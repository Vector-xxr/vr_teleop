"""Live OpenCV camera preview helpers for tcp_forward_grasp."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

WINDOW_NAME = "tcp_forward_grasp camera"


def preview_enabled(args) -> bool:
    if bool(getattr(args, "no_preview", False)):
        return False
    return bool(getattr(args, "preview", True))


def resize_preview(image: np.ndarray, max_width: int) -> np.ndarray:
    max_w = max(240, int(max_width))
    if image.shape[1] <= max_w:
        return image
    scale = max_w / float(image.shape[1])
    return cv2.resize(
        image,
        (max_w, int(round(image.shape[0] * scale))),
        interpolation=cv2.INTER_AREA,
    )


def grab_bgr(cam) -> np.ndarray | None:
    if hasattr(cam, "read_color"):
        frame = cam.read_color()
    elif hasattr(cam, "read"):
        frame = cam.read()
        if isinstance(frame, tuple) and len(frame) == 2:
            _ok, frame = frame
    elif hasattr(cam, "get_frame"):
        frame = cam.get_frame()
    else:
        raise RuntimeError("camera object has no read_color/read/get_frame")
    if frame is None:
        return None
    return frame


def draw_status(frame: np.ndarray, lines: list[str]) -> np.ndarray:
    out = frame.copy()
    y = 28
    for line in lines:
        cv2.putText(
            out,
            line,
            (16, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            lineType=cv2.LINE_AA,
        )
        y += 28
    return out


def annotate_detection(frame: np.ndarray, record: dict[str, Any] | None, args, servo) -> np.ndarray:
    """Draw white-plate ROI, gripper gate, red target, and green align center."""
    out = frame.copy()
    h, w = out.shape[:2]
    try:
        from .gripper_gate import draw_gate, load_gripper_gate
        from .grasp_lr_forward import detect_gripper_gate
        from .vision_align import build_white_paper_mask, detect_rightmost_red

        paper_mask, _info = build_white_paper_mask(frame, args)
        if paper_mask is not None and int(paper_mask.max()) > 0:
            tint = out.astype(np.float32)
            paper = paper_mask > 0
            tint[paper] = tint[paper] * 0.72 + np.array([40.0, 180.0, 40.0]) * 0.28
            out = tint.astype(np.uint8)
        gate = getattr(args, "locked_gripper_gate", None)
        if not (isinstance(gate, dict) and gate.get("locked")):
            gate = load_gripper_gate(getattr(args, "gripper_gate_file", None))
        if not (isinstance(gate, dict) and gate.get("u_left") is not None):
            gate = detect_gripper_gate(frame, args)
        if gate:
            locked = bool(gate.get("locked")) or str(gate.get("source", "")).startswith(
                ("calibrated", "locked", "soft_lock", "FIXED")
            )
            color = (0, 255, 255) if locked else (255, 200, 0)
            tag = "gate FIXED" if locked else f"gate {gate.get('source', 'live')}"
            draw_gate(out, gate, color=color, label=tag)
        live = detect_rightmost_red(frame, args) or {}
        if live.get("detected") and not (record and record.get("detected")):
            record = live
    except Exception:
        pass
    try:
        cx, cy = servo.target_center_px(args, w, h)
    except Exception:
        cx, cy = w * 0.5, h * 0.5
    cv2.drawMarker(
        out,
        (int(round(cx)), int(round(cy))),
        (0, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=2,
    )
    if record and record.get("detected"):
        center = record.get("center_px") or []
        if len(center) >= 2:
            ux, vy = float(center[0]), float(center[1])
            cv2.circle(out, (int(round(ux)), int(round(vy))), 12, (0, 0, 255), 2)
            for c in record.get("all_centers_left_to_right") or []:
                cv2.circle(out, (int(round(c[0])), int(round(c[1]))), 5, (255, 128, 0), 1)
            err = record.get("error_px") or [0.0, 0.0]
            bbox = record.get("bbox_xywh")
            if bbox and len(bbox) >= 4:
                x, y, bw, bh = [int(v) for v in bbox]
                cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.putText(
                out,
                f"rightmost red err=({float(err[0]):+.1f},{float(err[1]):+.1f})px",
                (16, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                lineType=cv2.LINE_AA,
            )
    return out


def show_frame(
    frame: np.ndarray,
    args,
    *,
    status: str = "",
    record: dict[str, Any] | None = None,
    servo=None,
) -> int:
    """Show one preview frame. Returns waitKey code, or -1 if preview off/failed."""
    if not preview_enabled(args):
        return -1
    try:
        out = frame
        if servo is not None:
            out = annotate_detection(frame, record, args, servo)
        lines = [x for x in [status] if x]
        if lines:
            out = draw_status(out, lines)
        max_w = int(getattr(args, "preview_max_width", 960) or 960)
        cv2.imshow(WINDOW_NAME, resize_preview(out, max_w))
        return int(cv2.waitKey(1) & 0xFF)
    except Exception as exc:
        print(f"[tcp-flow][WARN] camera preview failed: {exc}", flush=True)
        # Disable further attempts this run.
        args.preview = False
        args.no_preview = True
        return -1


def close_preview(args) -> None:
    if not preview_enabled(args) and not bool(getattr(args, "preview", False)):
        # Still try destroy in case window was opened earlier.
        pass
    try:
        cv2.destroyWindow(WINDOW_NAME)
        cv2.waitKey(1)
    except Exception:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
