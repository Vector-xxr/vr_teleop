#!/usr/bin/env python3
"""
Step-by-step 2D skewer endpoint alignment for OpenArm eye-in-hand top view.

This is intentionally simple and staged:

1. replay-observe: replay the recorded right-arm observation action.
2. detect: open the wrist camera and detect the black-painted grasp region.
3. align-once: reserved; currently computes and prints the 2D correction only.
4. grasp: reserved; no robot descent is executed yet.

The current robust path is to validate detection first. The detector finds a
dark connected region, reports its center, and compares it to the image center.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import ModuleType

if "wayland" in os.environ.get("XDG_SESSION_TYPE", "").lower():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
VR_ROOT = ROOT.parent
VENDOR_ROOT = ROOT / "vendor"
TEMPLATE_ROOT = (
    VENDOR_ROOT / "template_library"
    if (VENDOR_ROOT / "template_library").exists()
    else VR_ROOT / "template_library"
)
OPENARM_INPLACE = (
    VENDOR_ROOT / "openarm_inplace"
    if (VENDOR_ROOT / "openarm_inplace").exists()
    else VR_ROOT / "openarm_inplace"
)
DEFAULT_OBSERVE_REPLAY = ROOT / "outputs" / "right_observe_record_20260723_135046" / "real_follower_right_03.csv"


@dataclass
class Detection:
    center_px: tuple[float, float]
    image_center_px: tuple[float, float]
    target_center_px: tuple[float, float]
    error_px: tuple[float, float]
    area_px: float
    bbox_xywh: tuple[int, int, int, int]
    angle_deg: float
    elongation: float
    angle_from_vertical_deg: float
    mask: np.ndarray
    contour: np.ndarray
    source: str = "black"


def contour_axis_fraction_point(contour: np.ndarray, theta_deg: float, fraction: float) -> tuple[float, float]:
    pts = contour.reshape(-1, 2).astype(np.float64)
    if pts.size == 0:
        return 0.0, 0.0
    frac = min(1.0, max(0.0, float(fraction)))
    theta = np.deg2rad(float(theta_deg))
    axis = np.asarray([np.cos(theta), np.sin(theta)], dtype=np.float64)
    proj = pts @ axis
    lo = float(np.min(proj))
    hi = float(np.max(proj))
    target_proj = lo + frac * (hi - lo)
    center = np.mean(pts, axis=0)
    return tuple((center + (target_proj - float(center @ axis)) * axis).tolist())


def load_module(module_name: str, path: Path) -> ModuleType:
    if not path.exists():
        raise FileNotFoundError(path)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def send_udp(port: int, message: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message.encode("utf-8"), ("127.0.0.1", int(port)))
    finally:
        sock.close()


def with_sudo_password_env(env: dict[str, str], sudo_password: str) -> dict[str, str]:
    if sudo_password:
        env["OPENARM_SUDO_PASSWORD"] = sudo_password
    return env


def replay_duration_s(csv_path: Path) -> float:
    last_time = 0.0
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        header = f.readline()
        if "time" not in header.split(",")[0]:
            return 0.0
        for line in f:
            if not line.strip():
                continue
            try:
                last_time = float(line.split(",", 1)[0])
            except ValueError:
                continue
    return max(0.0, last_time)


def stop_process(proc: subprocess.Popen | None, label: str = "process") -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[{label}] stopping")
    proc.terminate()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)


def run_observe_replay(args: argparse.Namespace, *, hold_after_replay: bool = False) -> subprocess.Popen | None:
    replay = args.observe_replay.expanduser().resolve()
    if not replay.exists():
        raise FileNotFoundError(replay)
    script = OPENARM_INPLACE / "script" / "run_cpp_real_replay.sh"
    if not script.exists():
        raise FileNotFoundError(script)

    position_dump = args.output_dir / "observe_position_dump.txt"
    cmd = [
        str(script),
        str(replay),
        "--no-auto-home",
        "--initial-transition-duration",
        str(args.initial_transition_duration),
        "--replay-start-port",
        str(args.replay_start_port),
        "--record-start-port",
        str(args.record_start_port),
        "--position-dump",
        str(position_dump),
    ]
    if args.keep_enabled_on_exit:
        cmd.append("--keep-enabled-on-exit")

    env = with_sudo_password_env(
        {
            **os.environ,
            "OPENARM_TELEOP_ARM_SIDE": "right_arm",
            "OPENARM_FOLLOWER_CAN": args.follower_can,
            "OPENARM_FOLLOWER_ONLY": "1" if args.follower_only else "0",
        },
        args.sudo_password,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"[observe] replay={replay}")
    print(f"[observe] follower_can={args.follower_can} follower_only={args.follower_only}")
    print(f"[observe] command port={args.replay_start_port}")
    proc = subprocess.Popen(cmd, cwd=str(OPENARM_INPLACE), env=env)
    time.sleep(0.4)
    if proc.poll() is not None:
        raise RuntimeError(f"observe replay exited early with code {proc.returncode}")
    send_udp(args.replay_start_port, "start")
    wait_s = replay_duration_s(replay) + float(args.initial_transition_duration) + float(args.observe_settle_s)
    print(f"[observe] waiting {wait_s:.2f}s for replay to finish")
    time.sleep(wait_s)
    if proc.poll() is not None:
        if proc.returncode not in (0, -15):
            raise RuntimeError(f"observe replay failed with code {proc.returncode}")
        print(f"[observe] done returncode={proc.returncode}")
        return None
    if hold_after_replay:
        print("[observe] replay reached final time; keeping same process alive for next stages")
        return proc

    stopped_by_script = False
    if proc.poll() is None:
        print("[observe] replay reached expected final time; stopping hold process")
        stopped_by_script = True
        proc.terminate()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        stopped_by_script = True
        proc.kill()
        proc.wait(timeout=2.0)
    if proc.returncode not in (0, -15) and not stopped_by_script:
        raise RuntimeError(f"observe replay failed with code {proc.returncode}")
    print(f"[observe] done returncode={proc.returncode}")
    return None


class Camera:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.capture_module: ModuleType | None = None
        self.orbbec = None
        self.cap = None

    def __enter__(self):
        if self.args.camera_backend == "orbbec":
            self.capture_module = load_module(
                "skewer_2d_capture_rgbd",
                TEMPLATE_ROOT / "rgbd_samurai_minimal" / "01_capture_rgbd.py",
            )
            self.orbbec = self.capture_module.OrbbecRgbdCamera(
                serial=str(self.args.orbbec_serial or "") or None
            )
            self.orbbec.start()
        else:
            self.cap = cv2.VideoCapture(int(self.args.camera_index))
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open OpenCV camera index {self.args.camera_index}")
            if self.args.camera_width > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.args.camera_width))
            if self.args.camera_height > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.args.camera_height))
            if self.args.camera_fps > 0:
                self.cap.set(cv2.CAP_PROP_FPS, int(self.args.camera_fps))
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.orbbec is not None:
            self.orbbec.stop()
        if self.cap is not None:
            self.cap.release()

    def read_color(self) -> np.ndarray | None:
        if self.orbbec is not None:
            assert self.capture_module is not None
            latest = None
            frames = self.orbbec.pipeline.wait_for_frames(int(self.args.frame_timeout_ms))
            if frames is not None:
                color_frame = frames.get_color_frame()
                if color_frame is not None:
                    latest = self.capture_module.frame_to_bgr(color_frame)
            for _ in range(max(0, int(self.args.flush_frames))):
                frames = self.orbbec.pipeline.wait_for_frames(1)
                if frames is None:
                    break
                color_frame = frames.get_color_frame()
                if color_frame is not None:
                    latest = self.capture_module.frame_to_bgr(color_frame)
            return latest
        if self.cap is not None:
            ok, frame = self.cap.read()
            return frame if ok else None
        return None


def apply_detection_roi(
    mask: np.ndarray,
    args: argparse.Namespace,
    *,
    left: float | None = None,
    right: float | None = None,
    top: float | None = None,
    bottom: float | None = None,
) -> np.ndarray:
    h, w = mask.shape[:2]
    roi_left_f = float(args.roi_left) if left is None else float(left)
    roi_right_f = float(args.roi_right) if right is None else float(right)
    roi_top_f = float(args.roi_top) if top is None else float(top)
    roi_bottom_f = float(args.roi_bottom) if bottom is None else float(bottom)
    roi_left = int(round(w * roi_left_f))
    roi_right = int(round(w * roi_right_f))
    roi_top = int(round(h * roi_top_f))
    roi_bottom = int(round(h * roi_bottom_f))
    roi_left = max(0, min(w, roi_left))
    roi_right = max(0, min(w, roi_right))
    roi_top = max(0, min(h, roi_top))
    roi_bottom = max(0, min(h, roi_bottom))
    roi_mask = np.zeros_like(mask)
    if roi_right > roi_left and roi_bottom > roi_top:
        roi_mask[roi_top:roi_bottom, roi_left:roi_right] = 255
        mask = cv2.bitwise_and(mask, roi_mask)

    margin = max(0, int(args.ignore_border_px))
    if margin:
        mask[:margin, :] = 0
        mask[-margin:, :] = 0
        mask[:, :margin] = 0
        mask[:, -margin:] = 0
    return mask


def morph_mask(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    kernel_size = max(1, int(kernel_size))
    if kernel_size <= 1:
        return mask
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def contour_features(contour: np.ndarray) -> tuple[float, tuple[int, int, int, int], float, float, float, float]:
    area = float(cv2.contourArea(contour))
    x, y, bw, bh = cv2.boundingRect(contour)
    rect = cv2.minAreaRect(contour)
    (_, _), (rw, rh), angle = rect
    if rw < 1.0 or rh < 1.0:
        return area, (x, y, bw, bh), 0.0, 0.0, 90.0, 0.0
    major = float(max(rw, rh))
    minor = float(max(1.0, min(rw, rh)))
    elongation = major / minor
    theta = float(angle if rw >= rh else angle + 90.0)
    theta = (theta + 180.0) % 180.0
    angle_from_vertical = abs(theta - 90.0)
    fill_ratio = area / float(max(1, bw * bh))
    return area, (x, y, bw, bh), theta, elongation, angle_from_vertical, fill_ratio


def contour_center(contour: np.ndarray, bbox: tuple[int, int, int, int]) -> tuple[float, float]:
    m = cv2.moments(contour)
    if abs(m["m00"]) < 1.0e-9:
        return bbox[0] + bbox[2] * 0.5, bbox[1] + bbox[3] * 0.5
    return m["m10"] / m["m00"], m["m01"] / m["m00"]


def red_region_candidates(
    image_bgr: np.ndarray,
    args: argparse.Namespace,
) -> tuple[list[tuple[float, np.ndarray, tuple[int, int, int, int], float, float, float, float]], np.ndarray]:
    h, w = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    h_low = int(getattr(args, "red_h_low", 8))
    h_high = int(getattr(args, "red_h_high", 172))
    red_hue = (hue <= h_low) | (hue >= h_high)
    mask = (
        red_hue
        & (saturation >= int(getattr(args, "red_s_min", 130)))
        & (value >= int(getattr(args, "red_v_min", 45)))
    ).astype(np.uint8) * 255
    mask = apply_detection_roi(
        mask,
        args,
        left=float(getattr(args, "red_roi_left", args.roi_left)),
        right=float(getattr(args, "red_roi_right", args.roi_right)),
        top=float(getattr(args, "red_roi_top", args.roi_top)),
        bottom=float(getattr(args, "red_roi_bottom", 0.62)),
    )
    mask = morph_mask(mask, max(3, int(args.morph_kernel)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area, bbox, theta, elongation, angle_from_vertical, fill_ratio = contour_features(contour)
        if area < float(getattr(args, "red_min_area_px", 80.0)):
            continue
        if area > float(getattr(args, "red_max_area_px", 3000.0)):
            continue
        if fill_ratio < float(getattr(args, "red_min_fill_ratio", 0.18)):
            continue
        candidates.append((area, contour, bbox, theta, elongation, angle_from_vertical, fill_ratio))
    return candidates, mask


def detect_red_region(image_bgr: np.ndarray, args: argparse.Namespace) -> Detection | None:
    h, w = image_bgr.shape[:2]
    candidates, mask = red_region_candidates(image_bgr, args)
    if not candidates:
        return None

    cx, cy = target_center_px(args, w, h)

    def score(item):
        area, _contour, bbox, _theta, _elongation, _angle_from_vertical, fill_ratio = item
        ux, vy = contour_center(item[1], bbox)
        dist = float(np.hypot(ux - cx, vy - cy))
        bw, bh = bbox[2], bbox[3]
        size_bonus = min(float(max(bw, bh)), 120.0)
        return 2.0 * area + 120.0 * fill_ratio + size_bonus - 0.30 * dist

    ordered = sorted(candidates, key=score, reverse=True)
    candidate_index = int(getattr(args, "target_candidate_index", 0) or 0)
    if candidate_index > 0:
        if candidate_index > len(ordered):
            print(f"[detect][WARN] red target candidate {candidate_index} requested, but only {len(ordered)} candidate(s)")
            return None
        selected = ordered[candidate_index - 1]
    else:
        selected = ordered[0]

    area, contour, bbox, theta, elongation, angle_from_vertical, _fill_ratio = selected
    fraction = float(getattr(args, "red_grasp_point_fraction", 0.50))
    if 0.0 <= fraction <= 1.0 and elongation >= 1.15:
        ux, vy = contour_axis_fraction_point(contour, theta, fraction)
    else:
        ux, vy = contour_center(contour, bbox)
    if not np.isfinite(ux) or not np.isfinite(vy):
        ux = bbox[0] + bbox[2] * 0.5
        vy = bbox[1] + bbox[3] * 0.5
    return Detection(
        center_px=(float(ux), float(vy)),
        image_center_px=(float(w * 0.5), float(h * 0.5)),
        target_center_px=(float(cx), float(cy)),
        error_px=(float(ux - cx), float(vy - cy)),
        area_px=float(area),
        bbox_xywh=tuple(int(v) for v in bbox),
        angle_deg=float(theta),
        elongation=float(elongation),
        angle_from_vertical_deg=float(angle_from_vertical),
        mask=mask,
        contour=contour,
        source="red",
    )


def detect_black_region(image_bgr: np.ndarray, args: argparse.Namespace) -> Detection | None:
    h, w = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    saturation = hsv[:, :, 1]
    mask = (value <= int(args.black_v_max)).astype(np.uint8) * 255
    if args.black_s_min > 0:
        mask = cv2.bitwise_and(mask, ((saturation >= int(args.black_s_min)).astype(np.uint8) * 255))
    mask = apply_detection_roi(mask, args)
    mask = morph_mask(mask, int(args.morph_kernel))

    red_anchors: list[tuple[float, float]] = []
    if args.target_selection == "skewer" and bool(getattr(args, "red_fallback", True)):
        red_candidates, _red_mask = red_region_candidates(image_bgr, args)
        red_anchors = [contour_center(contour, bbox) for _area, contour, bbox, *_rest in red_candidates]

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    min_area_px = float(args.min_area_px)
    max_angle_from_vertical = float(args.max_angle_from_vertical_deg)
    if red_anchors:
        min_area_px = min(min_area_px, float(getattr(args, "skewer_min_area_px", 35.0)))
        max_angle_from_vertical = max(
            max_angle_from_vertical,
            float(getattr(args, "skewer_anchor_max_angle_from_vertical_deg", 60.0)),
        )
    for contour in contours:
        area, bbox, theta, elongation, angle_from_vertical, fill_ratio = contour_features(contour)
        if area < min_area_px or area > float(args.max_area_px):
            continue
        x, y, bw, bh = bbox
        if bw <= 0 or bh <= 0:
            continue
        if elongation < float(args.min_elongation):
            continue
        if angle_from_vertical > max_angle_from_vertical:
            continue
        if fill_ratio < float(args.min_fill_ratio):
            continue
        candidates.append((area, contour, (x, y, bw, bh), theta, elongation, angle_from_vertical, fill_ratio))
    if not candidates:
        if bool(getattr(args, "red_fallback", True)):
            return detect_red_region(image_bgr, args)
        return None

    # Reject the dark horizontal top border and the large dark gripper area at
    # the bottom. In servo mode the intended target is usually the valid dark
    # mark closest to the camera center.
    cx, cy = target_center_px(args, w, h)
    def center_and_score(item):
        area, contour, bbox, _theta, elongation, angle_from_vertical, _fill_ratio = item
        ux, vy = contour_center(contour, bbox)
        dist = float(np.hypot(ux - cx, vy - cy))
        score_value = area + 35.0 * elongation - 6.0 * angle_from_vertical - 1.8 * dist
        x, y, bw, bh = bbox
        major_px = float(max(bw, bh))
        nearest_red_dist = min((float(np.hypot(ux - ru, vy - rv)) for ru, rv in red_anchors), default=9999.0)
        skewer_score = (
            2.5 * min(area, float(getattr(args, "skewer_preferred_area_px", 600.0)))
            + 80.0 * elongation
            + 1.5 * major_px
            - 12.0 * angle_from_vertical
            - 0.35 * dist
            - 25.0 * max(0.0, nearest_red_dist - float(getattr(args, "skewer_red_anchor_px", 75.0)))
        )
        return ux, vy, dist, score_value, skewer_score, nearest_red_dist

    skip_centers = []
    for item in getattr(args, "target_skip_centers_px", []) or []:
        if item is None:
            continue
        try:
            su, sv = item
            skip_centers.append((float(su), float(sv)))
        except (TypeError, ValueError):
            continue
    skip_radius = max(0.0, float(getattr(args, "target_skip_radius_px", 45.0) or 0.0))
    if skip_centers and skip_radius > 0.0:
        kept = []
        for item in candidates:
            ux, vy, *_ = center_and_score(item)
            too_close = any(float(np.hypot(ux - su, vy - sv)) <= skip_radius for su, sv in skip_centers)
            if not too_close:
                kept.append(item)
        if kept:
            candidates = kept
        else:
            print("[detect][WARN] target skip removed all candidates; using full candidate set")

    candidate_index = int(getattr(args, "target_candidate_index", 0) or 0)
    selection_candidates = candidates
    if args.target_selection == "skewer" and red_anchors:
        anchor_radius = float(getattr(args, "skewer_red_anchor_px", 75.0))
        anchored = [item for item in candidates if center_and_score(item)[5] <= anchor_radius]
        if anchored:
            selection_candidates = anchored
    if args.target_selection == "largest":
        ordered = sorted(selection_candidates, key=lambda item: item[0], reverse=True)
    elif args.target_selection == "score":
        ordered = sorted(selection_candidates, key=lambda item: center_and_score(item)[3], reverse=True)
    elif args.target_selection == "skewer":
        ordered = sorted(selection_candidates, key=lambda item: center_and_score(item)[4], reverse=True)
    else:
        ordered = sorted(selection_candidates, key=lambda item: center_and_score(item)[2])
    if candidate_index > 0:
        if candidate_index > len(ordered):
            print(f"[detect][WARN] target candidate {candidate_index} requested, but only {len(ordered)} candidate(s)")
            return None
        else:
            selected = ordered[candidate_index - 1]
    else:
        selected = ordered[0]
    area, contour, bbox, theta, elongation, angle_from_vertical, _fill_ratio = selected
    fraction = float(getattr(args, "grasp_point_fraction", 0.70))
    if 0.0 <= fraction <= 1.0:
        ux, vy = contour_axis_fraction_point(contour, theta, fraction)
    else:
        m = cv2.moments(contour)
        if abs(m["m00"]) < 1.0e-9:
            ux = bbox[0] + bbox[2] * 0.5
            vy = bbox[1] + bbox[3] * 0.5
        else:
            ux = m["m10"] / m["m00"]
            vy = m["m01"] / m["m00"]
    if not np.isfinite(ux) or not np.isfinite(vy):
        ux = bbox[0] + bbox[2] * 0.5
        vy = bbox[1] + bbox[3] * 0.5
    return Detection(
        center_px=(float(ux), float(vy)),
        image_center_px=(float(w * 0.5), float(h * 0.5)),
        target_center_px=(float(cx), float(cy)),
        error_px=(float(ux - cx), float(vy - cy)),
        area_px=float(area),
        bbox_xywh=tuple(int(v) for v in bbox),
        angle_deg=float(theta),
        elongation=float(elongation),
        angle_from_vertical_deg=float(angle_from_vertical),
        mask=mask,
        contour=contour,
        source="black",
    )


def target_center_px(args: argparse.Namespace, width: int, height: int) -> tuple[float, float]:
    text = str(getattr(args, "target_center_px", "") or "").strip()
    if not text:
        return width * 0.5, height * 0.5
    values = [float(v.strip()) for v in text.split(",") if v.strip()]
    if len(values) != 2:
        raise RuntimeError("--target-center-px must be u,v, for example --target-center-px=320,360")
    return values[0], values[1]


def draw_detection(image_bgr: np.ndarray, det: Detection | None, args: argparse.Namespace) -> np.ndarray:
    out = image_bgr.copy()
    h, w = out.shape[:2]
    image_cx, image_cy = int(round(w * 0.5)), int(round(h * 0.5))
    cx_f, cy_f = target_center_px(args, w, h)
    cx, cy = int(round(cx_f)), int(round(cy_f))
    if (cx, cy) != (image_cx, image_cy):
        cv2.drawMarker(out, (image_cx, image_cy), (90, 90, 90), cv2.MARKER_CROSS, 20, 1)
    cv2.drawMarker(out, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 28, 2)
    cv2.line(out, (cx - 18, cy), (cx + 18, cy), (0, 160, 0), 1)
    cv2.line(out, (cx, cy - 18), (cx, cy + 18), (0, 160, 0), 1)
    if det is None:
        cv2.putText(out, "no skewer grasp region", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return out

    ux, vy = det.center_px
    x, y, bw, bh = det.bbox_xywh
    contour_color = (0, 0, 255) if det.source == "red" else (0, 180, 255)
    cv2.drawContours(out, [det.contour], -1, contour_color, 2)
    cv2.rectangle(out, (x, y), (x + bw, y + bh), (255, 0, 0), 2)
    cv2.circle(out, (int(round(ux)), int(round(vy))), 6, contour_color, -1)
    cv2.arrowedLine(out, (cx, cy), (int(round(ux)), int(round(vy))), (0, 255, 255), 2, tipLength=0.15)
    du, dv = det.error_px
    text = (
        f"{det.source} target=({ux:.1f},{vy:.1f}) err=({du:+.1f},{dv:+.1f})px "
        f"area={det.area_px:.0f} elong={det.elongation:.1f} frac={float(getattr(args, 'grasp_point_fraction', 0.70)):.2f}"
    )
    cv2.putText(out, text, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 255), 2, lineType=cv2.LINE_AA)
    if args.pixel_to_base_mm:
        dx, dy = pixel_error_to_base_mm(det.error_px, args)
        cv2.putText(out, f"planned base correction dx={dx:+.1f}mm dy={dy:+.1f}mm", (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2, lineType=cv2.LINE_AA)
    return out


def pixel_error_to_base_mm(error_px: tuple[float, float], args: argparse.Namespace) -> tuple[float, float]:
    values = [float(v.strip()) for v in str(args.pixel_to_base_mm).split(",") if v.strip()]
    if len(values) == 2:
        kx, ky = values
        du, dv = error_px
        return kx * du, ky * dv
    if len(values) == 4:
        a, b, c, d = values
        du, dv = error_px
        return a * du + b * dv, c * du + d * dv
    raise RuntimeError("--pixel-to-base-mm must be kx,ky or a,b,c,d")


def save_detection(run_dir: Path, image_bgr: np.ndarray, vis_bgr: np.ndarray, det: Detection | None, args: argparse.Namespace) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(run_dir / "latest_rgb.png"), image_bgr)
    cv2.imwrite(str(run_dir / "latest_vis.png"), vis_bgr)
    payload = {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "detected": det is not None,
        "image_shape": list(image_bgr.shape),
        "detector": {
            "black_v_max": args.black_v_max,
            "black_s_min": args.black_s_min,
            "min_area_px": args.min_area_px,
            "max_area_px": args.max_area_px,
            "morph_kernel": args.morph_kernel,
            "roi_left": args.roi_left,
            "roi_right": args.roi_right,
            "roi_top": args.roi_top,
            "roi_bottom": args.roi_bottom,
            "min_elongation": args.min_elongation,
            "max_angle_from_vertical_deg": args.max_angle_from_vertical_deg,
            "skewer_min_area_px": float(getattr(args, "skewer_min_area_px", 35.0)),
            "skewer_red_anchor_px": float(getattr(args, "skewer_red_anchor_px", 75.0)),
            "skewer_anchor_max_angle_from_vertical_deg": float(
                getattr(args, "skewer_anchor_max_angle_from_vertical_deg", 60.0)
            ),
            "skewer_preferred_area_px": float(getattr(args, "skewer_preferred_area_px", 600.0)),
            "target_selection": args.target_selection,
            "grasp_point_fraction": float(getattr(args, "grasp_point_fraction", 0.70)),
            "red_fallback": bool(getattr(args, "red_fallback", True)),
            "red_h_low": int(getattr(args, "red_h_low", 8)),
            "red_h_high": int(getattr(args, "red_h_high", 172)),
            "red_s_min": int(getattr(args, "red_s_min", 130)),
            "red_v_min": int(getattr(args, "red_v_min", 45)),
            "red_min_area_px": float(getattr(args, "red_min_area_px", 80.0)),
            "red_max_area_px": float(getattr(args, "red_max_area_px", 3000.0)),
            "red_grasp_point_fraction": float(getattr(args, "red_grasp_point_fraction", 0.50)),
            "red_roi_left": float(getattr(args, "red_roi_left", args.roi_left)),
            "red_roi_right": float(getattr(args, "red_roi_right", args.roi_right)),
            "red_roi_top": float(getattr(args, "red_roi_top", args.roi_top)),
            "red_roi_bottom": float(getattr(args, "red_roi_bottom", 0.62)),
        },
    }
    if det is not None:
        payload.update(
            {
                "center_px": list(det.center_px),
                "image_center_px": list(det.image_center_px),
                "target_center_px": list(det.target_center_px),
                "error_px": list(det.error_px),
                "area_px": det.area_px,
                "bbox_xywh": list(det.bbox_xywh),
                "angle_deg": det.angle_deg,
                "elongation": det.elongation,
                "angle_from_vertical_deg": det.angle_from_vertical_deg,
                "source": det.source,
            }
        )
        if args.pixel_to_base_mm:
            payload["planned_base_correction_mm"] = list(pixel_error_to_base_mm(det.error_px, args))
        cv2.imwrite(str(run_dir / "latest_mask.png"), det.mask)
    (run_dir / "latest_detection.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[detect] saved {run_dir / 'latest_detection.json'}")


def run_detect_loop(args: argparse.Namespace) -> None:
    run_dir = args.output_dir / datetime.now().strftime("detect_%Y%m%d_%H%M%S")
    print("[detect] keys: d detect | s save | a print align correction | q/Esc quit")
    print(f"[detect] output={run_dir}")
    last_frame = None
    last_det = None
    last_vis = None
    with Camera(args) as cam:
        cv2.namedWindow("Skewer 2D Align", cv2.WINDOW_NORMAL)
        while True:
            frame = cam.read_color()
            if frame is not None:
                last_frame = frame
            if last_frame is None:
                key = cv2.waitKey(1) & 0xFF
            else:
                display = draw_detection(last_frame, last_det, args) if last_det is not None else last_frame.copy()
                h, w = display.shape[:2]
                cv2.drawMarker(display, (w // 2, h // 2), (0, 255, 0), cv2.MARKER_CROSS, 28, 2)
                cv2.putText(display, "d:detect  s:save  a:align print only  q:quit", (18, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, lineType=cv2.LINE_AA)
                max_w = max(240, int(args.preview_max_width))
                if display.shape[1] > max_w:
                    scale = max_w / float(display.shape[1])
                    display = cv2.resize(display, (max_w, int(round(display.shape[0] * scale))), interpolation=cv2.INTER_AREA)
                cv2.imshow("Skewer 2D Align", display)
                key = cv2.waitKey(1) & 0xFF

            if key in (ord("d"), ord("D")) and last_frame is not None:
                last_det = detect_black_region(last_frame, args)
                last_vis = draw_detection(last_frame, last_det, args)
                if last_det is None:
                    print("[detect] no black region. Try increasing --black-v-max or lowering --min-area-px.")
                else:
                    du, dv = last_det.error_px
                    print(
                        "[detect] center="
                        f"({last_det.center_px[0]:.1f},{last_det.center_px[1]:.1f}) "
                        f"err=({du:+.1f},{dv:+.1f})px area={last_det.area_px:.0f} "
                        f"bbox={last_det.bbox_xywh} elong={last_det.elongation:.1f} "
                        f"angle_from_vertical={last_det.angle_from_vertical_deg:.1f}"
                    )
                    if args.pixel_to_base_mm:
                        dx, dy = pixel_error_to_base_mm(last_det.error_px, args)
                        print(f"[align-preview] base correction dx={dx:+.2f}mm dy={dy:+.2f}mm")

            if key in (ord("s"), ord("S")) and last_frame is not None:
                if last_vis is None:
                    last_vis = draw_detection(last_frame, last_det, args)
                save_detection(run_dir, last_frame, last_vis, last_det, args)

            if key in (ord("a"), ord("A")):
                if last_det is None:
                    print("[align-preview] no detection yet. Press d first.")
                elif not args.pixel_to_base_mm:
                    print("[align-preview] set --pixel-to-base-mm after calibration; no robot motion is executed.")
                else:
                    dx, dy = pixel_error_to_base_mm(last_det.error_px, args)
                    print(f"[align-preview] would move base dx={dx:+.2f}mm dy={dy:+.2f}mm (NOT executed)")

            if key in (ord("q"), 27):
                break
    cv2.destroyAllWindows()


def run_detect_image(args: argparse.Namespace) -> None:
    image_path = args.image.expanduser().resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")
    run_dir = args.output_dir / datetime.now().strftime("detect_image_%Y%m%d_%H%M%S_%f")
    det = detect_black_region(image, args)
    vis = draw_detection(image, det, args)
    save_detection(run_dir, image, vis, det, args)
    if det is None:
        print("[detect-image] no black region")
    else:
        du, dv = det.error_px
        print(
            "[detect-image] center="
            f"({det.center_px[0]:.1f},{det.center_px[1]:.1f}) "
            f"err=({du:+.1f},{dv:+.1f})px area={det.area_px:.0f} "
            f"bbox={det.bbox_xywh} elong={det.elongation:.1f} "
            f"angle_from_vertical={det.angle_from_vertical_deg:.1f}"
        )
    print(f"[detect-image] vis={run_dir / 'latest_vis.png'}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Step-by-step 2D skewer endpoint alignment.")
    parser.add_argument(
        "--stage",
        choices=("replay-observe", "detect", "detect-image", "observe-and-detect", "align-once", "grasp"),
        default="detect",
        help="Run one staged step. align-once/grasp are placeholders for now.",
    )
    parser.add_argument("--image", type=Path, default=None, help="--stage detect-image 时使用的离线 RGB 图片。")
    parser.add_argument("--observe-replay", type=Path, default=DEFAULT_OBSERVE_REPLAY)
    parser.add_argument("--follower-can", default="can1")
    parser.add_argument("--follower-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sudo-password", default="")
    parser.add_argument("--initial-transition-duration", type=float, default=5.0)
    parser.add_argument("--observe-settle-s", type=float, default=0.5)
    parser.add_argument("--replay-start-port", type=int, default=47920)
    parser.add_argument("--record-start-port", type=int, default=47931)
    parser.add_argument("--keep-enabled-on-exit", action="store_true")

    parser.add_argument("--camera-backend", choices=("orbbec", "opencv"), default="orbbec")
    parser.add_argument("--orbbec-serial", default="")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-width", type=int, default=0)
    parser.add_argument("--camera-height", type=int, default=0)
    parser.add_argument("--camera-fps", type=int, default=0)
    parser.add_argument("--frame-timeout-ms", type=int, default=30)
    parser.add_argument("--flush-frames", type=int, default=8)
    parser.add_argument("--preview-max-width", type=int, default=960)

    parser.add_argument("--black-v-max", type=int, default=80, help="HSV V threshold for black painted region.")
    parser.add_argument("--black-s-min", type=int, default=0, help="Optional HSV S lower bound.")
    parser.add_argument("--min-area-px", type=float, default=180.0)
    parser.add_argument("--max-area-px", type=float, default=8000.0)
    parser.add_argument("--morph-kernel", type=int, default=3)
    parser.add_argument("--ignore-border-px", type=int, default=8)
    parser.add_argument("--roi-left", type=float, default=0.0, help="检测 ROI 左边界比例。")
    parser.add_argument("--roi-right", type=float, default=1.0, help="检测 ROI 右边界比例。")
    parser.add_argument("--roi-top", type=float, default=0.12, help="检测 ROI 上边界比例；默认跳过顶部暗边。")
    parser.add_argument("--roi-bottom", type=float, default=0.82, help="检测 ROI 下边界比例；默认跳过底部夹爪。")
    parser.add_argument("--min-elongation", type=float, default=2.0, help="黑色涂层应是细长区域。")
    parser.add_argument("--max-angle-from-vertical-deg", type=float, default=35.0, help="候选长轴离图像竖直方向的最大角度。")
    parser.add_argument("--min-fill-ratio", type=float, default=0.08, help="轮廓面积 / bbox 面积下限。")
    parser.add_argument("--skewer-min-area-px", type=float, default=35.0, help="skewer 模式且有红色锚点时，允许的黑尖最小面积。")
    parser.add_argument("--skewer-red-anchor-px", type=float, default=75.0, help="skewer 模式下黑尖到红色套中心的最大锚定距离。")
    parser.add_argument(
        "--skewer-anchor-max-angle-from-vertical-deg",
        type=float,
        default=60.0,
        help="有红色锚点时，黑尖候选允许的最大竖直偏角。",
    )
    parser.add_argument("--skewer-preferred-area-px", type=float, default=600.0, help="skewer 打分中黑色面积的封顶值，避免大机械黑块压过小黑尖。")
    parser.add_argument(
        "--target-selection",
        choices=("skewer", "center", "score", "largest"),
        default="skewer",
        help="多个涂黑区域时的选择策略；skewer=优先选面积足够、细长、接近竖直的签子涂黑段。",
    )
    parser.add_argument(
        "--grasp-point-fraction",
        type=float,
        default=0.70,
        help="沿黑色涂层长轴取抓取点的位置。0.50 是中心；0.70 是从上端到下端 70%%，更靠后/靠下。",
    )
    parser.add_argument(
        "--red-fallback",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="黑色细长段检测不到时，自动检测红色标记区域。适合当前带红色套/红色标记的签子。",
    )
    parser.add_argument("--red-h-low", type=int, default=8, help="红色 HSV 低 hue 上限。")
    parser.add_argument("--red-h-high", type=int, default=172, help="红色 HSV 高 hue 下限。")
    parser.add_argument("--red-s-min", type=int, default=130, help="红色 HSV 饱和度下限。")
    parser.add_argument("--red-v-min", type=int, default=45, help="红色 HSV 亮度下限。")
    parser.add_argument("--red-min-area-px", type=float, default=80.0, help="红色候选最小面积。")
    parser.add_argument("--red-max-area-px", type=float, default=3000.0, help="红色候选最大面积。")
    parser.add_argument("--red-roi-left", type=float, default=0.0, help="红色 fallback ROI 左边界比例。")
    parser.add_argument("--red-roi-right", type=float, default=1.0, help="红色 fallback ROI 右边界比例。")
    parser.add_argument("--red-roi-top", type=float, default=0.12, help="红色 fallback ROI 上边界比例。")
    parser.add_argument("--red-roi-bottom", type=float, default=0.62, help="红色 fallback ROI 下边界比例；默认避开底部手和夹爪。")
    parser.add_argument(
        "--red-grasp-point-fraction",
        type=float,
        default=0.50,
        help="红色候选沿长轴取点位置。默认 0.50 使用红色区域中心。",
    )
    parser.add_argument(
        "--pixel-to-base-mm",
        default="",
        help="Optional calibration: kx,ky or 2x2 matrix a,b,c,d mapping [du,dv] px to [dx,dy] mm.",
    )
    parser.add_argument(
        "--target-center-px",
        default="",
        help="目标对齐像素 u,v。默认用图像中心；如果要对齐夹爪中心，传 --target-center-px=ux,vy。",
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "skewer_2d_servo")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.stage == "replay-observe":
        run_observe_replay(args)
        return 0
    if args.stage == "detect":
        run_detect_loop(args)
        return 0
    if args.stage == "detect-image":
        if args.image is None:
            raise RuntimeError("--stage detect-image requires --image")
        run_detect_image(args)
        return 0
    if args.stage == "observe-and-detect":
        replay_proc = run_observe_replay(args, hold_after_replay=True)
        try:
            run_detect_loop(args)
        finally:
            stop_process(replay_proc, label="observe")
        return 0
    if args.stage == "align-once":
        print("[align-once] placeholder: use --stage detect, press d, then a to print correction first.")
        return 0
    if args.stage == "grasp":
        print("[grasp] placeholder: fixed descent/grasp will be added after 2D alignment is verified.")
        return 0
    raise RuntimeError(f"unknown stage: {args.stage}")


if __name__ == "__main__":
    raise SystemExit(main())
