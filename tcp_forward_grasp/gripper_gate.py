"""Fixed gripper-gate (opening) box in image pixels.

White grippers are easy to confuse with the white plate, so we calibrate the
inner opening once (camera on, gripper open at approach scale) and reuse the
saved pixel box for every grasp — no online finger detection.
"""

from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from . import paths


DEFAULT_GATE_PATH = paths.MODULE_DIR / "config" / "gripper_gate_left.json"

GATE_KEYS = ("u_left", "u_right", "y_tip", "y_bottom")


def read_begin_gripper(begin_csv: Path | None = None) -> float:
    """Last-frame gripper (q7) from begin.csv — the open width used for approach."""
    path = Path(begin_csv or paths.DEFAULT_BEGIN_CSV).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    last = None
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            last = row
    if last is None:
        raise RuntimeError(f"empty replay CSV: {path}")
    if "q7" not in last:
        raise RuntimeError(f"no q7 column in {path}")
    return float(last["q7"])


def read_begin_last_q(begin_csv: Path | None = None) -> tuple[list[float], float]:
    """Return (q0..q6, gripper_q7) from the last frame of begin.csv."""
    path = Path(begin_csv or paths.DEFAULT_BEGIN_CSV).expanduser().resolve()
    last = None
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            last = row
    if last is None:
        raise RuntimeError(f"empty replay CSV: {path}")
    q = [float(last[f"q{i}"]) for i in range(7)]
    g = float(last["q7"])
    return q, g


def default_gate_path() -> Path:
    return Path(
        getattr(paths, "DEFAULT_GRIPPER_GATE_JSON", DEFAULT_GATE_PATH)
    ).expanduser().resolve()


def gate_is_valid(gate: dict[str, Any] | None) -> bool:
    if not isinstance(gate, dict):
        return False
    try:
        u0, u1 = float(gate["u_left"]), float(gate["u_right"])
        y0, y1 = float(gate["y_tip"]), float(gate["y_bottom"])
    except (KeyError, TypeError, ValueError):
        return False
    return u1 - u0 >= 10.0 and y1 - y0 >= 10.0


def normalize_gate(gate: dict[str, Any], *, source: str | None = None) -> dict[str, Any]:
    out = {
        "u_left": float(gate["u_left"]),
        "u_right": float(gate["u_right"]),
        "y_tip": float(gate["y_tip"]),
        "y_bottom": float(gate["y_bottom"]),
        "locked": True,
        "source": str(source or gate.get("source") or "calibrated_fixed"),
    }
    if out["u_right"] < out["u_left"]:
        out["u_left"], out["u_right"] = out["u_right"], out["u_left"]
    if out["y_bottom"] < out["y_tip"]:
        out["y_tip"], out["y_bottom"] = out["y_bottom"], out["y_tip"]
    for key in (
        "image_width",
        "image_height",
        "created_at",
        "note",
        "camera_index",
        "gripper_open_from_begin",
        "begin_csv",
        "tip_left_y",
        "tip_right_y",
    ):
        if key in gate:
            out[key] = gate[key]
    return out


def save_gripper_gate(path: Path, gate: dict[str, Any], *, note: str = "") -> Path:
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = normalize_gate(gate)
    payload["created_at"] = datetime.now().isoformat(timespec="seconds")
    if note:
        payload["note"] = note
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def load_gripper_gate(path: Path | None = None) -> dict[str, Any] | None:
    path = Path(path or default_gate_path()).expanduser().resolve()
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not gate_is_valid(data):
        return None
    out = normalize_gate(data)
    out["path"] = str(path)
    return out


def apply_fixed_gate_to_args(args, path: Path | None = None) -> dict[str, Any] | None:
    """Load calibrated gate onto args.locked_gripper_gate. Returns gate or None."""
    gate_path = path
    if gate_path is None:
        raw = getattr(args, "gripper_gate_file", None)
        gate_path = Path(raw) if raw else default_gate_path()
    gate = load_gripper_gate(gate_path)
    if gate is None:
        return None
    setattr(args, "locked_gripper_gate", dict(gate))
    setattr(args, "gripper_gate_file", str(gate_path))
    return gate


def draw_gate(
    frame: np.ndarray,
    gate: dict[str, Any] | None,
    *,
    color: tuple[int, int, int] = (0, 255, 255),
    label: str = "gate FIXED",
) -> np.ndarray:
    if not gate_is_valid(gate):
        return frame
    out = frame
    u0, u1 = int(round(gate["u_left"])), int(round(gate["u_right"]))
    y0, y1 = int(round(gate["y_tip"])), int(round(gate["y_bottom"]))
    cv2.rectangle(out, (u0, y0), (u1, y1), color, 2)
    cv2.line(out, (u0, y0), (u1, y0), (0, 255, 255), 2)
    cv2.putText(
        out,
        label,
        (u0, max(16, y0 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        1,
        lineType=cv2.LINE_AA,
    )
    return out


def propose_gate_from_green(args, width: int, height: int) -> dict[str, Any]:
    """Fallback: width around green; tip near white-finger tip depth.

    Green is recessed between fingers. Physical tip-to-tip grasp line is ahead
    of green (smaller image-y), but not past the white tip faces.
    """
    from .vision_align import target_center_px

    tx, ty = target_center_px(args, width, height)
    half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
    # ~75px: between recessed green and overshooting past tips.
    ahead = float(getattr(args, "calib_tip_ahead_of_green_px", 85.0))
    tip = float(ty - ahead)
    tip = float(np.clip(tip, 0.0, height - 20.0))
    bottom = float(height - 1)
    return normalize_gate(
        {
            "u_left": tx - half,
            "u_right": tx + half,
            "y_tip": tip,
            "y_bottom": bottom,
        },
        source="propose_green_tip_ahead",
    )


def _finger_tip_y(
    mask: np.ndarray,
    *,
    u0: int,
    u1: int,
    y0: int,
    y1: int,
    min_px: int = 12,
    percentile: float = 20.0,
) -> float | None:
    """Tip-face image-y in a strip (not the outermost noisy white pixel).

    Higher percentile = slightly toward the palm (larger y), matching the
    tip-to-tip grasp line rather than tape that sticks past the tips.
    """
    h, w = mask.shape[:2]
    u0 = int(np.clip(u0, 0, w - 1))
    u1 = int(np.clip(u1, u0 + 1, w))
    y0 = int(np.clip(y0, 0, h - 1))
    y1 = int(np.clip(y1, y0 + 1, h))
    strip = mask[y0:y1, u0:u1]
    ys, xs = np.where(strip > 0)
    if len(ys) < min_px:
        return None
    pct = float(np.clip(percentile, 1.0, 50.0))
    return float(y0 + int(np.percentile(ys.astype(np.float64), pct)))


def _white_finger_mask(image_bgr: np.ndarray, args, *, tx: float, ty: float) -> tuple[np.ndarray, int]:
    """White-tape mask; ROI keeps tip region ahead of green, drops white plate."""
    h, w = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    s_max = int(getattr(args, "calib_finger_s_max", 100))
    v_min = int(getattr(args, "calib_finger_v_min", 130))
    mask = ((hsv[:, :, 1] <= s_max) & (hsv[:, :, 2] >= v_min)).astype(np.uint8) * 255
    above = float(getattr(args, "calib_finger_above_green_px", 130.0))
    cut = int(max(0, min(h - 1, ty - above)))
    cut = max(cut, int(h * float(getattr(args, "calib_finger_roi_top_frac", 0.28))))
    cut = min(cut, int(max(0, ty - 40)))
    mask[:cut, :] = 0
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask, cut


def propose_gate_white_fingers(image_bgr: np.ndarray, args) -> dict[str, Any] | None:
    """Propose gate from white taped fingers; y_tip on tip-to-tip grasp line.

    That line is the best grasp depth (across white tip faces), not recessed
    green and not past the tips into free space.
    """
    h, w = image_bgr.shape[:2]
    from .vision_align import target_center_px

    tx, ty = target_center_px(args, w, h)
    mask, cut = _white_finger_mask(image_bgr, args, tx=tx, ty=ty)
    # Low percentile = foremost white (tip face). Higher values dig into the palm.
    tip_pct = float(getattr(args, "calib_finger_tip_percentile", 5.0))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = float(getattr(args, "calib_finger_min_area_px", 400.0))
    blobs = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if bh < 20 or bw < 6:
            continue
        aspect = float(bh) / float(max(1, bw))
        if aspect < 0.6 and bh < 40:
            continue
        blobs.append(
            {
                "bbox": (x, y, bw, bh),
                "cx": x + 0.5 * bw,
                "area": area,
                "top": y,
                "bottom": y + bh,
            }
        )

    left = right = None
    if len(blobs) >= 2:
        left_c = [b for b in blobs if b["cx"] < tx - 8]
        right_c = [b for b in blobs if b["cx"] > tx + 8]
        if left_c and right_c:
            left = max(left_c, key=lambda b: b["area"])
            right = max(right_c, key=lambda b: b["area"])

    half_search = float(getattr(args, "calib_finger_search_half_px", 160.0))
    if left is None or right is None:
        tip_l = _finger_tip_y(
            mask,
            u0=int(tx - half_search),
            u1=int(tx - 20),
            y0=cut,
            y1=h,
            min_px=20,
            percentile=tip_pct,
        )
        tip_r = _finger_tip_y(
            mask,
            u0=int(tx + 20),
            u1=int(tx + half_search),
            y0=cut,
            y1=h,
            min_px=20,
            percentile=tip_pct,
        )
        if tip_l is None or tip_r is None:
            return None
        gate_half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
        u_left = float(tx - gate_half)
        u_right = float(tx + gate_half)
        y_bottom = float(h - 1)
    else:
        lx, ly, lw, lh = left["bbox"]
        rx, ry, rw, rh = right["bbox"]
        u_left = float(lx + lw)
        u_right = float(rx)
        if u_right - u_left < 25:
            gate_half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
            u_left = float(tx - gate_half)
            u_right = float(tx + gate_half)
        tip_l = _finger_tip_y(
            mask, u0=lx, u1=lx + lw, y0=cut, y1=h, percentile=tip_pct
        )
        tip_r = _finger_tip_y(
            mask, u0=rx, u1=rx + rw, y0=cut, y1=h, percentile=tip_pct
        )
        if tip_l is None:
            tip_l = float(left["top"])
        if tip_r is None:
            tip_r = float(right["top"])
        y_bottom = float(max(left["bottom"], right["bottom"], h - 1))

    # Tip-to-tip grasp face ≈ mean of foremost whites, then slight palm inset.
    # Outermost white can stick a few px past the contact plane (looks "太靠上").
    y_tip = 0.5 * (float(tip_l) + float(tip_r))
    inset = float(getattr(args, "calib_tip_inset_px", 6.0))
    y_tip = float(np.clip(y_tip + inset, 0.0, h - 20.0))
    # Soft: stay ahead of recessed green (do not snap tip onto green).
    max_tip = float(ty - float(getattr(args, "calib_tip_min_ahead_of_green_px", 25.0)))
    if y_tip > max_tip:
        y_tip = max_tip

    return normalize_gate(
        {
            "u_left": u_left,
            "u_right": u_right,
            "y_tip": y_tip,
            "y_bottom": y_bottom,
            "image_width": w,
            "image_height": h,
            "tip_left_y": float(tip_l),
            "tip_right_y": float(tip_r),
        },
        source="propose_white_tips",
    )


def snap_gate_tip_to_white_fingers(
    image_bgr: np.ndarray,
    gate: dict[str, Any],
    args,
) -> dict[str, Any] | None:
    """Keep width from gate; move y_tip to detected white finger tip line."""
    proposed = propose_gate_white_fingers(image_bgr, args)
    if proposed is None or not gate_is_valid(gate):
        return None
    out = dict(gate)
    out["y_tip"] = float(proposed["y_tip"])
    out["tip_left_y"] = proposed.get("tip_left_y")
    out["tip_right_y"] = proposed.get("tip_right_y")
    if float(out["y_bottom"]) <= float(out["y_tip"]) + 10:
        out["y_bottom"] = float(proposed["y_bottom"])
    out["source"] = "snap_tip_white"
    return normalize_gate(out)
