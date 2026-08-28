"""LR align on rightmost red, then forward while keeping LR, close when ball enters fingers."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

from .camera_preview import close_preview, grab_bgr, preview_enabled, show_frame
from .flow_loader import load_flow_module
from .gripper_gate import apply_fixed_gate_to_args, default_gate_path, draw_gate, gate_is_valid
from .vision_align import LockedRedTracker, horizontal_only_error


def _apply_pacing(args) -> None:
    args.xy_correction_scale = float(getattr(args, "xy_correction_scale", 0.75) or 0.75)
    args.xy_correction_scale_x = float(getattr(args, "xy_correction_scale_x", 0.75) or 0.75)
    args.xy_correction_scale_y = float(getattr(args, "xy_correction_scale_y", 0.85) or 0.85)
    args.motion_duration = float(
        getattr(args, "xy_align_motion_duration", getattr(args, "motion_duration", 3.5))
    )
    args.settle_s = float(getattr(args, "xy_align_settle_s", getattr(args, "settle_s", 0.45)))
    args.max_move_mm = min(float(getattr(args, "max_move_mm", 45.0) or 45.0), 60.0)



def detect_gripper_gate_once(image_bgr: np.ndarray, args) -> dict[str, Any] | None:
    """One-shot estimate of the open region between the two black gripper fingers.

    Stricter black threshold + finger-shaped / lower-ROI filters reduce false
    locks on tray shadows.
    """
    h, w = image_bgr.shape[:2]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    # Lower V-max = only very dark pixels count as finger (stricter).
    v_max = int(getattr(args, "gripper_black_v_max", 45))
    s_max = int(getattr(args, "gripper_black_s_max", 90))
    mask = (
        (hsv[:, :, 2] <= v_max) & (hsv[:, :, 1] <= s_max)
    ).astype(np.uint8) * 255
    # Fingers live in the lower image; ignore upper tray/shadows.
    top_frac = float(getattr(args, "gripper_roi_top_frac", 0.45))
    y0 = int(round(h * top_frac))
    mask[:y0, :] = 0
    # Also ignore extreme left/right borders a bit.
    side = int(round(w * float(getattr(args, "gripper_roi_side_frac", 0.05))))
    if side > 0:
        mask[:, :side] = 0
        mask[:, w - side :] = 0
    k = max(3, int(getattr(args, "morph_kernel", 5)))
    kernel = np.ones((k, k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = float(getattr(args, "gripper_min_area_px", 1200.0))
    min_h = float(getattr(args, "gripper_min_height_px", 40.0))
    min_aspect = float(getattr(args, "gripper_min_aspect_h_w", 1.15))  # taller than wide
    blobs = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if bh < min_h:
            continue
        if float(bh) / float(max(1, bw)) < min_aspect:
            continue
        cx = x + 0.5 * bw
        cy = y + 0.5 * bh
        # Center of mass should stay in lower half.
        if cy < h * 0.40:
            continue
        blobs.append(
            {
                "area": area,
                "bbox": (x, y, bw, bh),
                "cx": cx,
                "cy": cy,
                "top": y,
                "bottom": y + bh,
            }
        )

    from .vision_align import target_center_px

    tx, ty = target_center_px(args, w, h)
    if len(blobs) < 2:
        half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
        tip = float(getattr(args, "gripper_gate_tip_y_frac", 0.58)) * h
        return {
            "u_left": float(tx - half),
            "u_right": float(tx + half),
            "y_tip": float(tip),
            "y_bottom": float(h - 1),
            "source": "fallback_around_green",
            "blob_count": len(blobs),
            "locked": False,
        }

    # Prefer one blob left of green and one right of green (true fingers).
    left_cands = [b for b in blobs if b["cx"] < tx - 8]
    right_cands = [b for b in blobs if b["cx"] > tx + 8]
    if left_cands and right_cands:
        left = max(left_cands, key=lambda b: b["area"])
        right = max(right_cands, key=lambda b: b["area"])
    else:
        # Fallback: two largest, sorted by x — but reject if both on same side of green.
        blobs.sort(key=lambda b: b["area"], reverse=True)
        pair = sorted(blobs[:2], key=lambda b: b["cx"])
        left, right = pair[0], pair[1]
        if (left["cx"] - tx) * (right["cx"] - tx) > 0:
            half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
            tip = float(getattr(args, "gripper_gate_tip_y_frac", 0.58)) * h
            return {
                "u_left": float(tx - half),
                "u_right": float(tx + half),
                "y_tip": float(tip),
                "y_bottom": float(h - 1),
                "source": "fallback_same_side_rejected",
                "blob_count": len(blobs),
                "locked": False,
            }

    lx, ly, lw, lh = left["bbox"]
    rx, ry, rw, rh = right["bbox"]
    u_left = float(lx + lw)
    u_right = float(rx)
    min_w = float(getattr(args, "gripper_gate_min_width_px", 35.0))
    max_w = float(getattr(args, "gripper_gate_max_width_px", 180.0))
    width = u_right - u_left
    if width < min_w or width > max_w:
        mid = 0.5 * (left["cx"] + right["cx"])
        half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
        u_left, u_right = mid - half, mid + half
        width = u_right - u_left
        if width < min_w or width > max_w:
            half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
            tip = float(getattr(args, "gripper_gate_tip_y_frac", 0.58)) * h
            return {
                "u_left": float(tx - half),
                "u_right": float(tx + half),
                "y_tip": float(tip),
                "y_bottom": float(h - 1),
                "source": "fallback_bad_width",
                "blob_count": len(blobs),
                "locked": False,
            }

    y_tip = float(min(left["top"], right["top"]))
    y_bottom = float(max(left["bottom"], right["bottom"]))
    # Tip should stay in lower image (reject tray-shadow "fingers").
    min_tip_y = float(h) * float(getattr(args, "gripper_min_tip_y_frac", 0.38))
    if y_tip < min_tip_y:
        half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
        tip = float(getattr(args, "gripper_gate_tip_y_frac", 0.58)) * h
        return {
            "u_left": float(tx - half),
            "u_right": float(tx + half),
            "y_tip": float(tip),
            "y_bottom": float(h - 1),
            "source": "fallback_tip_too_high",
            "blob_count": len(blobs),
            "locked": False,
        }

    return {
        "u_left": u_left,
        "u_right": u_right,
        "y_tip": y_tip,
        "y_bottom": y_bottom,
        "source": "black_fingers",
        "blob_count": len(blobs),
        "left_bbox": list(left["bbox"]),
        "right_bbox": list(right["bbox"]),
        "locked": False,
    }


class LockedGripperGate:
    """Stabilize gripper-middle gate over several frames, then freeze pixels.

    Do not trust frame-0. Collect black-finger detections for a short window;
    lock only when recent estimates agree (low spread). After lock, never update.
    """

    def __init__(self, args) -> None:
        self.args = args
        self.gate: dict[str, Any] | None = None
        self.require_fingers = bool(getattr(args, "gripper_gate_require_fingers", True))
        self._history: list[dict[str, Any]] = []
        self.min_samples = max(3, int(getattr(args, "gripper_gate_stable_samples", 12)))
        self.max_spread_px = float(getattr(args, "gripper_gate_stable_spread_px", 12.0))
        self.stable_window = max(self.min_samples, int(getattr(args, "gripper_gate_stable_window", 15)))
        self._last_live: dict[str, Any] = {}

    @property
    def locked(self) -> bool:
        return self.gate is not None and bool(self.gate.get("locked"))

    def _spread(self, samples: list[dict[str, Any]]) -> float:
        keys = ("u_left", "u_right", "y_tip", "y_bottom")
        spreads = [max(float(s[k]) for s in samples) - min(float(s[k]) for s in samples) for k in keys]
        return float(max(spreads))

    def _median_gate(self, samples: list[dict[str, Any]]) -> dict[str, Any]:
        def med(key: str) -> float:
            vals = sorted(float(s[key]) for s in samples)
            return float(vals[len(vals) // 2])

        return {
            "u_left": med("u_left"),
            "u_right": med("u_right"),
            "y_tip": med("y_tip"),
            "y_bottom": med("y_bottom"),
            "source": "locked_stable_median",
            "locked": True,
            "stable_samples": len(samples),
            "stable_spread_px": self._spread(samples),
        }

    def update(self, image_bgr: np.ndarray) -> dict[str, Any]:
        if self.gate is not None and self.gate.get("locked"):
            return dict(self.gate)

        est = detect_gripper_gate_once(image_bgr, self.args) or {}
        self._last_live = dict(est)

        # Only black-finger detections vote for lock (ignore green fallback).
        if est.get("source") == "black_fingers":
            self._history.append(
                {
                    "u_left": float(est["u_left"]),
                    "u_right": float(est["u_right"]),
                    "y_tip": float(est["y_tip"]),
                    "y_bottom": float(est["y_bottom"]),
                }
            )
            if len(self._history) > self.stable_window * 2:
                self._history = self._history[-self.stable_window * 2 :]

            window = self._history[-self.stable_window :]
            if len(window) >= self.min_samples:
                check = window[-self.min_samples :]
                spread = self._spread(check)
                if spread <= self.max_spread_px:
                    # Prefer a longer median if the whole window is still tight.
                    if len(window) >= self.min_samples + 3 and self._spread(window) <= self.max_spread_px * 1.25:
                        locked = self._median_gate(window)
                    else:
                        locked = self._median_gate(check)
                    self.gate = locked
                    setattr(self.args, "locked_gripper_gate", dict(locked))
                    print(
                        f"[tcp-flow] LOCKED gripper gate (stable) "
                        f"u=[{locked['u_left']:.0f},{locked['u_right']:.0f}] "
                        f"y=[{locked['y_tip']:.0f},{locked['y_bottom']:.0f}] "
                        f"n={locked['stable_samples']} spread={locked['stable_spread_px']:.1f}px",
                        flush=True,
                    )
                    return dict(locked)
                live = dict(est)
                live["locked"] = False
                live["source"] = "stabilizing"
                live["stable_n"] = len(self._history)
                live["stable_need"] = self.min_samples
                live["stable_spread_px"] = spread
                return live

        out = dict(est) if est else {}
        out["locked"] = False
        out["source"] = str(est.get("source", "none"))
        out["stable_n"] = len(self._history)
        out["stable_need"] = self.min_samples
        return out

    def get(self) -> dict[str, Any]:
        if self.gate is not None:
            return dict(self.gate)
        if self._last_live:
            return dict(self._last_live)
        return dict(getattr(self.args, "locked_gripper_gate", None) or {})


def detect_gripper_gate(image_bgr: np.ndarray, args) -> dict[str, Any] | None:
    """Return calibrated/locked gate if available; otherwise one-shot estimate."""
    locked = getattr(args, "locked_gripper_gate", None)
    if isinstance(locked, dict) and locked.get("locked"):
        return dict(locked)
    try:
        from .gripper_gate import load_gripper_gate

        fixed = load_gripper_gate(getattr(args, "gripper_gate_file", None))
        if fixed is not None:
            setattr(args, "locked_gripper_gate", dict(fixed))
            return dict(fixed)
    except Exception:
        pass
    return detect_gripper_gate_once(image_bgr, args)


def ball_gate_overlap_frac(
    record: dict[str, Any],
    gate: dict[str, Any],
    *,
    expand_px: float = 0.0,
    tip_inset_px: float = 0.0,
) -> float:
    """Fraction of red ball bbox overlapping the finger gate.

    tip_inset_px > 0 moves the effective tip toward the palm so close waits
    until the ball is deeper past the fingertip line (less shallow grasps).
    """
    bbox = record.get("bbox_xywh")
    if not bbox or len(bbox) < 4 or not gate:
        return 0.0
    x, y, bw, bh = [float(v) for v in bbox]
    if bw <= 1 or bh <= 1:
        return 0.0
    m = max(0.0, float(expand_px))
    inset = max(0.0, float(tip_inset_px))
    u_left = float(gate["u_left"]) - m
    u_right = float(gate["u_right"]) + m
    y_tip = float(gate["y_tip"]) + inset - m
    y_bottom = float(gate.get("y_bottom", y_tip + 1)) + m
    if u_right - u_left < 10 or y_bottom <= y_tip:
        return 0.0
    ix0 = max(x, u_left)
    iy0 = max(y, y_tip)
    ix1 = min(x + bw, u_right)
    iy1 = min(y + bh, y_bottom)
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    return float(inter / (bw * bh))


def _handle_top_half_px(record: dict[str, Any], args) -> float:
    """How far past tip the handle *top* (grape-side bead) must travel to close.

    Default: half of the smaller bbox side ≈ half a red bead diameter.
    Override with gripper_enter_top_half_px > 0.
    """
    fixed = float(getattr(args, "gripper_enter_top_half_px", 0.0) or 0.0)
    if fixed > 0.0:
        return fixed
    bbox = record.get("bbox_xywh")
    if not bbox or len(bbox) < 4:
        return 12.0
    _x, _y, bw, bh = [float(v) for v in bbox]
    return max(6.0, 0.5 * float(min(bw, bh)))


def ball_touches_gate(record: dict[str, Any], gate: dict[str, Any], args) -> bool:
    """True when enough of the red target is inside the finger opening to close.

    Default (red_handle_mode): prepare close once >= gripper_enter_frac (0.60)
    of the red bbox area overlaps the gripper gate. Top-bead half-in remains an
    optional early path if it fires first.
    """
    if not gate or "u_left" not in gate:
        return False
    margin = float(getattr(args, "gripper_touch_margin_px", 8.0))
    bbox = record.get("bbox_xywh")
    if not bbox or len(bbox) < 4:
        return False
    x, y, bw, bh = [float(v) for v in bbox]
    u_left = float(gate["u_left"]) - margin
    u_right = float(gate["u_right"]) + margin
    y_tip = float(gate["y_tip"])
    y_bottom = float(gate.get("y_bottom", y_tip + 1)) + margin
    h_ok = (x + bw) >= u_left and x <= u_right
    if not h_ok:
        return False

    handle_top = float(record.get("handle_top_y", y))
    handle_bottom = float(record.get("handle_bottom_y", y + bh))
    if bool(getattr(args, "red_handle_mode", True)):
        need = float(getattr(args, "gripper_enter_frac", 0.60))
        frac = ball_gate_overlap_frac(record, gate, expand_px=margin, tip_inset_px=0.0)
        if frac >= need:
            return True
        # Optional early: grape-side bead already half past tip.
        half = _handle_top_half_px(record, args)
        enter_top = y_tip + half - margin
        return handle_top >= enter_top and handle_top <= y_bottom

    # Legacy ball mode: bottom past tip + small overlap.
    past = float(getattr(args, "gripper_enter_past_tip_px", 8.0))
    need = float(getattr(args, "gripper_enter_frac", 0.08))
    frac = ball_gate_overlap_frac(
        record, gate, expand_px=margin, tip_inset_px=past
    )
    if need <= 0.0:
        if frac > 1e-6:
            return True
    elif frac >= need:
        return True
    y_enter = y_tip + past - margin
    if h_ok and handle_bottom >= y_enter and handle_top <= y_bottom:
        return True
    cx, cy = x + 0.5 * bw, y + 0.5 * bh
    return u_left <= cx <= u_right and y_enter <= cy <= y_bottom


def ball_half_in_gripper(record: dict[str, Any], gate: dict[str, Any], args) -> bool:
    """Alias: touch/overlap gate is enough to stop and close."""
    return ball_touches_gate(record, gate, args)

# Back-compat alias
def ball_fully_in_gripper(record: dict[str, Any], gate: dict[str, Any], args) -> bool:
    return ball_touches_gate(record, gate, args)


def _move_xy(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    name: str,
    current_T,
    current_q: list[float],
    current_gripper: float,
    move_mm: tuple[float, float],
    label: str,
    z_mm: float = 0.0,
    hold_z_m: float | None = None,
):
    """Plan/play a base XY move; keep base-Z fixed unless z_mm / hold_z_m says otherwise."""
    move_sample, executed_move, target_T, _ = core.generate_xy_sample_with_joint_limit(
        name=name,
        current_T=current_T,
        q_start=current_q,
        gripper=current_gripper,
        move_mm=move_mm,
        run_dir=run_dir,
        args=args,
    )
    target_T = target_T.copy()
    # Pin height: visual servo must not sink TCP after the initial lift.
    if hold_z_m is not None:
        target_T[2, 3] = float(hold_z_m)
    if abs(float(z_mm)) > 1e-6:
        target_T[2, 3] += float(z_mm) * 0.001
    if abs(float(target_T[2, 3]) - float(current_T[2, 3])) > 1e-6:
        move_sample = core.generate_sample_for_target(
            name=f"{name}_zpin",
            target_T=target_T,
            q_start=current_q,
            gripper=current_gripper,
            run_dir=run_dir,
            args=args,
        )
    print(
        f"[tcp-flow] {label} executed_mm=({executed_move[0]:+.1f},{executed_move[1]:+.1f}) "
        f"z={float(target_T[2, 3]):.4f}m",
        flush=True,
    )
    core.play_full_replay_on_server(
        server_proc,
        move_sample.replay_csv,
        args,
        label=label,
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    return list(move_sample.q), target_T.copy(), float(move_sample.gripper), executed_move



def _nudge_z(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    current_T,
    current_q: list[float],
    current_gripper: float,
    z_mm: float,
    name: str = "nudge_z_up",
):
    if abs(float(z_mm)) < 1e-6:
        return current_q, current_T, current_gripper
    target_T = current_T.copy()
    target_T[2, 3] += float(z_mm) * 0.001
    sample = core.generate_sample_for_target(
        name=name,
        target_T=target_T,
        q_start=current_q,
        gripper=current_gripper,
        run_dir=run_dir,
        args=args,
    )
    print(f"[tcp-flow] nudge base-Z {float(z_mm):+.1f}mm", flush=True)
    core.play_full_replay_on_server(
        server_proc,
        sample.replay_csv,
        args,
        label=name,
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    return list(sample.q), target_T.copy(), float(sample.gripper)


def _play_pose(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    current_T,
    current_q: list[float],
    current_gripper: float,
    target_T,
    name: str,
    label: str,
):
    sample = core.generate_sample_for_target(
        name=name,
        target_T=target_T,
        q_start=current_q,
        gripper=current_gripper,
        run_dir=run_dir,
        args=args,
    )
    xyz = target_T[:3, 3]
    print(
        f"[tcp-flow] {label} -> xyz=({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f})",
        flush=True,
    )
    core.play_full_replay_on_server(
        server_proc,
        sample.replay_csv,
        args,
        label=label,
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    return list(sample.q), target_T.copy(), float(sample.gripper)


def _try_play_pose(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    current_T,
    current_q: list[float],
    current_gripper: float,
    target_T,
    name: str,
    label: str,
) -> tuple[list[float], object, float] | None:
    """Like _play_pose, but return None on IK/plan failure instead of raising."""
    try:
        return _play_pose(
            core=core,
            server_proc=server_proc,
            args=args,
            run_dir=run_dir,
            current_T=current_T,
            current_q=current_q,
            current_gripper=current_gripper,
            target_T=target_T,
            name=name,
            label=label,
        )
    except Exception as exc:
        xyz = target_T[:3, 3]
        print(
            f"[tcp-flow][WARN] {label} IK/plan failed for "
            f"xyz=({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f}): {exc}",
            flush=True,
        )
        return None


def post_grasp_lift_and_retreat(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    current_T,
    current_q: list[float],
    current_gripper: float,
):
    """After close: lift +Z, then retreat as far as IK allows (never crash on IK fail)."""
    lift_m = float(getattr(args, "post_grasp_lift_m", 0.09))
    retreat_mm = float(getattr(args, "post_grasp_retreat_mm", 140.0))
    approach_axis = str(getattr(args, "approach_axis", "x")).lower()
    approach_sign = float(getattr(args, "approach_sign", 1.0))
    axis_i = 0 if approach_axis == "x" else 1 if approach_axis == "y" else 0
    retreat_sign = -1.0 if approach_sign >= 0.0 else 1.0

    saved_motion = float(getattr(args, "motion_duration", 2.2))
    saved_settle = float(getattr(args, "settle_s", 0.22))
    q, T, g = current_q, current_T, current_gripper
    executed_retreat_mm = 0.0
    try:
        if lift_m > 1e-6:
            args.motion_duration = float(getattr(args, "post_grasp_lift_duration", 1.0))
            args.settle_s = min(saved_settle, 0.12)
            # Prefer full lift; on IK fail, try shorter lifts rather than aborting.
            lift_candidates_mm = []
            full_lift_mm = abs(lift_m) * 1000.0
            for frac in (1.0, 0.75, 0.5, 0.35, 0.25):
                mm = full_lift_mm * frac
                if mm >= 15.0 and mm not in lift_candidates_mm:
                    lift_candidates_mm.append(mm)
            lifted = False
            for lift_try_mm in lift_candidates_mm:
                lift_T = T.copy()
                lift_T[2, 3] += (lift_try_mm * 0.001) * (1.0 if lift_m >= 0 else -1.0)
                print(
                    f"[tcp-flow] PHASE4 lift try +{lift_try_mm:.0f}mm base-Z",
                    flush=True,
                )
                got = _try_play_pose(
                    core=core,
                    server_proc=server_proc,
                    args=args,
                    run_dir=run_dir,
                    current_T=T,
                    current_q=q,
                    current_gripper=g,
                    target_T=lift_T,
                    name=f"post_grasp_lift_z_{int(round(lift_try_mm))}mm",
                    label=f"post-lift-{int(round(lift_try_mm))}mm",
                )
                if got is not None:
                    q, T, g = got
                    lifted = True
                    print(
                        f"[tcp-flow] PHASE4 lift ok +{lift_try_mm:.0f}mm",
                        flush=True,
                    )
                    break
            if not lifted:
                print(
                    "[tcp-flow][WARN] PHASE4 lift skipped (no reachable lift IK); "
                    "holding pose and continuing to retreat",
                    flush=True,
                )

        if abs(retreat_mm) > 1e-6:
            args.motion_duration = float(getattr(args, "post_grasp_retreat_duration", 1.0))
            args.settle_s = min(saved_settle, 0.12)
            # Requested distance first, then shorter fallbacks — "as far as possible".
            retreat_candidates_mm: list[float] = []
            full = abs(retreat_mm)
            for frac in (1.0, 0.75, 0.5, 0.35, 0.25, 0.15, 0.10):
                mm = full * frac
                if mm >= 10.0:
                    retreat_candidates_mm.append(round(mm, 1))
            # De-dupe while preserving order.
            seen: set[float] = set()
            ordered: list[float] = []
            for mm in retreat_candidates_mm:
                key = round(mm, 1)
                if key not in seen:
                    seen.add(key)
                    ordered.append(key)

            retreated = False
            for try_mm in ordered:
                retreat_T = T.copy()
                retreat_T[axis_i, 3] += try_mm * 0.001 * retreat_sign
                print(
                    f"[tcp-flow] PHASE4 retreat try {try_mm:.0f}mm along base "
                    f"{approach_axis} sign={retreat_sign:+.0f}",
                    flush=True,
                )
                got = _try_play_pose(
                    core=core,
                    server_proc=server_proc,
                    args=args,
                    run_dir=run_dir,
                    current_T=T,
                    current_q=q,
                    current_gripper=g,
                    target_T=retreat_T,
                    name=f"post_grasp_retreat_{int(round(try_mm))}mm",
                    label=f"post-retreat-{int(round(try_mm))}mm",
                )
                if got is not None:
                    q, T, g = got
                    executed_retreat_mm = try_mm
                    retreated = True
                    print(
                        f"[tcp-flow] PHASE4 retreat ok {try_mm:.0f}mm "
                        f"(requested {abs(retreat_mm):.0f}mm)",
                        flush=True,
                    )
                    break
            if not retreated:
                print(
                    f"[tcp-flow][WARN] PHASE4 retreat skipped — no reachable IK "
                    f"down to 10mm (requested {abs(retreat_mm):.0f}mm); holding",
                    flush=True,
                )
    finally:
        args.motion_duration = saved_motion
        args.settle_s = saved_settle
        args.post_grasp_retreat_mm_executed = float(executed_retreat_mm)
    return q, T, g


def _fresh_frame(cam, core, args, settle_extra: float = 0.0):
    """Flush stale camera buffers after motion (reduces lag-induced overshoot)."""
    core.flush_camera(cam, args)
    time.sleep(max(0.08, float(getattr(args, "settle_s", 0.18)) + float(settle_extra)))
    # Drop a couple frames so we don't servo on a pre-motion image.
    frame = None
    for _ in range(3):
        frame = grab_bgr(cam)
        time.sleep(0.015)
    return frame

def close_gripper_in_place(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    q: list[float],
    gripper_open: float,
) -> float:
    g_close = float(args.gripper_close)
    replay_csv = run_dir / "grasp_replays" / "close_gripper_only.csv"
    waypoints = [
        (list(q), float(gripper_open)),
        (list(q), g_close),
        (list(q), g_close),
    ]
    durations = [
        float(getattr(args, "gripper_close_duration", 1.2)),
        float(getattr(args, "gripper_close_hold_duration", 1.2)),
    ]
    core.write_replay_csv_8d(
        replay_csv, waypoints, durations, float(getattr(args, "replay_sample_period", 0.01))
    )
    print(f"[tcp-flow] closing gripper -> {g_close:+.3f}", flush=True)
    core.play_full_replay_on_server(
        server_proc,
        replay_csv,
        args,
        label="close-gripper",
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    return g_close


def run_lr_forward_close(
    server_proc,
    args,
    run_dir: Path,
    *,
    q_start: list[float],
    T_start,
    gripper_start: float,
    lr_only: bool = False,
) -> tuple[list[float], object, float]:
    """1) LR-only align  2) streaming tcp_vel approach until ball in fingers  3) close."""
    flow = load_flow_module()
    core = flow.core
    servo = flow.servo
    _apply_pacing(args)
    args.target_selection = "red"

    current_q = list(q_start)
    current_T = T_start.copy()
    # Keep gripper open for approach.
    current_gripper = float(getattr(args, "gripper_open", gripper_start))
    args.gripper = current_gripper
    args.gripper_current_after_align = current_gripper

    lr_done_err = float(getattr(args, "lr_align_done_err_px", 8.0))
    lr_passes = max(1, int(getattr(args, "lr_align_passes", 20)))

    delay_s = max(0.0, float(args.auto_start_delay_s))
    tracker = LockedRedTracker(args)
    # Prefer calibrated fixed gate file (white gripper) — no online finger detection.
    if hasattr(args, "locked_gripper_gate"):
        delattr(args, "locked_gripper_gate")
    fixed_gate = apply_fixed_gate_to_args(args)
    gate_locker = LockedGripperGate(args)
    if fixed_gate is not None:
        gate_locker.gate = dict(fixed_gate)
        print(
            f"[tcp-flow] using FIXED gripper gate file "
            f"{fixed_gate.get('path', default_gate_path())} "
            f"u=[{fixed_gate['u_left']:.0f},{fixed_gate['u_right']:.0f}] "
            f"y=[{fixed_gate['y_tip']:.0f},{fixed_gate['y_bottom']:.0f}]",
            flush=True,
        )
    else:
        print(
            f"[tcp-flow][WARN] no fixed gate at {default_gate_path()}; "
            "run ./run_calibrate_gate.sh first (online detect is legacy/fallback)",
            flush=True,
        )
    print(
        "[tcp-flow] pipeline: lock rightmost red + fixed gripper gate "
        "-> LR align green "
        + (
            "(lr-only)"
            if lr_only
            else "-> tcp_vel approach until ball touches gate -> close"
        )
        + f" (lr_err<={lr_done_err}px, approach_speed="
        f"{float(getattr(args, 'approach_speed_mm_s', 25.0)):.0f}mm/s)",
        flush=True,
    )

    with servo.Camera(args) as cam:
        core.flush_camera(cam, args)
        t_end = time.monotonic() + delay_s
        while time.monotonic() < t_end:
            frame = grab_bgr(cam)
            if frame is not None:
                if not gate_locker.locked:
                    gate_locker.update(frame)
                if preview_enabled(args):
                    show_frame(frame, args, status="pre-detect wait", servo=servo)
            time.sleep(0.03)

        # Raise whole TCP a few mm (base +Z) before LR / forward, then pin that height.
        z_nudge = float(getattr(args, "approach_lift_z_mm", 24.0))
        hold_z_m: float | None = None
        if abs(z_nudge) > 1e-6:
            print(
                f"[tcp-flow] raise TCP base-Z by {z_nudge:+.1f}mm before align/forward",
                flush=True,
            )
            current_q, current_T, current_gripper = _nudge_z(
                core=core,
                server_proc=server_proc,
                args=args,
                run_dir=run_dir,
                current_T=current_T,
                current_q=current_q,
                current_gripper=current_gripper,
                z_mm=z_nudge,
                name="000_tcp_lift_z",
            )
            hold_z_m = float(current_T[2, 3])
            print(
                f"[tcp-flow] pin TCP base-Z at {hold_z_m:.4f}m for all later XY moves",
                flush=True,
            )
            core.flush_camera(cam, args)
            time.sleep(max(0.2, float(args.settle_s)))
        else:
            hold_z_m = float(current_T[2, 3])

        # Stabilize online only if no calibrated file was loaded.
        if not gate_locker.locked:
            gate_timeout = float(getattr(args, "gripper_gate_stable_timeout_s", 4.0))
            print(
                f"[tcp-flow] stabilizing gripper gate "
                f"(need {gate_locker.min_samples} consistent frames, "
                f"spread<={gate_locker.max_spread_px:.0f}px, timeout={gate_timeout:.1f}s)",
                flush=True,
            )
            t_gate_end = time.monotonic() + gate_timeout
            while not gate_locker.locked and time.monotonic() < t_gate_end:
                frame = grab_bgr(cam)
                if frame is None:
                    time.sleep(0.05)
                    continue
                live_gate = gate_locker.update(frame)
                if preview_enabled(args):
                    n = live_gate.get("stable_n", 0)
                    need = live_gate.get("stable_need", gate_locker.min_samples)
                    sp = live_gate.get("stable_spread_px")
                    sp_s = f" spread={sp:.1f}px" if sp is not None else ""
                    show_frame(
                        frame,
                        args,
                        status=f"gate stabilize {n}/{need}{sp_s}",
                        servo=servo,
                    )
                time.sleep(0.05)
            if not gate_locker.locked:
                print(
                    "[tcp-flow][WARN] gripper gate did not stabilize; "
                    "locking best available estimate",
                    flush=True,
                )
                if len(gate_locker._history) >= 3:
                    soft = gate_locker._median_gate(
                        gate_locker._history[-gate_locker.min_samples :]
                    )
                    soft["source"] = "soft_lock_history_median"
                else:
                    from .vision_align import target_center_px

                    frame = grab_bgr(cam)
                    h, w = (frame.shape[:2] if frame is not None else (480, 640))
                    tx, ty = target_center_px(args, w, h)
                    half = float(getattr(args, "gripper_gate_half_width_px", 55.0))
                    tip = float(getattr(args, "gripper_gate_tip_y_frac", 0.58)) * h
                    soft = {
                        "u_left": float(tx - half),
                        "u_right": float(tx + half),
                        "y_tip": float(tip),
                        "y_bottom": float(h - 1),
                        "source": "soft_lock_green_fixed",
                        "locked": True,
                        "stable_samples": 0,
                        "stable_spread_px": 0.0,
                    }
                soft["locked"] = True
                gate_locker.gate = soft
                setattr(args, "locked_gripper_gate", dict(soft))
                print(
                    f"[tcp-flow] soft-locked gate src={soft['source']} "
                    f"u=[{soft['u_left']:.0f},{soft['u_right']:.0f}] "
                    f"y=[{soft['y_tip']:.0f},{soft['y_bottom']:.0f}]",
                    flush=True,
                )
        else:
            print("[tcp-flow] skip online gate detect — using calibrated fixed box", flush=True)

        # ---- Phase 1: left-right only ----
        print("[tcp-flow] PHASE1 LR-only align (no forward); lock rightmost then track", flush=True)
        for pass_i in range(lr_passes):
            frame = grab_bgr(cam)
            if frame is None:
                raise RuntimeError("camera read failed in LR phase")
            if not gate_locker.locked:
                gate_locker.update(frame)
            record = tracker.update(frame, near_gripper=False) or {"detected": False}
            if not record.get("detected"):
                if preview_enabled(args):
                    show_frame(
                        frame,
                        args,
                        status="LR: waiting lock" if not tracker.locked else "LR: track lost",
                        servo=servo,
                    )
                time.sleep(0.3)
                continue
            if record.get("selection") == "locked_rightmost_initial":
                print(
                    f"[tcp-flow] LOCKED rightmost ball at {record['center_px']} "
                    f"(will track this one only)",
                    flush=True,
                )
            record_lr = horizontal_only_error(record)
            err_u = float(record_lr["error_px"][0])
            if preview_enabled(args):
                show_frame(
                    frame,
                    args,
                    status=f"PHASE1 LR track err_u={err_u:+.1f}px",
                    record=record_lr,
                    servo=servo,
                )
            print(
                f"[tcp-flow] LR {pass_i + 1}/{lr_passes} tracked={record['center_px']} "
                f"green={record['target_center_px']} err_u={err_u:+.1f} "
                f"sel={record.get('selection')} jump={record.get('track_dist_px', 0):.1f}px",
                flush=True,
            )
            core.save_json(
                run_dir / f"lr_pass_{pass_i + 1:02d}.json",
                {k: v for k, v in record_lr.items() if k != "paper_mask"},
            )
            if abs(err_u) <= lr_done_err:
                print(f"[tcp-flow] LR done (|err_u|={abs(err_u):.1f}px)", flush=True)
                break
            plan = flow.compute_plan(record_lr, args)
            move = (float(plan["move_mm"][0]), float(plan["move_mm"][1]))
            current_q, current_T, current_gripper, _ = _move_xy(
                core=core,
                server_proc=server_proc,
                args=args,
                run_dir=run_dir,
                name=f"{pass_i + 1:03d}_lr_align",
                current_T=current_T,
                current_q=current_q,
                current_gripper=current_gripper,
                move_mm=move,
                label=f"lr-{pass_i + 1}",
                hold_z_m=hold_z_m,
            )
            tracker.notify_motion()
            core.flush_camera(cam, args)
            time.sleep(max(0.2, float(args.settle_s)))
        else:
            print("[tcp-flow][WARN] LR max passes reached; continuing", flush=True)

        if lr_only:
            print("[tcp-flow] --lr-only: skip forward/close, hold open gripper", flush=True)
            core.save_json(
                run_dir / "lr_forward_close_result.json",
                {
                    "lr_only": True,
                    "entered_gripper": False,
                    "lock_center_px": list(tracker.lock_center) if tracker.lock_center else None,
                    "q": list(current_q),
                    "gripper": float(current_gripper),
                },
            )
            flow.log_pose_state("after-lr-only", current_q, current_gripper, current_T)
            return current_q, current_T, current_gripper

        # ---- Phase 2: streaming TCP approach (no CSV micro-jog) ----
        from .approach_visual_servo import run_continuous_approach

        current_q, current_T, current_gripper, entered = run_continuous_approach(
            core=core,
            server_proc=server_proc,
            args=args,
            run_dir=run_dir,
            cam=cam,
            tracker=tracker,
            gate_locker=gate_locker,
            servo=servo,
            current_q=current_q,
            current_T=current_T,
            current_gripper=current_gripper,
            hold_z_m=hold_z_m,
        )

        # ---- Phase 3: close only if ball entered fingers ----
        if not entered:
            print(
                "[tcp-flow] PHASE3 skipped close (ball never entered finger gate); holding open",
                flush=True,
            )
            core.save_json(
                run_dir / "lr_forward_close_result.json",
                {
                    "entered_gripper": False,
                    "closed": False,
                    "lock_center_px": list(tracker.lock_center) if tracker.lock_center else None,
                    "q": list(current_q),
                    "gripper": float(current_gripper),
                },
            )
            flow.log_pose_state("after-forward-no-close", current_q, current_gripper, current_T)
            return current_q, current_T, current_gripper

        print("[tcp-flow] PHASE3 close gripper (entered=True)", flush=True)
        g_closed = close_gripper_in_place(
            core=core,
            server_proc=server_proc,
            args=args,
            run_dir=run_dir,
            q=current_q,
            gripper_open=current_gripper,
        )
        current_gripper = g_closed
        args.gripper_current_after_align = current_gripper

        # ---- Phase 4: lift then horizontal retreat ----
        current_q, current_T, current_gripper = post_grasp_lift_and_retreat(
            core=core,
            server_proc=server_proc,
            args=args,
            run_dir=run_dir,
            current_T=current_T,
            current_q=current_q,
            current_gripper=current_gripper,
        )
        core.save_json(
            run_dir / "lr_forward_close_result.json",
            {
                "entered_gripper": True,
                "closed": True,
                "lift_m": float(getattr(args, "post_grasp_lift_m", 0.09)),
                "retreat_mm_requested": float(
                    getattr(args, "post_grasp_retreat_mm", 140.0)
                ),
                "retreat_mm_executed": float(
                    getattr(args, "post_grasp_retreat_mm_executed", 0.0)
                ),
                "lock_center_px": list(tracker.lock_center) if tracker.lock_center else None,
                "q": list(current_q),
                "gripper": float(current_gripper),
            },
        )

    flow.log_pose_state("after-lift-retreat", current_q, current_gripper, current_T)
    return current_q, current_T, current_gripper
