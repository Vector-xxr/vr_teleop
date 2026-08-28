#!/usr/bin/env python3
"""One-shot real robot 2D alignment test for the top-down skewer workflow."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OBSERVE_REPLAY = ROOT / "outputs" / "right_observe_record_20260723_135046" / "real_follower_right_03.csv"
DEFAULT_CONFIG = ROOT / "config" / "skewer_align_config.json"
DEFAULT_LEFT_CONFIG = ROOT / "config" / "skewer_align_config_left.json"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


calib = load_module("skewer_auto_calib", ROOT / "scripts" / "auto_calibrate_pixel_to_motion.py")
servo = calib.servo


class JointDeltaLimitError(RuntimeError):
    pass


def save_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def apply_config_defaults(args) -> None:
    config_path = args.config.expanduser().resolve() if args.config else None
    if config_path is None or not config_path.exists():
        return
    data = load_json(config_path)
    if not args.pixel_to_base_mm and data.get("pixel_to_base_mm"):
        value = data["pixel_to_base_mm"]
        if isinstance(value, list):
            value = ",".join(str(v) for v in value)
        args.pixel_to_base_mm = str(value)
    if not args.pixel_to_base_model and data.get("pixel_to_base_model"):
        args.pixel_to_base_model = str(data["pixel_to_base_model"])
    if args.quadratic_offset_from_error_coeff is None and data.get("quadratic_offset_from_error_coeff"):
        args.quadratic_offset_from_error_coeff = data["quadratic_offset_from_error_coeff"]
    if not args.target_center_px and data.get("target_center_px"):
        value = data["target_center_px"]
        if isinstance(value, list):
            value = ",".join(str(v) for v in value)
        args.target_center_px = str(value)
    print(f"[config] loaded {config_path}")


def parse_matrix(text: str) -> np.ndarray:
    values = [float(v.strip()) for v in str(text).split(",") if v.strip()]
    if len(values) != 4:
        raise RuntimeError("--pixel-to-base-mm must be a,b,c,d")
    return np.asarray(values, dtype=np.float64).reshape(2, 2)


def quadratic_features(error_px: np.ndarray) -> np.ndarray:
    du = float(error_px[0])
    dv = float(error_px[1])
    return np.asarray([du, dv, du * du, dv * dv, du * dv, 1.0], dtype=np.float64)


def correction_from_error(error_px: np.ndarray, args) -> tuple[np.ndarray, str]:
    if (
        str(args.pixel_to_base_model) == "quadratic_offset_delta"
        and args.quadratic_offset_from_error_coeff is not None
    ):
        coeff = np.asarray(args.quadratic_offset_from_error_coeff, dtype=np.float64)
        if coeff.shape != (6, 2):
            raise RuntimeError(f"quadratic_offset_from_error_coeff must have shape 6x2, got {coeff.shape}")
        offset_current = quadratic_features(error_px) @ coeff
        offset_zero = quadratic_features(np.asarray([0.0, 0.0], dtype=np.float64)) @ coeff
        return offset_zero - offset_current, "quadratic"
    return parse_matrix(args.pixel_to_base_mm) @ error_px, "linear"


def limit_vector_mm(vec: np.ndarray, max_norm_mm: float | None) -> tuple[np.ndarray, float]:
    if max_norm_mm is None or max_norm_mm <= 0.0:
        return vec, 1.0
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm_mm or norm <= 1.0e-9:
        return vec, 1.0
    scale = max_norm_mm / norm
    return vec * scale, scale


def xy_correction_scales(args) -> np.ndarray:
    fallback = float(args.xy_correction_scale)
    scale_x = fallback if args.xy_correction_scale_x is None else float(args.xy_correction_scale_x)
    scale_y = fallback if args.xy_correction_scale_y is None else float(args.xy_correction_scale_y)
    return np.asarray([scale_x, scale_y], dtype=np.float64)


def rotz(rad: float) -> np.ndarray:
    c = float(np.cos(rad))
    s = float(np.sin(rad))
    return np.asarray(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def wrap_deg(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def line_axis_delta_deg(current_deg: float, target_deg: float) -> float:
    # A skewer line has 180-degree symmetry, so use the shortest axis delta.
    return (float(target_deg) - float(current_deg) + 90.0) % 180.0 - 90.0


def flush_camera(cam, args) -> None:
    for _ in range(max(0, int(args.capture_flush_frames))):
        cam.read_color()


def detect_once(cam, sample, run_dir: Path, args, previous_record=None) -> dict:
    return calib.capture_detection(cam, sample, run_dir, args, previous_record=previous_record)


def detect_full_frame_once(cam, sample, run_dir: Path, args) -> dict:
    full_args = argparse.Namespace(**vars(args))
    full_args.roi_left = 0.0
    full_args.roi_right = 1.0
    full_args.roi_top = 0.12
    full_args.roi_bottom = 0.82
    full_args.red_roi_left = float(getattr(args, "red_roi_left", 0.0))
    full_args.red_roi_right = float(getattr(args, "red_roi_right", 1.0))
    full_args.red_roi_top = float(getattr(args, "red_roi_top", 0.12))
    full_args.red_roi_bottom = float(getattr(args, "red_roi_bottom", 0.62))
    return calib.capture_detection(cam, sample, run_dir, full_args, previous_record=None)


def tracking_record_at_target_center(frame: np.ndarray | None, args, fallback: dict | None = None) -> dict | None:
    if frame is None:
        return fallback
    h, w = frame.shape[:2]
    u, v = servo.target_center_px(args, w, h)
    record = {
        "detected": True,
        "center_px": [float(u), float(v)],
    }
    if fallback:
        record.update({k: v2 for k, v2 in fallback.items() if k not in record})
    return record


def compute_plan(record: dict, args) -> dict:
    if not record.get("detected"):
        raise RuntimeError("No detection record available for planning")
    err = np.asarray(record["error_px"], dtype=np.float64)
    raw_move, model_name = correction_from_error(err, args)
    correction_scales = xy_correction_scales(args)
    scaled_move = raw_move * correction_scales
    max_move = None if bool(args.no_limit) else float(args.max_move_mm)
    move, limit_scale = limit_vector_mm(scaled_move, max_move)
    return {
        "error_px": err,
        "raw_move_mm": raw_move,
        "model": model_name,
        "scaled_move_mm": scaled_move,
        "move_mm": move,
        "xy_correction_scale": float(args.xy_correction_scale),
        "xy_correction_scale_x": float(correction_scales[0]),
        "xy_correction_scale_y": float(correction_scales[1]),
        "limit_scale": limit_scale,
        "max_move_mm": max_move,
    }


def draw_preview(frame: np.ndarray, last_record: dict | None, plan: dict | None, args) -> np.ndarray:
    det = None
    draw_args = args
    if last_record and last_record.get("detected"):
        draw_args = calib.make_tracking_args(args, frame, last_record)
        det = servo.detect_black_region(frame, draw_args)
    out = servo.draw_detection(frame, det, draw_args) if det is not None else frame.copy()
    h, _w = out.shape[:2]
    color = (0, 255, 0)
    if plan is None:
        if getattr(args, "auto_once", False):
            if getattr(args, "after_grasp_action", "") == "return-observe":
                text1 = "AUTO-ONCE  wait -> detect -> xy -> grasp -> lift -> observe"
            else:
                text1 = "AUTO-ONCE  wait -> detect -> xy -> grasp -> lift, keep gripper closed"
        elif getattr(args, "auto_cycle", False):
            text1 = "AUTO  detect+xy+grasp+place+home loop running  q:quit"
        elif getattr(args, "quick_grasp_on_d", False):
            text1 = "QUICK  d:detect+angle+xy+grasp  h:release/home/close  r:reset  q:quit"
        else:
            text1 = "d:detect/lock  a:angle  g:xy  j:grasp  h:home  r:reset-lock  q:quit"
    else:
        err = plan["error_px"]
        move = plan["move_mm"]
        raw = plan["raw_move_mm"]
        text1 = (
            f"err=({err[0]:+.1f},{err[1]:+.1f})px "
            f"go=({move[0]:+.1f},{move[1]:+.1f})mm raw=({raw[0]:+.1f},{raw[1]:+.1f})mm "
            f"kx={plan['xy_correction_scale_x']:.2f} ky={plan['xy_correction_scale_y']:.2f} {plan['model']}"
        )
    cv2.putText(out, text1, (18, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, lineType=cv2.LINE_AA)
    if getattr(args, "auto_once", False):
        if getattr(args, "after_grasp_action", "") == "return-observe":
            text2 = "auto-once: return observe after grasp; press d again    q/Esc:stop"
        else:
            text2 = "auto-once: stop after vertical lift, no release, no repeat    q/Esc:stop"
    elif getattr(args, "auto_cycle", False):
        text2 = "auto: wait -> detect -> xy -> grasp -> place -> observe -> close    q/Esc:stop"
    else:
        text2 = "d:detect/lock  a:angle  g:xy  j:grasp  h:release/home/close  r:reset-lock  q:quit"
    cv2.putText(out, text2, (18, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, lineType=cv2.LINE_AA)
    return out


def resize_preview(image: np.ndarray, max_width: int) -> np.ndarray:
    max_w = max(240, int(max_width))
    if image.shape[1] <= max_w:
        return image
    scale = max_w / float(image.shape[1])
    return cv2.resize(image, (max_w, int(round(image.shape[0] * scale))), interpolation=cv2.INTER_AREA)


def show_saved_detection_preview(record: dict | None, plan: dict | None, args, *, label: str) -> None:
    if float(args.quick_detect_preview_s) <= 0.0:
        return
    if not record or not record.get("detection_dir"):
        return
    vis_path = Path(record["detection_dir"]) / "latest_vis.png"
    vis = cv2.imread(str(vis_path))
    if vis is None:
        return
    out = vis.copy()
    h, _w = out.shape[:2]
    if record.get("detected") and plan is not None:
        err = plan["error_px"]
        move = plan["move_mm"]
        text = f"{label} err=({err[0]:+.1f},{err[1]:+.1f})px go=({move[0]:+.1f},{move[1]:+.1f})mm"
    elif record.get("detected"):
        text = f"{label} detected"
    else:
        text = f"{label} no detection"
    cv2.putText(out, text, (18, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 255), 2, lineType=cv2.LINE_AA)
    cv2.putText(out, "preview before quick action", (18, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, lineType=cv2.LINE_AA)
    cv2.imshow("Skewer Align Live", resize_preview(out, args.preview_max_width))
    delay_ms = max(1, int(round(float(args.quick_detect_preview_s) * 1000.0)))
    cv2.waitKey(delay_ms)


def generate_sample_for_target(
    *,
    name: str,
    target_T: np.ndarray,
    q_start: list[float],
    gripper: float,
    run_dir: Path,
    args,
) -> calib.PoseSample:
    q_goal = calib.solve_ik(target_T, q_start, run_dir, name, args)
    max_delta = max(abs(a - b) for a, b in zip(q_start, q_goal))
    if max_delta > float(args.max_step_joint_delta_rad):
        raise JointDeltaLimitError(
            f"{name} step joint delta too large: {max_delta:.3f} rad > {args.max_step_joint_delta_rad}. "
            "Reduce the motion or increase the limit only after checking workspace."
        )
    replay_csv = run_dir / "jog_replays" / f"{name}.csv"
    calib.write_replay_csv(replay_csv, q_start, q_goal, gripper, args)
    return calib.PoseSample(name, (0.0, 0.0), q_goal, gripper, replay_csv, run_dir / f"T_base_ee_{name}.json")


def generate_xy_sample_with_joint_limit(
    *,
    name: str,
    current_T: np.ndarray,
    q_start: list[float],
    gripper: float,
    move_mm: np.ndarray,
    run_dir: Path,
    args,
) -> tuple[calib.PoseSample, np.ndarray, np.ndarray, float]:
    scale = 1.0
    min_scale = max(0.05, float(args.min_xy_joint_limit_scale))
    retry_scale = min(0.95, max(0.1, float(args.xy_joint_limit_retry_scale)))
    last_error: JointDeltaLimitError | None = None
    for attempt in range(max(1, int(args.max_xy_joint_limit_retries)) + 1):
        scaled_move = np.asarray(move_mm, dtype=np.float64) * scale
        target_T = current_T.copy()
        target_T[0, 3] += float(scaled_move[0]) * 0.001
        target_T[1, 3] += float(scaled_move[1]) * 0.001
        attempt_name = name if attempt == 0 else f"{name}_scale{int(round(scale * 100)):03d}"
        try:
            sample = generate_sample_for_target(
                name=attempt_name,
                target_T=target_T,
                q_start=q_start,
                gripper=gripper,
                run_dir=run_dir,
                args=args,
            )
            if attempt > 0:
                print(
                    f"[align-live][WARN] XY move scaled to {scale:.2f} to satisfy joint limit: "
                    f"({move_mm[0]:+.1f},{move_mm[1]:+.1f})mm -> ({scaled_move[0]:+.1f},{scaled_move[1]:+.1f})mm"
                )
            return sample, scaled_move, target_T, scale
        except JointDeltaLimitError as exc:
            last_error = exc
            next_scale = scale * retry_scale
            if next_scale < min_scale:
                break
            print(f"[align-live][WARN] {exc}; retry XY scale {next_scale:.2f}")
            scale = next_scale
    if last_error is not None:
        raise last_error
    raise RuntimeError("failed to generate XY sample")


def write_replay_csv_8d(
    path: Path,
    waypoints: list[tuple[list[float], float]],
    durations: list[float],
    sample_period: float,
) -> None:
    if len(durations) != max(0, len(waypoints) - 1):
        raise RuntimeError(f"Need {max(0, len(waypoints) - 1)} durations, got {len(durations)}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        header = ["time"]
        for i in range(8):
            header.extend([f"q{i}", f"dq{i}"])
        writer.writerow(header)

        def write_row(t: float, q8: list[float], dq8: list[float]) -> None:
            row = [f"{t:.6f}"]
            for q, dq in zip(q8, dq8):
                row.extend([f"{float(q):.8f}", f"{float(dq):.8f}"])
            writer.writerow(row)

        time_s = 0.0
        first_q, first_g = waypoints[0]
        write_row(time_s, [*first_q, float(first_g)], [0.0] * 8)
        for duration, start, goal in zip(durations, waypoints[:-1], waypoints[1:]):
            if duration <= 0.0:
                raise RuntimeError(f"Replay segment duration must be positive, got {duration}")
            start_q8 = [float(v) for v in [*start[0], start[1]]]
            goal_q8 = [float(v) for v in [*goal[0], goal[1]]]
            delta = [b - a for a, b in zip(start_q8, goal_q8)]
            steps = max(1, int(round(duration / sample_period)))
            for step in range(1, steps + 1):
                u = step / steps
                s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
                ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
                q8 = [a + s * d for a, d in zip(start_q8, delta)]
                dq8 = [(ds_du / duration) * d for d in delta]
                time_s += duration / steps
                write_row(time_s, q8, dq8)


def replay_csv_duration_s(path: Path) -> float:
    last_time = 0.0
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row and row.get("time"):
                last_time = float(row["time"])
    return last_time


def read_first_replay_q(path: Path) -> tuple[list[float], float]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        first = next((row for row in reader if row), None)
    if first is None:
        raise RuntimeError(f"Replay CSV is empty: {path}")
    q = []
    for i in range(7):
        key = f"q{i}"
        if key not in first:
            raise RuntimeError(f"{path} missing column {key}")
        q.append(float(first[key]))
    return q, float(first.get("q7", 0.0))


def play_full_replay_on_server(server_proc, replay_csv: Path, args, *, label: str) -> None:
    if server_proc.poll() is not None:
        raise RuntimeError(f"OpenArm replay server is not running, code={server_proc.returncode}")
    duration_s = replay_csv_duration_s(replay_csv)
    message = f"play|{replay_csv.resolve()}"
    print(f"[{label}] UDP {message} -> port {args.replay_command_port}")
    print(f"[{label}] waiting full replay duration {duration_s:.2f}s + settle {float(args.settle_s):.2f}s")
    calib.send_udp(args.replay_command_port, message)
    time.sleep(duration_s + float(args.settle_s))
    if server_proc.poll() is not None:
        raise RuntimeError(f"OpenArm replay server exited during {label}, code={server_proc.returncode}")


def parse_q_values(text: str) -> tuple[list[float], float | None]:
    values = [float(v.strip()) for v in str(text).split(",") if v.strip()]
    if len(values) == 7:
        return values, None
    if len(values) == 8:
        return values[:7], values[7]
    raise RuntimeError("--place-q must contain 7 arm joints, or 8 values where the gripper value is ignored for placing")


def read_last_position_snapshot(path: Path, arm_side: str = "right_arm") -> tuple[list[float], float]:
    if not path.exists():
        raise FileNotFoundError(path)
    last = ""
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if f"arm_side={arm_side}" in line and "follower={" in line:
            last = line
    if not last:
        raise RuntimeError(f"No {arm_side} follower position snapshot found in {path}")
    values = []
    for i in range(1, 8):
        m = re.search(rf"arm_m{i}=([-+0-9.eE]+)", last)
        if not m:
            raise RuntimeError(f"Missing arm_m{i} in snapshot: {last}")
        values.append(float(m.group(1)))
    gm = re.search(r"gripper_m1=([-+0-9.eE]+)", last)
    if not gm:
        raise RuntimeError(f"Missing gripper_m1 in snapshot: {last}")
    return values, float(gm.group(1))


def resolve_place_q(args) -> tuple[list[float] | None, float | None, str]:
    if args.place_q:
        q, gripper = parse_q_values(args.place_q)
        return q, gripper, "--place-q"
    if args.place_replay:
        q, gripper = calib.read_last_replay_q(args.place_replay.expanduser().resolve())
        return q, gripper, f"--place-replay {args.place_replay}"
    if args.place_snapshot:
        q, gripper = read_last_position_snapshot(args.place_snapshot.expanduser().resolve(), args.arm_side)
        return q, gripper, f"--place-snapshot {args.place_snapshot}"
    return None, None, ""


def execute_grasp_sequence(
    *,
    q_aligned: list[float],
    T_aligned: np.ndarray,
    server_proc,
    run_dir: Path,
    args,
) -> tuple[list[float], np.ndarray, float]:
    args.last_grasp_reverse_return_waypoints = []
    down_T = T_aligned.copy()
    down_T[2, 3] -= float(args.grasp_descent_m)
    q_down = calib.solve_ik(down_T, q_aligned, run_dir, "grasp_down", args)

    requested_lift_m = float(args.post_grasp_lift_m)
    min_lift_m = max(0.0, float(args.min_post_grasp_lift_m))
    lift_candidates = []
    for value in (
        requested_lift_m,
        requested_lift_m * 0.75,
        requested_lift_m * 0.50,
        requested_lift_m * 0.25,
        min_lift_m,
    ):
        lift_m = round(float(value), 4)
        if lift_m < min_lift_m or lift_m <= 0.0 or lift_m in lift_candidates:
            continue
        lift_candidates.append(lift_m)
    if not lift_candidates:
        raise RuntimeError("--post-grasp-lift-m/--min-post-grasp-lift-m leave no positive lift candidate")

    retreat_m = max(0.0, float(args.post_grasp_retreat_mm) * 0.001)
    retreat_order = str(getattr(args, "post_grasp_retreat_order", "before-lift"))
    pre_lift_T = down_T.copy()
    q_pre_lift = list(q_down)
    if retreat_m > 0.0 and retreat_order == "before-lift":
        # Same direction as calibration jog key "s": base -X.
        pre_lift_T[0, 3] -= retreat_m
        q_pre_lift = calib.solve_ik(pre_lift_T, q_down, run_dir, "grasp_retreat_x_minus", args)

    q_lift = None
    lift_T = None
    actual_lift_m = None
    last_lift_error: Exception | None = None
    for lift_m in lift_candidates:
        candidate_T = pre_lift_T.copy()
        candidate_T[2, 3] += lift_m
        try:
            candidate_q = calib.solve_ik(candidate_T, q_pre_lift, run_dir, f"grasp_lift_{int(round(lift_m * 1000.0)):03d}mm", args)
        except Exception as exc:
            last_lift_error = exc
            print(f"[grasp][WARN] lift IK failed at {lift_m * 1000.0:.0f}mm; trying a smaller vertical lift.")
            continue
        q_lift = candidate_q
        lift_T = candidate_T
        actual_lift_m = lift_m
        if lift_m < requested_lift_m:
            print(f"[grasp][WARN] using smaller post-grasp vertical lift: {lift_m * 1000.0:.0f}mm instead of {requested_lift_m * 1000.0:.0f}mm")
        break
    if q_lift is None or lift_T is None or actual_lift_m is None:
        raise RuntimeError(
            "post-grasp lift IK failed for all lift heights; robot has not executed the grasp replay. "
            "Try smaller --grasp-descent-m, smaller --post-grasp-lift-m, or move the observe pose away from joint limits."
        ) from last_lift_error

    retreat_T = pre_lift_T.copy()
    q_retreat = list(q_pre_lift)
    final_q = list(q_lift)
    final_T = lift_T.copy()
    if retreat_m > 0.0 and retreat_order == "after-lift":
        retreat_T = lift_T.copy()
        retreat_T[0, 3] -= retreat_m
        q_retreat = calib.solve_ik(retreat_T, q_lift, run_dir, "grasp_retreat_after_lift_x_minus", args)
        final_q = list(q_retreat)
        final_T = retreat_T.copy()
    elif retreat_m > 0.0:
        retreat_T = pre_lift_T.copy()
        final_q = list(q_lift)
        final_T = lift_T.copy()

    max_delta_down = max(abs(a - b) for a, b in zip(q_aligned, q_down))
    max_delta_retreat = (
        max(abs(a - b) for a, b in zip(q_down, q_pre_lift))
        if retreat_order == "before-lift"
        else max(abs(a - b) for a, b in zip(q_lift, q_retreat))
    )
    max_delta_lift = max(abs(a - b) for a, b in zip(q_pre_lift, q_lift))
    max_delta = max(max_delta_down, max_delta_retreat, max_delta_lift)
    if max_delta > float(args.max_grasp_joint_delta_rad):
        raise RuntimeError(
            f"grasp joint delta too large: {max_delta:.3f} rad > {args.max_grasp_joint_delta_rad}. "
            "Reduce --grasp-descent-m or check the observe pose."
        )

    replay_csv = run_dir / "grasp_replays" / "align_down_close_lift.csv"
    waypoints = [
        (q_aligned, float(args.gripper_current_after_align)),
        (q_aligned, float(args.gripper_open)),
        (q_down, float(args.gripper_open)),
        (q_down, float(args.gripper_open)),
        (q_down, float(args.gripper_close)),
        (q_down, float(args.gripper_close)),
    ]
    durations = [
        float(args.gripper_open_duration),
        float(args.grasp_descent_duration),
        float(args.pre_close_hold_duration),
        float(args.gripper_close_duration),
        float(args.gripper_close_hold_duration),
    ]
    if retreat_m > 0.0:
        if retreat_order == "after-lift":
            waypoints.append((q_lift, float(args.gripper_close)))
            durations.append(float(args.post_grasp_lift_duration))
            waypoints.append((q_retreat, float(args.gripper_close)))
            durations.append(float(args.post_grasp_retreat_duration))
        else:
            waypoints.append((q_retreat, float(args.gripper_close)))
            durations.append(float(args.post_grasp_retreat_duration))
            waypoints.append((q_lift, float(args.gripper_close)))
            durations.append(float(args.post_grasp_lift_duration))
    else:
        waypoints.append((q_lift, float(args.gripper_close)))
        durations.append(float(args.post_grasp_lift_duration))

    reverse_return_waypoints = []
    if retreat_m > 0.0 and retreat_order == "after-lift":
        # The retreat is already the safe post-grasp exit. Do not undo it and
        # carry the skewer back over the grasp point; return to observe from
        # the retreated pose.
        reverse_return_waypoints = []
    elif retreat_m > 0.0:
        reverse_return_waypoints.append(
            {
                "name": "reverse_lift_to_retreat",
                "q": q_retreat,
                "gripper": float(args.gripper_close),
                "duration": float(args.post_grasp_lift_duration),
            }
        )
        reverse_return_waypoints.append(
            {
                "name": "reverse_retreat_to_down",
                "q": q_down,
                "gripper": float(args.gripper_close),
                "duration": float(args.post_grasp_retreat_duration),
            }
        )
        reverse_return_waypoints.append(
            {
                "name": "reverse_descent_to_aligned",
                "q": q_aligned,
                "gripper": float(args.gripper_close),
                "duration": float(args.grasp_descent_duration),
            }
        )
    else:
        reverse_return_waypoints.append(
            {
                "name": "reverse_lift_to_aligned",
                "q": q_aligned,
                "gripper": float(args.gripper_close),
                "duration": float(args.post_grasp_lift_duration),
            }
        )
    args.last_grasp_reverse_return_waypoints = reverse_return_waypoints

    write_replay_csv_8d(replay_csv, waypoints, durations, float(args.replay_sample_period))
    save_json(
        run_dir / "grasp_plan.json",
        {
            "grasp_descent_m": float(args.grasp_descent_m),
            "post_grasp_lift_m": float(actual_lift_m),
            "requested_post_grasp_lift_m": float(requested_lift_m),
            "post_grasp_retreat_m": float(retreat_m),
            "post_grasp_retreat_direction": "base_x_minus",
            "post_grasp_retreat_order": retreat_order,
            "post_grasp_retreat_duration": float(args.post_grasp_retreat_duration),
            "gripper_open": float(args.gripper_open),
            "gripper_close": float(args.gripper_close),
            "pre_close_hold_duration": float(args.pre_close_hold_duration),
            "gripper_close_hold_duration": float(args.gripper_close_hold_duration),
            "q_aligned": q_aligned,
            "q_down": q_down,
            "q_retreat": q_retreat,
            "q_lift": q_lift,
            "reverse_return_waypoints": reverse_return_waypoints,
            "T_base_ee_aligned": calib.matrix_to_list(T_aligned),
            "T_base_ee_retreat": calib.matrix_to_list(retreat_T),
            "replay_csv": str(replay_csv),
        },
    )
    print(
        "[grasp] open -> down "
        f"{float(args.grasp_descent_m) * 1000.0:.0f}mm -> close"
        f" -> {'lift ' + str(round(actual_lift_m * 1000.0)) + 'mm -> retreat -X ' + str(round(retreat_m * 1000.0)) + 'mm' if retreat_order == 'after-lift' else 'retreat -X ' + str(round(retreat_m * 1000.0)) + 'mm -> lift ' + str(round(actual_lift_m * 1000.0)) + 'mm'}"
    )
    play_full_replay_on_server(server_proc, replay_csv, args, label="grasp")
    return final_q, final_T, float(args.gripper_close)


def execute_release_and_return_observe(
    *,
    current_q: list[float],
    q_observe: list[float],
    current_gripper: float,
    server_proc,
    run_dir: Path,
    args,
) -> float:
    place_q, ignored_place_gripper, place_source = resolve_place_q(args)
    if place_q is not None:
        place_hold_gripper = float(current_gripper)
        replay_csv = run_dir / "return_replays" / "place_open_return_observe_close.csv"
        waypoints = [
            (current_q, place_hold_gripper),
            (place_q, place_hold_gripper),
            (place_q, float(args.gripper_open)),
            (q_observe, float(args.gripper_open)),
            (q_observe, float(args.gripper_close)),
        ]
        durations = [
            float(args.place_duration),
            float(args.gripper_open_duration),
            float(args.return_observe_duration),
            float(args.gripper_close_duration),
        ]
    else:
        replay_csv = run_dir / "return_replays" / "return_observe_open_close.csv"
        waypoints = [
            (current_q, float(current_gripper)),
            (q_observe, float(current_gripper)),
            (q_observe, float(args.gripper_open)),
            (q_observe, float(args.gripper_close)),
        ]
        durations = [
            float(args.return_observe_duration),
            float(args.gripper_open_duration),
            float(args.gripper_close_duration),
        ]
    write_replay_csv_8d(replay_csv, waypoints, durations, float(args.replay_sample_period))
    save_json(
        run_dir / "return_observe_plan.json",
        {
            "q_start": current_q,
            "q_place": place_q,
            "place_source": place_source,
            "ignored_place_gripper": ignored_place_gripper,
            "place_hold_gripper": float(current_gripper),
            "q_observe": q_observe,
            "gripper_open": float(args.gripper_open),
            "gripper_close": float(args.gripper_close),
            "replay_csv": str(replay_csv),
        },
    )
    if place_q is not None:
        print(
            "[home] keep grasp gripper "
            f"{float(current_gripper):+.4f} -> place position -> open gripper -> return observe -> close gripper"
        )
    else:
        print("[home] return observe -> open gripper -> close gripper")
    play_full_replay_on_server(server_proc, replay_csv, args, label="home")
    return float(args.gripper_close)


def execute_return_place_keep_gripper(
    *,
    current_q: list[float],
    current_gripper: float,
    server_proc,
    run_dir: Path,
    args,
) -> float:
    place_q, ignored_place_gripper, place_source = resolve_place_q(args)
    if place_q is None:
        raise RuntimeError("--auto-once 需要放置位：请传 --place-snapshot、--place-q 或 --place-replay")
    replay_csv = run_dir / "return_replays" / "return_place_keep_gripper.csv"
    waypoints = [
        (current_q, float(current_gripper)),
        (place_q, float(current_gripper)),
    ]
    durations = [float(args.place_duration)]
    write_replay_csv_8d(replay_csv, waypoints, durations, float(args.replay_sample_period))
    save_json(
        run_dir / "return_place_keep_gripper_plan.json",
        {
            "q_start": current_q,
            "q_place": place_q,
            "place_source": place_source,
            "ignored_place_gripper": ignored_place_gripper,
            "held_gripper": float(current_gripper),
            "place_duration": float(args.place_duration),
            "replay_csv": str(replay_csv),
        },
    )
    print(
        "[place-once] move to place while keeping gripper closed at "
        f"{float(current_gripper):+.4f}; no release"
    )
    play_full_replay_on_server(server_proc, replay_csv, args, label="place-once")
    return float(current_gripper)


def execute_return_observe_keep_gripper(
    *,
    current_q: list[float],
    q_observe: list[float],
    current_gripper: float,
    server_proc,
    run_dir: Path,
    args,
) -> float:
    replay_csv = run_dir / "return_replays" / "return_observe_keep_gripper.csv"
    reverse_return_waypoints = list(getattr(args, "last_grasp_reverse_return_waypoints", []) or [])
    waypoints = [(current_q, float(current_gripper))]
    durations = []
    for item in reverse_return_waypoints:
        waypoint_q = item.get("q")
        if waypoint_q is None:
            continue
        waypoints.append((list(waypoint_q), float(item.get("gripper", current_gripper))))
        durations.append(float(item.get("duration", args.return_observe_duration)))
    waypoints.append((q_observe, float(current_gripper)))
    durations.append(float(args.return_observe_duration))
    write_replay_csv_8d(replay_csv, waypoints, durations, float(args.replay_sample_period))
    save_json(
        run_dir / "return_observe_keep_gripper_plan.json",
        {
            "q_start": current_q,
            "q_observe": q_observe,
            "held_gripper": float(current_gripper),
            "reverse_return_waypoints": reverse_return_waypoints,
            "return_observe_duration": float(args.return_observe_duration),
            "replay_csv": str(replay_csv),
        },
    )
    if reverse_return_waypoints:
        print(
            "[return-observe] reverse grasp path -> observe while keeping gripper at "
            f"{float(current_gripper):+.4f}; press d for the next grasp"
        )
    else:
        print(
            "[return-observe] return to observe while keeping gripper at "
            f"{float(current_gripper):+.4f}; press d for the next grasp"
        )
    play_full_replay_on_server(server_proc, replay_csv, args, label="return-observe")
    args.last_grasp_reverse_return_waypoints = []
    return float(current_gripper)


def execute_joint_transition(
    *,
    name: str,
    label: str,
    q_start: list[float],
    gripper_start: float,
    q_goal: list[float],
    gripper_goal: float,
    duration_s: float,
    server_proc,
    run_dir: Path,
    args,
) -> None:
    replay_csv = run_dir / "transition_replays" / f"{name}.csv"
    waypoints = [
        (q_start, float(gripper_start)),
        (q_goal, float(gripper_goal)),
    ]
    write_replay_csv_8d(replay_csv, waypoints, [float(duration_s)], float(args.replay_sample_period))
    save_json(
        run_dir / "transition_replays" / f"{name}.json",
        {
            "q_start": q_start,
            "gripper_start": float(gripper_start),
            "q_goal": q_goal,
            "gripper_goal": float(gripper_goal),
            "duration_s": float(duration_s),
            "replay_csv": str(replay_csv),
        },
    )
    print(f"[{label}] smooth joint transition {float(duration_s):.2f}s: {replay_csv}")
    play_full_replay_on_server(server_proc, replay_csv, args, label=label)


def execute_post_replay(
    *,
    current_q: list[float],
    current_gripper: float,
    server_proc,
    run_dir: Path,
    args,
) -> tuple[list[float], float]:
    post_replay = args.post_replay.expanduser().resolve()
    q_post_start, gripper_post_start = read_first_replay_q(post_replay)
    q_post_end, gripper_post_end = calib.read_last_replay_q(post_replay)
    execute_joint_transition(
        name="lift_to_post_replay_start",
        label="post-prep",
        q_start=current_q,
        gripper_start=float(current_gripper),
        q_goal=q_post_start,
        gripper_goal=float(gripper_post_start),
        duration_s=float(args.post_replay_transition_duration),
        server_proc=server_proc,
        run_dir=run_dir,
        args=args,
    )
    print(f"[post-replay] playing recorded post action: {post_replay}")
    play_full_replay_on_server(server_proc, post_replay, args, label="post-replay")
    return q_post_end, float(gripper_post_end)


def run_interactive(args, run_dir: Path, q_observe: list[float], gripper: float, T_observe: np.ndarray) -> None:
    observe_replay = args.observe_replay.expanduser().resolve()
    center_sample = calib.PoseSample("000_observe", (0.0, 0.0), q_observe, float(gripper), observe_replay)

    current_q = list(q_observe)
    current_T = T_observe.copy()
    current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
    current_gripper = float(gripper)
    args.gripper_current_after_align = float(gripper)
    last_frame = None
    last_record = None
    locked_record = None
    last_plan = None
    move_index = 1
    next_auto_grasp_at = None
    auto_cycle_index = 0
    auto_once_done = False

    print("[align-live] Camera window keys:")
    print(f"  d  detect {args.target_selection} target and compute correction")
    print("  a  rotate wrist/camera to make the skewer parallel to the desired angle")
    print("  g  move XY to align detected target center with gripper center")
    print("  j  open, descend, close, and lift")
    print("  h  open gripper, return to observe pose, then close gripper for the next skewer")
    print("  r  reset locked target only when you want to switch to another skewer")
    print("  s  save current RGB")
    print("  q  quit")
    if args.quick_grasp_on_d:
        quick_steps = "detect -> angle -> redetect -> XY -> grasp/lift -> return/open/close" if args.quick_align_angle else "detect -> XY -> grasp/lift -> return/open/close"
        print(f"[align-live] QUICK mode: d will run {quick_steps}. Press d again for the next skewer.")
    if args.auto_cycle:
        print(
            "[align-live] AUTO mode: after observe waits "
            f"{args.auto_start_delay_s:.1f}s, then repeatedly detects, grasps, places, returns observe, and closes."
        )
        print("[align-live] AUTO mode still shows preview. Press q/Esc in the camera window to stop.")
    if args.auto_once:
        print(
            "[align-live] AUTO-ONCE mode: after observe waits "
            f"{args.auto_start_delay_s:.1f}s, then detects, grasps, lifts vertically, and keeps the gripper closed."
        )
        print("[align-live] AUTO-ONCE mode stops after grasp/lift by default. Press q/Esc after it finishes.")
    if args.no_limit:
        print("[align-live][WARN] no motion limit is enabled.")
    else:
        print(f"[align-live] max move per g = {args.max_move_mm:.1f} mm")
    print("[align-live] action order: d -> a -> d -> g -> j -> h -> next skewer")

    server_proc = None
    try:
        if args.execute:
            if args.pre_replay is not None:
                pre_replay = args.pre_replay.expanduser().resolve()
                print(f"[pre-replay] playing recorded pre action: {pre_replay}")
                server_proc = calib.start_replay_server(pre_replay, args)
                calib.play_on_replay_server(server_proc, pre_replay, args, initial_start=True)
                q_pre_end, gripper_pre_end = calib.read_last_replay_q(pre_replay)
                print(f"[pre-replay] smooth transition from pre action end to observe pose: {observe_replay}")
                execute_joint_transition(
                    name="pre_replay_end_to_observe",
                    label="pre-to-observe",
                    q_start=q_pre_end,
                    gripper_start=float(gripper_pre_end),
                    q_goal=q_observe,
                    gripper_goal=float(gripper),
                    duration_s=float(args.pre_to_observe_duration),
                    server_proc=server_proc,
                    run_dir=run_dir,
                    args=args,
                )
            else:
                print(f"[align-live] moving to observe using recorded observe replay: {observe_replay}")
                server_proc = calib.start_replay_server(observe_replay, args)
                calib.play_on_replay_server(server_proc, observe_replay, args, initial_start=True)
        else:
            print("[align-live] dry run only. Add --execute to move on g.")

        with servo.Camera(args) as cam:
            cv2.namedWindow("Skewer Align Live", cv2.WINDOW_NORMAL)
            if args.auto_cycle or args.auto_once:
                next_auto_grasp_at = time.time() + float(args.auto_start_delay_s)
            while True:
                frame = cam.read_color()
                if frame is not None:
                    last_frame = frame
                if last_frame is not None:
                    cv2.imshow("Skewer Align Live", resize_preview(draw_preview(last_frame, last_record, last_plan, args), args.preview_max_width))

                key = cv2.waitKey(1) & 0xFF
                terminal_key = calib.poll_terminal_key()
                if terminal_key:
                    key = ord(terminal_key)
                if (
                    (args.auto_cycle or (args.auto_once and not auto_once_done))
                    and key == 255
                    and next_auto_grasp_at is not None
                    and time.time() >= next_auto_grasp_at
                ):
                    auto_cycle_index += 1
                    key = ord("d")
                    next_auto_grasp_at = None
                    mode_name = "once" if args.auto_once else f"cycle {auto_cycle_index}"
                    print(f"[auto] {mode_name}: auto detect/grasp triggered.")

                if key in (ord("d"), ord("D")):
                    if last_frame is None:
                        print("[align-live][WARN] no camera frame yet")
                        if args.auto_cycle or args.auto_once:
                            next_auto_grasp_at = time.time() + float(args.auto_detect_retry_s)
                        continue
                    sample = calib.PoseSample(
                        f"{move_index:03d}_detect",
                        (float(current_offset[0]), float(current_offset[1])),
                        current_q,
                        float(gripper),
                        run_dir / "unused.csv",
                    )
                    track_from = locked_record if locked_record and locked_record.get("detected") else last_record
                    if track_from and track_from.get("detected"):
                        print("[align-live] locked detect: searching only near the previous target")
                    last_record = detect_once(cam, sample, run_dir, args, previous_record=track_from)
                    if not last_record.get("detected"):
                        if track_from and bool(args.manual_d_full_retry):
                            print("[align-live][WARN] locked ROI missed. Clearing lock and retrying full-image detection once.")
                            retry_sample = calib.PoseSample(
                                f"{move_index:03d}_detect_full_retry",
                                (float(current_offset[0]), float(current_offset[1])),
                                current_q,
                                float(gripper),
                                run_dir / "unused.csv",
                            )
                            last_record = detect_full_frame_once(cam, retry_sample, run_dir, args)
                        if not last_record.get("detected"):
                            locked_record = None
                            last_plan = None
                            print(f"[align-live][WARN] no {args.target_selection} target detected; adjust view/threshold then press d again")
                            if args.auto_cycle or args.auto_once:
                                next_auto_grasp_at = time.time() + float(args.auto_detect_retry_s)
                                print(f"[auto] detection failed; retry in {args.auto_detect_retry_s:.1f}s.")
                            continue
                    locked_record = last_record
                    last_plan = compute_plan(last_record, args)
                    err = last_plan["error_px"]
                    raw = last_plan["raw_move_mm"]
                    move = last_plan["move_mm"]
                    print(
                        f"[align-live] detected err=({err[0]:+.1f},{err[1]:+.1f})px "
                        f"raw=({raw[0]:+.2f},{raw[1]:+.2f})mm go=({move[0]:+.2f},{move[1]:+.2f})mm "
                        f"scale={last_plan['limit_scale']:.3f}"
                    )
                    save_json(
                        run_dir / f"plans/{move_index:03d}_plan.json",
                        {
                            "record": last_record,
                            "raw_move_mm": raw.tolist(),
                            "scaled_move_mm": last_plan["scaled_move_mm"].tolist(),
                            "move_mm": move.tolist(),
                            "model": last_plan["model"],
                            "xy_correction_scale": float(last_plan["xy_correction_scale"]),
                            "limit_scale": float(last_plan["limit_scale"]),
                            "current_offset_xy_mm": current_offset.tolist(),
                            "execute": bool(args.execute),
                        },
                    )
                    if args.quick_grasp_on_d:
                        if not args.execute:
                            print("[quick][WARN] dry run: detected only. Re-run with --execute.")
                            if args.auto_cycle or args.auto_once:
                                next_auto_grasp_at = time.time() + float(args.auto_detect_retry_s)
                            continue
                        show_saved_detection_preview(last_record, last_plan, args, label="quick detect")
                        print("[quick] d detected target. Running XY -> grasp." if not args.quick_align_angle else "[quick] d detected target. Running angle -> redetect -> XY -> grasp.")

                        if args.quick_align_angle:
                            current_angle = float(last_record.get("angle_deg", 90.0))
                            delta_axis = line_axis_delta_deg(current_angle, float(args.desired_skewer_angle_deg))
                            delta_axis = max(
                                -float(args.max_angle_correction_deg),
                                min(float(args.max_angle_correction_deg), delta_axis),
                            )
                            command_delta = float(args.angle_align_sign) * delta_axis
                            if abs(command_delta) >= float(args.min_angle_correction_deg):
                                angle_T = current_T.copy()
                                angle_T[:3, :3] = angle_T[:3, :3] @ rotz(np.deg2rad(command_delta))
                                angle_sample = generate_sample_for_target(
                                    name=f"{move_index:03d}_quick_angle_align",
                                    target_T=angle_T,
                                    q_start=current_q,
                                    gripper=float(current_gripper),
                                    run_dir=run_dir,
                                    args=args,
                                )
                                print(
                                    f"[quick] angle align: detected={current_angle:.1f}deg "
                                    f"target={args.desired_skewer_angle_deg:.1f}deg command={command_delta:+.1f}deg"
                                )
                                calib.play_on_replay_server(server_proc, angle_sample.replay_csv, args)
                                current_q = list(angle_sample.q)
                                current_T = angle_T
                                move_index += 1
                                angle_detect_args = argparse.Namespace(**vars(args))
                                angle_detect_args.track_roi_px = max(float(args.track_roi_px), float(args.angle_track_roi_px))
                                angle_record = detect_once(
                                    cam,
                                    angle_sample,
                                    run_dir,
                                    angle_detect_args,
                                    previous_record=locked_record,
                                )
                                if not angle_record.get("detected"):
                                    print("[quick][WARN] after angle alignment detection failed. Press d again.")
                                    last_record = angle_record
                                    last_plan = None
                                    if args.auto_cycle or args.auto_once:
                                        next_auto_grasp_at = time.time() + float(args.auto_detect_retry_s)
                                        print(f"[auto] redetection failed; retry in {args.auto_detect_retry_s:.1f}s.")
                                    continue
                                last_record = angle_record
                                locked_record = angle_record
                                last_plan = compute_plan(angle_record, args)
                                show_saved_detection_preview(last_record, last_plan, args, label="quick redetect")
                                print(
                                    f"[quick] redetect after angle: angle={angle_record.get('angle_deg', 0.0):.1f}deg "
                                    f"err=({angle_record['error_px'][0]:+.1f},{angle_record['error_px'][1]:+.1f})px"
                                )
                            else:
                                print(f"[quick] angle correction {command_delta:+.1f}deg below threshold; skip rotate")
                        else:
                            print("[quick] angle alignment disabled; skipping wrist rotation.")

                        move_sample, executed_move, target_T, _xy_scale = generate_xy_sample_with_joint_limit(
                            name=f"{move_index:03d}_quick_xy_align",
                            current_T=current_T,
                            q_start=current_q,
                            gripper=float(current_gripper),
                            move_mm=last_plan["move_mm"],
                            run_dir=run_dir,
                            args=args,
                        )
                        next_offset = current_offset + executed_move
                        print(
                            f"[quick] moving XY to offset=({next_offset[0]:+.2f},{next_offset[1]:+.2f})mm "
                            f"csv={move_sample.replay_csv}"
                        )
                        calib.play_on_replay_server(server_proc, move_sample.replay_csv, args)
                        current_q = list(move_sample.q)
                        current_T = target_T
                        current_offset = next_offset
                        if bool(args.quick_verify_after_xy):
                            predicted_record = tracking_record_at_target_center(last_frame, args, fallback=last_record)
                            after = detect_once(cam, move_sample, run_dir, args, previous_record=predicted_record)
                            save_json(
                                run_dir / f"results/{move_index:03d}_quick_result.json",
                                {"before": last_record, "after": after, "executed_offset_xy_mm": current_offset.tolist()},
                            )
                            if after.get("detected"):
                                e0 = np.asarray(last_record["error_px"], dtype=np.float64)
                                e1 = np.asarray(after["error_px"], dtype=np.float64)
                                print(
                                    f"[quick] after XY err=({e1[0]:+.1f},{e1[1]:+.1f})px "
                                    f"norm {np.linalg.norm(e0):.1f}px -> {np.linalg.norm(e1):.1f}px"
                                )
                                last_record = after
                                locked_record = after
                                last_plan = compute_plan(after, args)
                                show_saved_detection_preview(last_record, last_plan, args, label="quick after xy")
                            else:
                                print("[quick][WARN] after XY detection failed; continuing to grasp from aligned pose.")
                                last_record = after
                                last_plan = None
                        else:
                            save_json(
                                run_dir / f"results/{move_index:03d}_quick_result.json",
                                {"before": last_record, "after": None, "executed_offset_xy_mm": current_offset.tolist(), "verify_after_xy": False},
                            )
                            locked_record = None
                            last_record = None
                            last_plan = None
                            print("[quick] XY verify skipped for speed; continuing directly to grasp.")
                        move_index += 1

                        args.gripper_current_after_align = float(current_gripper)
                        current_q, current_T, current_gripper = execute_grasp_sequence(
                            q_aligned=current_q,
                            T_aligned=current_T,
                            server_proc=server_proc,
                            run_dir=run_dir,
                            args=args,
                        )
                        ran_post_replay = False
                        if args.post_replay is not None:
                            current_q, current_gripper = execute_post_replay(
                                current_q=current_q,
                                current_gripper=current_gripper,
                                server_proc=server_proc,
                                run_dir=run_dir,
                                args=args,
                            )
                            current_T = calib.solve_fk(current_q, run_dir, args)
                            ran_post_replay = True
                        elif args.auto_once and args.after_grasp_action == "place":
                            current_gripper = execute_return_place_keep_gripper(
                                current_q=current_q,
                                current_gripper=current_gripper,
                                server_proc=server_proc,
                                run_dir=run_dir,
                                args=args,
                            )
                        elif args.auto_once and args.after_grasp_action == "return-observe":
                            current_gripper = execute_return_observe_keep_gripper(
                                current_q=current_q,
                                q_observe=q_observe,
                                current_gripper=current_gripper,
                                server_proc=server_proc,
                                run_dir=run_dir,
                                args=args,
                            )
                        elif args.auto_once:
                            print("[auto-once] after-grasp-action=lift-only; keeping robot at lifted grasp pose.")
                        else:
                            current_gripper = execute_return_observe_keep_gripper(
                                current_q=current_q,
                                q_observe=q_observe,
                                current_gripper=current_gripper,
                                server_proc=server_proc,
                                run_dir=run_dir,
                                args=args,
                            )
                        if ran_post_replay:
                            current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
                        elif args.auto_once and args.after_grasp_action == "place":
                            place_q, _place_gripper, _place_source = resolve_place_q(args)
                            current_q = list(place_q)
                            current_T = calib.solve_fk(current_q, run_dir, args)
                            current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
                        elif args.auto_once and args.after_grasp_action == "lift-only":
                            current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
                        else:
                            current_q = list(q_observe)
                            current_T = T_observe.copy()
                            current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
                        last_record = None
                        locked_record = None
                        last_plan = None
                        move_index += 1
                        if args.auto_once:
                            auto_once_done = True
                            next_auto_grasp_at = None
                            if ran_post_replay:
                                print("[auto-once] finished. Post replay completed. Press q/Esc to exit.")
                            elif args.after_grasp_action == "place":
                                print("[auto-once] finished. Robot is at place pose and gripper remains closed. Press q/Esc to exit.")
                            elif args.after_grasp_action == "return-observe":
                                print("[auto-once] finished. Robot returned to observe with gripper held. Press d for the next grasp or q/Esc to exit.")
                            else:
                                print("[auto-once] finished. Robot is at lifted grasp pose and gripper remains closed. Press q/Esc to exit.")
                        elif args.auto_cycle:
                            next_auto_grasp_at = time.time() + float(args.auto_cycle_delay_s)
                            print(
                                "[auto] grasp cycle finished. Returned to observe and closed gripper. "
                                f"Next cycle in {args.auto_cycle_delay_s:.1f}s."
                            )
                        else:
                            print("[quick] grasp cycle finished. Returned to observe with gripper held. Press d for next skewer.")

                if key in (ord("r"), ord("R")):
                    locked_record = None
                    last_record = None
                    last_plan = None
                    print("[align-live] target lock reset. Next d will choose a target from the full image.")

                if key in (ord("a"), ord("A")):
                    if not last_record or not last_record.get("detected"):
                        print("[align-live][WARN] press d first; no detected angle to align")
                        continue
                    if not args.execute:
                        print("[align-live] dry run: not moving. Re-run with --execute.")
                        continue
                    current_angle = float(last_record.get("angle_deg", 90.0))
                    delta_axis = line_axis_delta_deg(current_angle, float(args.desired_skewer_angle_deg))
                    delta_axis = max(
                        -float(args.max_angle_correction_deg),
                        min(float(args.max_angle_correction_deg), delta_axis),
                    )
                    command_delta = float(args.angle_align_sign) * delta_axis
                    if abs(command_delta) < float(args.min_angle_correction_deg):
                        print(f"[align-live] angle correction {command_delta:+.1f}deg below threshold; skip rotate")
                        continue
                    angle_T = current_T.copy()
                    angle_T[:3, :3] = angle_T[:3, :3] @ rotz(np.deg2rad(command_delta))
                    angle_sample = generate_sample_for_target(
                        name=f"{move_index:03d}_angle_align",
                        target_T=angle_T,
                        q_start=current_q,
                        gripper=float(current_gripper),
                        run_dir=run_dir,
                        args=args,
                    )
                    print(
                        f"[align-live] angle align: detected={current_angle:.1f}deg "
                        f"target={args.desired_skewer_angle_deg:.1f}deg command={command_delta:+.1f}deg"
                    )
                    calib.play_on_replay_server(server_proc, angle_sample.replay_csv, args)
                    current_q = list(angle_sample.q)
                    current_T = angle_T
                    # Keep locked_record so the next d searches near the same skewer,
                    # not another target in a multi-skewer scene.
                    last_record = locked_record
                    last_plan = None
                    move_index += 1
                    print("[align-live] angle move finished. Press d to redetect before XY alignment.")

                if key in (ord("g"), ord("G")):
                    if last_plan is None:
                        print("[align-live][WARN] press d first; no planned XY correction to execute")
                        continue
                    if not args.execute:
                        print("[align-live] dry run: not moving. Re-run with --execute.")
                        continue

                    move_sample, executed_move, target_T, _xy_scale = generate_xy_sample_with_joint_limit(
                        name=f"{move_index:03d}_xy_align",
                        current_T=current_T,
                        q_start=current_q,
                        gripper=float(current_gripper),
                        move_mm=last_plan["move_mm"],
                        run_dir=run_dir,
                        args=args,
                    )
                    next_offset = current_offset + executed_move
                    print(
                        f"[align-live] moving to offset=({next_offset[0]:+.2f},{next_offset[1]:+.2f})mm "
                        f"csv={move_sample.replay_csv}"
                    )
                    calib.play_on_replay_server(server_proc, move_sample.replay_csv, args)
                    current_q = list(move_sample.q)
                    current_T = target_T
                    current_offset = next_offset
                    if bool(args.verify_after_xy):
                        predicted_record = tracking_record_at_target_center(last_frame, args, fallback=last_record)
                        after = detect_once(cam, move_sample, run_dir, args, previous_record=predicted_record)
                        save_json(
                            run_dir / f"results/{move_index:03d}_result.json",
                            {"before": last_record, "after": after, "executed_offset_xy_mm": current_offset.tolist()},
                        )
                        if after.get("detected"):
                            e0 = np.asarray(last_record["error_px"], dtype=np.float64)
                            e1 = np.asarray(after["error_px"], dtype=np.float64)
                            print(
                                f"[align-live] after err=({e1[0]:+.1f},{e1[1]:+.1f})px "
                                f"norm {np.linalg.norm(e0):.1f}px -> {np.linalg.norm(e1):.1f}px"
                            )
                            last_record = after
                            locked_record = after
                            last_plan = compute_plan(after, args)
                        else:
                            print("[align-live][WARN] after move detection failed; press d again after image is clear")
                            last_record = after
                            last_plan = None
                    else:
                        save_json(
                            run_dir / f"results/{move_index:03d}_result.json",
                            {"before": last_record, "after": None, "executed_offset_xy_mm": current_offset.tolist(), "verify_after_xy": False},
                        )
                        locked_record = None
                        last_record = None
                        last_plan = None
                        print("[align-live] XY verify skipped for speed. Press j to grasp, or d to check.")
                    move_index += 1

                if key in (ord("j"), ord("J")):
                    if not args.execute:
                        print("[align-live] dry run: not grasping. Re-run with --execute.")
                        continue
                    args.gripper_current_after_align = float(current_gripper)
                    current_q, current_T, current_gripper = execute_grasp_sequence(
                        q_aligned=current_q,
                        T_aligned=current_T,
                        server_proc=server_proc,
                        run_dir=run_dir,
                        args=args,
                    )
                    last_record = None
                    locked_record = None
                    last_plan = None
                    move_index += 1
                    print("[align-live] grasp/lift finished. Press h to release, return observe, and close gripper.")

                if key in (ord("h"), ord("H")):
                    if not args.execute:
                        print("[align-live] dry run: not returning. Re-run with --execute.")
                        continue
                    current_gripper = execute_release_and_return_observe(
                        current_q=current_q,
                        q_observe=q_observe,
                        current_gripper=current_gripper,
                        server_proc=server_proc,
                        run_dir=run_dir,
                        args=args,
                    )
                    current_q = list(q_observe)
                    current_T = T_observe.copy()
                    current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
                    last_record = None
                    locked_record = None
                    last_plan = None
                    move_index += 1
                    print("[align-live] back at observe with gripper closed. Place next skewer, then press d.")

                if key in (ord("s"), ord("S")):
                    if last_frame is None:
                        print("[align-live][WARN] no camera frame yet")
                    else:
                        save_dir = run_dir / "manual_saves"
                        save_dir.mkdir(parents=True, exist_ok=True)
                        out = save_dir / datetime.now().strftime("rgb_%Y%m%d_%H%M%S.png")
                        cv2.imwrite(str(out), last_frame)
                        print(f"[align-live] saved {out}")

                if key in (ord("q"), 27):
                    print("[align-live] quit requested")
                    break
            cv2.destroyAllWindows()
    finally:
        calib.stop_process(server_proc)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Move OpenArm once for 2D skewer alignment test.")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Default: right_arm uses config/skewer_align_config.json; left_arm uses config/skewer_align_config_left.json.",
    )
    parser.add_argument("--observe-replay", type=Path, default=DEFAULT_OBSERVE_REPLAY)
    parser.add_argument("--pre-replay", type=Path, default=None, help="Optional recorded action to play before moving smoothly to observe.")
    parser.add_argument("--post-replay", type=Path, default=None, help="Optional recorded action to play after grasp/lift, with a smooth transition to its first row.")
    parser.add_argument("--pixel-to-base-mm", default="")
    parser.add_argument(
        "--pixel-to-base-model",
        choices=("linear", "quadratic_offset_delta"),
        default="",
        help="Default comes from config. quadratic_offset_delta uses fitted nonlinear pixel correction.",
    )
    parser.add_argument("--quadratic-offset-from-error-coeff", default=None)
    parser.add_argument("--max-move-mm", type=float, default=60.0)
    parser.add_argument(
        "--xy-correction-scale",
        type=float,
        default=1.0,
        help="Fallback scale applied to both base X/Y corrections when an axis-specific scale is omitted.",
    )
    parser.add_argument(
        "--xy-correction-scale-x",
        type=float,
        default=None,
        help="Scale only the base X correction.",
    )
    parser.add_argument(
        "--xy-correction-scale-y",
        type=float,
        default=None,
        help="Scale only the base Y correction.",
    )
    parser.add_argument("--no-limit", action="store_true", help="Disable max move clipping. Use only after verifying detection.")
    parser.add_argument("--interactive", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "align_once")

    parser.add_argument("--arm-side", choices=("right_arm", "left_arm"), default="right_arm")
    parser.add_argument("--follower-can", default="can0")
    parser.add_argument("--follower-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sudo-password", default="")
    parser.add_argument("--motion-duration", type=float, default=1.5)
    parser.add_argument("--initial-transition-duration", type=float, default=1.0)
    parser.add_argument("--replay-sample-period", type=float, default=0.01)
    parser.add_argument("--settle-s", type=float, default=0.2)
    parser.add_argument("--replay-command-port", type=int, default=47920)
    parser.add_argument(
        "--auto-replay-command-port",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="如果 47920 被旧进程占用，自动换到后面的空闲 UDP 端口。",
    )
    parser.add_argument("--replay-command-port-scan", type=int, default=50)
    parser.add_argument("--server-startup-s", type=float, default=3.0)
    parser.add_argument("--auto-build-kdl", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ik-max-iter", type=int, default=700)
    parser.add_argument("--max-step-joint-delta-rad", type=float, default=0.85)
    parser.add_argument("--max-xy-joint-limit-retries", type=int, default=4)
    parser.add_argument("--xy-joint-limit-retry-scale", type=float, default=0.75)
    parser.add_argument("--min-xy-joint-limit-scale", type=float, default=0.25)
    parser.add_argument("--gripper", type=float, default=None)
    parser.add_argument("--grasp-after-go", action="store_true", help="Deprecated; actions are split into a/g/j/h keys now.")
    parser.add_argument(
        "--quick-grasp-on-d",
        action="store_true",
        help="Quick mode: pressing d runs detect, angle alignment, redetect, XY alignment, and grasp/lift.",
    )
    parser.add_argument(
        "--auto-cycle",
        action="store_true",
        help="Fully automatic loop: after observe, auto trigger quick grasp, place, return observe, then repeat. q/Esc exits.",
    )
    parser.add_argument(
        "--auto-once",
        action="store_true",
        help="Single automatic grasp: after observe, auto trigger quick grasp, then stop after vertical lift by default.",
    )
    parser.add_argument(
        "--after-grasp-action",
        choices=("lift-only", "return-observe", "place"),
        default="lift-only",
        help="Auto-once post-grasp action. lift-only stops after vertical lift; return-observe moves back while holding gripper; place moves to --place-*.",
    )
    parser.add_argument(
        "--auto-start-delay-s",
        type=float,
        default=1.0,
        help="Auto-cycle wait time after reaching observe before the first detection.",
    )
    parser.add_argument(
        "--auto-cycle-delay-s",
        type=float,
        default=1.0,
        help="Auto-cycle wait time after returning observe before starting the next skewer.",
    )
    parser.add_argument(
        "--auto-detect-retry-s",
        type=float,
        default=1.0,
        help="Auto-cycle retry wait time when no target is detected.",
    )
    parser.add_argument(
        "--quick-align-angle",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="In quick mode, rotate wrist before XY alignment. Use --no-quick-align-angle for faster straight grasp.",
    )
    parser.add_argument(
        "--quick-detect-preview-s",
        type=float,
        default=0.25,
        help="In quick mode, show each detection result for this many seconds before continuing.",
    )
    parser.add_argument(
        "--quick-verify-after-xy",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="In quick mode, redetect after XY alignment before grasping. Default off for speed.",
    )
    parser.add_argument(
        "--verify-after-xy",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="In manual mode, redetect after g. Default off for speed; press d manually when needed.",
    )
    parser.add_argument(
        "--align-angle-before-xy",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Pressing g rotates wrist to match skewer angle first, then redetects and translates.",
    )
    parser.add_argument(
        "--desired-skewer-angle-deg",
        type=float,
        default=90.0,
        help="Desired target/skewer axis angle in the image after wrist rotation. 90 means vertical.",
    )
    parser.add_argument(
        "--angle-align-sign",
        type=float,
        default=1.0,
        help="Use 1 or -1. If wrist rotation makes image angle worse, flip this sign.",
    )
    parser.add_argument("--max-angle-correction-deg", type=float, default=25.0)
    parser.add_argument("--min-angle-correction-deg", type=float, default=2.0)
    parser.add_argument("--gripper-open", type=float, default=-1.13)
    parser.add_argument("--gripper-close", type=float, default=-0.018)
    parser.add_argument("--gripper-open-duration", type=float, default=0.35)
    parser.add_argument("--gripper-close-duration", type=float, default=1.2)
    parser.add_argument("--pre-close-hold-duration", type=float, default=0.6)
    parser.add_argument("--gripper-close-hold-duration", type=float, default=1.2)
    parser.add_argument("--grasp-descent-m", type=float, default=0.06)
    parser.add_argument("--grasp-descent-duration", type=float, default=3.0)
    parser.add_argument("--post-grasp-lift-m", type=float, default=0.06)
    parser.add_argument("--min-post-grasp-lift-m", type=float, default=0.02)
    parser.add_argument("--post-grasp-lift-duration", type=float, default=3.0)
    parser.add_argument(
        "--post-grasp-retreat-mm",
        type=float,
        default=0.0,
        help="After closing gripper, move base -X by this many mm before vertical lift. Same direction as calibration jog key s.",
    )
    parser.add_argument("--post-grasp-retreat-duration", type=float, default=1.5)
    parser.add_argument(
        "--post-grasp-retreat-order",
        choices=("before-lift", "after-lift"),
        default="before-lift",
        help="Order for post-grasp retreat. after-lift means close -> vertical lift -> base -X retreat.",
    )
    parser.add_argument("--place-q", default="", help="Optional place pose: 7 arm joints, or 8 values including gripper.")
    parser.add_argument("--place-replay", type=Path, default=None, help="Optional replay CSV; use its last row as place pose.")
    parser.add_argument("--place-snapshot", type=Path, default=None, help="Optional follower_position_snapshots.txt; use its last right-arm follower row as place pose.")
    parser.add_argument("--place-duration", type=float, default=3.0)
    parser.add_argument("--return-observe-duration", type=float, default=2.5)
    parser.add_argument("--pre-to-observe-duration", type=float, default=3.0)
    parser.add_argument("--post-replay-transition-duration", type=float, default=3.0)
    parser.add_argument("--max-grasp-joint-delta-rad", type=float, default=1.2)

    parser.add_argument("--camera-backend", choices=("orbbec", "opencv"), default="opencv")
    parser.add_argument("--orbbec-serial", default="")
    parser.add_argument("--camera-index", type=int, default=8)
    parser.add_argument("--camera-width", type=int, default=0)
    parser.add_argument("--camera-height", type=int, default=0)
    parser.add_argument("--camera-fps", type=int, default=0)
    parser.add_argument("--frame-timeout-ms", type=int, default=30)
    parser.add_argument("--flush-frames", type=int, default=8)
    parser.add_argument("--capture-attempts", type=int, default=4)
    parser.add_argument("--capture-flush-frames", type=int, default=4)
    parser.add_argument("--capture-flush-dt-s", type=float, default=0.005)
    parser.add_argument("--capture-use-latest", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--preview-max-width", type=int, default=960)

    parser.add_argument("--black-v-max", type=int, default=80)
    parser.add_argument("--black-s-min", type=int, default=0)
    parser.add_argument("--min-area-px", type=float, default=180.0)
    parser.add_argument("--max-area-px", type=float, default=8000.0)
    parser.add_argument("--morph-kernel", type=int, default=3)
    parser.add_argument("--ignore-border-px", type=int, default=8)
    parser.add_argument("--roi-left", type=float, default=0.0)
    parser.add_argument("--roi-right", type=float, default=1.0)
    parser.add_argument("--roi-top", type=float, default=0.12)
    parser.add_argument("--roi-bottom", type=float, default=0.82)
    parser.add_argument("--min-elongation", type=float, default=2.0)
    parser.add_argument("--max-angle-from-vertical-deg", type=float, default=35.0)
    parser.add_argument("--min-fill-ratio", type=float, default=0.08)
    parser.add_argument("--skewer-min-area-px", type=float, default=35.0)
    parser.add_argument("--skewer-red-anchor-px", type=float, default=75.0)
    parser.add_argument("--skewer-anchor-max-angle-from-vertical-deg", type=float, default=60.0)
    parser.add_argument("--skewer-preferred-area-px", type=float, default=600.0)
    parser.add_argument("--target-selection", choices=("red", "skewer", "center", "score", "largest"), default="skewer")
    parser.add_argument(
        "--grasp-point-fraction",
        type=float,
        default=0.70,
        help="沿检测目标长轴取抓取点的位置。0.50 是中心；0.70 是更靠后/靠下。",
    )
    parser.add_argument("--red-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--red-h-low", type=int, default=8)
    parser.add_argument("--red-h-high", type=int, default=172)
    parser.add_argument("--red-s-min", type=int, default=130)
    parser.add_argument("--red-v-min", type=int, default=45)
    parser.add_argument("--red-min-area-px", type=float, default=80.0)
    parser.add_argument("--red-max-area-px", type=float, default=3000.0)
    parser.add_argument("--red-preferred-area-px", type=float, default=700.0)
    parser.add_argument("--red-target-distance-weight", type=float, default=1.15)
    parser.add_argument("--red-roi-left", type=float, default=0.0)
    parser.add_argument("--red-roi-right", type=float, default=1.0)
    parser.add_argument("--red-roi-top", type=float, default=0.12)
    parser.add_argument("--red-roi-bottom", type=float, default=0.62)
    parser.add_argument("--red-max-target-dist-px", type=float, default=0.0, help="红色候选离 target-center 的最大距离；0 表示不硬过滤。")
    parser.add_argument("--red-grasp-point-fraction", type=float, default=0.50)
    parser.add_argument("--track-target", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--track-roi-px", type=float, default=90.0)
    parser.add_argument(
        "--angle-track-roi-px",
        type=float,
        default=180.0,
        help="Temporary tracking ROI radius used for redetection after wrist rotation in quick mode.",
    )
    parser.add_argument(
        "--track-fallback-to-full",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="If locked target ROI misses, allow full-image fallback. Default off to avoid switching skewers.",
    )
    parser.add_argument(
        "--manual-d-full-retry",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When manual d uses a stale locked ROI and misses, clear lock and retry one full-image detection.",
    )
    parser.add_argument(
        "--target-center-px",
        default="",
        help="目标对齐像素 u,v。默认相机中心；传夹爪中心后，g 会把检测目标移动到夹爪中心。",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.config is None:
        args.config = DEFAULT_LEFT_CONFIG if args.arm_side == "left_arm" else DEFAULT_CONFIG
    apply_config_defaults(args)
    for scale_name, scale_value in (
        ("--xy-correction-scale", args.xy_correction_scale),
        ("--xy-correction-scale-x", args.xy_correction_scale_x),
        ("--xy-correction-scale-y", args.xy_correction_scale_y),
    ):
        if scale_value is not None and float(scale_value) < 0.0:
            raise RuntimeError(f"{scale_name} must be >= 0")
    if args.auto_cycle and args.auto_once:
        raise RuntimeError("--auto-cycle 和 --auto-once 不能同时使用：一个是循环抓取，一个是只抓一根。")
    if args.auto_cycle and not args.quick_grasp_on_d:
        raise RuntimeError("--auto-cycle 需要同时传 --quick-grasp-on-d，这样才能复用已验证的快抓流程。")
    if args.auto_cycle and not args.execute:
        raise RuntimeError("--auto-cycle 是真机自动循环模式，请同时传 --execute；如果只想预览，请先用手动 d 模式。")
    if args.auto_once and not args.quick_grasp_on_d:
        raise RuntimeError("--auto-once 需要同时传 --quick-grasp-on-d，这样才能复用已验证的快抓流程。")
    if args.auto_once and not args.execute:
        raise RuntimeError("--auto-once 是真机单次自动抓取模式，请同时传 --execute；如果只想预览，请先用手动 d 模式。")
    if not args.pixel_to_base_mm:
        raise RuntimeError(
            "缺少 pixel_to_base_mm。请在 config/skewer_align_config.json 写入，"
            "或命令行传 --pixel-to-base-mm=a,b,c,d"
        )
    run_dir = args.output_dir.expanduser().resolve() / datetime.now().strftime("align_once_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir = run_dir

    observe_replay = args.observe_replay.expanduser().resolve()
    if not observe_replay.exists():
        raise FileNotFoundError(observe_replay)
    if args.pre_replay is not None and not args.pre_replay.expanduser().resolve().exists():
        raise FileNotFoundError(args.pre_replay)
    if args.post_replay is not None and not args.post_replay.expanduser().resolve().exists():
        raise FileNotFoundError(args.post_replay)

    q_observe, gripper_from_csv = calib.read_last_replay_q(observe_replay)
    if args.gripper is None:
        args.gripper = gripper_from_csv
    place_q, ignored_place_gripper, place_source = resolve_place_q(args)
    if args.auto_once and args.after_grasp_action == "place" and place_q is None:
        raise RuntimeError("--auto-once --after-grasp-action place 需要放置位：请传 --place-snapshot、--place-q 或 --place-replay。")
    if place_q is not None:
        print(f"[place] loaded place pose from {place_source}: {','.join(f'{v:.8f}' for v in place_q)}")
        if ignored_place_gripper is not None:
            print(f"[place] snapshot/place gripper {ignored_place_gripper:+.4f} will be ignored; using grasped gripper until release.")
    T_observe = calib.solve_fk(q_observe, run_dir, args)
    save_json(run_dir / "T_base_ee_observe.json", {"T_base_ee": calib.matrix_to_list(T_observe)})
    if args.interactive:
        run_interactive(args, run_dir, q_observe, float(args.gripper), T_observe)
        return 0

    observe_replay = args.observe_replay.expanduser().resolve()
    center_sample = calib.PoseSample("000_observe", (0.0, 0.0), q_observe, float(args.gripper), observe_replay)

    server_proc = None
    try:
        if args.execute:
            print(f"[align-once] moving to observe using recorded observe replay: {observe_replay}")
            server_proc = calib.start_replay_server(observe_replay, args)
            calib.play_on_replay_server(server_proc, observe_replay, args, initial_start=True)

        with servo.Camera(args) as cam:
            before = detect_once(cam, center_sample, run_dir, args)
            if not before.get("detected"):
                raise RuntimeError("observe pose detection failed; not moving")
            err = np.asarray(before["error_px"], dtype=np.float64)
            raw_move, model_name = correction_from_error(err, args)
            limited_move, scale = limit_vector_mm(raw_move, float(args.max_move_mm))
            print(
                "[align-once] before err="
                f"({err[0]:+.1f},{err[1]:+.1f})px model={model_name} raw_move=({raw_move[0]:+.2f},{raw_move[1]:+.2f})mm "
                f"limited=({limited_move[0]:+.2f},{limited_move[1]:+.2f})mm scale={scale:.3f}"
            )
            save_json(
                run_dir / "align_once_plan.json",
                {
                    "before": before,
                    "raw_move_mm": raw_move.tolist(),
                    "limited_move_mm": limited_move.tolist(),
                    "limit_scale": scale,
                    "max_move_mm": args.max_move_mm,
                    "execute": bool(args.execute),
                },
            )
            if not args.execute:
                print("[align-once] dry run only. Add --execute to move.")
                return 0

            move_sample = calib.generate_sample_for_offset(
                name="001_align_move",
                offset_xy_mm=(float(limited_move[0]), float(limited_move[1])),
                q_start=q_observe,
                q_observe=q_observe,
                gripper=float(args.gripper),
                T_observe=T_observe,
                run_dir=run_dir,
                args=args,
            )
            calib.play_on_replay_server(server_proc, move_sample.replay_csv, args)
            after = detect_once(cam, move_sample, run_dir, args, previous_record=before)
            if after.get("detected"):
                e2 = np.asarray(after["error_px"], dtype=np.float64)
                print(f"[align-once] after err=({e2[0]:+.1f},{e2[1]:+.1f})px")
                print(f"[align-once] improvement norm: {np.linalg.norm(err):.1f}px -> {np.linalg.norm(e2):.1f}px")
            save_json(run_dir / "align_once_result.json", {"before": before, "after": after})
    finally:
        calib.stop_process(server_proc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
