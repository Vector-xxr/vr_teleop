#!/usr/bin/env python3
"""
Automatically collect pixel-vs-motion samples for top-down 2D skewer alignment.

This script does not grasp. It:

1. Reads q_observe from the last row of a right-arm replay CSV.
2. Uses the existing OpenArm KDL FK/IK tool to generate small base-frame
   horizontal offsets around q_observe.
3. In execute mode, opens a camera preview. Press d to detect the current pose;
   press m to move to the next calibration pose.
4. Estimates a 2x2 mapping from image error pixels to robot base XY correction.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import os
import select
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import ModuleType
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
VR_ROOT = ROOT.parent
VENDOR_ROOT = ROOT / "vendor"
OPENARM_VISION = (
    VENDOR_ROOT / "openarm_vision_grasp_yolo"
    if (VENDOR_ROOT / "openarm_vision_grasp_yolo").exists()
    else VR_ROOT / "openarm_vision_grasp_yolo"
)
OPENARM_INPLACE = (
    VENDOR_ROOT / "openarm_inplace"
    if (VENDOR_ROOT / "openarm_inplace").exists()
    else VR_ROOT / "openarm_inplace"
)
SOLVE_FK_IK = OPENARM_VISION / "scripts" / "solve_openarm_fk_ik.py"
DEFAULT_OBSERVE_REPLAY = ROOT / "outputs" / "right_observe_record_20260723_135046" / "real_follower_right_03.csv"
DEFAULT_ALIGN_CONFIG = ROOT / "config" / "skewer_align_config.json"


@dataclass(frozen=True)
class PoseSample:
    name: str
    delta_xy_mm: tuple[float, float]
    q: list[float]
    gripper: float
    replay_csv: Path
    target_json: Path | None = None


@dataclass
class SampleRecord:
    name: str
    payload: dict[str, Any]


def load_module(module_name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


servo = load_module("skewer_topdown_2d_servo", ROOT / "run_skewer_2d_servo.py")


def save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def apply_align_config_defaults(args: argparse.Namespace) -> None:
    config_path = args.align_config.expanduser().resolve() if args.align_config else None
    if config_path is None or not config_path.exists():
        return
    data = load_json(config_path)
    if not args.target_center_px and data.get("target_center_px"):
        value = data["target_center_px"]
        if isinstance(value, list):
            value = ",".join(str(v) for v in value)
        args.target_center_px = str(value)
    print(f"[calib] loaded align config defaults: {config_path}")


def update_align_config(mapping: dict[str, Any], args: argparse.Namespace) -> None:
    if not bool(args.update_align_config):
        return
    linear_rms = float(mapping.get("rms_fit_error_mm", float("inf")))
    quadratic_rms = mapping.get("quadratic_rms_fit_error_mm")
    best_rms = linear_rms if quadratic_rms is None else min(linear_rms, float(quadratic_rms))
    if best_rms > float(args.max_update_rms_mm):
        print(
            f"[calib][WARN] fitted RMS {best_rms:.2f}mm > --max-update-rms-mm {args.max_update_rms_mm:.2f}; "
            "NOT updating align config."
        )
        return
    config_path = args.align_config.expanduser().resolve()
    payload: dict[str, Any] = {}
    if config_path.exists():
        payload = load_json(config_path)
    payload["pixel_to_base_mm"] = np.asarray(mapping["pixel_to_base_xy_mm_matrix"], dtype=np.float64).reshape(-1).tolist()
    if mapping.get("quadratic_offset_from_error_coeff"):
        payload["pixel_to_base_model"] = "quadratic_offset_delta"
        payload["quadratic_offset_from_error_coeff"] = mapping["quadratic_offset_from_error_coeff"]
        payload["quadratic_feature_order"] = mapping["quadratic_feature_order"]
        payload["quadratic_rms_fit_error_mm"] = mapping["quadratic_rms_fit_error_mm"]
    if args.target_center_px:
        values = [float(v.strip()) for v in str(args.target_center_px).split(",") if v.strip()]
        if len(values) != 2:
            raise RuntimeError("--target-center-px must be u,v")
        payload["target_center_px"] = values
    else:
        payload.setdefault("target_center_px", [348.0, 364.0])
    payload["last_calibration_updated_at"] = datetime.now().isoformat(timespec="seconds")
    payload["last_calibration_rms_fit_error_mm"] = mapping.get("rms_fit_error_mm")
    payload["note"] = (
        "pixel_to_base_mm maps image error [du,dv] px to base XY correction [dx,dy] mm. "
        "target_center_px is the fixed gripper-center pixel."
    )
    save_json(config_path, payload)
    print(f"[calib] updated align config: {config_path}")


def read_last_replay_q(path: Path) -> tuple[list[float], float]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        last: dict[str, str] | None = None
        for row in reader:
            if row:
                last = row
    if last is None:
        raise RuntimeError(f"Replay CSV is empty: {path}")
    q = []
    for i in range(7):
        key = f"q{i}"
        if key not in last:
            raise RuntimeError(f"{path} missing column {key}")
        q.append(float(last[key]))
    gripper = float(last.get("q7", 0.0))
    return q, gripper


def write_plain_joints(path: Path, q: list[float]) -> None:
    if len(q) != 7:
        raise RuntimeError(f"Need 7 arm joints, got {len(q)}")
    save_json(path, {"arm": [float(v) for v in q]})


def matrix_to_list(T: np.ndarray) -> list[list[float]]:
    return [[float(T[r, c]) for c in range(4)] for r in range(4)]


def run_checked(command: list[str], *, cwd: Path | None = None) -> None:
    print("[run]", " ".join(str(x) for x in command))
    subprocess.run(command, cwd=str(cwd) if cwd else None, check=True)


def solve_fk(q: list[float], run_dir: Path, args: argparse.Namespace) -> np.ndarray:
    joints_path = run_dir / "observe_joints.json"
    write_plain_joints(joints_path, q)
    command = [
        sys.executable,
        str(SOLVE_FK_IK),
        "--joints",
        str(joints_path),
        "--joint-source",
        "plain",
        "--arm-side",
        "right_arm",
        "--output-dir",
        str(run_dir / "fk_observe"),
        "--max-iter",
        str(args.ik_max_iter),
    ]
    if args.auto_build_kdl:
        command.append("--auto-build")
    run_checked(command, cwd=OPENARM_VISION)
    data = load_json(run_dir / "fk_observe" / "T_base_ee_from_seed.json")
    return np.asarray(data["T_base_ee"], dtype=np.float64)


def solve_ik(target_T: np.ndarray, seed: list[float], run_dir: Path, name: str, args: argparse.Namespace) -> list[float]:
    target_path = run_dir / f"T_base_ee_{name}.json"
    save_json(
        target_path,
        {
            "T_base_ee": matrix_to_list(target_T),
            "note": "Target generated by skewer_topdown_2d automatic XY calibration.",
        },
    )
    seed_path = run_dir / f"seed_{name}.json"
    write_plain_joints(seed_path, seed)
    out_dir = run_dir / f"ik_{name}"
    replay_csv = run_dir / "generated_replays" / f"{name}.csv"
    command = [
        sys.executable,
        str(SOLVE_FK_IK),
        "--joints",
        str(seed_path),
        "--joint-source",
        "plain",
        "--arm-side",
        "right_arm",
        "--target",
        str(target_path),
        "--target-key",
        "T_base_ee",
        "--output-dir",
        str(out_dir),
        "--replay-csv",
        str(replay_csv),
        "--replay-duration",
        str(args.motion_duration),
        "--replay-sample-period",
        str(args.replay_sample_period),
        "--gripper",
        str(args.gripper),
        "--max-iter",
        str(args.ik_max_iter),
    ]
    if args.auto_build_kdl:
        command.append("--auto-build")
    run_checked(command, cwd=OPENARM_VISION)
    result = load_json(out_dir / "ik_solution_joints.json")
    return [float(v) for v in result["ik_solution_joints"]]


def write_replay_csv(path: Path, q_start: list[float], q_goal: list[float], gripper: float, args: argparse.Namespace) -> None:
    # Reuse the old project's writer so the CSV shape matches the OpenArm replay controller.
    if str(OPENARM_VISION) not in sys.path:
        sys.path.insert(0, str(OPENARM_VISION))
    from robot.openarm_adapter import write_single_arm_replay_csv  # noqa: WPS433

    write_single_arm_replay_csv(
        path,
        [q_start, q_goal],
        gripper=float(gripper),
        dt=float(args.motion_duration),
        sample_period=float(args.replay_sample_period),
    )


def generate_samples(q_observe: list[float], gripper: float, run_dir: Path, args: argparse.Namespace) -> list[PoseSample]:
    T_observe = solve_fk(q_observe, run_dir, args)
    save_json(
        run_dir / "T_base_ee_observe.json",
        {
            "T_base_ee": matrix_to_list(T_observe),
            "q_observe": q_observe,
            "gripper": gripper,
        },
    )

    step_m = float(args.step_mm) * 0.001
    plan: list[tuple[str, tuple[float, float]]] = [
        ("center", (0.0, 0.0)),
        ("x_plus", (float(args.step_mm), 0.0)),
        ("x_minus", (-float(args.step_mm), 0.0)),
        ("y_plus", (0.0, float(args.step_mm))),
        ("y_minus", (0.0, -float(args.step_mm))),
    ]
    samples: list[PoseSample] = []
    previous_q = q_observe
    for name, (dx_mm, dy_mm) in plan:
        if name == "center":
            q = list(q_observe)
            target_json = None
        else:
            target = T_observe.copy()
            target[0, 3] += dx_mm * 0.001
            target[1, 3] += dy_mm * 0.001
            q = solve_ik(target, q_observe, run_dir, name, args)
            target_json = run_dir / f"T_base_ee_{name}.json"
            max_delta = max(abs(a - b) for a, b in zip(q_observe, q))
            if max_delta > float(args.max_joint_delta_rad):
                raise RuntimeError(
                    f"{name} IK joint delta too large: {max_delta:.3f} rad > {args.max_joint_delta_rad}. "
                    "Reduce --step-mm or check observe pose."
                )
        replay_csv = run_dir / "calib_replays" / f"{name}.csv"
        write_replay_csv(replay_csv, previous_q, q, gripper, args)
        samples.append(PoseSample(name, (dx_mm, dy_mm), q, gripper, replay_csv, target_json))
        previous_q = q
    # Return to center at the end when executing.
    return_csv = run_dir / "calib_replays" / "return_center.csv"
    write_replay_csv(return_csv, previous_q, q_observe, gripper, args)
    save_json(
        run_dir / "calibration_motion_plan.json",
        {
            "step_mm": args.step_mm,
            "samples": [
                {
                    "name": s.name,
                    "delta_xy_mm": list(s.delta_xy_mm),
                    "q": s.q,
                    "replay_csv": str(s.replay_csv),
                    "target_json": str(s.target_json) if s.target_json else None,
                }
                for s in samples
            ],
            "return_center_replay": str(return_csv),
        },
    )
    return samples


def load_observe_transform(run_dir: Path) -> np.ndarray:
    data = load_json(run_dir / "T_base_ee_observe.json")
    return np.asarray(data["T_base_ee"], dtype=np.float64)


def generate_sample_for_offset(
    *,
    name: str,
    offset_xy_mm: tuple[float, float],
    q_start: list[float],
    q_observe: list[float],
    gripper: float,
    T_observe: np.ndarray,
    run_dir: Path,
    args: argparse.Namespace,
) -> PoseSample:
    dx_mm, dy_mm = offset_xy_mm
    if abs(dx_mm) < 1.0e-9 and abs(dy_mm) < 1.0e-9:
        q_goal = list(q_observe)
        target_json = None
    else:
        target = T_observe.copy()
        target[0, 3] += dx_mm * 0.001
        target[1, 3] += dy_mm * 0.001
        q_goal = solve_ik(target, q_observe, run_dir, name, args)
        target_json = run_dir / f"T_base_ee_{name}.json"
        max_delta = max(abs(a - b) for a, b in zip(q_start, q_goal))
        if max_delta > float(args.max_step_joint_delta_rad):
            raise RuntimeError(
                f"{name} step joint delta too large: {max_delta:.3f} rad > {args.max_step_joint_delta_rad}. "
                "Reduce --step-mm or use shorter repeated moves."
            )
    replay_csv = run_dir / "jog_replays" / f"{name}.csv"
    write_replay_csv(replay_csv, q_start, q_goal, gripper, args)
    return PoseSample(name, offset_xy_mm, q_goal, gripper, replay_csv, target_json)


def replay_and_wait(csv_path: Path, args: argparse.Namespace) -> subprocess.Popen:
    script = OPENARM_INPLACE / "script" / "run_cpp_real_replay.sh"
    command = [
        str(script),
        str(csv_path.resolve()),
        "--no-auto-home",
        "--initial-transition-duration",
        str(args.initial_transition_duration),
    ]
    env = {
        **os.environ,
        "OPENARM_TELEOP_ARM_SIDE": "right_arm",
        "OPENARM_FOLLOWER_CAN": args.follower_can,
        "OPENARM_FOLLOWER_ONLY": "1" if args.follower_only else "0",
    }
    if args.sudo_password:
        env["OPENARM_SUDO_PASSWORD"] = args.sudo_password
    print(f"[motion] START replay {csv_path}", flush=True)
    print(
        f"[motion] follower_can={args.follower_can} follower_only={args.follower_only} "
        f"duration={args.motion_duration}s initial_transition={args.initial_transition_duration}s",
        flush=True,
    )
    proc = subprocess.Popen(command, cwd=str(OPENARM_INPLACE), env=env)
    wait_s = float(args.initial_transition_duration) + float(args.motion_duration) + float(args.settle_s)
    time.sleep(wait_s)
    if proc.poll() is not None:
        if proc.returncode not in (0, -15):
            raise RuntimeError(f"Replay failed early: {csv_path} code={proc.returncode}")
        print(f"[motion] replay process exited code={proc.returncode}", flush=True)
    else:
        print("[motion] replay expected motion time elapsed; process is holding, will stop after this step", flush=True)
    return proc


def send_udp(port: int, message: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(message.encode("utf-8"), ("127.0.0.1", int(port)))
    finally:
        sock.close()


def build_replay_command(csv_path: Path, args: argparse.Namespace, *, label: str = "motion", side: str | None = None) -> str:
    if str(getattr(args, "replay_command_format", "single")) == "dual":
        replay_side = side or str(getattr(args, "replay_command_side", "right"))
        telemetry_port = int(getattr(args, "replay_telemetry_port", 0))
        replay_speed = float(getattr(args, "replay_speed", 1.0))
        return f"play|{csv_path.resolve()}|{telemetry_port}|{label}|{replay_side}|{replay_speed}"
    return f"play|{csv_path.resolve()}"


def udp_port_available(port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", int(port)))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def choose_replay_command_port(args: argparse.Namespace) -> int:
    requested = int(args.replay_command_port)
    if not bool(getattr(args, "auto_replay_command_port", True)):
        return requested
    scan_count = max(1, int(getattr(args, "replay_command_port_scan", 50)))
    for port in range(requested, requested + scan_count):
        if udp_port_available(port):
            if port != requested:
                print(
                    f"[motion-server][WARN] UDP port {requested} is busy; using free port {port} instead.",
                    flush=True,
                )
            args.replay_command_port = port
            return port
    raise RuntimeError(
        f"No free UDP replay command port found in [{requested}, {requested + scan_count - 1}]. "
        "Close old replay/record processes or pass --replay-command-port to a free port."
    )


def start_replay_server(initial_csv: Path, args: argparse.Namespace) -> subprocess.Popen:
    script = OPENARM_INPLACE / "script" / "run_cpp_real_replay.sh"
    choose_replay_command_port(args)
    command = [
        str(script),
        str(initial_csv.resolve()),
        "--no-auto-home",
        "--initial-transition-duration",
        str(args.initial_transition_duration),
        "--replay-start-port",
        str(args.replay_command_port),
        "--keep-enabled-on-exit",
    ]
    env = {
        **os.environ,
        "OPENARM_TELEOP_ARM_SIDE": "right_arm",
        "OPENARM_FOLLOWER_CAN": args.follower_can,
        "OPENARM_FOLLOWER_ONLY": "1" if args.follower_only else "0",
    }
    if args.sudo_password:
        env["OPENARM_SUDO_PASSWORD"] = args.sudo_password
    print("[motion-server] START one persistent OpenArm replay process", flush=True)
    print("[motion-server]", " ".join(str(x) for x in command), flush=True)
    proc = subprocess.Popen(command, cwd=str(OPENARM_INPLACE), env=env)
    time.sleep(float(args.server_startup_s))
    if proc.poll() is not None:
        raise RuntimeError(f"OpenArm replay server exited early with code {proc.returncode}")
    return proc


def play_on_replay_server(
    proc: subprocess.Popen,
    csv_path: Path,
    args: argparse.Namespace,
    *,
    initial_start: bool = False,
    label: str = "motion",
    side: str | None = None,
) -> None:
    if proc.poll() is not None:
        raise RuntimeError(f"OpenArm replay server is not running, code={proc.returncode}")
    if initial_start:
        print(f"[motion-server] UDP start center on port {args.replay_command_port}", flush=True)
        send_udp(args.replay_command_port, "start")
        wait_s = float(args.initial_transition_duration) + float(args.motion_duration) + float(args.settle_s)
    else:
        message = build_replay_command(csv_path, args, label=label, side=side)
        print(f"[motion-server] UDP {message} -> port {args.replay_command_port}", flush=True)
        send_udp(args.replay_command_port, message)
        wait_s = float(args.motion_duration) + float(args.settle_s)
    time.sleep(wait_s)
    if proc.poll() is not None:
        raise RuntimeError(f"OpenArm replay server exited during motion, code={proc.returncode}")
    print("[motion-server] motion time elapsed; same process is still holding", flush=True)


def stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    print("[motion] stopping replay hold process", flush=True)
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)


def make_tracking_args(
    args: argparse.Namespace,
    image_bgr: np.ndarray,
    previous_record: dict[str, Any] | None,
) -> argparse.Namespace:
    if not getattr(args, "track_target", True):
        return args
    if not previous_record or not previous_record.get("detected"):
        return args
    if "center_px" not in previous_record:
        return args
    h, w = image_bgr.shape[:2]
    u, v = [float(x) for x in previous_record["center_px"]]
    radius = max(20.0, float(args.track_roi_px))
    tracked = argparse.Namespace(**vars(args))
    tracked.roi_left = max(0.0, (u - radius) / float(w))
    tracked.roi_right = min(1.0, (u + radius) / float(w))
    tracked.roi_top = max(0.0, (v - radius) / float(h))
    tracked.roi_bottom = min(1.0, (v + radius) / float(h))
    tracked.target_selection = "center"
    return tracked


def capture_detection(
    cam: Any,
    sample: PoseSample,
    run_dir: Path,
    args: argparse.Namespace,
    previous_record: dict[str, Any] | None = None,
    skip_centers_px: list[tuple[float, float]] | None = None,
    candidate_index: int = 0,
) -> dict[str, Any]:
    frame = None
    # USB/OpenCV cameras may keep buffered frames from before the robot motion.
    # Always drain a few frames after reaching a new pose, then detect on the
    # newest frame we can read.
    for _ in range(max(0, int(args.capture_flush_frames))):
        flushed = cam.read_color()
        if flushed is not None:
            frame = flushed
        time.sleep(max(0.0, float(args.capture_flush_dt_s)))
    for _ in range(max(1, int(args.capture_attempts))):
        frame = cam.read_color()
        if frame is not None:
            if not bool(args.capture_use_latest):
                break
        time.sleep(0.05)
    if frame is None:
        raise RuntimeError("Camera did not return a frame")
    detect_args = make_tracking_args(args, frame, previous_record)
    if skip_centers_px:
        detect_args = argparse.Namespace(**vars(detect_args))
        detect_args.target_skip_centers_px = [(float(u), float(v)) for u, v in skip_centers_px]
        detect_args.target_skip_radius_px = float(getattr(args, "multi_target_skip_radius_px", 55.0))
    if int(candidate_index) > 0:
        detect_args = argparse.Namespace(**vars(detect_args))
        detect_args.target_candidate_index = int(candidate_index)
    det = servo.detect_black_region(frame, detect_args)
    if det is None and detect_args is not args and bool(getattr(args, "track_fallback_to_full", True)):
        print(f"[detect][WARN] tracking ROI missed {sample.name}; fallback to full ROI once")
        detect_args = args
        det = servo.detect_black_region(frame, detect_args)
    vis = servo.draw_detection(frame, det, detect_args)
    sample_dir = run_dir / "detections" / sample.name
    servo.save_detection(sample_dir, frame, vis, det, detect_args)
    if det is None:
        return {
            "name": sample.name,
            "detected": False,
            "delta_xy_mm": list(sample.delta_xy_mm),
            "detection_dir": str(sample_dir),
            "reason": "no skewer black region detected",
            "tracking_roi": {
                "roi_left": detect_args.roi_left,
                "roi_right": detect_args.roi_right,
                "roi_top": detect_args.roi_top,
                "roi_bottom": detect_args.roi_bottom,
            },
            "skip_centers_px": [list(p) for p in (skip_centers_px or [])],
            "target_candidate_index": int(candidate_index),
        }
    return {
        "name": sample.name,
        "detected": True,
        "delta_xy_mm": list(sample.delta_xy_mm),
        "center_px": list(det.center_px),
        "image_center_px": list(det.image_center_px),
        "error_px": list(det.error_px),
        "area_px": det.area_px,
        "bbox_xywh": list(det.bbox_xywh),
        "angle_deg": det.angle_deg,
        "elongation": det.elongation,
        "angle_from_vertical_deg": det.angle_from_vertical_deg,
        "detection_dir": str(sample_dir),
        "tracking_roi": {
            "roi_left": detect_args.roi_left,
            "roi_right": detect_args.roi_right,
            "roi_top": detect_args.roi_top,
            "roi_bottom": detect_args.roi_bottom,
        },
        "skip_centers_px": [list(p) for p in (skip_centers_px or [])],
        "target_candidate_index": int(candidate_index),
    }


def same_offset(a: list[float] | tuple[float, float], b: np.ndarray | tuple[float, float], tol_mm: float = 0.25) -> bool:
    return abs(float(a[0]) - float(b[0])) <= tol_mm and abs(float(a[1]) - float(b[1])) <= tol_mm


def sampled_centers_at_offset(records: list[dict[str, Any]], current_offset: np.ndarray) -> list[tuple[float, float]]:
    centers: list[tuple[float, float]] = []
    for record in records:
        if not record.get("detected"):
            continue
        if not same_offset(record.get("delta_xy_mm", [999999.0, 999999.0]), current_offset):
            continue
        center = record.get("center_px")
        if not center or len(center) != 2:
            continue
        centers.append((float(center[0]), float(center[1])))
    return centers


def draw_record_marker(frame: np.ndarray, record: dict[str, Any] | None, args: argparse.Namespace) -> np.ndarray:
    out = frame.copy()
    if not record or not record.get("detected"):
        return out
    h, w = out.shape[:2]
    target_u, target_v = servo.target_center_px(args, w, h)
    u, v = [float(x) for x in record["center_px"]]
    x, y, bw, bh = [int(round(float(x))) for x in record.get("bbox_xywh", [u - 8.0, v - 8.0, 16.0, 16.0])]
    cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
    cv2.drawMarker(out, (int(round(u)), int(round(v))), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=28, thickness=2)
    cv2.drawMarker(
        out,
        (int(round(target_u)), int(round(target_v))),
        (80, 80, 80),
        markerType=cv2.MARKER_CROSS,
        markerSize=28,
        thickness=1,
    )
    cv2.line(out, (int(round(target_u)), int(round(target_v))), (int(round(u)), int(round(v))), (0, 255, 255), 2)
    du, dv = [float(x) for x in record["error_px"]]
    candidate_index = int(record.get("target_candidate_index", 0) or 0)
    label = f"saved candidate #{candidate_index}" if candidate_index > 0 else "saved target"
    text = f"{label} center=({u:.1f},{v:.1f}) err=({du:+.1f},{dv:+.1f})px"
    cv2.putText(out, text, (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, lineType=cv2.LINE_AA)
    return out


def estimate_mapping(records: list[dict[str, Any]]) -> dict[str, Any]:
    valid_records = [r for r in records if r.get("detected")]
    if len(valid_records) < 3:
        raise RuntimeError("Need at least three detected samples to estimate affine pixel-to-motion mapping")
    unique_offsets = {
        (round(float(r["delta_xy_mm"][0]), 1), round(float(r["delta_xy_mm"][1]), 1))
        for r in valid_records
    }
    if len(unique_offsets) < 4:
        raise RuntimeError(
            f"Need samples from at least four different robot offsets, got {len(unique_offsets)}. "
            "Press w/s/d/a/c to move, then press 1/2/3 at each position."
        )
    errors = []
    offsets = []
    for r in valid_records:
        errors.append([float(r["error_px"][0]), float(r["error_px"][1]), 1.0])
        offsets.append([float(r["delta_xy_mm"][0]), float(r["delta_xy_mm"][1])])
    X = np.asarray(errors, dtype=np.float64)
    Y = np.asarray(offsets, dtype=np.float64)
    coeff, residuals, rank, _ = np.linalg.lstsq(X, Y, rcond=None)
    # Fit: absolute_offset_xy_mm = err_px @ A + b.
    # For visual servoing from the current pose, we need the incremental move
    # that drives err toward zero: correction_xy_mm = -A.T @ err_px.
    offset_from_error = coeff[:2, :].T
    correction_from_error = -offset_from_error
    intercept = coeff[2, :]
    predicted = X @ coeff
    residual_xy = Y - predicted
    rms_mm = float(np.sqrt(np.mean(np.sum(residual_xy**2, axis=1))))
    quadratic_payload: dict[str, Any] = {}
    if len(valid_records) >= 6:
        Xq = []
        for r in valid_records:
            du = float(r["error_px"][0])
            dv = float(r["error_px"][1])
            Xq.append([du, dv, du * du, dv * dv, du * dv, 1.0])
        Xq_arr = np.asarray(Xq, dtype=np.float64)
        coeff_q, residuals_q, rank_q, _ = np.linalg.lstsq(Xq_arr, Y, rcond=None)
        pred_q = Xq_arr @ coeff_q
        residual_q = Y - pred_q
        quadratic_payload = {
            "quadratic_feature_order": ["du", "dv", "du2", "dv2", "du_dv", "bias"],
            "quadratic_offset_from_error_coeff": coeff_q.tolist(),
            "quadratic_rank": int(rank_q),
            "quadratic_residuals": residuals_q.tolist(),
            "quadratic_rms_fit_error_mm": float(np.sqrt(np.mean(np.sum(residual_q**2, axis=1)))),
            "quadratic_note": (
                "coeff maps feature row [du,dv,du^2,dv^2,du*dv,1] to absolute "
                "base offset [x,y] mm. Runtime correction is offset_at_zero - offset_at_current_error."
            ),
        }
    return {
        "pixel_to_base_xy_mm_matrix": correction_from_error.tolist(),
        "pixel_to_base_xy_mm_arg": ",".join(f"{v:.8g}" for v in correction_from_error.reshape(-1)),
        "affine_offset_from_error_matrix": offset_from_error.tolist(),
        "affine_offset_intercept_mm": intercept.tolist(),
        "rank": int(rank),
        "residuals": residuals.tolist(),
        "rms_fit_error_mm": rms_mm,
        "used_sample_names": [r["name"] for r in valid_records],
        "note": (
            "pixel_to_base_xy_mm_matrix maps current image error [du,dv]px to an incremental "
            "base XY correction [dx,dy]mm. Use --pixel-to-base-mm a,b,c,d after verifying "
            "the fitted RMS error and motion sign."
        ),
        **quadratic_payload,
    }


def execute_collection(samples: list[PoseSample], run_dir: Path, args: argparse.Namespace) -> None:
    records: list[dict[str, Any]] = []
    with servo.Camera(args) as cam:
        for sample in samples:
            proc = None
            try:
                proc = replay_and_wait(sample.replay_csv, args)
                record = capture_detection(cam, sample, run_dir, args)
                records.append(record)
                print(
                    f"[sample] {sample.name}: delta_xy_mm={record['delta_xy_mm']} "
                    f"err_px=({record['error_px'][0]:+.1f},{record['error_px'][1]:+.1f})"
                )
            finally:
                stop_process(proc)
        if args.return_center:
            return_csv = run_dir / "calib_replays" / "return_center.csv"
            proc = None
            try:
                proc = replay_and_wait(return_csv, args)
            finally:
                stop_process(proc)
    mapping = estimate_mapping(records)
    save_json(
        run_dir / "pixel_to_base_xy_calibration.json",
        {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "step_mm": args.step_mm,
            "records": records,
            **mapping,
        },
    )
    print("[calib] wrote", run_dir / "pixel_to_base_xy_calibration.json")
    print(f"[calib] RMS fit error: {mapping['rms_fit_error_mm']:.2f} mm")
    print("[calib] use this after verification:")
    print("  --pixel-to-base-mm", mapping["pixel_to_base_xy_mm_arg"])
    update_align_config(mapping, args)


def draw_manual_preview(frame: np.ndarray, sample: PoseSample, record: dict[str, Any] | None, args: argparse.Namespace) -> np.ndarray:
    det = None
    if record is not None and record.get("detected"):
        det = servo.detect_black_region(frame, args)
    out = servo.draw_detection(frame, det, args) if det is not None else frame.copy()
    h, w = out.shape[:2]
    color = (0, 255, 0)
    status = "detected" if record and record.get("detected") else "not detected"
    if record is None:
        status = "not sampled"
    text1 = f"sample={sample.name} delta=({sample.delta_xy_mm[0]:+.1f},{sample.delta_xy_mm[1]:+.1f})mm {status}"
    text2 = "d:detect/save  m:move next  f:fit if enough  q:quit"
    cv2.putText(out, text1, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, lineType=cv2.LINE_AA)
    cv2.putText(out, text2, (18, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, lineType=cv2.LINE_AA)
    return out


def save_records_and_maybe_fit(records: list[dict[str, Any]], run_dir: Path, args: argparse.Namespace, *, final: bool) -> None:
    save_json(
        run_dir / "pixel_to_base_xy_samples.json",
        {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "step_mm": args.step_mm,
            "records": records,
            "note": "Manual collection: press d to detect/save, m to move. Missing samples are allowed.",
        },
    )
    try:
        mapping = estimate_mapping(records)
    except RuntimeError as exc:
        if final:
            print(f"[calib][WARN] mapping not estimated: {exc}")
        return
    save_json(
        run_dir / "pixel_to_base_xy_calibration.json",
        {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "step_mm": args.step_mm,
            "records": records,
            **mapping,
        },
    )
    print("[calib] wrote", run_dir / "pixel_to_base_xy_calibration.json")
    print("[calib] use this after verification:")
    print("  --pixel-to-base-mm", mapping["pixel_to_base_xy_mm_arg"])
    update_align_config(mapping, args)


def poll_terminal_key() -> str:
    if not sys.stdin.isatty():
        return ""
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    except (OSError, ValueError):
        return ""
    if not ready:
        return ""
    text = sys.stdin.readline().strip().lower()
    return text[:1] if text else ""


def execute_collection_manual(samples: list[PoseSample], run_dir: Path, args: argparse.Namespace) -> None:
    records_by_name: dict[str, dict[str, Any]] = {}
    index = 0
    print("[manual] Camera window keys:")
    print("  d  detect and save current sample")
    print("  m  move to next sample only after you confirm the workspace is clear")
    print("  f  fit calibration from currently detected samples")
    print("  q  quit")
    print("[manual] Terminal also works: type d/m/f/q then press Enter.")
    print("[manual] IMPORTANT: keep your hand out of the camera view before pressing d.")
    server_proc = None
    try:
        server_proc = start_replay_server(samples[0].replay_csv, args)
        if args.initial_center_move:
            print("[manual] moving to center/observe first using the persistent replay process.")
            play_on_replay_server(server_proc, samples[0].replay_csv, args, initial_start=True)
            print("[manual] arrived at center. Now press d after the image is clear.")
        with servo.Camera(args) as cam:
            cv2.namedWindow("Skewer XY Manual Calibration", cv2.WINDOW_NORMAL)
            last_frame = None
            while True:
                sample = samples[min(index, len(samples) - 1)]
                frame = cam.read_color()
                if frame is not None:
                    last_frame = frame
                if last_frame is not None:
                    display = draw_manual_preview(last_frame, sample, records_by_name.get(sample.name), args)
                    max_w = max(240, int(args.preview_max_width))
                    if display.shape[1] > max_w:
                        scale = max_w / float(display.shape[1])
                        display = cv2.resize(display, (max_w, int(round(display.shape[0] * scale))), interpolation=cv2.INTER_AREA)
                    cv2.imshow("Skewer XY Manual Calibration", display)
                key = cv2.waitKey(1) & 0xFF
                terminal_key = poll_terminal_key()
                if terminal_key:
                    key = ord(terminal_key)

                if key in (ord("d"), ord("D")):
                    if last_frame is None:
                        print("[manual][WARN] no camera frame yet")
                        continue
                    record = capture_detection(cam, sample, run_dir, args)
                    records_by_name[sample.name] = record
                    if record.get("detected"):
                        du, dv = record["error_px"]
                        print(f"[manual] {sample.name} detected err=({du:+.1f},{dv:+.1f})px")
                    else:
                        print(f"[manual][WARN] {sample.name} no detection. Move your hand away or adjust threshold, then press d again.")
                    save_records_and_maybe_fit(list(records_by_name.values()), run_dir, args, final=False)

                if key in (ord("m"), ord("M")):
                    if index >= len(samples) - 1:
                        print("[manual] last sample reached. Moving back to center in the same replay process.")
                        return_csv = run_dir / "calib_replays" / "return_center.csv"
                        play_on_replay_server(server_proc, return_csv, args)
                        break
                    next_sample = samples[index + 1]
                    print(f"[manual] moving to {next_sample.name}; clear workspace now.")
                    play_on_replay_server(server_proc, next_sample.replay_csv, args)
                    index += 1
                    print(f"[manual] arrived at {next_sample.name}. Press d only after the image is clear.")

                if key in (ord("f"), ord("F")):
                    save_records_and_maybe_fit(list(records_by_name.values()), run_dir, args, final=True)

                if key in (ord("q"), 27):
                    print("[manual] quit requested. Stopping the single replay process once.")
                    break
            cv2.destroyAllWindows()
    finally:
        stop_process(server_proc)
    save_records_and_maybe_fit(list(records_by_name.values()), run_dir, args, final=True)


def draw_jog_preview(
    frame: np.ndarray,
    current_offset_xy_mm: tuple[float, float],
    last_record: dict[str, Any] | None,
    sample_count: int,
    args: argparse.Namespace,
) -> np.ndarray:
    out = draw_record_marker(frame, last_record, args)
    h, _w = out.shape[:2]
    color = (0, 255, 0)
    status = "no sample"
    if last_record is not None:
        status = "detected" if last_record.get("detected") else "not detected"
    text1 = (
        f"offset=({current_offset_xy_mm[0]:+.1f},{current_offset_xy_mm[1]:+.1f})mm "
        f"samples={sample_count} last={status}"
    )
    text2 = "w/s/d/a/c:move only  1/2/3:save candidate  f:fit q:quit"
    cv2.putText(out, text1, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, lineType=cv2.LINE_AA)
    cv2.putText(out, text2, (18, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, lineType=cv2.LINE_AA)
    return out


def execute_collection_jog(samples: list[PoseSample], run_dir: Path, args: argparse.Namespace) -> None:
    records: list[dict[str, Any]] = []
    q_observe = list(samples[0].q)
    current_q = list(q_observe)
    gripper = float(samples[0].gripper)
    T_observe = load_observe_transform(run_dir)
    current_offset = np.asarray([0.0, 0.0], dtype=np.float64)
    sequence = 0
    last_record: dict[str, Any] | None = None
    key_to_delta = {
        "w": np.asarray([float(args.step_mm), 0.0], dtype=np.float64),
        "s": np.asarray([-float(args.step_mm), 0.0], dtype=np.float64),
        "d": np.asarray([0.0, float(args.step_mm)], dtype=np.float64),
        "a": np.asarray([0.0, -float(args.step_mm)], dtype=np.float64),
    }

    print("[jog] One persistent replay process; move keys only move by default, then press 1/2/3 to save a candidate.")
    print("[jog] Keys in camera window or terminal + Enter:")
    print("  w  move base +X by step-mm")
    print("  s  move base -X by step-mm")
    print("  d  move base +Y by step-mm")
    print("  a  move base -Y by step-mm")
    print("  c  return center")
    print("  1  save candidate #1 at current robot offset")
    print("  2  save candidate #2 at current robot offset")
    print("  3  save candidate #3 at current robot offset")
    print("  m  save another target at the current robot offset without moving; auto skips targets already sampled here")
    print("  r  reset target lock only; sampled targets at this offset are still skipped by m")
    print("  f  fit calibration now")
    print("  q  quit and fit")
    print("[jog] IMPORTANT: keep your hand out of the camera view before pressing a move key.")

    server_proc = None
    try:
        server_proc = start_replay_server(samples[0].replay_csv, args)
        if args.initial_center_move:
            print("[jog] moving to center/observe first.")
            play_on_replay_server(server_proc, samples[0].replay_csv, args, initial_start=True)
        with servo.Camera(args) as cam:
            cv2.namedWindow("Skewer XY Jog Calibration", cv2.WINDOW_NORMAL)
            last_frame = None
            while True:
                frame = cam.read_color()
                if frame is not None:
                    last_frame = frame
                if last_frame is not None:
                    display = draw_jog_preview(
                        last_frame,
                        (float(current_offset[0]), float(current_offset[1])),
                        last_record,
                        len([r for r in records if r.get("detected")]),
                        args,
                    )
                    max_w = max(240, int(args.preview_max_width))
                    if display.shape[1] > max_w:
                        scale = max_w / float(display.shape[1])
                        display = cv2.resize(display, (max_w, int(round(display.shape[0] * scale))), interpolation=cv2.INTER_AREA)
                    cv2.imshow("Skewer XY Jog Calibration", display)
                key = cv2.waitKey(1) & 0xFF
                terminal_key = poll_terminal_key()
                if terminal_key:
                    key = ord(terminal_key)
                if key == 255:
                    continue

                char = chr(key).lower() if 0 <= key < 256 else ""
                if char in key_to_delta or char == "c":
                    if char == "c":
                        next_offset = np.asarray([0.0, 0.0], dtype=np.float64)
                        move_name = f"{sequence:03d}_center"
                    else:
                        next_offset = current_offset + key_to_delta[char]
                        axis_name = {"w": "x_plus", "s": "x_minus", "d": "y_plus", "a": "y_minus"}[char]
                        move_name = f"{sequence:03d}_{axis_name}_{next_offset[0]:+.0f}_{next_offset[1]:+.0f}".replace("+", "p").replace("-", "m")
                    if float(np.linalg.norm(next_offset, ord=np.inf)) > float(args.max_offset_mm):
                        print(f"[jog][WARN] requested offset {next_offset.tolist()} exceeds --max-offset-mm {args.max_offset_mm}")
                        continue
                    sample = generate_sample_for_offset(
                        name=move_name,
                        offset_xy_mm=(float(next_offset[0]), float(next_offset[1])),
                        q_start=current_q,
                        q_observe=q_observe,
                        gripper=gripper,
                        T_observe=T_observe,
                        run_dir=run_dir,
                        args=args,
                    )
                    print(f"[jog] moving {move_name} to offset=({next_offset[0]:+.1f},{next_offset[1]:+.1f})mm")
                    play_on_replay_server(server_proc, sample.replay_csv, args)
                    current_q = list(sample.q)
                    current_offset = next_offset
                    sequence += 1
                    if bool(args.auto_capture_after_move):
                        last_record = capture_detection(cam, sample, run_dir, args, previous_record=last_record)
                        records.append(last_record)
                        if last_record.get("detected"):
                            du, dv = last_record["error_px"]
                            print(f"[jog] saved {move_name}: err=({du:+.1f},{dv:+.1f})px")
                        else:
                            print(f"[jog][WARN] saved {move_name}: no detection")
                        save_records_and_maybe_fit(records, run_dir, args, final=False)
                    else:
                        last_record = None
                        print("[jog] arrived. Press 1/2/3 to save a specific skewer candidate at this offset.")

                if char in ("1", "2", "3", "4", "5", "6", "7", "8", "9"):
                    candidate_index = int(char)
                    sample_name = (
                        f"{sequence:03d}_candidate{candidate_index}_current_"
                        f"{current_offset[0]:+.0f}_{current_offset[1]:+.0f}"
                    ).replace("+", "p").replace("-", "m")
                    sample = PoseSample(
                        sample_name,
                        (float(current_offset[0]), float(current_offset[1])),
                        list(current_q),
                        gripper,
                        run_dir / "unused_current.csv",
                    )
                    print(
                        f"[jog] saving candidate #{candidate_index} at "
                        f"offset=({current_offset[0]:+.1f},{current_offset[1]:+.1f})mm"
                    )
                    last_record = capture_detection(
                        cam,
                        sample,
                        run_dir,
                        args,
                        previous_record=None,
                        candidate_index=candidate_index,
                    )
                    records.append(last_record)
                    sequence += 1
                    if last_record.get("detected"):
                        du, dv = last_record["error_px"]
                        center = last_record["center_px"]
                        print(
                            f"[jog] saved candidate #{candidate_index}: "
                            f"center=({center[0]:.1f},{center[1]:.1f}) err=({du:+.1f},{dv:+.1f})px"
                        )
                    else:
                        print(f"[jog][WARN] candidate #{candidate_index}: no detection")
                    save_records_and_maybe_fit(records, run_dir, args, final=False)

                if char == "m":
                    sample_name = f"{sequence:03d}_multi_current_{current_offset[0]:+.0f}_{current_offset[1]:+.0f}".replace("+", "p").replace("-", "m")
                    sample = PoseSample(
                        sample_name,
                        (float(current_offset[0]), float(current_offset[1])),
                        list(current_q),
                        gripper,
                        run_dir / "unused_current.csv",
                    )
                    skip_centers = sampled_centers_at_offset(records, current_offset)
                    if skip_centers:
                        print(
                            f"[jog] current-position multi sample: full-image detection, "
                            f"skipping {len(skip_centers)} target(s) already sampled at this offset"
                        )
                    else:
                        print("[jog] current-position multi sample: full-image detection")
                    last_record = capture_detection(
                        cam,
                        sample,
                        run_dir,
                        args,
                        previous_record=None,
                        skip_centers_px=skip_centers,
                    )
                    records.append(last_record)
                    sequence += 1
                    if last_record.get("detected"):
                        du, dv = last_record["error_px"]
                        print(
                            f"[jog] saved {sample_name}: offset=({current_offset[0]:+.1f},{current_offset[1]:+.1f})mm "
                            f"err=({du:+.1f},{dv:+.1f})px"
                        )
                    else:
                        print(f"[jog][WARN] saved {sample_name}: no detection")
                    save_records_and_maybe_fit(records, run_dir, args, final=False)

                if char == "r":
                    last_record = None
                    print("[jog] target lock reset. Next m uses full image and still skips targets already sampled at this offset.")

                if char == "f":
                    save_records_and_maybe_fit(records, run_dir, args, final=True)

                if char == "q" or key == 27:
                    print("[jog] quit requested.")
                    if args.return_center and float(np.linalg.norm(current_offset)) > 1.0e-9:
                        sample = generate_sample_for_offset(
                            name=f"{sequence:03d}_return_center",
                            offset_xy_mm=(0.0, 0.0),
                            q_start=current_q,
                            q_observe=q_observe,
                            gripper=gripper,
                            T_observe=T_observe,
                            run_dir=run_dir,
                            args=args,
                        )
                        play_on_replay_server(server_proc, sample.replay_csv, args)
                    break
            cv2.destroyAllWindows()
    finally:
        stop_process(server_proc)
    save_records_and_maybe_fit(records, run_dir, args, final=True)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Auto collect 2D pixel-to-OpenArm-base XY calibration samples.")
    parser.add_argument("--observe-replay", type=Path, default=DEFAULT_OBSERVE_REPLAY)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "auto_xy_calib")
    parser.add_argument("--step-mm", type=float, default=10.0, help="Base-frame +/- X/Y probe distance.")
    parser.add_argument("--execute", action="store_true", help="Actually move robot and capture detections.")
    parser.add_argument(
        "--auto-run",
        action="store_true",
        help="Old unattended mode: automatically move/detect every sample. Manual d/m mode is the default and safer.",
    )
    parser.add_argument(
        "--manual-sequence",
        action="store_true",
        help="Use old d/m fixed sequence mode instead of jog mode.",
    )
    parser.add_argument("--auto-build-kdl", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ik-max-iter", type=int, default=700)
    parser.add_argument("--max-joint-delta-rad", type=float, default=0.45)
    parser.add_argument("--max-step-joint-delta-rad", type=float, default=0.45)
    parser.add_argument("--max-offset-mm", type=float, default=40.0)
    parser.add_argument("--motion-duration", type=float, default=3.0)
    parser.add_argument("--initial-transition-duration", type=float, default=2.0)
    parser.add_argument("--replay-sample-period", type=float, default=0.01)
    parser.add_argument("--settle-s", type=float, default=1.2)
    parser.add_argument("--replay-command-port", type=int, default=47920, help="UDP port for the one persistent replay process.")
    parser.add_argument(
        "--auto-replay-command-port",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="If --replay-command-port is busy, automatically use the next free UDP port.",
    )
    parser.add_argument("--replay-command-port-scan", type=int, default=50)
    parser.add_argument("--server-startup-s", type=float, default=1.0)
    parser.add_argument(
        "--initial-center-move",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="In manual execute mode, first replay center/observe pose so robot communication is explicit.",
    )
    parser.add_argument("--return-center", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--auto-capture-after-move",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="If enabled, jog move keys also auto-save one target. Default off: move first, then press 1/2/3.",
    )
    parser.add_argument("--follower-can", default="can1")
    parser.add_argument("--follower-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sudo-password", default="")
    parser.add_argument("--gripper", type=float, default=None, help="Default: use q7 from observe replay.")

    # Camera and detector args are intentionally aligned with run_skewer_2d_servo.py.
    parser.add_argument("--camera-backend", choices=("orbbec", "opencv"), default="opencv")
    parser.add_argument("--orbbec-serial", default="")
    parser.add_argument("--camera-index", type=int, default=8)
    parser.add_argument("--camera-width", type=int, default=0)
    parser.add_argument("--camera-height", type=int, default=0)
    parser.add_argument("--camera-fps", type=int, default=0)
    parser.add_argument("--frame-timeout-ms", type=int, default=30)
    parser.add_argument("--flush-frames", type=int, default=8)
    parser.add_argument("--capture-attempts", type=int, default=20)
    parser.add_argument("--capture-flush-frames", type=int, default=15)
    parser.add_argument("--capture-flush-dt-s", type=float, default=0.03)
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
    parser.add_argument("--target-selection", choices=("skewer", "center", "score", "largest"), default="skewer")
    parser.add_argument(
        "--grasp-point-fraction",
        type=float,
        default=0.70,
        help="沿黑色涂层长轴取抓取点的位置。0.50 是中心；0.70 是更靠后/靠下。",
    )
    parser.add_argument("--red-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--red-h-low", type=int, default=8)
    parser.add_argument("--red-h-high", type=int, default=172)
    parser.add_argument("--red-s-min", type=int, default=130)
    parser.add_argument("--red-v-min", type=int, default=45)
    parser.add_argument("--red-min-area-px", type=float, default=80.0)
    parser.add_argument("--red-max-area-px", type=float, default=3000.0)
    parser.add_argument("--red-roi-left", type=float, default=0.0)
    parser.add_argument("--red-roi-right", type=float, default=1.0)
    parser.add_argument("--red-roi-top", type=float, default=0.12)
    parser.add_argument("--red-roi-bottom", type=float, default=0.62)
    parser.add_argument("--red-grasp-point-fraction", type=float, default=0.50)
    parser.add_argument("--track-target", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--track-roi-px", type=float, default=140.0)
    parser.add_argument("--track-fallback-to-full", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--multi-target-skip-radius-px",
        type=float,
        default=55.0,
        help="When pressing m repeatedly at one robot offset, skip already sampled target centers within this pixel radius.",
    )
    parser.add_argument(
        "--target-center-px",
        default="",
        help="目标对齐像素 u,v。标定时应与抓取使用同一个夹爪中心；默认从 --align-config 读取。",
    )
    parser.add_argument("--pixel-to-base-mm", default="")
    parser.add_argument("--update-align-config", action="store_true", help="Write fitted matrix into config/skewer_align_config.json.")
    parser.add_argument(
        "--max-update-rms-mm",
        type=float,
        default=4.0,
        help="Only update align config when the best fitted RMS error is below this threshold.",
    )
    parser.add_argument("--align-config", type=Path, default=DEFAULT_ALIGN_CONFIG)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    apply_align_config_defaults(args)
    stamp = datetime.now().strftime("xy_calib_%Y%m%d_%H%M%S")
    run_dir = args.output_dir.expanduser().resolve() / stamp
    run_dir.mkdir(parents=True, exist_ok=True)

    q_observe, gripper_from_csv = read_last_replay_q(args.observe_replay.expanduser().resolve())
    if args.gripper is None:
        args.gripper = gripper_from_csv
    save_json(
        run_dir / "inputs.json",
        {
            "observe_replay": str(args.observe_replay),
            "q_observe": q_observe,
            "gripper": args.gripper,
            "step_mm": args.step_mm,
            "execute": args.execute,
            "target_center_px": args.target_center_px,
        },
    )
    print(f"[calib] run_dir={run_dir}")
    print(f"[calib] q_observe={','.join(f'{v:.8f}' for v in q_observe)}")
    print(f"[calib] gripper={args.gripper:.8f}")

    samples = generate_samples(q_observe, float(args.gripper), run_dir, args)
    print("[calib] generated motion plan:")
    for sample in samples:
        print(f"  {sample.name:8s} delta_xy_mm={sample.delta_xy_mm} replay={sample.replay_csv}")
    if not args.execute:
        print("[calib] dry-run only. Add --execute to move robot and capture detections.")
        return 0
    if args.auto_run:
        execute_collection(samples, run_dir, args)
    elif args.manual_sequence:
        execute_collection_manual(samples, run_dir, args)
    else:
        execute_collection_jog(samples, run_dir, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
