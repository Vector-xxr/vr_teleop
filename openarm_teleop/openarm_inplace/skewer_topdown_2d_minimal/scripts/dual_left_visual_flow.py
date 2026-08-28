#!/usr/bin/env python3
"""Dual-arm task flow with left-arm visual grasp.

Flow:
  steady replay (dual)
  -> action1/pre replay (dual)
  -> left arm directly interpolates from action1 end to observe replay last row
  -> wait, detect target, XY align, grasp/lift/retreat (left arm only)
  -> left arm interpolates to action2/post replay first row
  -> action2/post replay (dual)

The low-level replay, IK, feedback, and grasp primitives are reused from the
existing align_once_move implementation so server-side waiting and side routing
stay consistent with the tested workflow.
"""

from __future__ import annotations

import argparse
import atexit
import importlib.util
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CORE_SCRIPT = ROOT / "scripts" / "align_once_move.py"


def load_core():
    spec = importlib.util.spec_from_file_location("dual_left_visual_flow_core", CORE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load core alignment module: {CORE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


core = load_core()
calib = core.calib
servo = core.servo

_SERVER_PROC = None
_CLEANUP_IN_PROGRESS = False


def register_server_proc(proc) -> None:
    global _SERVER_PROC
    _SERVER_PROC = proc


def clear_server_proc(proc=None) -> None:
    global _SERVER_PROC
    if proc is None or proc is _SERVER_PROC:
        _SERVER_PROC = None


def cleanup_running_processes(reason: str = "exit") -> None:
    global _CLEANUP_IN_PROGRESS
    if _CLEANUP_IN_PROGRESS:
        return
    proc = _SERVER_PROC
    if proc is None:
        return
    _CLEANUP_IN_PROGRESS = True
    try:
        if proc.poll() is None:
            print(f"[cleanup] stopping replay process because of {reason}")
            calib.stop_process(proc)
        clear_server_proc(proc)
    finally:
        _CLEANUP_IN_PROGRESS = False


def install_cleanup_handlers() -> None:
    atexit.register(cleanup_running_processes, "process exit")

    def handle_signal(signum, _frame):
        name = signal.Signals(signum).name
        cleanup_running_processes(name)
        raise SystemExit(128 + int(signum))

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)


def parser_has_option(parser: argparse.ArgumentParser, option: str) -> bool:
    return any(option in action.option_strings for action in parser._actions)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = core.build_arg_parser()
    parser.description = "Dual-arm steady/action flow with automatic left-arm visual grasp."
    if not parser_has_option(parser, "--xy-correction-scale-x"):
        parser.add_argument(
            "--xy-correction-scale-x",
            type=float,
            default=None,
            help="Scale the planned base X correction. Defaults to --xy-correction-scale.",
        )
    if not parser_has_option(parser, "--xy-correction-scale-y"):
        parser.add_argument(
            "--xy-correction-scale-y",
            type=float,
            default=None,
            help="Scale the planned base Y correction. Defaults to --xy-correction-scale.",
        )
    if not parser_has_option(parser, "--auto-detect-timeout-s"):
        parser.add_argument(
            "--auto-detect-timeout-s",
            type=float,
            default=0.0,
            help="Maximum seconds to retry detection after reaching observe; 0 means no timeout.",
        )
    parser.set_defaults(
        preview=False,
        auto_once=True,
        auto_cycle=False,
        quick_grasp_on_d=True,
        after_grasp_action="lift-only",
        post_grasp_lift_m=0.05,
        post_grasp_retreat_mm=130.0,
        post_grasp_retreat_order="after-lift",
        no_preview=True,
    )
    return parser


def apply_runtime_defaults(args: argparse.Namespace) -> None:
    args.dual_replay = True
    args.arm_side = "left_arm"
    args.quick_grasp_on_d = True
    args.auto_once = True
    args.auto_cycle = False
    args.after_grasp_action = "lift-only"
    if getattr(args, "no_preview", False):
        args.preview = False
    if getattr(args, "xy_correction_scale_x", None) is None:
        args.xy_correction_scale_x = float(args.xy_correction_scale)
    if getattr(args, "xy_correction_scale_y", None) is None:
        args.xy_correction_scale_y = float(args.xy_correction_scale)

    # Snappier pacing for dual visual flow: clamp long CLI leftovers (e.g. 10s
    # smooth joints / 3s C++ prepend on every play). Shorter explicit values win.
    def _cap(name: str, limit: float) -> None:
        cur = float(getattr(args, name, limit))
        if cur > limit:
            print(f"[flow-pace] {name}: {cur:.2f}s -> {limit:.2f}s", flush=True)
            setattr(args, name, limit)

    _cap("pre_to_observe_duration", 5.0)
    _cap("post_replay_transition_duration", 5.0)
    _cap("initial_transition_duration", 1.2)
    _cap("auto_start_delay_s", 0.5)
    _cap("settle_s", 0.1)
    if float(getattr(args, "server_startup_s", 3.0)) > 2.0:
        _cap("server_startup_s", 2.0)
    if float(getattr(args, "grasp_descent_duration", 5.0)) > 3.5:
        _cap("grasp_descent_duration", 3.5)


def xy_correction_scales(args: argparse.Namespace) -> np.ndarray:
    return np.asarray(
        [float(args.xy_correction_scale_x), float(args.xy_correction_scale_y)],
        dtype=np.float64,
    )


def compute_plan(record: dict, args: argparse.Namespace) -> dict:
    if not record.get("detected"):
        raise RuntimeError("No detection record available for planning")
    err = np.asarray(record["error_px"], dtype=np.float64)
    raw_move, model_name = core.correction_from_error(err, args)
    correction_scales = xy_correction_scales(args)
    scaled_move = raw_move * correction_scales
    max_move = None if bool(args.no_limit) else float(args.max_move_mm)
    move, limit_scale = core.limit_vector_mm(scaled_move, max_move)
    return {
        "error_px": err,
        "raw_move_mm": raw_move,
        "model": model_name,
        "scaled_move_mm": scaled_move,
        "move_mm": move,
        "xy_correction_scale": float(args.xy_correction_scale),
        "xy_correction_scale_x": float(correction_scales[0]),
        "xy_correction_scale_y": float(correction_scales[1]),
        "limit_scale": float(limit_scale),
        "max_move_mm": max_move,
    }


def path_arg(value: Path | None, name: str) -> Path:
    if value is None:
        raise RuntimeError(f"{name} is required")
    path = value.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def resolve_gripper_value(*values, default: float = 0.0) -> float:
    for value in values:
        if value is None:
            continue
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return float(default)


def matrix_to_list(T: np.ndarray) -> list[list[float]]:
    if hasattr(calib, "matrix_to_list"):
        return calib.matrix_to_list(T)
    return [[float(T[r, c]) for c in range(4)] for r in range(4)]


def format_q(q: list[float]) -> str:
    return "[" + ", ".join(f"{float(v):+.4f}" for v in q) + "]"


def log_pose_state(label: str, q: list[float], gripper: float, T: np.ndarray | None = None) -> None:
    print(f"[{label}] q={format_q(q)} gripper={float(gripper):+.6f}")
    if T is not None:
        print(
            f"[{label}] T_base_ee xyz="
            f"({float(T[0, 3]):+.4f}, {float(T[1, 3]):+.4f}, {float(T[2, 3]):+.4f}) m"
        )


def log_visual_config(args: argparse.Namespace, run_dir: Path) -> None:
    print(f"[visual-config] run_dir={run_dir}")
    print(
        "[visual-config] camera="
        f"{args.camera_backend} index={args.camera_index} size={args.camera_width}x{args.camera_height} "
        f"fps={args.camera_fps} preview={bool(args.preview)}"
    )
    print(
        "[visual-config] target="
        f"{args.target_selection} target_center_px={args.target_center_px} "
        f"track_target={bool(args.track_target)} track_roi_px={args.track_roi_px}"
    )
    print(
        "[visual-config] red hsv/area="
        f"h=[{args.red_h_low},{args.red_h_high}] s_min={args.red_s_min} v_min={args.red_v_min} "
        f"area=[{args.red_min_area_px},{args.red_max_area_px}] max_dist={args.red_max_target_dist_px}"
    )
    print(
        "[visual-config] red roi="
        f"left={args.red_roi_left} right={args.red_roi_right} top={args.red_roi_top} bottom={args.red_roi_bottom}"
    )
    print(
        "[visual-config] motion="
        f"max_move_mm={args.max_move_mm} max_step_joint_delta_rad={args.max_step_joint_delta_rad} "
        f"xy_scale_x={args.xy_correction_scale_x} xy_scale_y={args.xy_correction_scale_y}"
    )
    print(
        "[visual-config] grasp="
        f"open={args.gripper_open} close={args.gripper_close} descent_m={args.grasp_descent_m} "
        f"lift_m={args.post_grasp_lift_m} retreat_mm={args.post_grasp_retreat_mm} "
        f"retreat_order={args.post_grasp_retreat_order}"
    )


def save_run_plan(
    run_dir: Path,
    args: argparse.Namespace,
    *,
    steady_replay: Path,
    pre_replay: Path,
    observe_replay: Path,
    post_replay: Path,
) -> None:
    core.save_json(
        run_dir / "dual_left_visual_flow_plan.json",
        {
            "flow": [
                "steady_replay_dual",
                "pre_replay_dual",
                "left_pre_end_to_observe",
                "auto_detect_after_delay",
                "left_xy_align",
                "left_grasp_lift_retreat",
                "left_to_post_first",
                "post_replay_dual",
            ],
            "steady_replay": str(steady_replay),
            "pre_replay": str(pre_replay),
            "observe_replay": str(observe_replay),
            "post_replay": str(post_replay),
            "active_arm_side": "left_arm",
            "active_replay_side": "left",
            "auto_start_delay_s": float(args.auto_start_delay_s),
            "xy_correction_scale_x": float(args.xy_correction_scale_x),
            "xy_correction_scale_y": float(args.xy_correction_scale_y),
            "post_grasp_lift_m": float(args.post_grasp_lift_m),
            "post_grasp_retreat_mm": float(args.post_grasp_retreat_mm),
            "post_grasp_retreat_order": str(args.post_grasp_retreat_order),
        },
    )


def detect_until_success(cam, sample, run_dir: Path, args: argparse.Namespace) -> dict:
    retry_s = max(0.05, float(getattr(args, "auto_detect_retry_s", 1.0)))
    timeout_s = max(0.0, float(getattr(args, "auto_detect_timeout_s", 0.0)))
    deadline = None if timeout_s <= 0.0 else time.monotonic() + timeout_s
    attempt = 1
    while True:
        print(f"[auto-detect] attempt {attempt}: capturing and detecting")
        record = core.detect_once(cam, sample, run_dir, args, previous_record=None)
        if record.get("detected"):
            center = record.get("center_px")
            error = record.get("error_px")
            print(f"[auto-detect] detected center={center} error_px={error}")
            return record
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError(f"auto detection timed out after {timeout_s:.1f}s")
        print(f"[auto-detect][WARN] no target detected; retry in {retry_s:.1f}s")
        time.sleep(retry_s)
        attempt += 1


def _raise_if_aborted(abort_check) -> None:
    if abort_check is not None and abort_check():
        raise RuntimeError("abort requested")


def prepare_observe_state(
    args: argparse.Namespace,
    run_dir: Path,
    observe_replay: Path,
) -> tuple[list[float], float, np.ndarray]:
    """Resolve observe joint/gripper/FK used by job segment and CLI."""
    q_observe, gripper_from_observe = calib.read_last_replay_q(observe_replay)
    gripper_observe = resolve_gripper_value(
        args.gripper,
        gripper_from_observe,
        getattr(args, "gripper_open", None),
        default=0.0,
    )
    args.gripper = gripper_observe
    print(
        "[observe] gripper resolved: "
        f"args.gripper={args.gripper!r} observe_csv={gripper_from_observe!r} final={gripper_observe:+.6f}"
    )
    T_observe = calib.solve_fk(q_observe, run_dir, args)
    log_pose_state("observe", q_observe, gripper_observe, T_observe)
    core.save_json(
        run_dir / "observe_state.json",
        {
            "observe_replay": str(observe_replay),
            "q_observe": q_observe,
            "gripper_from_observe_replay": gripper_from_observe,
            "gripper_observe": float(gripper_observe),
            "T_base_ee_observe": matrix_to_list(T_observe),
        },
    )
    return list(q_observe), float(gripper_observe), T_observe


def start_server_and_steady(
    args: argparse.Namespace,
    run_dir: Path,
    steady_replay: Path | None = None,
):
    """Start one dual replay server and play the startup steady CSV once.

    Returns the server process. Caller owns lifetime (CLI cleans up in finally;
    persistent job service keeps it alive across jobs).
    """
    steady = path_arg(
        steady_replay if steady_replay is not None else args.steady_replay,
        "--steady-replay",
    )
    if not args.execute:
        raise RuntimeError("start_server_and_steady requires --execute")

    log_visual_config(args, run_dir)
    print("[motion-server] starting one persistent dual replay server")
    server_proc = calib.start_replay_server(steady, args)
    register_server_proc(server_proc)

    print(f"[steady-replay] playing dual steady action: {steady}")
    core.play_full_replay_on_server(
        server_proc,
        steady,
        args,
        label="steady-replay",
        side=None,
        feedback_dir=run_dir / "feedback_dumps",
    )
    return server_proc


def run_job_segment(
    server_proc,
    args: argparse.Namespace,
    run_dir: Path,
    pre_replay: Path,
    observe_replay: Path,
    post_replay: Path,
    *,
    abort_check=None,
    skip_action2_on_grasp_fail: bool = True,
) -> None:
    """Play action1 -> quintic to observe -> left 2D grasp -> quintic -> action2.

    Uses an already-running dual replay server (no steady replay here).
    """
    pre = path_arg(pre_replay, "--pre-replay")
    observe = path_arg(observe_replay, "--observe-replay")
    post = path_arg(post_replay, "--post-replay")
    _raise_if_aborted(abort_check)

    q_observe, gripper_observe, T_observe = prepare_observe_state(args, run_dir, observe)

    print(f"[pre-replay] playing dual action1: {pre}")
    core.play_full_replay_on_server(
        server_proc,
        pre,
        args,
        label="pre-replay",
        side=None,
        feedback_dir=run_dir / "feedback_dumps",
    )
    _raise_if_aborted(abort_check)
    q_pre_end, gripper_pre_end = core.read_recorded_active_replay_last_q(pre, args)
    log_pose_state("pre-replay-end-left", q_pre_end, gripper_pre_end)

    print(f"[pre-to-observe] left arm quintic transition to observe: {observe}")
    core.execute_joint_transition(
        name="left_pre_replay_end_to_observe",
        label="pre-to-observe",
        q_start=q_pre_end,
        gripper_start=float(gripper_pre_end),
        q_goal=q_observe,
        gripper_goal=gripper_observe,
        duration_s=float(args.pre_to_observe_duration),
        server_proc=server_proc,
        run_dir=run_dir,
        args=args,
    )
    _raise_if_aborted(abort_check)

    current_q = list(q_observe)
    current_T = T_observe.copy()
    current_gripper = gripper_observe
    args.gripper_current_after_align = current_gripper
    log_pose_state("pre-to-observe-done", current_q, current_gripper, current_T)
    center_sample = calib.PoseSample(
        "000_observe",
        (0.0, 0.0),
        current_q,
        current_gripper,
        observe,
    )

    try:
        delay_s = max(0.0, float(args.auto_start_delay_s))
        print(f"[auto-detect] observe reached; waiting {delay_s:.1f}s before opening camera/detecting")
        time.sleep(delay_s)
        _raise_if_aborted(abort_check)
        print("[auto-detect] opening camera now")
        with servo.Camera(args) as cam:
            core.flush_camera(cam, args)
            record = detect_until_success(cam, center_sample, run_dir, args)
        print("[auto-detect] camera closed after detection")
        _raise_if_aborted(abort_check)

        plan = compute_plan(record, args)
        core.save_json(
            run_dir / "auto_xy_plan.json",
            {
                "detection": record,
                "plan": {
                    k: (v.tolist() if hasattr(v, "tolist") else v)
                    for k, v in plan.items()
                },
            },
        )
        print(
            "[auto-xy] move_mm="
            f"({plan['move_mm'][0]:+.1f},{plan['move_mm'][1]:+.1f}) "
            f"raw=({plan['raw_move_mm'][0]:+.1f},{plan['raw_move_mm'][1]:+.1f}) "
            f"scaled=({plan['scaled_move_mm'][0]:+.1f},{plan['scaled_move_mm'][1]:+.1f}) "
            f"kx={plan['xy_correction_scale_x']:.2f} ky={plan['xy_correction_scale_y']:.2f} "
            f"limit_scale={plan['limit_scale']:.3f}"
        )
        move_sample, executed_move, target_T, _xy_scale = core.generate_xy_sample_with_joint_limit(
            name="001_auto_xy_align",
            current_T=current_T,
            q_start=current_q,
            gripper=current_gripper,
            move_mm=plan["move_mm"],
            run_dir=run_dir,
            args=args,
        )
        print(
            "[auto-xy] generated replay="
            f"{move_sample.replay_csv} executed_move_mm=({executed_move[0]:+.1f},{executed_move[1]:+.1f})"
        )
        core.play_full_replay_on_server(
            server_proc,
            move_sample.replay_csv,
            args,
            label="auto-xy",
            side="active",
            feedback_dir=run_dir / "feedback_dumps",
        )
        current_q = list(move_sample.q)
        current_T = target_T.copy()
        current_gripper = float(move_sample.gripper)
        args.gripper_current_after_align = current_gripper
        log_pose_state("auto-xy-done", current_q, current_gripper, current_T)
        _raise_if_aborted(abort_check)

        print("[grasp] executing left-arm visual grasp/lift/retreat")
        current_q, current_T, current_gripper = core.execute_grasp_sequence(
            q_aligned=current_q,
            T_aligned=current_T,
            server_proc=server_proc,
            run_dir=run_dir,
            args=args,
        )
        log_pose_state("grasp-done", current_q, current_gripper, current_T)
    except Exception as exc:
        if skip_action2_on_grasp_fail:
            raise RuntimeError(f"grasp failed; skipped action2: {exc}") from exc
        print(f"[WARN] grasp failed but continue_action2 enabled: {exc}", flush=True)

    _raise_if_aborted(abort_check)
    q_post_start, gripper_post_start = core.read_recorded_active_replay_first_q(post, args)
    log_pose_state("post-replay-start-left", q_post_start, gripper_post_start)
    print(f"[post-prep] left arm quintic transition to action2 first row: {post}")
    core.execute_joint_transition(
        name="left_lift_retreat_to_post_replay_start",
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
    _raise_if_aborted(abort_check)

    print(f"[post-replay] playing dual action2: {post}")
    core.play_full_replay_on_server(
        server_proc,
        post,
        args,
        label="post-replay",
        side=None,
        feedback_dir=run_dir / "feedback_dumps",
    )
    print("[flow] completed: action1 -> left visual grasp -> action2")


def run_flow(args: argparse.Namespace, run_dir: Path) -> None:
    """One-shot CLI flow: start server + steady, then one job segment, then stop."""
    steady_replay = path_arg(args.steady_replay, "--steady-replay")
    pre_replay = path_arg(args.pre_replay, "--pre-replay")
    observe_replay = path_arg(args.observe_replay, "--observe-replay")
    post_replay = path_arg(args.post_replay, "--post-replay")
    save_run_plan(
        run_dir,
        args,
        steady_replay=steady_replay,
        pre_replay=pre_replay,
        observe_replay=observe_replay,
        post_replay=post_replay,
    )

    if not args.execute:
        prepare_observe_state(args, run_dir, observe_replay)
        print("[flow] dry run only. Add --execute to move the robot.")
        return

    server_proc = None
    try:
        server_proc = start_server_and_steady(args, run_dir, steady_replay)
        run_job_segment(
            server_proc,
            args,
            run_dir,
            pre_replay,
            observe_replay,
            post_replay,
            skip_action2_on_grasp_fail=False,
        )
        print("[flow] completed: steady -> action1 -> left visual grasp -> action2")
    finally:
        if server_proc is not None:
            cleanup_running_processes("flow finished or failed")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    apply_runtime_defaults(args)
    core.apply_config_defaults(args)
    if not getattr(args, "pixel_to_base_mm", None):
        raise RuntimeError("--pixel-to-base-mm is required, or provide it through --config")
    run_dir = args.output_dir.expanduser().resolve() / (
        "dual_left_visual_flow_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    run_flow(args, run_dir)
    return 0


if __name__ == "__main__":
    install_cleanup_handlers()
    raise SystemExit(main())
