#!/usr/bin/env python3
"""steady -> action1 -> quintic to begin -> LR/forward grasp -> quintic -> action2.

Default flow:
  0) Play tasks/steady_action_on_table.csv (same prep as VR teleop startup)
  1) Play action1.csv
  2) Quintic joint transition to begin.csv last pose (grasp start, gripper open)
  3) LR align + forward until gate touch + close + lift + retreat
  4) Quintic joint transition to action2.csv first pose
  5) Play action2

Use --grasp-only for begin.csv + grasp (no steady/action1/2).
Use --skip-steady to skip the prep steady.
Use --lr-only to stop after LR align (debug).
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from . import defaults, paths
from .camera_preview import close_preview, grab_bgr, preview_enabled, show_frame
from .flow_loader import build_flow_args, load_flow_module
from .grasp_lr_forward import run_lr_forward_close
from .grasp_tcp_approach import execute_grasp_sequence_tcp_approach
from .gripper_gate import read_begin_gripper
from .ensure_motor_log import begin_temp_logging
from .motor_temp_monitor import MotorTempMonitor
from .vision_align import detect_rightmost_red, horizontal_only_error



def _camera_grab_bgr(cam) -> object:
    if hasattr(cam, "read_color"):
        return cam.read_color()
    if hasattr(cam, "read"):
        frame = cam.read()
        if isinstance(frame, tuple) and len(frame) == 2:
            _ok, frame = frame
            return frame
        return frame
    if hasattr(cam, "get_frame"):
        return cam.get_frame()
    raise RuntimeError("camera object has no read_color/read/get_frame")


def detect_rightmost_until_success(cam, args, run_dir: Path, core) -> dict:
    retry_s = max(0.05, float(getattr(args, "auto_detect_retry_s", 1.0)))
    timeout_s = max(0.0, float(getattr(args, "auto_detect_timeout_s", 0.0)))
    deadline = None if timeout_s <= 0.0 else time.monotonic() + timeout_s
    attempt = 1
    while True:
        print(f"[tcp-flow] detect rightmost red attempt {attempt}", flush=True)
        frame = _camera_grab_bgr(cam)
        if frame is None:
            raise RuntimeError("camera read failed")
        record = detect_rightmost_red(frame, args)
        if record and record.get("detected"):
            centers = record.get("all_centers_left_to_right") or []
            print(
                f"[tcp-flow] rightmost red center={record['center_px']} "
                f"error_full={record['error_px']} candidates={len(centers)} "
                f"ltr={centers}",
                flush=True,
            )
            core.save_json(run_dir / "detect_rightmost_red.json", record)
            try:
                import cv2

                cv2.imwrite(str(run_dir / "detect_frame.jpg"), frame)
            except Exception:
                pass
            return record
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError(f"rightmost red detection timed out after {timeout_s:.1f}s")
        print(f"[tcp-flow][WARN] no red ball; retry in {retry_s:.1f}s", flush=True)
        time.sleep(retry_s)
        attempt += 1


def start_and_play_begin(
    args,
    run_dir: Path,
    begin_csv: Path,
):
    """Start replay server and play begin.csv with VR-like smooth prepend.

    No separate steady / quintic prelude — only begin.csv.
    """
    flow = load_flow_module()
    core = flow.core
    flow.log_visual_config(args, run_dir)
    print(
        f"[tcp-flow] start server; play begin.csv "
        f"(initial_transition={float(args.initial_transition_duration):.1f}s "
        f"from current pose -> first frame): {begin_csv}",
        flush=True,
    )
    server_proc = flow.calib.start_replay_server(begin_csv, args)
    flow.register_server_proc(server_proc)
    core.play_full_replay_on_server(
        server_proc,
        begin_csv,
        args,
        label="begin-replay",
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    q_end, g_end = core.read_recorded_active_replay_last_q(begin_csv, args)
    T_end = flow.calib.solve_fk(q_end, run_dir, args)
    args.gripper = float(g_end)
    flow.log_pose_state("begin-end", q_end, g_end, T_end)
    return server_proc, list(q_end), T_end, float(g_end)


def detect_red_until_success(cam, sample, run_dir: Path, args, flow, servo) -> dict:
    """Detect rightmost red on white paper with optional live camera preview."""
    core = flow.core
    retry_s = max(0.05, float(getattr(args, "auto_detect_retry_s", 1.0)))
    timeout_s = max(0.0, float(getattr(args, "auto_detect_timeout_s", 0.0)))
    deadline = None if timeout_s <= 0.0 else time.monotonic() + timeout_s
    attempt = 1
    det_dir = run_dir / "detections" / "rightmost_red_on_tray"
    det_dir.mkdir(parents=True, exist_ok=True)
    while True:
        print(f"[tcp-flow] detect rightmost tray red attempt {attempt}", flush=True)
        if preview_enabled(args):
            t_end = time.monotonic() + (0.0 if attempt == 1 else retry_s)
            while True:
                frame = grab_bgr(cam)
                if frame is not None:
                    key = show_frame(
                        frame,
                        args,
                        status=f"detect tray red... attempt {attempt}",
                        servo=servo,
                    )
                    if key in (27, ord("q")):
                        raise RuntimeError("preview quit (q/Esc) during detect")
                if time.monotonic() >= t_end:
                    break
                time.sleep(0.03)

        frame = grab_bgr(cam)
        if frame is None:
            raise RuntimeError("camera read failed during red detect")
        record = detect_rightmost_red(frame, args) or {"detected": False}
        if preview_enabled(args):
            status = (
                f"tray rightmost={record.get('center_px')} n={record.get('candidate_count')}"
                if record.get("detected")
                else f"no tray red ({record.get('reason', 'miss')} attempt {attempt})"
            )
            show_frame(frame, args, status=status, record=record, servo=servo)
        if record.get("detected"):
            centers = record.get("all_centers_left_to_right") or []
            print(
                f"[tcp-flow] rightmost tray red center={record.get('center_px')} "
                f"error_px={record.get('error_px')} candidates={len(centers)} "
                f"ltr={centers}",
                flush=True,
            )
            save_rec = {k: v for k, v in record.items() if k != "paper_mask"}
            core.save_json(det_dir / "detect_rightmost_red_on_tray.json", save_rec)
            try:
                import cv2

                cv2.imwrite(str(det_dir / "detect_frame.jpg"), frame)
                paper_mask = record.get("paper_mask")
                if paper_mask is not None:
                    overlay = frame.copy()
                    overlay[paper_mask > 0] = (
                        overlay[paper_mask > 0] * 0.65 + np.array([0, 180, 0], dtype=np.float32) * 0.35
                    ).astype(np.uint8)
                    cv2.imwrite(str(det_dir / "white_paper_overlay.jpg"), overlay)
            except Exception:
                pass
            _ = sample
            return {k: v for k, v in record.items() if k != "paper_mask"}
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError(f"rightmost red-on-white timed out after {timeout_s:.1f}s")
        if not preview_enabled(args):
            print(
                f"[tcp-flow][WARN] no red on white ({record.get('reason')}); retry in {retry_s:.1f}s",
                flush=True,
            )
            time.sleep(retry_s)
        attempt += 1


def xy_align_target(
    server_proc,
    args,
    run_dir: Path,
    *,
    q_start: list[float],
    T_start,
    gripper_start: float,
) -> tuple[list[float], object, float]:
    """Detect rightmost tray red and XY-align green crosshair onto it (may refine once)."""
    flow = load_flow_module()
    core = flow.core
    servo = flow.servo
    calib = flow.calib

    current_q = list(q_start)
    current_T = T_start.copy()
    current_gripper = float(gripper_start)
    args.gripper_current_after_align = current_gripper
    args.target_selection = "red"

    center_sample = calib.PoseSample(
        "000_begin_wait",
        (0.0, 0.0),
        current_q,
        current_gripper,
        Path(args.observe_replay),
    )

    delay_s = max(0.0, float(args.auto_start_delay_s))
    print(
        "[tcp-flow] wait "
        f"{delay_s:.1f}s then detect rightmost red on tray above gripper "
        "(full XY -> overlap green crosshair)",
        flush=True,
    )
    with servo.Camera(args) as cam:
        core.flush_camera(cam, args)
        if preview_enabled(args):
            print(
                "[tcp-flow] live camera preview ON (window: tcp_forward_grasp camera)",
                flush=True,
            )
            t_end = time.monotonic() + delay_s
            while time.monotonic() < t_end:
                frame = grab_bgr(cam)
                if frame is not None:
                    remain = max(0.0, t_end - time.monotonic())
                    key = show_frame(
                        frame,
                        args,
                        status=f"pre-detect wait {remain:.1f}s",
                        servo=servo,
                    )
                    if key in (27, ord("q")):
                        raise RuntimeError("preview quit (q/Esc) before detect")
                time.sleep(0.03)
        else:
            time.sleep(delay_s)

        refine_passes = max(1, int(getattr(args, "xy_align_passes", 30)))
        done_err = float(getattr(args, "xy_align_done_err_px", 8.0))
        # Slow, damped XY: prefer overlap over speed. Restore pacing after flow-pace caps.
        args.xy_correction_scale = float(getattr(args, "xy_correction_scale", 0.75) or 0.75)
        args.xy_correction_scale_x = float(getattr(args, "xy_correction_scale_x", 0.75) or 0.75)
        args.xy_correction_scale_y = float(getattr(args, "xy_correction_scale_y", 0.85) or 0.85)
        args.motion_duration = float(
            getattr(args, "xy_align_motion_duration", getattr(args, "motion_duration", 3.5))
        )
        args.settle_s = float(getattr(args, "xy_align_settle_s", getattr(args, "settle_s", 0.45)))
        args.max_move_mm = min(float(getattr(args, "max_move_mm", 45.0) or 45.0), 60.0)
        print(
            f"[tcp-flow] align pacing: passes<={refine_passes} done_err<={done_err:.1f}px "
            f"motion={args.motion_duration:.1f}s settle={args.settle_s:.2f}s "
            f"max_step={args.max_move_mm:.1f}mm scales="
            f"({args.xy_correction_scale_x:.2f},{args.xy_correction_scale_y:.2f}) "
            "(no strict time limit; stop when green≈red)",
            flush=True,
        )

        for pass_i in range(refine_passes):
            record = detect_red_until_success(cam, center_sample, run_dir, args, flow, servo)
            centers = record.get("all_centers_left_to_right") or []
            print(
                f"[tcp-flow] pass {pass_i + 1}/{refine_passes}: tray reds L->R={centers} "
                f"SELECT rightmost={record.get('center_px')} "
                f"green={record.get('target_center_px')} "
                f"err_px={record.get('error_px')}",
                flush=True,
            )
            if len(centers) < 2:
                print(
                    "[tcp-flow][WARN] fewer than 2 tray reds found; "
                    "rightmost may be wrong if balls are missed",
                    flush=True,
                )
            plan = flow.compute_plan(record, args)
            err = np.asarray(plan["error_px"], dtype=np.float64)
            move = np.asarray(plan["move_mm"], dtype=np.float64)
            err_norm = float(np.hypot(err[0], err[1]))
            # No aggressive boost — keep each step moderate and iterate to overlap.
            core.save_json(
                run_dir / f"auto_xy_plan_pass{pass_i + 1}.json",
                {
                    "detection": {k: v for k, v in record.items() if k != "paper_mask"},
                    "plan": {
                        k: (v.tolist() if hasattr(v, "tolist") else v)
                        for k, v in plan.items()
                    },
                    "move_mm_commanded": move.tolist(),
                    "note": "slow multi-pass XY until green overlaps rightmost tray red",
                    "pass": pass_i + 1,
                    "done_err_px": done_err,
                },
            )
            print(
                f"[tcp-flow] XY pass {pass_i + 1}/{refine_passes} "
                f"err_px=({err[0]:+.1f},{err[1]:+.1f}) norm={err_norm:.1f} "
                f"move_mm=({move[0]:+.1f},{move[1]:+.1f}) "
                f"duration={args.motion_duration:.1f}s",
                flush=True,
            )
            if err_norm <= done_err:
                print(
                    f"[tcp-flow] overlap OK (|err|={err_norm:.1f}px <= {done_err:.1f}); "
                    "holding aligned pose",
                    flush=True,
                )
                break
            if float(np.hypot(move[0], move[1])) < 0.3:
                print("[tcp-flow] planned move tiny; stop XY passes", flush=True)
                break

            move_sample, executed_move, target_T, _ = core.generate_xy_sample_with_joint_limit(
                name=f"{pass_i + 1:03d}_xy_align_rightmost_tray_red",
                current_T=current_T,
                q_start=current_q,
                gripper=current_gripper,
                move_mm=(float(move[0]), float(move[1])),
                run_dir=run_dir,
                args=args,
            )
            print(
                f"[tcp-flow] XY executed_mm=({executed_move[0]:+.1f},{executed_move[1]:+.1f})",
                flush=True,
            )
            core.play_full_replay_on_server(
                server_proc,
                move_sample.replay_csv,
                args,
                label=f"xy-align-{pass_i + 1}",
                side="active",
                feedback_dir=run_dir / "feedback_dumps",
            )
            current_q = list(move_sample.q)
            current_T = target_T.copy()
            current_gripper = float(move_sample.gripper)
            args.gripper_current_after_align = current_gripper
            center_sample = calib.PoseSample(
                f"{pass_i + 1:03d}_after_xy",
                (0.0, 0.0),
                current_q,
                current_gripper,
                Path(args.observe_replay),
            )
            core.flush_camera(cam, args)
            time.sleep(max(0.2, float(args.settle_s)))
        else:
            print(
                f"[tcp-flow][WARN] reached max passes ({refine_passes}) without |err|<={done_err:.1f}px; "
                "holding best effort",
                flush=True,
            )

    flow.log_pose_state("after-xy-align", current_q, current_gripper, current_T)
    return current_q, current_T, current_gripper


def lr_align_and_grasp(
    server_proc,
    args,
    run_dir: Path,
    *,
    q_start: list[float],
    T_start,
    gripper_start: float,
) -> tuple[list[float], object, float]:
    """Legacy: LR-only align on rightmost red, then forward grasp + lift + return."""
    flow = load_flow_module()
    core = flow.core
    servo = flow.servo

    current_q = list(q_start)
    current_T = T_start.copy()
    current_gripper = float(gripper_start)
    args.gripper_current_after_align = current_gripper

    delay_s = max(0.0, float(args.auto_start_delay_s))
    print(f"[tcp-flow] wait {delay_s:.1f}s then detect rightmost red", flush=True)
    time.sleep(delay_s)
    with servo.Camera(args) as cam:
        core.flush_camera(cam, args)
        record = detect_rightmost_until_success(cam, args, run_dir, core)

    record_lr = horizontal_only_error(record)
    plan = flow.compute_plan(record_lr, args)
    core.save_json(
        run_dir / "auto_lr_plan.json",
        {
            "detection": record_lr,
            "plan": {
                k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in plan.items()
            },
            "note": "error_px[1] forced to 0 (no up-down align)",
        },
    )
    print(
        "[tcp-flow] LR-only align "
        f"err_px=({record_lr['error_px'][0]:+.1f},0) "
        f"move_mm=({plan['move_mm'][0]:+.1f},{plan['move_mm'][1]:+.1f})",
        flush=True,
    )

    move_sample, executed_move, target_T, _ = core.generate_xy_sample_with_joint_limit(
        name="001_lr_align_rightmost_red",
        current_T=current_T,
        q_start=current_q,
        gripper=current_gripper,
        move_mm=plan["move_mm"],
        run_dir=run_dir,
        args=args,
    )
    print(
        f"[tcp-flow] LR move executed_mm=({executed_move[0]:+.1f},{executed_move[1]:+.1f})",
        flush=True,
    )
    core.play_full_replay_on_server(
        server_proc,
        move_sample.replay_csv,
        args,
        label="lr-align",
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    current_q = list(move_sample.q)
    current_T = target_T.copy()
    current_gripper = float(move_sample.gripper)
    args.gripper_current_after_align = current_gripper

    print("[tcp-flow] forward grasp -> vertical lift -> return previous pose", flush=True)
    return execute_grasp_sequence_tcp_approach(
        q_aligned=current_q,
        T_aligned=current_T,
        server_proc=server_proc,
        run_dir=run_dir,
        args=args,
        core=core,
    )


def hold_aligned_pose(server_proc, args, *, label: str = "aligned pose") -> None:
    """Keep replay server alive so motors hold the current joint command.

    With preview enabled, also show a live camera window until Ctrl+C / q / Esc.
    """
    flow = load_flow_module()
    servo = flow.servo
    use_preview = preview_enabled(args)
    if use_preview:
        print(
            f"[tcp-flow] HOLDING {label} + live camera. "
            "Press Ctrl+C or q/Esc in the camera window to stop.",
            flush=True,
        )
    else:
        print(
            f"[tcp-flow] HOLDING {label} — joints stay at last command. "
            "Press Ctrl+C when done.",
            flush=True,
        )

    cam_cm = servo.Camera(args) if use_preview else None
    try:
        cam = cam_cm.__enter__() if cam_cm is not None else None
        if cam is not None:
            flow.core.flush_camera(cam, args)
        while True:
            if server_proc is None or server_proc.poll() is not None:
                code = None if server_proc is None else server_proc.returncode
                raise RuntimeError(
                    f"replay server exited while holding {label}, code={code}"
                )
            if cam is not None:
                frame = grab_bgr(cam)
                if frame is not None:
                    key = show_frame(
                        frame,
                        args,
                        status=f"HOLDING {label}",
                        servo=servo,
                    )
                    if key in (27, ord("q")):
                        print("[tcp-flow] preview quit (q/Esc); leaving hold", flush=True)
                        break
                time.sleep(0.03)
            else:
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("[tcp-flow] Ctrl+C received; leaving hold and shutting down", flush=True)
    finally:
        if cam_cm is not None:
            try:
                cam_cm.__exit__(None, None, None)
            except Exception:
                pass
        close_preview(args)


def run_wait_grasp_only(args) -> Path:
    """begin.csv -> LR align -> forward+LR until ball in fingers -> close -> hold."""
    flow = load_flow_module()
    flow.install_cleanup_handlers()
    begin_csv = flow.path_arg(args.observe_replay, "--wait-pose / begin.csv")
    lr_only = bool(getattr(args, "lr_only", False))

    run_dir = Path(args.output_dir).expanduser().resolve() / (
        "lr_fwd_close_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    core = flow.core
    core.save_json(
        run_dir / "mode.json",
        {
            "mode": "lr_only" if lr_only else "lr_align_then_forward_close",
            "begin_csv": str(begin_csv),
            "selection": "rightmost_red_on_white_plate",
            "phase1": "left_right_only_to_green_crosshair",
            "phase2": None if lr_only else "forward_while_keeping_lr_until_ball_in_fingers",
            "phase3": None if lr_only else "close_gripper",
            "initial_transition_duration": float(args.initial_transition_duration),
        },
    )

    if not args.execute:
        print("[tcp-flow] dry-run; begin would be", begin_csv, flush=True)
        return run_dir

    server_proc = None
    temp_mon: MotorTempMonitor | None = None
    try:
        temp_mon, _csv = begin_temp_logging(flow, run_dir)
        server_proc, q_wait, T_wait, g_wait = start_and_play_begin(
            args, run_dir, begin_csv
        )
        q_end, T_end, g_end = run_lr_forward_close(
            server_proc,
            args,
            run_dir,
            q_start=list(q_wait),
            T_start=T_wait,
            gripper_start=float(g_wait),
            lr_only=lr_only,
        )
        core.save_json(
            run_dir / "final_hold.json",
            {"q": list(q_end), "gripper": float(g_end)},
        )
        hold_aligned_pose(
            server_proc,
            args,
            label="after LR" if lr_only else "after lift/retreat",
        )
    finally:
        if temp_mon is not None:
            temp_mon.stop()
        if server_proc is not None:
            flow.cleanup_running_processes("lr_fwd_close finished or failed")
    return run_dir


def _raise_if_aborted(abort_check) -> None:
    if abort_check is not None and abort_check():
        raise RuntimeError("abort requested")


def run_job_segment_on_server(
    server_proc,
    args,
    run_dir: Path,
    pre_replay: Path,
    begin_csv: Path,
    post_replay: Path,
    *,
    abort_check=None,
    skip_action2_on_grasp_fail: bool = True,
    lr_only: bool = False,
    hold_after: bool = False,
) -> None:
    """action1 -> quintic to begin -> tcp_forward grasp -> quintic -> action2.

    Uses an already-running dual replay server (no steady / no process shutdown).
    """
    flow = load_flow_module()
    core = flow.core
    pre = flow.path_arg(pre_replay, "--pre-replay / action1.csv")
    begin = flow.path_arg(begin_csv, "--wait-pose / begin.csv")
    post = flow.path_arg(post_replay, "--post-replay / action2.csv")
    _raise_if_aborted(abort_check)

    gripper_open = float(getattr(args, "gripper_open", None) or read_begin_gripper(begin))
    args.gripper_open = gripper_open
    args.gripper = gripper_open
    q_obs, _g_obs_csv, T_obs = flow.prepare_observe_state(args, run_dir, begin)
    g_obs = float(gripper_open)

    print(f"[tcp-flow] play action1: {pre}", flush=True)
    core.play_full_replay_on_server(
        server_proc,
        pre,
        args,
        label="pre-replay",
        side=None,
        feedback_dir=run_dir / "feedback_dumps",
    )
    _raise_if_aborted(abort_check)
    q_pre_end, g_pre = core.read_recorded_active_replay_last_q(pre, args)
    flow.log_pose_state("action1-end", q_pre_end, g_pre)

    print(
        f"[tcp-flow] quintic action1-end -> begin/grasp-start "
        f"({float(args.pre_to_observe_duration):.1f}s, gripper->{g_obs:+.4f})",
        flush=True,
    )
    core.execute_joint_transition(
        name="action1_to_begin",
        label="pre-to-observe",
        q_start=list(q_pre_end),
        gripper_start=float(g_pre),
        q_goal=list(q_obs),
        gripper_goal=float(g_obs),
        duration_s=float(args.pre_to_observe_duration),
        server_proc=server_proc,
        run_dir=run_dir,
        args=args,
    )
    _raise_if_aborted(abort_check)
    T_obs = flow.calib.solve_fk(q_obs, run_dir, args)
    flow.log_pose_state("begin-grasp-start", q_obs, g_obs, T_obs)

    q_end = list(q_obs)
    g_end = float(g_obs)
    try:
        q_end, _T_end, g_end = run_lr_forward_close(
            server_proc,
            args,
            run_dir,
            q_start=list(q_obs),
            T_start=T_obs,
            gripper_start=float(g_obs),
            lr_only=lr_only,
        )
    except Exception as exc:
        if skip_action2_on_grasp_fail:
            raise RuntimeError(f"grasp failed; skipped action2: {exc}") from exc
        print(f"[WARN] grasp failed but continue_action2 enabled: {exc}", flush=True)

    _raise_if_aborted(abort_check)
    if lr_only:
        core.save_json(
            run_dir / "final_hold.json",
            {"q": list(q_end), "gripper": float(g_end), "note": "lr_only"},
        )
        if hold_after:
            hold_aligned_pose(server_proc, args, label="after LR (no action2)")
        print("[tcp-flow] completed: action1 -> begin -> LR-only", flush=True)
        return

    q_post, g_post = core.read_recorded_active_replay_first_q(post, args)
    print(
        f"[tcp-flow] quintic after-grasp -> action2-start "
        f"({float(args.post_replay_transition_duration):.1f}s)",
        flush=True,
    )
    core.execute_joint_transition(
        name="grasp_to_action2",
        label="post-prep",
        q_start=list(q_end),
        gripper_start=float(g_end),
        q_goal=list(q_post),
        gripper_goal=float(g_post),
        duration_s=float(args.post_replay_transition_duration),
        server_proc=server_proc,
        run_dir=run_dir,
        args=args,
    )
    _raise_if_aborted(abort_check)

    print(f"[tcp-flow] play action2: {post}", flush=True)
    core.play_full_replay_on_server(
        server_proc,
        post,
        args,
        label="post-replay",
        side=None,
        feedback_dir=run_dir / "feedback_dumps",
    )
    core.save_json(
        run_dir / "final_hold.json",
        {
            "q": list(q_post),
            "gripper": float(g_post),
            "post_replay": str(post),
        },
    )
    if hold_after:
        hold_aligned_pose(server_proc, args, label="after action2")
    print("[tcp-flow] completed: action1 -> begin grasp -> action2", flush=True)


def run_full_with_actions(args) -> Path:
    """action1 -> quintic to begin -> LR/forward/close/lift/retreat -> quintic -> action2."""
    flow = load_flow_module()
    flow.install_cleanup_handlers()
    pre = flow.path_arg(args.pre_replay, "--pre-replay / action1.csv")
    begin_csv = flow.path_arg(args.observe_replay, "--wait-pose / begin.csv")
    post = flow.path_arg(args.post_replay, "--post-replay / action2.csv")
    lr_only = bool(getattr(args, "lr_only", False))
    skip_steady = bool(getattr(args, "skip_steady", False))

    run_dir = Path(args.output_dir).expanduser().resolve() / (
        "action_grasp_action_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    core = flow.core
    core.save_json(
        run_dir / "mode.json",
        {
            "mode": "steady_action1_quintic_grasp_quintic_action2",
            "pre_replay": str(pre),
            "begin_csv": str(begin_csv),
            "post_replay": str(post),
            "skip_steady": skip_steady,
            "steady_replay": str(getattr(args, "steady_replay", paths.STEADY_CSV)),
            "lr_only": lr_only,
            "pre_to_observe_duration": float(args.pre_to_observe_duration),
            "post_replay_transition_duration": float(args.post_replay_transition_duration),
            "post_grasp_retreat_mm": float(getattr(args, "post_grasp_retreat_mm", 140.0)),
        },
    )
    if not args.execute:
        print(
            "[tcp-flow] dry-run; would run steady -> action1 -> begin -> grasp -> action2",
            flush=True,
        )
        return run_dir

    server_proc = None
    temp_mon: MotorTempMonitor | None = None
    try:
        temp_mon, _csv = begin_temp_logging(flow, run_dir)
        if skip_steady:
            print(
                f"[tcp-flow] start server on action1 (skip steady): {pre}",
                flush=True,
            )
            flow.log_visual_config(args, run_dir)
            server_proc = flow.calib.start_replay_server(pre, args)
            flow.register_server_proc(server_proc)
        else:
            steady = flow.path_arg(
                getattr(args, "steady_replay", None) or paths.STEADY_CSV,
                "--steady-replay",
            )
            print(
                f"[tcp-flow] prep steady (same as VR teleop startup): {steady}",
                flush=True,
            )
            server_proc = flow.start_server_and_steady(args, run_dir, steady)

        run_job_segment_on_server(
            server_proc,
            args,
            run_dir,
            pre,
            begin_csv,
            post,
            lr_only=lr_only,
            skip_action2_on_grasp_fail=True,
            hold_after=True,
        )
    finally:
        if temp_mon is not None:
            temp_mon.stop()
        if server_proc is not None:
            flow.cleanup_running_processes("action_grasp_action finished or failed")
    return run_dir


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "steady -> action1 -> quintic to begin -> grasp -> "
            "quintic to action2 (default). Use --grasp-only for begin+grasp only."
        )
    )
    p.add_argument(
        "--grasp-only",
        "--wait-grasp-only",
        dest="grasp_only",
        action="store_true",
        help="Only begin.csv + grasp (skip steady/action1/action2)",
    )
    p.add_argument(
        "--with-action12",
        action="store_true",
        default=True,
        help="steady + action1 -> quintic -> grasp -> quintic -> action2 (default)",
    )
    p.add_argument(
        "--with-steady",
        action="store_true",
        default=True,
        help="Play tasks/steady_action_on_table.csv before action1 (default on; same as VR)",
    )
    p.add_argument(
        "--skip-steady",
        action="store_true",
        help="Skip prep steady; start directly on action1",
    )
    p.add_argument(
        "--lr-only",
        action="store_true",
        help="Stop after left-right align (no forward / no close); for debug",
    )
    p.add_argument(
        "--align-only",
        action="store_true",
        help="Alias of --lr-only (legacy name)",
    )
    p.add_argument(
        "--full-grasp",
        action="store_true",
        help="Legacy flag; ignored (close already in grasp pipeline)",
    )
    p.add_argument(
        "--wait-pose",
        "--observe-replay",
        "--begin-csv",
        dest="observe_replay",
        type=Path,
        default=None,
        help="begin.csv / grasp start pose (default: left_grasp_exec_bundle/begin.csv)",
    )
    p.add_argument("--steady-replay", type=Path, default=None)
    p.add_argument("--pre-replay", type=Path, default=None, help="action1.csv")
    p.add_argument("--post-replay", type=Path, default=None, help="action2.csv")
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--output-dir", type=Path, default=paths.OUTPUT_ROOT)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument(
        "--begin-transition-duration",
        type=float,
        default=2.0,
        help="Smooth seconds from current pose into first replay CSV (C++ prepend)",
    )
    p.add_argument(
        "--pre-to-observe-duration",
        type=float,
        default=3.0,
        help="Quintic seconds: action1 end -> begin/grasp start",
    )
    p.add_argument(
        "--post-replay-transition-duration",
        type=float,
        default=3.0,
        help="Quintic seconds: after-grasp -> action2 start",
    )
    p.add_argument("--grasp-approach-m", type=float, default=defaults.GRASP_APPROACH_M)
    p.add_argument("--approach-frame", choices=["base", "tcp"], default=defaults.APPROACH_FRAME)
    p.add_argument("--approach-axis", choices=["x", "y", "z"], default=defaults.APPROACH_AXIS)
    p.add_argument("--approach-sign", type=float, default=defaults.APPROACH_SIGN)
    p.add_argument("--grasp-approach-duration", type=float, default=3.5)
    p.add_argument("--post-grasp-lift-m", type=float, default=0.09)
    p.add_argument(
        "--post-grasp-retreat-mm",
        type=float,
        default=140.0,
        help="After lift, retreat this many mm opposite approach (default 140)",
    )
    p.add_argument("--camera-index", type=int, default=None)
    p.add_argument(
        "--preview",
        action="store_true",
        help="Force live camera window on (default: on)",
    )
    p.add_argument(
        "--no-preview",
        action="store_true",
        help="Disable live camera window",
    )
    p.add_argument("--preview-max-width", type=int, default=960)
    p.add_argument("--sudo-password", type=str, default=None)
    return p


def main(argv: list[str] | None = None) -> int:
    cli = build_parser().parse_args(argv)
    lr_only = bool(cli.lr_only) or bool(cli.align_only)
    grasp_only = bool(getattr(cli, "grasp_only", False))
    overrides = {
        "grasp_approach_m": float(cli.grasp_approach_m),
        "grasp_approach_duration": float(cli.grasp_approach_duration),
        "approach_frame": str(cli.approach_frame),
        "approach_axis": str(cli.approach_axis),
        "approach_sign": float(cli.approach_sign),
        "tcp_approach_axis": str(cli.approach_axis),
        "tcp_approach_sign": float(cli.approach_sign),
        "post_grasp_lift_m": float(cli.post_grasp_lift_m),
        "post_grasp_return": "none",
        "post_grasp_retreat_mm": float(cli.post_grasp_retreat_mm),
        "target_selection": "red",
        "red_fallback": False,
        "lr_only": lr_only,
        "align_only": lr_only,
        "initial_transition_duration": float(cli.begin_transition_duration),
        "pre_to_observe_duration": float(cli.pre_to_observe_duration),
        "post_replay_transition_duration": float(cli.post_replay_transition_duration),
        "skip_steady": bool(cli.skip_steady),
        "preview_max_width": int(cli.preview_max_width),
        "output_dir": cli.output_dir,
        "execute": not bool(cli.dry_run),
        "observe_replay": paths.DEFAULT_BEGIN_CSV,
        "pre_replay": paths.DEFAULT_ACTION1,
        "post_replay": paths.DEFAULT_ACTION2,
        "steady_replay": paths.STEADY_CSV,
    }
    if cli.observe_replay is not None:
        overrides["observe_replay"] = cli.observe_replay
    if cli.steady_replay is not None:
        overrides["steady_replay"] = cli.steady_replay
    if cli.pre_replay is not None:
        overrides["pre_replay"] = cli.pre_replay
    if cli.post_replay is not None:
        overrides["post_replay"] = cli.post_replay
    if cli.config is not None:
        overrides["config"] = cli.config
    if cli.camera_index is not None:
        overrides["camera_index"] = cli.camera_index
    if cli.sudo_password is not None:
        overrides["sudo_password"] = cli.sudo_password

    args = build_flow_args(overrides=overrides, output_dir=Path(cli.output_dir))
    args.post_grasp_return = "aligned"
    args.lr_only = lr_only
    args.align_only = lr_only
    args.skip_steady = bool(cli.skip_steady)
    args.initial_transition_duration = float(cli.begin_transition_duration)
    args.pre_to_observe_duration = float(cli.pre_to_observe_duration)
    args.post_replay_transition_duration = float(cli.post_replay_transition_duration)
    args.post_grasp_retreat_mm = float(cli.post_grasp_retreat_mm)
    # Preview: default from flow_loader (on). Only CLI flags override.
    if bool(cli.no_preview):
        args.preview = False
        args.no_preview = True
    elif bool(cli.preview):
        args.preview = True
        args.no_preview = False
    args.preview_max_width = int(cli.preview_max_width)
    args.motion_duration = float(getattr(args, "xy_align_motion_duration", 2.2))
    args.settle_s = float(getattr(args, "xy_align_settle_s", 0.22))
    args.lr_align_passes = int(getattr(args, "lr_align_passes", 20))
    args.lr_align_done_err_px = float(getattr(args, "lr_align_done_err_px", 8.0))
    args.forward_align_passes = int(getattr(args, "forward_align_passes", 80))
    args.forward_step_mm = float(getattr(args, "forward_step_mm", 8.0))
    args.max_move_mm = float(getattr(args, "max_move_mm", 50.0))
    args.auto_detect_timeout_s = 0.0

    if grasp_only:
        run_wait_grasp_only(args)
    else:
        run_full_with_actions(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
