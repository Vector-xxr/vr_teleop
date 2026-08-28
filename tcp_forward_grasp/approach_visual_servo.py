"""Continuous visual-servo approach via dual_replay TCP velocity stream.

Does NOT use play|csv / micro-jog CSVs for Phase2 forward. Speaks:
  approach_on|side|timeout|accel
  tcp_vel|side|vx|vy|vz   (base frame, m/s)
  approach_stop|side
  approach_get_pose|side
"""

from __future__ import annotations

import socket
import time
from pathlib import Path
from typing import Any

import numpy as np

from .camera_preview import grab_bgr, preview_enabled, show_frame
from .vision_align import LockedRedTracker, horizontal_only_error


def _active_side(args) -> str:
    try:
        from .flow_loader import load_flow_module

        flow = load_flow_module()
        side = flow.core.active_replay_side(args)
        if side:
            return str(side)
    except Exception:
        pass
    raw = str(getattr(args, "active_replay_side", "") or getattr(args, "arm_side", "left") or "left")
    if "right" in raw:
        return "right"
    return "left"


def _status_port(args) -> int:
    return int(getattr(args, "replay_command_port", 47970)) + 1


def _send_and_wait_status(
    *,
    core,
    args,
    message: str,
    expect_status: str,
    timeout_s: float = 2.0,
    label: str = "approach",
) -> str:
    """Send UDP command and wait for a matching status|detail reply."""
    calib = core.calib
    port = int(args.replay_command_port)
    status_port = _status_port(args)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("127.0.0.1", status_port))
    except OSError:
        # Another listener may own the port; still send and best-effort recv.
        pass
    sock.settimeout(0.05)
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    detail_out = ""
    print(f"[{label}] UDP {message} -> port {port}", flush=True)
    calib.send_udp(port, message)
    while time.monotonic() < deadline:
        try:
            data, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        text = data.decode("utf-8", errors="ignore")
        status, _sep, detail = text.partition("|")
        if status == expect_status:
            detail_out = detail
            print(f"[{label}] status {status}|{detail}", flush=True)
            sock.close()
            return detail_out
        if status in ("busy", "approach_error"):
            sock.close()
            raise RuntimeError(f"approach command failed: {text}")
    sock.close()
    raise TimeoutError(f"no {expect_status} for {message!r} within {timeout_s:.1f}s")


def _parse_pose_detail(detail: str) -> tuple[list[float], float, np.ndarray | None]:
    """Parse approach_pose detail: side|q=q0,..|g=g|tcp=x,y,z"""
    q: list[float] = []
    gripper = 0.0
    tcp = None
    for part in detail.split("|"):
        if part.startswith("q="):
            q = [float(v) for v in part[2:].split(",") if v.strip()]
        elif part.startswith("g="):
            gripper = float(part[2:])
        elif part.startswith("tcp="):
            vals = [float(v) for v in part[4:].split(",") if v.strip()]
            if len(vals) == 3:
                tcp = np.asarray(vals, dtype=np.float64)
    return q, gripper, tcp


def _refresh_pose_from_stream(
    *,
    core,
    args,
    run_dir: Path,
    side: str,
    current_T,
    current_q: list[float],
    current_gripper: float,
) -> tuple[list[float], object, float]:
    detail = _send_and_wait_status(
        core=core,
        args=args,
        message=f"approach_get_pose|{side}",
        expect_status="approach_pose",
        timeout_s=1.5,
        label="approach-pose",
    )
    q, gripper, tcp = _parse_pose_detail(detail)
    if len(q) >= 7:
        current_q = list(q[:7])
    if gripper is not None:
        current_gripper = float(gripper)
    try:
        current_T = core.calib.solve_fk(current_q, run_dir, args)
        if tcp is not None and current_T is not None:
            # Prefer FK orientation; optionally sync translation from stream.
            current_T = current_T.copy()
            current_T[:3, 3] = tcp
    except Exception as exc:
        print(f"[approach][WARN] solve_fk after pose: {exc}", flush=True)
    return current_q, current_T, current_gripper


def _velocity_from_error(
    *,
    core,
    args,
    record_lr: dict[str, Any],
    approach_axis: str,
    approach_sign: float,
    speed_mm_s: float,
    lat_gain_mm_s_per_px: float,
    tracked: bool,
) -> tuple[float, float, float]:
    """Return base-frame tip velocity (m/s)."""
    forward_idx = 0 if approach_axis == "x" else 1 if approach_axis == "y" else 0
    v_mm = np.zeros(2, dtype=np.float64)
    v_mm[forward_idx] = float(approach_sign) * float(speed_mm_s)

    if tracked and record_lr.get("detected"):
        err = np.asarray(record_lr.get("error_px", [0.0, 0.0]), dtype=np.float64)
        err_u = float(err[0])
        try:
            raw_move, _model = core.correction_from_error(
                np.asarray([err_u, 0.0], dtype=np.float64), args
            )
            raw_move = np.asarray(raw_move, dtype=np.float64).reshape(-1)[:2]
            scales = np.asarray(
                [
                    float(getattr(args, "xy_correction_scale_x", 0.75)),
                    float(getattr(args, "xy_correction_scale_y", 0.85)),
                ],
                dtype=np.float64,
            )
            lat_vec = raw_move * scales
            # Zero the forward axis contribution from LR so forward stays constant speed.
            lat_vec[forward_idx] = 0.0
            norm = float(np.linalg.norm(lat_vec))
            lat_speed = float(lat_gain_mm_s_per_px) * abs(err_u)
            if norm > 1e-9 and lat_speed > 1e-9:
                lat_vec = (lat_vec / norm) * lat_speed
            else:
                lat_vec = np.zeros(2, dtype=np.float64)
            v_mm = v_mm + lat_vec
        except Exception as exc:
            print(f"[approach][WARN] lat vel from error failed: {exc}", flush=True)

    # Cap lateral a bit so forward dominates near contact.
    max_lat = max(float(speed_mm_s) * 1.5, 20.0)
    for i in (0, 1):
        if i == forward_idx:
            continue
        v_mm[i] = float(np.clip(v_mm[i], -max_lat, max_lat))

    return float(v_mm[0]) * 0.001, float(v_mm[1]) * 0.001, 0.0


def run_continuous_approach(
    *,
    core,
    server_proc,
    args,
    run_dir: Path,
    cam,
    tracker: LockedRedTracker,
    gate_locker,
    servo,
    current_q: list[float],
    current_T,
    current_gripper: float,
    hold_z_m: float | None = None,
) -> tuple[list[float], object, float, bool]:
    """Phase2: constant-speed TCP approach with live vision until gate touch.

    Returns (q, T, gripper, entered).
    """
    # Late import avoids circular dependency with grasp_lr_forward.
    from .grasp_lr_forward import ball_gate_overlap_frac, ball_touches_gate

    if server_proc is not None and server_proc.poll() is not None:
        raise RuntimeError(
            f"OpenArm replay server is not running, code={server_proc.returncode}"
        )

    side = _active_side(args)
    approach_axis = str(getattr(args, "approach_axis", "x")).lower()
    approach_sign = float(getattr(args, "approach_sign", 1.0))
    speed_mm_s = float(getattr(args, "approach_speed_mm_s", 25.0))
    lat_gain = float(getattr(args, "approach_lat_gain_mm_s_per_px", 0.5))
    accel_mm_s2 = float(getattr(args, "approach_accel_mm_s2", 200.0))
    cmd_timeout_s = float(getattr(args, "approach_cmd_timeout_s", 0.15))
    loop_hz = float(getattr(args, "approach_loop_hz", 25.0))
    max_s = float(getattr(args, "approach_max_s", 25.0))
    enter_confirm = max(1, int(getattr(args, "gripper_enter_confirm_frames", 2)))
    max_openloop = int(getattr(args, "forward_max_openloop_streak", 3))
    period = 1.0 / max(5.0, loop_hz)
    accel_mps2 = max(0.01, accel_mm_s2 * 0.001)

    print(
        f"[tcp-flow] PHASE2 continuous approach stream side={side} "
        f"base {approach_axis} sign={approach_sign:+.0f} "
        f"speed={speed_mm_s:.1f}mm/s lat_gain={lat_gain:.2f}mm/s/px "
        f"accel={accel_mm_s2:.0f}mm/s^2 timeout={cmd_timeout_s:.2f}s "
        f"(no CSV micro-jog)",
        flush=True,
    )

    _send_and_wait_status(
        core=core,
        args=args,
        message=f"approach_on|{side}|{cmd_timeout_s:.3f}|{accel_mps2:.4f}",
        expect_status="approach_on",
        timeout_s=2.0,
        label="approach-on",
    )

    entered = False
    lost_streak = 0
    enter_hits = 0
    t0 = time.monotonic()
    frame_i = 0
    port = int(args.replay_command_port)

    # Keep a long-lived status socket so we can drain noise; vel cmds are fire-and-forget.
    status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        status_sock.bind(("127.0.0.1", _status_port(args)))
    except OSError:
        pass
    status_sock.settimeout(0.0)

    try:
        while time.monotonic() - t0 < max_s:
            if server_proc is not None and server_proc.poll() is not None:
                raise RuntimeError(
                    f"OpenArm replay server exited during approach, "
                    f"code={server_proc.returncode}"
                )
            loop_start = time.monotonic()
            frame_i += 1

            # Drain status noise.
            try:
                while True:
                    status_sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError, OSError):
                pass

            frame = grab_bgr(cam)
            if frame is None:
                raise RuntimeError("camera read failed in approach phase")

            record = tracker.update(frame, near_gripper=True) or {"detected": False}
            if not gate_locker.locked:
                gate_locker.update(frame)
            gate = gate_locker.get() or {}
            tracked = bool(record.get("detected"))

            if not tracked:
                lost_streak += 1
                if lost_streak > max_openloop:
                    print(
                        "[tcp-flow][WARN] approach: too many frames without track; stop",
                        flush=True,
                    )
                    break
                if preview_enabled(args):
                    show_frame(
                        frame,
                        args,
                        status=f"APPROACH lost track {lost_streak}/{max_openloop}",
                        servo=servo,
                    )
                vx, vy, vz = _velocity_from_error(
                    core=core,
                    args=args,
                    record_lr={"detected": False},
                    approach_axis=approach_axis,
                    approach_sign=approach_sign,
                    speed_mm_s=speed_mm_s * 0.6,
                    lat_gain_mm_s_per_px=0.0,
                    tracked=False,
                )
                core.calib.send_udp(port, f"tcp_vel|{side}|{vx:.6f}|{vy:.6f}|{vz:.6f}")
                elapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, period - elapsed))
                continue

            lost_streak = 0
            record_lr = horizontal_only_error(record)
            err_u = float(record_lr["error_px"][0])
            touch_margin = float(getattr(args, "gripper_touch_margin_px", 8.0))
            # Match ball_touches_gate: area fraction inside gate (no tip inset).
            overlap = ball_gate_overlap_frac(
                record, gate, expand_px=touch_margin, tip_inset_px=0.0
            )
            in_grip = ball_touches_gate(record, gate, args)
            if in_grip:
                enter_hits += 1
            else:
                enter_hits = 0

            status = (
                f"APPROACH touch={in_grip} overlap={overlap:.0%} "
                f"hits={enter_hits}/{enter_confirm} err_u={err_u:+.1f}"
            )
            if preview_enabled(args):
                show_frame(frame, args, status=status, record=record_lr, servo=servo)

            if frame_i % 5 == 0 or enter_hits > 0:
                print(
                    f"[tcp-flow] APPROACH t={time.monotonic() - t0:.1f}s "
                    f"ball={record.get('center_px')} err_u={err_u:+.1f} "
                    f"touch={in_grip} overlap={overlap:.0%} "
                    f"hits={enter_hits}/{enter_confirm}",
                    flush=True,
                )

            core.save_json(
                run_dir / f"approach_frame_{frame_i:04d}.json",
                {
                    "detection": {k: v for k, v in record.items() if k != "paper_mask"},
                    "gate": gate,
                    "overlap_frac": overlap,
                    "touch": in_grip,
                    "enter_hits": enter_hits,
                    "mode": "tcp_vel_stream",
                },
            )

            if enter_hits >= enter_confirm:
                print(
                    "[tcp-flow] red handle >=60% in gate — approach_stop & close",
                    flush=True,
                )
                entered = True
                break

            bbox = record.get("bbox_xywh") or [0, 0, 0, 0]
            need_frac = float(getattr(args, "gripper_enter_frac", 0.60))
            if overlap >= need_frac and abs(err_u) <= max(
                25.0, float(getattr(args, "lr_align_done_err_px", 8.0)) * 3.0
            ):
                print(
                    f"[tcp-flow] overlap={overlap:.0%} >= {need_frac:.0%} "
                    f"— approach_stop & close",
                    flush=True,
                )
                entered = True
                break

            if len(bbox) >= 4 and gate.get("y_tip") is not None:
                handle_top = float(record.get("handle_top_y", bbox[1]))
                half = float(getattr(args, "gripper_enter_top_half_px", 0.0) or 0.0)
                if half <= 0.0:
                    half = max(6.0, 0.5 * float(min(float(bbox[2]), float(bbox[3]))))
                enter_top = float(gate["y_tip"]) + half - touch_margin
                if handle_top >= enter_top and abs(err_u) <= max(
                    25.0, float(getattr(args, "lr_align_done_err_px", 8.0)) * 3.0
                ):
                    print(
                        f"[tcp-flow] handle top y={handle_top:.0f} past enter_top "
                        f"{enter_top:.0f} (half={half:.0f}) — approach_stop & close",
                        flush=True,
                    )
                    entered = True
                    break

            vx, vy, vz = _velocity_from_error(
                core=core,
                args=args,
                record_lr=record_lr,
                approach_axis=approach_axis,
                approach_sign=approach_sign,
                speed_mm_s=speed_mm_s,
                lat_gain_mm_s_per_px=lat_gain,
                tracked=True,
            )
            # Hold Z: never command vz; C++ keeps orientation / seeded Z via IK.
            _ = hold_z_m
            core.calib.send_udp(port, f"tcp_vel|{side}|{vx:.6f}|{vy:.6f}|{vz:.6f}")

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period - elapsed))
        else:
            print(
                f"[tcp-flow][WARN] approach max_s={max_s:.1f}s without enter",
                flush=True,
            )
    finally:
        try:
            status_sock.close()
        except Exception:
            pass
        try:
            _send_and_wait_status(
                core=core,
                args=args,
                message=f"approach_stop|{side}",
                expect_status="approach_stopped",
                timeout_s=2.0,
                label="approach-stop",
            )
        except Exception as exc:
            print(f"[approach][WARN] approach_stop: {exc}", flush=True)
            try:
                core.calib.send_udp(port, f"approach_stop|{side}")
            except Exception:
                pass

    try:
        current_q, current_T, current_gripper = _refresh_pose_from_stream(
            core=core,
            args=args,
            run_dir=run_dir,
            side=side,
            current_T=current_T,
            current_q=current_q,
            current_gripper=current_gripper,
        )
    except Exception as exc:
        print(f"[approach][WARN] get_pose after stop: {exc}", flush=True)
        # Fall back: keep last commanded Python state.

    tracker.notify_motion()
    return current_q, current_T, current_gripper, entered
