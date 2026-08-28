#!/usr/bin/env python3
"""Send bimanual joint-space references with position and velocity output."""

from __future__ import annotations

import argparse
import csv
import math
import socket
import time
from pathlib import Path

INITIAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MIN_LIMIT = [-2.0944, -1.5708, -1.5708, 0.0, -1.5708, -1.5708, -1.5708, 0.0]
MAX_LIMIT = [2.0944, 3.1416, 1.5708, 3.1416, 1.5708, 1.5708, 1.5708, 0.044]
JOINT_SCALE = [1.0, 0.50, 0.35, 0.55, 0.30, 0.45, 0.25, 0.0]


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def triangle_unit(t: float, duration: float, cycles: float) -> tuple[float, float]:
    """Continuous position, mostly constant velocity, starting at home."""
    phase = (cycles * t / duration) % 1.0
    dphase_dt = cycles / duration
    if phase < 0.25:
        return 4.0 * phase, 4.0 * dphase_dt
    if phase < 0.75:
        return 2.0 - 4.0 * phase, -4.0 * dphase_dt
    return -4.0 + 4.0 * phase, 4.0 * dphase_dt


def sine_unit(t: float, duration: float, cycles: float) -> tuple[float, float]:
    phase = 2.0 * math.pi * cycles * t / duration
    return math.sin(phase), (2.0 * math.pi * cycles / duration) * math.cos(phase)


def make_pose_and_velocity(
    t: float, duration: float, cycles: float, amplitude: float, gripper: float, profile: str
) -> tuple[list[float], list[float]]:
    if profile == "sine":
        unit, unit_velocity = sine_unit(t, duration, cycles)
    else:
        unit, unit_velocity = triangle_unit(t, duration, cycles)

    q: list[float] = []
    dq: list[float] = []
    for idx, (home, lo, hi, scale) in enumerate(zip(INITIAL, MIN_LIMIT, MAX_LIMIT, JOINT_SCALE)):
        if idx == 7:
            q.append(clamp(gripper, lo, hi))
            dq.append(0.0)
            continue
        raw_q = home + amplitude * scale * unit
        clamped_q = clamp(raw_q, lo, hi)
        q.append(clamped_q)
        dq.append(amplitude * scale * unit_velocity if clamped_q == raw_q else 0.0)
    return q, dq


def interleave_position_velocity(q: list[float], dq: list[float]) -> list[float]:
    values: list[float] = []
    for position, velocity in zip(q, dq):
        values.extend([position, velocity])
    return values


def maybe_open_velocity_log(path: str | None):
    if not path:
        return None, None
    log_path = Path(path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = log_path.open("w", newline="")
    writer = csv.writer(handle)
    header = ["time"]
    for arm in ("left", "right"):
        for idx in range(8):
            name = f"{arm}_j{idx + 1}" if idx < 7 else f"{arm}_gripper"
            header.extend([f"{name}_q", f"{name}_dq"])
    writer.writerow(header)
    return handle, writer


def main() -> int:
    parser = argparse.ArgumentParser(description="Joint-space motion source for C++ MuJoCo teleop.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47910)
    parser.add_argument("--hz", type=float, default=500.0)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--cycles", type=float, default=2.0)
    parser.add_argument("--amplitude", type=float, default=0.75)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--profile", choices=("constant", "sine"), default="constant")
    parser.add_argument("--gripper", type=float, default=0.0)
    parser.add_argument("--positions-only", action="store_true")
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--velocity-log", default="")
    args = parser.parse_args()

    dt = 1.0 / max(args.hz, 1.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    next_send = start
    next_print = start
    log_handle, log_writer = maybe_open_velocity_log(args.velocity_log)

    print(
        "[smooth] sending "
        f"profile={args.profile} side={args.side} duration={args.duration}s "
        f"cycles={args.cycles} amplitude={args.amplitude}rad hz={args.hz}"
    )
    if args.positions_only:
        print("[smooth] UDP payload: 16 positions only")
    else:
        print("[smooth] UDP payload: 32 values, q/dq interleaved")

    try:
        while True:
            now = time.monotonic()
            t = now - start
            if t > args.duration:
                break

            moving_q, moving_dq = make_pose_and_velocity(
                t, args.duration, args.cycles, args.amplitude, args.gripper, args.profile
            )
            zero_dq = [0.0] * len(INITIAL)
            left_q = moving_q if args.side in ("left", "both") else INITIAL
            left_dq = moving_dq if args.side in ("left", "both") else zero_dq
            right_q = moving_q if args.side in ("right", "both") else INITIAL
            right_dq = moving_dq if args.side in ("right", "both") else zero_dq

            positions = left_q + right_q
            velocities = left_dq + right_dq
            if args.positions_only:
                values = positions
            else:
                values = interleave_position_velocity(positions, velocities)
            payload = ",".join(f"{value:.6f}" for value in values).encode("ascii")
            sock.sendto(payload, (args.host, args.port))

            if log_writer is not None:
                row = [f"{t:.6f}"]
                for position, velocity in zip(positions, velocities):
                    row.extend([f"{position:.6f}", f"{velocity:.6f}"])
                log_writer.writerow(row)

            if now >= next_print:
                arm_dq = [abs(v) for idx, v in enumerate(velocities) if idx % 8 < 7]
                max_dq = max(arm_dq) if arm_dq else 0.0
                active_dq = [v for v in arm_dq if v > 1e-9]
                min_active_dq = min(active_dq) if active_dq else 0.0
                print(f"[smooth] t={t:6.2f}s min_active|dq|={min_active_dq:.4f} max|dq|={max_dq:.4f} rad/s")
                next_print = now + max(args.print_every, dt)

            next_send += dt
            time.sleep(max(0.0, next_send - time.monotonic()))

        positions = INITIAL + INITIAL
        velocities = [0.0] * len(positions)
        values = positions if args.positions_only else interleave_position_velocity(positions, velocities)
        payload = ",".join(f"{value:.6f}" for value in values).encode("ascii")
        for _ in range(20):
            sock.sendto(payload, (args.host, args.port))
            time.sleep(dt)
    finally:
        if log_handle is not None:
            log_handle.close()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
