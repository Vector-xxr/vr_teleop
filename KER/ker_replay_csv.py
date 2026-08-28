#!/usr/bin/env python3
"""KER CSV replay at native sample rate (~200Hz) — no interpolation / upsample.

Plays the exact MIT joint commands recorded during KER teleop (record == control).
Do NOT use Quest dual_replay (1kHz) for these files; that staircase feels choppy.

Usage:
  ./run_ker_replay.sh recordings/sessions/ker_xxx/01/real_teleop_dual_merged.csv
  ./run_ker_replay.sh recordings/sessions/ker_xxx/01   # picks real_teleop_dual_merged.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

try:
    import openarm_can as oa
except ImportError as e:
    print("[ker-replay] openarm_can not installed", file=sys.stderr)
    raise SystemExit(1) from e

ARM_TYPES = [
    oa.MotorType.DM8009,
    oa.MotorType.DM8009,
    oa.MotorType.DM4340,
    oa.MotorType.DM4340,
    oa.MotorType.DM4310,
    oa.MotorType.DM4310,
    oa.MotorType.DM4310,
]
ARM_SEND = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GRIP_TYPE = oa.MotorType.DM4310
GRIP_SEND = 0x08
GRIP_RECV = 0x18

DEFAULT_KP = [80.0, 80.0, 80.0, 80.0, 20.0, 24.0, 20.0, 8.0]
DEFAULT_KD = [2.0, 2.0, 2.0, 2.0, 0.25, 0.25, 0.25, 0.35]


def make_arm(iface: str) -> oa.OpenArm:
    arm = oa.OpenArm(iface, True)
    arm.init_arm_motors(ARM_TYPES, ARM_SEND, ARM_RECV)
    arm.init_gripper_motor(GRIP_TYPE, GRIP_SEND, GRIP_RECV)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    arm.enable_all()
    for _ in range(30):
        arm.refresh_all()
        arm.recv_all(200)
        q = read_q8(arm)
        send_mit(arm, q, kp=[0.0] * 8, kd=[0.0] * 8)
        time.sleep(0.005)
    return arm


def read_q8(arm: oa.OpenArm) -> np.ndarray:
    arm.refresh_all()
    arm.recv_all(200)
    q = [m.get_position() for m in arm.get_arm().get_motors()]
    g = [m.get_position() for m in arm.get_gripper().get_motors()]
    if len(q) != 7 or not g:
        raise RuntimeError(f"unexpected motor count arm={len(q)} grip={len(g)}")
    return np.asarray(q + [g[0]], dtype=np.float64)


def send_mit(arm: oa.OpenArm, q8: np.ndarray, kp=None, kd=None, qd=None):
    kp = kp if kp is not None else DEFAULT_KP
    kd = kd if kd is not None else DEFAULT_KD
    if qd is None:
        qd = np.zeros(8, dtype=np.float64)
    arm_cmds = [
        oa.MITParam(float(kp[i]), float(kd[i]), float(q8[i]), float(qd[i]), 0.0)
        for i in range(7)
    ]
    grip_cmds = [
        oa.MITParam(float(kp[7]), float(kd[7]), float(q8[7]), float(qd[7]), 0.0)
    ]
    arm.get_arm().mit_control_all(arm_cmds)
    arm.get_gripper().mit_control_all(grip_cmds)
    arm.recv_all(200)


def resolve_csv(path: Path) -> Path:
    if path.is_dir():
        cand = path / "real_teleop_dual_merged.csv"
        if cand.is_file():
            return cand
        csvs = sorted(path.glob("*.csv"))
        if not csvs:
            raise FileNotFoundError(f"no CSV in {path}")
        return csvs[0]
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


LEGACY_L_CMD = [f"L_cmd_j{i}" for i in range(1, 8)] + ["L_cmd_grip"]
LEGACY_R_CMD = [f"R_cmd_j{i}" for i in range(1, 8)] + ["R_cmd_grip"]


def load_quest_dual(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (times, q_leftNx8, q_rightNx8). Exact rows only — no interp/upsample.

    Accepts:
      - Quest dual-merged: time,q0,dq0,tau0,...,q15
      - Legacy KER: timestamp_ns,L_cmd_*,R_cmd_*,... (cmd columns; remap only)
    """
    times: list[float] = []
    left: list[list[float]] = []
    right: list[list[float]] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fields = set(reader.fieldnames or [])

        if "time" in fields and "q0" in fields:
            fmt = "quest"
        elif "timestamp_ns" in fields and all(c in fields for c in LEGACY_L_CMD + LEGACY_R_CMD):
            fmt = "legacy"
            print(f"[ker-replay] legacy KER CSV detected → column remap (no upsample)")
        else:
            raise ValueError(
                f"{path} unsupported CSV.\n"
                "  Need Quest dual-merged (time,q0..) or legacy KER "
                "(timestamp_ns,L_cmd_*,R_cmd_*)."
            )

        t0_ns: int | None = None
        for row in reader:
            try:
                if fmt == "quest":
                    t = float(row["time"])
                    ql = [float(row[f"q{i}"]) for i in range(8)]
                    qr = [float(row[f"q{i}"]) for i in range(8, 16)]
                else:
                    ts = int(float(row["timestamp_ns"]))
                    if t0_ns is None:
                        t0_ns = ts
                    t = (ts - t0_ns) * 1e-9
                    ql = [float(row[c]) for c in LEGACY_L_CMD]
                    qr = [float(row[c]) for c in LEGACY_R_CMD]
            except (KeyError, TypeError, ValueError):
                continue
            times.append(t)
            left.append(ql)
            right.append(qr)

    if len(times) < 2:
        raise ValueError(f"{path}: need ≥2 frames, got {len(times)}")
    return (
        np.asarray(times, dtype=np.float64),
        np.asarray(left, dtype=np.float64),
        np.asarray(right, dtype=np.float64),
    )


def median_dt(times: np.ndarray) -> float:
    dts = np.diff(times)
    dts = dts[dts > 1e-6]
    if len(dts) == 0:
        return 0.005
    return float(np.median(dts))


def soft_goto(arm, target: np.ndarray, *, step: float, hz: float) -> None:
    """Rate-limit from current pose to target (full gains) — no hard snap."""
    cmd = read_q8(arm)
    dt = 1.0 / max(hz, 1.0)
    while True:
        err = target - cmd
        if float(np.max(np.abs(err))) < 0.02:
            send_mit(arm, target)
            return
        cmd = cmd + np.clip(err, -step, step)
        send_mit(arm, cmd)
        time.sleep(dt)


def parse_args():
    p = argparse.ArgumentParser(description="KER native-rate CSV replay (no upsample)")
    p.add_argument("path", type=Path, help="CSV file or session segment directory")
    p.add_argument("--right-can", default=os.getenv("OPENARM_RIGHT_CAN", "can0"))
    p.add_argument("--left-can", default=os.getenv("OPENARM_LEFT_CAN", "can1"))
    p.add_argument("--sides", choices=["both", "right", "left"], default="both")
    p.add_argument(
        "--hz",
        type=float,
        default=0.0,
        help="Fixed playback Hz if >0; else use CSV time deltas (default).",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Time scale (2.0 = 2x faster). Does not invent frames.",
    )
    p.add_argument(
        "--align-step",
        type=float,
        default=0.004,
        help="Max rad/tick for start soft-goto (default 0.004).",
    )
    p.add_argument(
        "--no-align",
        action="store_true",
        help="Skip soft-goto to first frame (not recommended).",
    )
    p.add_argument("--print-every", type=float, default=1.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = resolve_csv(args.path)
    times, q_l, q_r = load_quest_dual(csv_path)
    dt_med = median_dt(times)
    n = len(times)
    dur = float(times[-1] - times[0])
    eff_hz = (n - 1) / max(dur, 1e-9)
    print(
        f"[ker-replay] {csv_path}\n"
        f"[ker-replay] frames={n} duration={dur:.3f}s "
        f"median_dt={dt_med * 1000:.2f}ms effective_Hz≈{eff_hz:.1f} "
        f"speed={args.speed}"
    )
    if eff_hz > 400:
        print(
            "[ker-replay][WARN] CSV looks denser than typical KER (~200Hz). "
            "Still playing exact rows (no downsample).",
            file=sys.stderr,
        )

    right = left = None
    try:
        if args.sides in ("both", "right"):
            print(f"[ker-replay] enabling right on {args.right_can} ...")
            right = make_arm(args.right_can)
        if args.sides in ("both", "left"):
            print(f"[ker-replay] enabling left on {args.left_can} ...")
            left = make_arm(args.left_can)

        if not args.no_align:
            align_hz = 1.0 / dt_med if args.hz <= 0 else args.hz
            print(
                f"[ker-replay] soft-goto first frame "
                f"(step={args.align_step}, hz≈{align_hz:.0f}) ..."
            )
            if right is not None:
                soft_goto(right, q_r[0], step=args.align_step, hz=align_hz)
            if left is not None:
                soft_goto(left, q_l[0], step=args.align_step, hz=align_hz)
            print("[ker-replay] aligned → playing exact frames")

        t_wall0 = time.perf_counter()
        last_print = 0.0
        i = 0
        while i < n:
            # Schedule from CSV timeline (scaled); never invent intermediate q.
            if args.hz > 0:
                target_wall = t_wall0 + (i / args.hz) / max(args.speed, 1e-6)
            else:
                target_wall = t_wall0 + ((times[i] - times[0]) / max(args.speed, 1e-6))

            now = time.perf_counter()
            sleep = target_wall - now
            if sleep > 0:
                time.sleep(sleep)

            if right is not None:
                send_mit(right, q_r[i])
            if left is not None:
                send_mit(left, q_l[i])

            now = time.perf_counter()
            if args.print_every > 0 and now - last_print >= args.print_every:
                pct = 100.0 * i / max(n - 1, 1)
                print(
                    f"[ker-replay] i={i}/{n - 1} ({pct:.0f}%) "
                    f"t_csv={times[i]:.3f}s wall={now - t_wall0:.3f}s"
                )
                last_print = now
            i += 1

        print("[ker-replay] done — holding last frame 1s")
        t_end = time.perf_counter() + 1.0
        while time.perf_counter() < t_end:
            if right is not None:
                send_mit(right, q_r[-1])
            if left is not None:
                send_mit(left, q_l[-1])
            time.sleep(dt_med)

    except KeyboardInterrupt:
        print("\n[ker-replay] interrupted")
    finally:
        for name, arm in (("right", right), ("left", left)):
            if arm is None:
                continue
            try:
                q = read_q8(arm)
                for _ in range(10):
                    send_mit(arm, q, kp=[0.0] * 8, kd=[0.0] * 8)
                    time.sleep(0.005)
                arm.disable_all()
                print(f"[ker-replay] {name} disabled")
            except Exception as exc:
                print(f"[ker-replay] {name} shutdown error: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
