#!/usr/bin/env python3
"""Replay a single-motor OpenArm CSV trajectory on a Gloria/DM motor.

Expected CSV format:
    time,q,dq

The script keeps the original CSV timing, sends MIT commands, synchronizes the
motor PMAX/VMAX/TMAX before replay, and ramps smoothly from the current motor
position to the first CSV point.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from glob import glob

import numpy as np
import serial

from CAN import DM_Motor_Type, DM_variable, Motor, MotorControl, float_to_uint


def get_default_port() -> str:
    for pattern in (
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/serial/by-id/*",
    ):
        matches = sorted(glob(pattern))
        if matches:
            return matches[0]
    if os.name == "nt":
        return "COM3"
    return "/dev/ttyACM0"


@dataclass
class Frame:
    t: float
    q: float
    dq: float


@dataclass
class ElectricalParams:
    uv_threshold_v: float | None = None
    ov_threshold_v: float | None = None
    oc_threshold: float | None = None
    kt_nm_per_a: float | None = None


class TelemetryLogger:
    def __init__(self, path: str, electrical: ElectricalParams):
        self.path = os.path.abspath(path)
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.electrical = electrical
        self.writer.writerow(
            [
                "wall_time",
                "phase",
                "frame_index",
                "target_time",
                "send_late_s",
                "q_cmd",
                "dq_cmd",
                "tau_cmd",
                "q_fb",
                "dq_fb",
                "tau_fb",
                "current_est_a",
                "voltage_v",
                "uv_threshold_v",
                "ov_threshold_v",
                "oc_threshold",
                "kt_nm_per_a",
                "t_mos_c",
                "t_rotor_c",
            ]
        )

    def close(self) -> None:
        self.file.flush()
        self.file.close()

    def write(
        self,
        phase: str,
        frame_index: int,
        target_time: float,
        send_late_s: float,
        q_cmd: float,
        dq_cmd: float,
        tau_cmd: float,
        motor: Motor,
    ) -> None:
        q_fb = float(motor.getPosition())
        dq_fb = float(motor.getVelocity())
        tau_fb = float(motor.getTorque())
        kt = self.electrical.kt_nm_per_a
        current_est = tau_fb / kt if kt is not None and abs(kt) > 1e-9 else ""

        # DM4310 feedback used by this SDK does not expose real-time bus voltage.
        voltage_v = ""

        self.writer.writerow(
            [
                f"{time.time():.9f}",
                phase,
                frame_index,
                f"{target_time:.9f}",
                f"{send_late_s:.9f}",
                f"{q_cmd:.9f}",
                f"{dq_cmd:.9f}",
                f"{tau_cmd:.9f}",
                f"{q_fb:.9f}",
                f"{dq_fb:.9f}",
                f"{tau_fb:.9f}",
                f"{current_est:.9f}" if current_est != "" else "",
                voltage_v,
                _fmt_optional(self.electrical.uv_threshold_v),
                _fmt_optional(self.electrical.ov_threshold_v),
                _fmt_optional(self.electrical.oc_threshold),
                _fmt_optional(self.electrical.kt_nm_per_a),
                int(motor.T_Mos),
                int(motor.T_Roto),
            ]
        )


def _fmt_optional(value: float | None) -> str:
    return "" if value is None else f"{float(value):.9f}"


def load_csv(path: str) -> list[Frame]:
    frames: list[Frame] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"time", "q", "dq"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"CSV must contain columns: {', '.join(sorted(required))}")
        for line_no, row in enumerate(reader, start=2):
            try:
                t = float(row["time"])
                q = float(row["q"])
                dq = float(row["dq"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid numeric value at line {line_no}") from exc
            if not (math.isfinite(t) and math.isfinite(q) and math.isfinite(dq)):
                raise ValueError(f"non-finite value at line {line_no}")
            frames.append(Frame(t, q, dq))

    if len(frames) < 2:
        raise ValueError("CSV must contain at least two frames")
    if frames[0].t < -1e-9:
        raise ValueError("first timestamp must be non-negative")
    for prev, cur in zip(frames, frames[1:]):
        if cur.t <= prev.t:
            raise ValueError("timestamps must be strictly increasing")
    return frames


def sort_key(path: str) -> tuple[int, int | str, str]:
    stem = os.path.splitext(os.path.basename(path))[0]
    head = stem.split("_", 1)[0]
    if head.isdigit():
        return (0, int(head), stem)
    return (1, stem, stem)


def collect_csv_paths(path: str) -> list[str]:
    if os.path.isdir(path):
        paths = [
            os.path.join(path, name)
            for name in os.listdir(path)
            if name.lower().endswith(".csv")
        ]
        paths = sorted(paths, key=sort_key)
        if not paths:
            raise ValueError(f"directory contains no CSV files: {path}")
        return paths
    return [path]


def summarize_frames(frames: list[Frame]) -> None:
    dts = [b.t - a.t for a, b in zip(frames, frames[1:])]
    qs = [f.q for f in frames]
    dqs = [f.dq for f in frames]
    print(f"[CSV] frames={len(frames)} duration={frames[-1].t - frames[0].t:.3f}s")
    print(
        f"[CSV] dt min/avg/max={min(dts):.6f}/"
        f"{sum(dts) / len(dts):.6f}/{max(dts):.6f}s"
    )
    print(f"[CSV] q range=[{min(qs):.6f}, {max(qs):.6f}] rad")
    print(f"[CSV] dq range=[{min(dqs):.6f}, {max(dqs):.6f}] rad/s")


def sync_limits(mc: MotorControl, motor: Motor, motor_type: DM_Motor_Type) -> tuple[float, float, float]:
    """Read real PMAX/VMAX/TMAX and update SDK encoding limits when available."""
    pmax = mc.read_motor_param(motor, DM_variable.PMAX)
    vmax = mc.read_motor_param(motor, DM_variable.VMAX)
    tmax = mc.read_motor_param(motor, DM_variable.TMAX)
    if pmax is not None and vmax is not None and tmax is not None:
        mc.change_limit_param(motor_type, float(pmax), float(vmax), float(tmax))
        print(f"[SYNC] PMAX={float(pmax):.6f} VMAX={float(vmax):.6f} TMAX={float(tmax):.6f}")
    else:
        pmax, vmax, tmax = mc.Limit_Param[motor_type]
        print(
            "[WARN] failed to read real PMAX/VMAX/TMAX; "
            f"using SDK defaults PMAX={pmax:.6f} VMAX={vmax:.6f} TMAX={tmax:.6f}"
        )
    return float(pmax), float(vmax), float(tmax)


def read_electrical_params(mc: MotorControl, motor: Motor) -> ElectricalParams:
    params = ElectricalParams()
    values = [
        ("uv_threshold_v", DM_variable.UV_Value),
        ("ov_threshold_v", DM_variable.OV_Value),
        ("oc_threshold", DM_variable.OC_Value),
        ("kt_nm_per_a", DM_variable.KT_Value),
    ]
    for attr, rid in values:
        value = mc.read_motor_param(motor, rid)
        if value is not None:
            setattr(params, attr, float(value))

    kt_text = "unavailable"
    if params.kt_nm_per_a is not None:
        kt_text = f"{params.kt_nm_per_a:.6f} Nm/A"
    print(
        "[ELEC] "
        f"UV={_fmt_optional(params.uv_threshold_v) or 'NA'}V "
        f"OV={_fmt_optional(params.ov_threshold_v) or 'NA'}V "
        f"OC={_fmt_optional(params.oc_threshold) or 'NA'} "
        f"KT={kt_text}"
    )
    print("[ELEC] real-time voltage is not present in this SDK feedback frame; voltage_v will be blank")
    return params


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def send_mit_no_recv(
    mc: MotorControl,
    motor: Motor,
    kp: float,
    kd: float,
    q: float,
    dq: float,
    tau: float,
    q_limit: float,
    dq_limit: float,
    tau_limit: float,
) -> None:
    """MIT send path without per-frame recv(), for stable replay timing."""
    q = clamp(q, -q_limit, q_limit)
    dq = clamp(dq, -dq_limit, dq_limit)
    tau = clamp(tau, -tau_limit, tau_limit)

    motor_type = motor.MotorType
    q_max = mc.Limit_Param[motor_type][0]
    dq_max = mc.Limit_Param[motor_type][1]
    tau_max = mc.Limit_Param[motor_type][2]

    q_uint = float_to_uint(q, -q_max, q_max, 16)
    dq_uint = float_to_uint(dq, -dq_max, dq_max, 12)
    kp_uint = float_to_uint(kp, 0, 500, 12)
    kd_uint = float_to_uint(kd, 0, 5, 12)
    tau_uint = float_to_uint(tau, -tau_max, tau_max, 12)

    data_buf = np.array([0x00] * 8, np.uint8)
    data_buf[0] = (q_uint >> 8) & 0xFF
    data_buf[1] = q_uint & 0xFF
    data_buf[2] = dq_uint >> 4
    data_buf[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
    data_buf[4] = kp_uint & 0xFF
    data_buf[5] = kd_uint >> 4
    data_buf[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
    data_buf[7] = tau_uint & 0xFF

    mc._MotorControl__send_data(motor.SlaveID, data_buf)


def ramp_to_start(
    mc: MotorControl,
    motor: Motor,
    q_start: float,
    args: argparse.Namespace,
    q_limit: float,
    dq_limit: float,
    tau_limit: float,
    logger: TelemetryLogger | None = None,
) -> None:
    mc.refresh_motor_status(motor)
    q0 = float(motor.getPosition())
    distance = abs(q_start - q0)
    duration = max(args.ramp_time, distance / max(args.ramp_speed, 1e-6))
    steps = max(2, int(math.ceil(duration * args.hz)))
    print(f"[RAMP] current={q0:.6f} target={q_start:.6f} duration={duration:.3f}s steps={steps}")

    period = 1.0 / args.hz
    next_send = time.monotonic()
    for i in range(steps + 1):
        s = i / steps
        blend = 3.0 * s * s - 2.0 * s * s * s
        q = q0 + (q_start - q0) * blend
        dq = 0.0 if i == 0 or i == steps else (q_start - q0) * (6.0 * s - 6.0 * s * s) / duration
        send_mit_no_recv(mc, motor, args.kp, args.kd, q, dq, 0.0, q_limit, dq_limit, tau_limit)
        mc.recv()
        if logger is not None:
            logger.write("ramp", i, i * period, 0.0, q, dq, 0.0, motor)
        next_send += period
        sleep_time = next_send - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)


def enable_and_hold_current(
    mc: MotorControl,
    motor: Motor,
    args: argparse.Namespace,
    q_limit: float,
    dq_limit: float,
    tau_limit: float,
    logger: TelemetryLogger | None = None,
) -> float:
    """Enable gently and immediately hold the current measured position.

    The SDK enable() helper sleeps after sending the enable command. On a real
    gripper, that gap can let the drive react to an old/default target before
    the first hold command arrives. This path sends the raw enable command and
    immediately streams current-position MIT commands.
    """
    mc.refresh_motor_status(motor)
    q_hold = float(motor.getPosition())
    print(
        f"[ARM] current={q_hold:.6f}; "
        f"hold_time={args.arm_time:.3f}s kp={args.arm_kp:.3f} kd={args.arm_kd:.3f}"
    )

    # Preload a few current-position frames. Some firmwares ignore them while
    # disabled, but it is harmless and helps if commands are latched.
    for _ in range(3):
        send_mit_no_recv(mc, motor, args.arm_kp, args.arm_kd, q_hold, 0.0, 0.0, q_limit, dq_limit, tau_limit)
        time.sleep(1.0 / args.hz)

    if args.sdk_enable:
        mc.enable(motor)
    else:
        mc._MotorControl__control_cmd(motor, np.uint8(0xFC))

    steps = max(1, int(math.ceil(args.arm_time * args.hz)))
    next_send = time.monotonic()
    for i in range(steps):
        send_mit_no_recv(mc, motor, args.arm_kp, args.arm_kd, q_hold, 0.0, 0.0, q_limit, dq_limit, tau_limit)
        mc.recv()
        if logger is not None:
            logger.write("arm_hold", i, i / args.hz, 0.0, q_hold, 0.0, 0.0, motor)
        next_send += 1.0 / args.hz
        sleep_time = next_send - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return q_hold


def replay(
    mc: MotorControl,
    motor: Motor,
    frames: list[Frame],
    args: argparse.Namespace,
    q_limit: float,
    dq_limit: float,
    tau_limit: float,
    logger: TelemetryLogger | None = None,
) -> None:
    print("[REPLAY] start")
    start_wall = time.monotonic()
    last_recv = start_wall
    max_late = 0.0
    for i, frame in enumerate(frames):
        target_time = start_wall + (frame.t - frames[0].t) / args.speed
        now = time.monotonic()
        sleep_time = target_time - now
        send_late = 0.0
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            send_late = -sleep_time
            max_late = max(max_late, send_late)

        q_cmd = frame.q
        dq_cmd = frame.dq * args.speed
        tau_cmd = args.tau
        send_mit_no_recv(
            mc,
            motor,
            args.kp,
            args.kd,
            q_cmd,
            dq_cmd,
            tau_cmd,
            q_limit,
            dq_limit,
            tau_limit,
        )

        now = time.monotonic()
        if args.log_hz > 0:
            mc.recv()
            last_recv = now
            if logger is not None and i % args.log_decimation == 0:
                logger.write("replay", i, frame.t, send_late, q_cmd, dq_cmd, tau_cmd, motor)
        elif now - last_recv >= args.recv_period:
            mc.recv()
            last_recv = now
        if args.print_every > 0 and i % args.print_every == 0:
            print(f"[REPLAY] {i:5d}/{len(frames)} t={frame.t:.3f} q={frame.q:+.5f} dq={frame.dq:+.5f}")

    mc.recv()
    print(f"[REPLAY] done; max_late={max_late * 1000.0:.3f} ms")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay openarm_inplace time,q,dq CSV on a real Gloria/DM motor.")
    parser.add_argument("csv", nargs="?", default="../openarm_inplace/recordings/test_real.csv")
    parser.add_argument("--port", default=None, help="serial port; default uses platform_utils.get_default_port()")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--motor-type", default="DM4310", choices=[m.name for m in DM_Motor_Type])
    parser.add_argument("--slave-id", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--master-id", type=lambda x: int(x, 0), default=0x101)
    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--tau", type=float, default=0.0)
    parser.add_argument("--hz", type=float, default=200.0)
    parser.add_argument("--speed", type=float, default=1.0, help="time scale; 1.0 keeps original timing")
    parser.add_argument("--ramp-time", type=float, default=1.5)
    parser.add_argument("--ramp-speed", type=float, default=0.8, help="max approximate ramp speed rad/s")
    parser.add_argument("--arm-time", type=float, default=0.6, help="seconds to hold current position immediately after enable")
    parser.add_argument("--arm-kp", type=float, default=4.0, help="low gain used during immediate enable hold")
    parser.add_argument("--arm-kd", type=float, default=1.0, help="damping used during immediate enable hold")
    parser.add_argument("--sdk-enable", action="store_true", help="use SDK enable() helper instead of fast enable command")
    parser.add_argument("--q-limit", type=float, default=None, help="software q limit; default PMAX")
    parser.add_argument("--dq-limit", type=float, default=None, help="software dq limit; default min(VMAX, csv max)")
    parser.add_argument("--tau-limit", type=float, default=None, help="software tau limit; default TMAX")
    parser.add_argument("--recv-period", type=float, default=0.05, help="seconds between nonblocking recv calls")
    parser.add_argument("--print-every", type=int, default=200)
    parser.add_argument("--log", default=None, help="telemetry CSV path; default creates motor/replay_logs/*.csv")
    parser.add_argument("--log-hz", type=float, default=200.0, help="telemetry rate; default records every 200Hz frame")
    parser.add_argument("--dry-run", action="store_true", help="only validate CSV and print settings")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        raise ValueError("--hz must be positive")
    if args.speed <= 0:
        raise ValueError("--speed must be positive")
    if args.arm_time < 0:
        raise ValueError("--arm-time must be non-negative")
    if args.log_hz < 0:
        raise ValueError("--log-hz must be non-negative")
    args.log_decimation = 1
    if args.log_hz > 0:
        args.log_decimation = max(1, int(round(args.hz / args.log_hz)))

    replay_path = os.path.abspath(args.csv)
    csv_paths = collect_csv_paths(replay_path)
    sequences = []
    for csv_path in csv_paths:
        frames = load_csv(csv_path)
        sequences.append((csv_path, frames))
        print(f"[CSV] file={csv_path}")
        summarize_frames(frames)

    if args.dry_run:
        print(f"[DRY] validated {len(sequences)} CSV file(s); no motor command sent")
        return 0

    port = args.port or get_default_port()
    motor_type = DM_Motor_Type[args.motor_type]
    print(f"[CONN] port={port} baud={args.baud} motor={args.motor_type} slave=0x{args.slave_id:X} master=0x{args.master_id:X}")

    serial_device = serial.Serial(port, args.baud, timeout=0.02)
    mc = MotorControl(serial_device)
    motor = Motor(motor_type, args.slave_id, args.master_id)
    mc.addMotor(motor)
    logger = None
    pmax = vmax = tmax = 1.0

    try:
        pmax, vmax, tmax = sync_limits(mc, motor, motor_type)
        electrical = read_electrical_params(mc, motor)
        log_path = args.log
        if log_path is None:
            stem = os.path.splitext(os.path.basename(replay_path.rstrip(os.sep)))[0] or "replay"
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(os.path.dirname(__file__), "replay_logs", f"{stem}_telemetry_{stamp}.csv")
        if args.log_hz > 0:
            logger = TelemetryLogger(log_path, electrical)
            print(f"[LOG] telemetry -> {logger.path}")
        all_frames = [frame for _, frames in sequences for frame in frames]
        q_abs_max = max(abs(f.q) for f in all_frames)
        dq_abs_max = max(abs(f.dq * args.speed) for f in all_frames)
        q_limit = args.q_limit if args.q_limit is not None else pmax
        dq_limit = args.dq_limit if args.dq_limit is not None else min(vmax, max(dq_abs_max * 1.2, 0.1))
        tau_limit = args.tau_limit if args.tau_limit is not None else tmax
        if q_abs_max > q_limit + 1e-9:
            raise ValueError(f"CSV q max {q_abs_max:.6f} exceeds q limit {q_limit:.6f}")
        if dq_abs_max > dq_limit + 1e-9:
            raise ValueError(f"CSV dq max {dq_abs_max:.6f} exceeds dq limit {dq_limit:.6f}")
        print(f"[LIMIT] q_limit={q_limit:.6f} dq_limit={dq_limit:.6f} tau_limit={tau_limit:.6f}")

        time.sleep(0.2)
        enable_and_hold_current(mc, motor, args, q_limit, dq_limit, tau_limit, logger)
        last_q = None
        for seq_index, (csv_path, frames) in enumerate(sequences, start=1):
            print(f"[SEQUENCE] {seq_index}/{len(sequences)} {csv_path}")
            ramp_to_start(mc, motor, frames[0].q, args, q_limit, dq_limit, tau_limit, logger)
            replay(mc, motor, frames, args, q_limit, dq_limit, tau_limit, logger)
            last_q = frames[-1].q
        hold_q = last_q if last_q is not None else all_frames[-1].q
        for _ in range(int(args.hz * 0.5)):
            send_mit_no_recv(mc, motor, args.kp, args.kd, hold_q, 0.0, 0.0, q_limit, dq_limit, tau_limit)
            if logger is not None:
                mc.recv()
                logger.write("hold", -1, 0.0, 0.0, hold_q, 0.0, 0.0, motor)
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print("\n[STOP] interrupted")
    finally:
        try:
            for _ in range(20):
                send_mit_no_recv(mc, motor, 0.0, min(args.kd, 1.0), 0.0, 0.0, 0.0, pmax, vmax, tmax)
                time.sleep(0.005)
            mc.disable(motor)
        finally:
            if logger is not None:
                logger.close()
                print(f"[LOG] saved {logger.path}")
            serial_device.close()
            print("[EXIT] motor disabled, serial closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
