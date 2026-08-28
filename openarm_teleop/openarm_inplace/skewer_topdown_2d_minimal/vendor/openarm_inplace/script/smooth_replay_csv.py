#!/usr/bin/env python3
"""Offline smoothing for OpenArm replay CSV files.

Input/output format:
    time,q0,dq0,q1,dq1,...,q15,dq15

The script smooths joint positions with a forward-backward exponential
low-pass filter, then recomputes velocities from the smoothed positions.
It uses only the Python standard library so it can run in the project conda
environment even when numpy/scipy are unavailable.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


DEFAULT_INPUT = (
    Path(__file__).resolve().parents[1]
    / "recordings"
    / "sessions"
    / "1test_taiqi"
    / "real_teleop_dual_merged.csv"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smooth an OpenArm replay CSV without modifying the original file."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"input replay CSV (default: {DEFAULT_INPUT})",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output CSV path (default: INPUT_STEM_smooth.csv next to input)",
    )
    parser.add_argument(
        "--cutoff-hz",
        type=float,
        default=6.0,
        help="position low-pass cutoff frequency in Hz (default: 6.0)",
    )
    parser.add_argument(
        "--velocity-cutoff-hz",
        type=float,
        default=10.0,
        help="velocity low-pass cutoff frequency in Hz; <=0 disables it (default: 10.0)",
    )
    parser.add_argument(
        "--no-preserve-endpoints",
        action="store_true",
        help="do not force smoothed positions to keep the original first/last positions",
    )
    parser.add_argument(
        "--no-zero-end-velocity",
        action="store_true",
        help="do not force first/last velocity commands to zero",
    )
    parser.add_argument(
        "--max-velocity",
        type=float,
        default=0.0,
        help="optional absolute velocity clamp in rad/s; <=0 disables it",
    )
    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=0.0,
        help="optional absolute acceleration clamp in rad/s^2; <=0 disables it",
    )
    return parser.parse_args()


def load_replay_csv(path: Path) -> tuple[list[str], list[float], list[list[float]]]:
    with path.open(newline="") as file:
        reader = csv.reader(file)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: empty CSV")
        if len(header) < 3 or (len(header) - 1) % 2 != 0:
            raise ValueError(
                f"{path}: expected time,q0,dq0,... header, got {len(header)} columns"
            )

        joint_count = (len(header) - 1) // 2
        times: list[float] = []
        positions: list[list[float]] = []
        for line_no, row in enumerate(reader, start=2):
            if not row:
                continue
            if len(row) != len(header):
                raise ValueError(
                    f"{path}:{line_no}: expected {len(header)} columns, got {len(row)}"
                )
            values = [float(cell) for cell in row]
            times.append(values[0])
            positions.append([values[1 + 2 * i] for i in range(joint_count)])

    if len(times) < 3:
        raise ValueError(f"{path}: need at least 3 frames, got {len(times)}")
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            raise ValueError(f"{path}: timestamps must be strictly increasing")
    return header, times, positions


def alpha_from_dt(dt: float, cutoff_hz: float) -> float:
    if cutoff_hz <= 0.0:
        return 1.0
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (tau + dt)


def ema_pass(values: list[float], times: list[float], cutoff_hz: float) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    prev = values[0]
    for i in range(1, len(values)):
        dt = times[i] - times[i - 1]
        alpha = alpha_from_dt(dt, cutoff_hz)
        prev = prev + alpha * (values[i] - prev)
        out.append(prev)
    return out


def zero_phase_lowpass(values: list[float], times: list[float], cutoff_hz: float) -> list[float]:
    if cutoff_hz <= 0.0:
        return values[:]
    forward = ema_pass(values, times, cutoff_hz)
    reverse_times = [-t for t in reversed(times)]
    backward = ema_pass(list(reversed(forward)), reverse_times, cutoff_hz)
    return list(reversed(backward))


def preserve_endpoints(raw: list[float], smooth: list[float]) -> list[float]:
    if len(raw) != len(smooth) or len(raw) < 2:
        return smooth
    start_error = raw[0] - smooth[0]
    end_error = raw[-1] - smooth[-1]
    denom = len(raw) - 1
    corrected: list[float] = []
    for i, value in enumerate(smooth):
        s = i / denom
        corrected.append(value + (1.0 - s) * start_error + s * end_error)
    return corrected


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(col) for col in zip(*matrix)]


def smooth_positions(
    times: list[float],
    positions: list[list[float]],
    cutoff_hz: float,
    preserve: bool,
) -> list[list[float]]:
    smoothed_columns: list[list[float]] = []
    for raw_col in transpose(positions):
        smooth_col = zero_phase_lowpass(raw_col, times, cutoff_hz)
        if preserve:
            smooth_col = preserve_endpoints(raw_col, smooth_col)
        smoothed_columns.append(smooth_col)
    return transpose(smoothed_columns)


def recompute_velocities(times: list[float], positions: list[list[float]]) -> list[list[float]]:
    joint_count = len(positions[0])
    velocities = [[0.0 for _ in range(joint_count)] for _ in positions]
    last = len(times) - 1

    for j in range(joint_count):
        velocities[0][j] = (positions[1][j] - positions[0][j]) / (times[1] - times[0])
        velocities[last][j] = (positions[last][j] - positions[last - 1][j]) / (
            times[last] - times[last - 1]
        )
        for i in range(1, last):
            velocities[i][j] = (positions[i + 1][j] - positions[i - 1][j]) / (
                times[i + 1] - times[i - 1]
            )
    return velocities


def smooth_velocities(
    times: list[float], velocities: list[list[float]], cutoff_hz: float
) -> list[list[float]]:
    if cutoff_hz <= 0.0:
        return velocities
    smoothed_columns = [
        zero_phase_lowpass(raw_col, times, cutoff_hz) for raw_col in transpose(velocities)
    ]
    return transpose(smoothed_columns)


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def clamp_velocities(
    times: list[float],
    velocities: list[list[float]],
    max_velocity: float,
    max_acceleration: float,
) -> list[list[float]]:
    joint_count = len(velocities[0])
    out = [row[:] for row in velocities]

    if max_velocity > 0.0:
        for row in out:
            for j in range(joint_count):
                row[j] = clamp(row[j], max_velocity)

    if max_acceleration > 0.0:
        for j in range(joint_count):
            for i in range(1, len(out)):
                dt = times[i] - times[i - 1]
                max_delta = max_acceleration * dt
                delta = out[i][j] - out[i - 1][j]
                out[i][j] = out[i - 1][j] + clamp(delta, max_delta)
            for i in range(len(out) - 2, -1, -1):
                dt = times[i + 1] - times[i]
                max_delta = max_acceleration * dt
                delta = out[i][j] - out[i + 1][j]
                out[i][j] = out[i + 1][j] + clamp(delta, max_delta)
    return out


def write_replay_csv(
    path: Path,
    header: list[str],
    times: list[float],
    positions: list[list[float]],
    velocities: list[list[float]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        for t, q_row, dq_row in zip(times, positions, velocities):
            row: list[str] = [f"{t:.9f}"]
            for q, dq in zip(q_row, dq_row):
                row.extend([f"{q:.9f}", f"{dq:.9f}"])
            writer.writerow(row)


def max_abs_delta(raw: list[list[float]], smooth: list[list[float]]) -> float:
    return max(
        abs(a - b)
        for raw_row, smooth_row in zip(raw, smooth)
        for a, b in zip(raw_row, smooth_row)
    )


def max_abs(values: list[list[float]]) -> float:
    return max(abs(value) for row in values for value in row)


def main() -> int:
    args = parse_args()
    input_path = args.input.expanduser().resolve()
    output_path = (
        args.output.expanduser().resolve()
        if args.output
        else input_path.with_name(f"{input_path.stem}_smooth.csv")
    )

    if not input_path.exists():
        print(f"[smooth] input not found: {input_path}", file=sys.stderr)
        return 1
    if input_path == output_path:
        print("[smooth] output must not overwrite input", file=sys.stderr)
        return 1

    header, times, positions = load_replay_csv(input_path)
    smooth_q = smooth_positions(
        times,
        positions,
        args.cutoff_hz,
        preserve=not args.no_preserve_endpoints,
    )
    smooth_dq = recompute_velocities(times, smooth_q)
    smooth_dq = smooth_velocities(times, smooth_dq, args.velocity_cutoff_hz)
    smooth_dq = clamp_velocities(
        times,
        smooth_dq,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
    )

    if not args.no_zero_end_velocity:
        for j in range(len(smooth_dq[0])):
            smooth_dq[0][j] = 0.0
            smooth_dq[-1][j] = 0.0

    write_replay_csv(output_path, header, times, smooth_q, smooth_dq)

    print(f"[smooth] input : {input_path}")
    print(f"[smooth] output: {output_path}")
    print(f"[smooth] frames: {len(times)}, joints: {(len(header) - 1) // 2}")
    print(f"[smooth] position cutoff: {args.cutoff_hz:.3f} Hz")
    print(f"[smooth] velocity cutoff: {args.velocity_cutoff_hz:.3f} Hz")
    print(f"[smooth] max |q_raw - q_smooth|: {max_abs_delta(positions, smooth_q):.6f} rad")
    print(f"[smooth] max |dq_smooth|: {max_abs(smooth_dq):.6f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
