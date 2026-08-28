#!/usr/bin/env python3
"""Merge left/right real teleop recording CSVs into one 16-joint replay CSV.

Input files are the per-arm CSVs written by CsvRecorder:
    time,q0,dq0,tau0,...,q7,dq7,tau7
Legacy files without tau columns are also accepted.

Output order is compatible with the project 16-joint convention:
    time,
    q0,dq0,tau0,...,q7,dq7,tau7,       # left arm J1..J7 + gripper
    q8,dq8,tau8,...,q15,dq15,tau15     # right arm J1..J7 + gripper
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path


def load_recording(path: Path, joint_count: int = 8) -> tuple[list[float], list[list[float]], list[list[float]], list[list[float]]]:
    times: list[float] = []
    qs: list[list[float]] = []
    dqs: list[list[float]] = []
    taus: list[list[float]] = []
    skipped = 0
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        expected = ["time"]
        for i in range(joint_count):
            expected += [f"q{i}", f"dq{i}"]
        tau_columns = [f"tau{i}" for i in range(joint_count)]
        has_tau = all(name in (reader.fieldnames or []) for name in tau_columns)
        missing = [name for name in expected if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path} missing columns: {', '.join(missing)}")
        for line_no, row in enumerate(reader, start=2):
            try:
                if any(row.get(name) in (None, "") for name in expected):
                    skipped += 1
                    continue
                t = float(row["time"])
                q = [float(row[f"q{i}"]) for i in range(joint_count)]
                dq = [float(row[f"dq{i}"]) for i in range(joint_count)]
                tau = [float(row[f"tau{i}"]) for i in range(joint_count)] if has_tau else [0.0] * joint_count
            except (TypeError, ValueError):
                skipped += 1
                continue
            if not math.isfinite(t) or any(not math.isfinite(v) for v in q + dq + tau):
                skipped += 1
                continue
            if times and t <= times[-1]:
                continue
            times.append(t)
            qs.append(q)
            dqs.append(dq)
            taus.append(tau)
    if skipped:
        print(f"[merge][WARN] skipped {skipped} incomplete/invalid row(s) in {path}")
    if len(times) < 2:
        raise ValueError(f"{path}: not enough valid frames")
    return times, qs, dqs, taus


def interp(times: list[float], values: list[list[float]], t: float) -> list[float]:
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    hi = bisect.bisect_right(times, t)
    lo = hi - 1
    t0 = times[lo]
    t1 = times[hi]
    if t1 <= t0:
        return values[lo]
    a = (t - t0) / (t1 - t0)
    return [v0 + (v1 - v0) * a for v0, v1 in zip(values[lo], values[hi])]


def infer_hz(left_times: list[float], right_times: list[float], fallback: float) -> float:
    dts = []
    for ts in (left_times, right_times):
        for a, b in zip(ts, ts[1:]):
            dt = b - a
            if 1e-5 < dt < 1.0:
                dts.append(dt)
    if not dts:
        return fallback
    dts.sort()
    median_dt = dts[len(dts) // 2]
    hz = 1.0 / median_dt
    return max(1.0, min(1000.0, hz))


def merge(left_path: Path, right_path: Path, output_path: Path, hz: float | None, crop: str) -> None:
    lt, lq, ldq, ltau = load_recording(left_path)
    rt, rq, rdq, rtau = load_recording(right_path)

    if crop == "min":
        duration = min(lt[-1] - lt[0], rt[-1] - rt[0])
    elif crop == "max":
        duration = max(lt[-1] - lt[0], rt[-1] - rt[0])
    else:
        raise ValueError("--crop must be min or max")
    if duration <= 0:
        raise ValueError("merged duration is non-positive")

    out_hz = hz if hz is not None else infer_hz(lt, rt, 1000.0)
    dt = 1.0 / out_hz
    steps = int(math.floor(duration / dt))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["time"]
        for i in range(16):
            header += [f"q{i}", f"dq{i}", f"tau{i}"]
        writer.writerow(header)

        for step in range(steps + 1):
            t = min(step * dt, duration)
            lq_t = interp(lt, lq, lt[0] + t)
            ldq_t = interp(lt, ldq, lt[0] + t)
            ltau_t = interp(lt, ltau, lt[0] + t)
            rq_t = interp(rt, rq, rt[0] + t)
            rdq_t = interp(rt, rdq, rt[0] + t)
            rtau_t = interp(rt, rtau, rt[0] + t)
            row = [f"{t:.6f}"]
            for q, dq, tau in zip(lq_t, ldq_t, ltau_t):
                row += [f"{q:.9f}", f"{dq:.9f}", f"{tau:.9f}"]
            for q, dq, tau in zip(rq_t, rdq_t, rtau_t):
                row += [f"{q:.9f}", f"{dq:.9f}", f"{tau:.9f}"]
            writer.writerow(row)

    print(f"[merge] left : {left_path} frames={len(lt)} duration={lt[-1] - lt[0]:.3f}s")
    print(f"[merge] right: {right_path} frames={len(rt)} duration={rt[-1] - rt[0]:.3f}s")
    print(f"[merge] out  : {output_path} hz={out_hz:.3f} duration={duration:.3f}s rows={steps + 1}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Merge left/right OpenArm real teleop recordings.")
    parser.add_argument("--left", required=True, type=Path)
    parser.add_argument("--right", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--hz", type=float, default=None, help="output sampling rate; default infers from inputs")
    parser.add_argument("--crop", choices=("min", "max"), default="min", help="min keeps only overlapping duration")
    args = parser.parse_args()
    if args.hz is not None and args.hz <= 0:
        raise ValueError("--hz must be positive")
    merge(args.left, args.right, args.output, args.hz, args.crop)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
