#!/usr/bin/env python3
"""Read CSV action files from recordings directory and send via UDP to leader.

This script mimics the real-hardware Live mode: it reads recorded action
trajectories and sends them as UDP joint references to the C++ MuJoCo teleop
process. The C++ side will set leader references from these UDP values, then
read leader actual state as follower reference — exactly matching the real
hardware recording pipeline.

The sending mechanism borrows from send_smooth_joint_motion.py: fixed-rate
timing loop with next_send scheduling, but instead of generating synthetic
trajectories, it reads frames from CSV files and matches their original
timestamps.

Usage:
    # Terminal 1: Start C++ MuJoCo with recording enabled
    OPENARM_RECORD_PATH=recordings/re_recorded.csv \
    script/run_cpp_mujoco_panel.sh

    # Terminal 2: Send action files as leader input
    python script/send_csv_actions.py recordings/

    # Or send a single file
    python script/send_csv_actions.py recordings/1hello.csv

    # Loop playback
    python script/send_csv_actions.py recordings/ --loop

    # Repeat N times
    python script/send_csv_actions.py recordings/ --repeat 3

    # Speed up 2x
    python script/send_csv_actions.py recordings/ --speed 2.0
"""

from __future__ import annotations

import argparse
import csv
import re
import socket
import sys
import time
from pathlib import Path


def extract_seq(filename: str) -> int:
    """Extract leading number from filename for sorting."""
    m = re.match(r"(\d+)", Path(filename).stem)
    return int(m.group(1)) if m else 0


def is_transition_file(filename: str) -> bool:
    """Check if file matches transition pattern {num}_{num}.csv."""
    return bool(re.match(r"^\d+_\d+\.csv$", Path(filename).name))


def load_csv_frames(path: Path) -> list[dict]:
    """Load frames from a CSV file.

    Each frame: {"time": float, "values": [q0,dq0,q1,dq1,...,q15,dq15]}
    """
    frames = []
    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return frames
        n_cols = len(header)
        for row in reader:
            if len(row) != n_cols:
                continue
            try:
                values = [float(v) for v in row]
            except ValueError:
                continue
            t = values[0]
            joint_values = values[1:]
            frames.append({"time": t, "values": joint_values})
    return frames


def find_ordered_files(dir_path: Path) -> list[Path]:
    """Find and sort all CSV files in directory, including transitions.

    Order: action files sorted by leading number, with transition files
    inserted between the corresponding action files.

    Example order: 0_1.csv, 1_action.csv, 1_2.csv, 2_action.csv, 2_3.csv, 3_action.csv
    """
    # Collect action files (non-transition) sorted by sequence number
    action_files = []
    for p in dir_path.glob("*.csv"):
        if not is_transition_file(p.name):
            action_files.append(p)
    action_files.sort(key=lambda p: extract_seq(p.name))

    if not action_files:
        return []

    # Build ordered list: initial transition + action1 + transition + action2 + ...
    ordered = []
    seqs = [extract_seq(f.name) for f in action_files]

    # Check for initial transition (0_{first_seq}.csv)
    first_seq = seqs[0]
    init_trans = dir_path / f"0_{first_seq}.csv"
    if init_trans.exists():
        ordered.append(init_trans)

    for i, (f, seq) in enumerate(zip(action_files, seqs)):
        ordered.append(f)
        # Check for transition to next action: {seq}_{next_seq}.csv
        if i + 1 < len(seqs):
            next_seq = seqs[i + 1]
            trans = dir_path / f"{seq}_{next_seq}.csv"
            if trans.exists():
                ordered.append(trans)

    return ordered


def interleave_position_velocity(q: list[float], dq: list[float]) -> list[float]:
    """Interleave position and velocity: [q0,dq0,q1,dq1,...]."""
    values: list[float] = []
    for position, velocity in zip(q, dq):
        values.extend([position, velocity])
    return values


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send recorded CSV action files via UDP as leader input."
    )
    parser.add_argument("path", help="CSV file or directory containing action files")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47910)
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed factor (default: 1.0)")
    parser.add_argument("--positions-only", action="store_true",
                        help="Send only 16 position values (no velocity)")
    parser.add_argument("--loop", action="store_true", help="Loop playback")
    parser.add_argument("--repeat", type=int, default=1,
                        help="Number of playback cycles")
    parser.add_argument("--print-every", type=float, default=2.0,
                        help="Print progress interval in seconds")
    args = parser.parse_args()

    path = Path(args.path)
    if not path.exists():
        print(f"[send_csv] path not found: {path}", file=sys.stderr)
        return 1

    # Collect files
    if path.is_file():
        files = [path]
    elif path.is_dir():
        files = find_ordered_files(path)
        if not files:
            print(f"[send_csv] no action CSV files found in: {path}", file=sys.stderr)
            return 1
    else:
        print(f"[send_csv] invalid path: {path}", file=sys.stderr)
        return 1

    # Load all frames and concatenate with time offsets
    all_frames = []
    time_offset = 0.0
    for f in files:
        frames = load_csv_frames(f)
        if not frames:
            print(f"[send_csv] no frames in {f.name}, skipping")
            continue
        # Shift timestamps so files play sequentially
        for frame in frames:
            frame["time"] += time_offset
        all_frames.extend(frames)
        duration = frames[-1]["time"] - frames[0]["time"]
        time_offset += duration
        print(f"[send_csv] loaded {len(frames)} frames from {f.name} "
              f"(duration {duration:.2f}s, total so far {time_offset:.2f}s)")

    if not all_frames:
        print("[send_csv] no frames loaded", file=sys.stderr)
        return 1

    total_duration = all_frames[-1]["time"]
    total_frames = len(all_frames)
    print(f"[send_csv] total {total_frames} frames, duration {total_duration:.2f}s")
    if args.positions_only:
        print("[send_csv] UDP payload: 16 positions only")
    else:
        print("[send_csv] UDP payload: 32 values, q/dq interleaved")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    cycle = 0
    try:
        while True:
            cycle += 1
            print(f"[send_csv] === cycle {cycle} ===")

            # Use the same timing mechanism as send_smooth_joint_motion.py
            start = time.monotonic()
            next_send = start
            next_print = start
            frame_idx = 0

            while frame_idx < total_frames:
                now = time.monotonic()
                elapsed = now - start

                # Advance to the frame matching current elapsed time (adjusted by speed)
                target_t = elapsed * args.speed
                while frame_idx < total_frames and all_frames[frame_idx]["time"] <= target_t:
                    frame = all_frames[frame_idx]
                    values = frame["values"]

                    if args.positions_only:
                        # Extract only positions: q0,q1,...,q15 (every other value starting from 0)
                        positions = [values[i] for i in range(0, len(values), 2)]
                        payload = ",".join(f"{v:.6f}" for v in positions).encode("ascii")
                    else:
                        # Send all q,dq interleaved as-is from CSV
                        payload = ",".join(f"{v:.6f}" for v in values).encode("ascii")

                    sock.sendto(payload, addr)
                    frame_idx += 1

                # Print progress periodically
                if now >= next_print:
                    progress = frame_idx / total_frames * 100
                    print(f"[send_csv] t={elapsed:6.2f}s/{total_duration/args.speed:.1f}s "
                          f"frame {frame_idx}/{total_frames} ({progress:.0f}%)")
                    next_print = now + args.print_every

                # Sleep until next frame time (like send_smooth_joint_motion.py)
                if frame_idx < total_frames:
                    next_frame_t = all_frames[frame_idx]["time"] / args.speed
                    next_send = start + next_frame_t
                    sleep_time = next_send - time.monotonic()
                    if sleep_time > 0:
                        time.sleep(min(sleep_time, 0.002))  # Cap sleep to stay responsive
                    else:
                        # We're behind, yield briefly
                        time.sleep(0.0001)

            print(f"[send_csv] cycle {cycle} complete")

            if not args.loop and cycle >= args.repeat:
                break

    except KeyboardInterrupt:
        print("\n[send_csv] interrupted")
    finally:
        # Send home position at the end (like send_smooth_joint_motion.py)
        home = [0.0] * 16
        if args.positions_only:
            payload = ",".join(f"{v:.6f}" for v in home).encode("ascii")
        else:
            home_values = interleave_position_velocity(home, [0.0] * 16)
            payload = ",".join(f"{v:.6f}" for v in home_values).encode("ascii")
        for _ in range(20):
            sock.sendto(payload, addr)
            time.sleep(0.005)
        sock.close()

    print("[send_csv] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
