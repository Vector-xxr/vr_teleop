#!/usr/bin/env python3
"""Prepare paired left/right replay directories for dual-arm real replay.

Accepted input:
- A single session directory containing a synchronized merged file:
    real_teleop_dual_merged.csv
- A single session directory containing split files:
    real_teleop_split/right/real_teleop_right.csv
    real_teleop_split/left/real_teleop_left.csv
- A sessions root directory containing multiple such session directories.

The script copies each paired action into temporary left/right sequence directories
with numeric prefixes so CsvReplay::load_directory() can execute them in order and
generate inter-action transitions.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import tempfile
from pathlib import Path


MERGED_CANDIDATES = (
    Path("real_teleop_dual_merged.csv"),
    Path("dual_merged.csv"),
    Path("merged.csv"),
)
RIGHT_CANDIDATES = (
    Path("real_teleop_split/right/real_teleop_right.csv"),
    Path("right/real_teleop_right.csv"),
    Path("real_teleop_right.csv"),
    Path("right.csv"),
)
LEFT_CANDIDATES = (
    Path("real_teleop_split/left/real_teleop_left.csv"),
    Path("left/real_teleop_left.csv"),
    Path("real_teleop_left.csv"),
    Path("left.csv"),
)


def find_merged(session: Path) -> Path | None:
    return next((session / rel for rel in MERGED_CANDIDATES if (session / rel).is_file()), None)


def find_pair(session: Path) -> tuple[Path | None, Path | None, Path | None] | None:
    merged = find_merged(session)
    if merged is not None:
        return merged, None, None

    right = next((session / rel for rel in RIGHT_CANDIDATES if (session / rel).is_file()), None)
    left = next((session / rel for rel in LEFT_CANDIDATES if (session / rel).is_file()), None)
    if right is None or left is None:
        return None
    return None, right, left


def collect_pairs(root: Path) -> list[tuple[str, Path | None, Path | None, Path | None]]:
    direct = find_pair(root)
    if direct is not None:
        return [(root.name, direct[0], direct[1], direct[2])]

    pairs: list[tuple[str, Path | None, Path | None, Path | None]] = []
    for child in sorted([p for p in root.iterdir() if p.is_dir()], key=lambda p: p.name):
        pair = find_pair(child)
        if pair is not None:
            pairs.append((child.name, pair[0], pair[1], pair[2]))
    if not pairs:
        raise ValueError(f"no paired left/right recordings found under {root}")
    return pairs


def safe_name(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name) or "action"


def split_merged_csv(merged: Path, left_out: Path, right_out: Path) -> int:
    """Split a 16-joint merged CSV into synchronized left/right 8-joint CSVs."""
    expected = ["time"]
    for i in range(16):
        expected += [f"q{i}", f"dq{i}"]

    left_out.parent.mkdir(parents=True, exist_ok=True)
    right_out.parent.mkdir(parents=True, exist_ok=True)

    rows = 0
    with merged.open(newline="") as src, left_out.open("w", newline="") as left_file, right_out.open("w", newline="") as right_file:
        reader = csv.DictReader(src)
        missing = [name for name in expected if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{merged} missing columns: {', '.join(missing)}")

        header = ["time"]
        for i in range(8):
            header += [f"q{i}", f"dq{i}"]
        left_writer = csv.DictWriter(left_file, fieldnames=header)
        right_writer = csv.DictWriter(right_file, fieldnames=header)
        left_writer.writeheader()
        right_writer.writeheader()

        for row in reader:
            left_row = {"time": row["time"]}
            right_row = {"time": row["time"]}
            for i in range(8):
                left_row[f"q{i}"] = row[f"q{i}"]
                left_row[f"dq{i}"] = row[f"dq{i}"]
                right_row[f"q{i}"] = row[f"q{i + 8}"]
                right_row[f"dq{i}"] = row[f"dq{i + 8}"]
            left_writer.writerow(left_row)
            right_writer.writerow(right_row)
            rows += 1
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare dual-arm replay directory sequence.")
    parser.add_argument("root", type=Path)
    parser.add_argument("--output-root", type=Path, default=None)
    args = parser.parse_args()

    root = args.root.resolve()
    if not root.is_dir():
        raise ValueError(f"not a directory: {root}")

    pairs = collect_pairs(root)
    out_root = args.output_root or Path(tempfile.mkdtemp(prefix="openarm_dual_replay_"))
    out_root.mkdir(parents=True, exist_ok=True)
    right_dir = out_root / "right"
    left_dir = out_root / "left"
    merged_dir = out_root / "merged"
    right_dir.mkdir(parents=True, exist_ok=True)
    left_dir.mkdir(parents=True, exist_ok=True)
    merged_dir.mkdir(parents=True, exist_ok=True)

    merged_count = 0
    for idx, (name, merged, right, left) in enumerate(pairs, start=1):
        stem = f"{idx:03d}_{safe_name(name)}.csv"
        right_out = right_dir / stem
        left_out = left_dir / stem
        merged_out = merged_dir / stem
        print(f"[prepare] {idx:03d} {name}")
        if merged is not None:
            shutil.copy2(merged, merged_out)
            merged_count += 1
            rows = split_merged_csv(merged, left_out, right_out)
            print(f"[prepare]   merged: {merged}")
            print(f"[prepare]   split : synchronized time axis, rows={rows}")
        else:
            if right is None or left is None:
                raise ValueError(f"{name}: missing left/right replay inputs")
            shutil.copy2(right, right_out)
            shutil.copy2(left, left_out)
            print("[prepare]   merged: not found, using split files")
            print(f"[prepare]   right: {right}")
            print(f"[prepare]   left : {left}")
        print(f"[prepare]   out right: {right_out}")
        print(f"[prepare]   out left : {left_out}")
        if merged is not None:
            print(f"[prepare]   out merged: {merged_out}")

    print(f"RIGHT_DIR={right_dir}")
    print(f"LEFT_DIR={left_dir}")
    if merged_count == len(pairs):
        print(f"MERGED_DIR={merged_dir}")
    else:
        print("MERGED_DIR=")
    print(f"PAIR_COUNT={len(pairs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
