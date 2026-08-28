#!/usr/bin/env python3
"""Convert legacy KER record CSV → Quest dual-merged column layout.

Column remap only — does NOT upsample / interpolate (preserves record↔replay).
For playback use: ./run_ker_replay.sh <converted.csv>
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


QUEST_HEADER = ["time"] + [
    col for i in range(16) for col in (f"q{i}", f"dq{i}", f"tau{i}")
]

LEGACY_L_CMD = [f"L_cmd_j{i}" for i in range(1, 8)] + ["L_cmd_grip"]
LEGACY_R_CMD = [f"R_cmd_j{i}" for i in range(1, 8)] + ["R_cmd_grip"]
LEGACY_L_OBS = [f"L_obs_j{i}" for i in range(1, 8)] + ["L_obs_grip"]
LEGACY_R_OBS = [f"R_obs_j{i}" for i in range(1, 8)] + ["R_obs_grip"]


def convert(src: Path, dst: Path, *, use_obs: bool = False) -> int:
    with src.open(newline="", encoding="utf-8") as fin:
        reader = csv.DictReader(fin)
        fields = reader.fieldnames or []
        if "time" in fields and "q0" in fields:
            raise SystemExit(f"already Quest-format: {src}")
        need = LEGACY_L_CMD + LEGACY_R_CMD + ["timestamp_ns"]
        missing = [c for c in need if c not in fields]
        if missing:
            raise SystemExit(f"{src} missing columns: {missing}")

        l_cols = LEGACY_L_OBS if use_obs else LEGACY_L_CMD
        r_cols = LEGACY_R_OBS if use_obs else LEGACY_R_CMD

        dst.parent.mkdir(parents=True, exist_ok=True)
        n = 0
        t0 = None
        with dst.open("w", newline="", encoding="utf-8") as fout:
            writer = csv.writer(fout)
            writer.writerow(QUEST_HEADER)
            for row in reader:
                ts = int(float(row["timestamp_ns"]))
                if t0 is None:
                    t0 = ts
                t_s = (ts - t0) * 1e-9
                q16 = [float(row[c]) for c in l_cols] + [float(row[c]) for c in r_cols]
                out = [f"{t_s:.6f}"]
                for q in q16:
                    out.extend([f"{q:.9f}", "0.000000000", "0.000000000"])
                writer.writerow(out)
                n += 1
    return n


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("src", type=Path)
    p.add_argument("-o", "--output", type=Path, default=None)
    p.add_argument(
        "--obs",
        action="store_true",
        help="Use measured obs joints instead of cmd (default: cmd).",
    )
    args = p.parse_args()
    src = args.src
    if args.output is None:
        # Place next to source in Quest layout
        out = src.parent / "01" / "real_teleop_dual_merged.csv"
        if src.name != "1.csv" and src.stem.isdigit():
            out = src.parent / f"{int(src.stem):02d}" / "real_teleop_dual_merged.csv"
    else:
        out = args.output
    n = convert(src, out, use_obs=args.obs)
    print(f"[convert] {src} → {out}  rows={n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
