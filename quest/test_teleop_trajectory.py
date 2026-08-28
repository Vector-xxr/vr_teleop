#!/usr/bin/env python3
"""Record and visualize OpenArm Quest teleop TCP / joint trajectories.

The C++ controller writes CSV logs when OPENARM_TELEOP_TRAJ_LOG is set.

Usage:
  # One-shot sim test (starts bridge + MuJoCo bimanual, logs, plots on Ctrl+C):
  ./test_teleop_trajectory.sh

  # Real hardware bimanual:
  ./test_teleop_trajectory.sh --real

  # Plot existing logs only:
  python test_teleop_trajectory.py plot --dir /tmp/teleop_traj_xxx
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib is required. Install with: pip install matplotlib"
    ) from exc


def load_arm_csv(path: Path) -> Dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty log: {path}")

    def col(name: str) -> np.ndarray:
        return np.array([float(row[name]) for row in rows], dtype=float)

    data: Dict[str, np.ndarray] = {
        "t": col("t_s"),
        "state": np.array([row["state"] for row in rows], dtype=object),
        "grip": col("grip"),
        "tcp_x": col("tcp_x"),
        "tcp_y": col("tcp_y"),
        "tcp_z": col("tcp_z"),
        "tgt_x": col("tgt_x"),
        "tgt_y": col("tgt_y"),
        "tgt_z": col("tgt_z"),
    }

    joint_cols = sorted(
        [key for key in rows[0].keys() if key.startswith("q") and key[1:].isdigit()],
        key=lambda name: int(name[1:]),
    )
    for prefix in ("q", "ref", "dq"):
        cols = sorted(
            [key for key in rows[0].keys() if key.startswith(prefix) and key[len(prefix) :].isdigit()],
            key=lambda name: int(name[len(prefix) :]),
        )
        if cols:
            data[prefix] = np.stack([col(name) for name in cols], axis=1)

    data["path"] = np.array([str(path)])
    return data


def discover_logs(log_dir: Path) -> Dict[str, Dict[str, np.ndarray]]:
    logs: Dict[str, Dict[str, np.ndarray]] = {}
    for csv_path in sorted(log_dir.glob("*_tcp.csv")):
        label = csv_path.stem.replace("_tcp", "")
        logs[label] = load_arm_csv(csv_path)
    if not logs:
        raise FileNotFoundError(f"no *_tcp.csv found in {log_dir}")
    return logs


def summarize_arm(label: str, data: Dict[str, np.ndarray]) -> str:
  t = data["t"]
  dt = np.diff(t)
  tcp = np.stack([data["tcp_x"], data["tcp_y"], data["tcp_z"]], axis=1)
  d_tcp = np.linalg.norm(np.diff(tcp, axis=0), axis=1)

  lines = [
      f"[{label}] samples={len(t)} duration={t[-1]-t[0]:.2f}s rate~{len(t)/max(t[-1]-t[0],1e-6):.1f}Hz",
      f"  TCP range X[{data['tcp_x'].min():.3f},{data['tcp_x'].max():.3f}] "
      f"Y[{data['tcp_y'].min():.3f},{data['tcp_y'].max():.3f}] "
      f"Z[{data['tcp_z'].min():.3f},{data['tcp_z'].max():.3f}]",
      f"  max |ΔTCP|/step={d_tcp.max():.5f} m, max |Δt|={dt.max()*1e3:.2f} ms",
  ]

  if "q" in data:
      q = data["q"]
      ref = data["ref"]
      dq = data["dq"]
      joint_step = np.abs(np.diff(q, axis=0)).max(axis=1)
      ref_gap = np.abs(ref - q).max(axis=1)
      lines.append(
          f"  joints max |Δq|/step={joint_step.max():.4f} rad, max |ref-q|={ref_gap.max():.4f} rad"
      )
      lines.append(
          f"  joint velocity max={np.abs(dq).max(axis=0)}"
      )
  active = np.sum(data["state"] == "active")
  degraded = np.sum(data["state"] == "degraded")
  lines.append(f"  states: active={active} degraded={degraded} grip_on={int(data['grip'].sum())}")
  return "\n".join(lines)


def plot_logs(log_dir: Path, save: Path | None = None, show: bool = True) -> None:
    logs = discover_logs(log_dir)

    print("=== Trajectory summary ===")
    for label, data in logs.items():
        print(summarize_arm(label, data))
        print()

    n_arms = len(logs)
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"OpenArm teleop trajectory: {log_dir}", fontsize=12)

    # 3D TCP paths
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    colors = {"left": "tab:blue", "right": "tab:orange"}
    for label, data in logs.items():
        c = colors.get(label, "tab:green")
        ax3d.plot(data["tcp_x"], data["tcp_y"], data["tcp_z"], label=f"{label} tcp", color=c, lw=1)
        ax3d.plot(data["tgt_x"], data["tgt_y"], data["tgt_z"], "--", color=c, alpha=0.5, lw=0.8)
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("TCP actual (solid) vs target (dashed)")
    ax3d.legend()

    # TCP axes vs time
    ax_tcp = fig.add_subplot(2, 2, 2)
    axis_names = ("x", "y", "z")
    for label, data in logs.items():
        for i, axis in enumerate(axis_names):
            ax_tcp.plot(
                data["t"],
                data[f"tcp_{axis}"],
                label=f"{label}.{axis}",
                alpha=0.85,
                lw=0.9,
            )
    ax_tcp.set_xlabel("time (s)")
    ax_tcp.set_ylabel("TCP position (m)")
    ax_tcp.set_title("TCP position per axis")
    ax_tcp.grid(True, alpha=0.3)
    ax_tcp.legend(fontsize=8, ncol=2)

    # Joint positions (first arm with joints, or left)
    ax_j = fig.add_subplot(2, 2, 3)
    label0 = "left" if "left" in logs else next(iter(logs))
    data0 = logs[label0]
    if "q" in data0:
        for j in range(data0["q"].shape[1]):
            ax_j.plot(data0["t"], data0["q"][:, j], lw=0.9, label=f"q{j}")
            ax_j.plot(data0["t"], data0["ref"][:, j], "--", lw=0.7, alpha=0.6)
    ax_j.set_title(f"Joint q solid / ref dashed ({label0})")
    ax_j.set_xlabel("time (s)")
    ax_j.set_ylabel("rad")
    ax_j.grid(True, alpha=0.3)
    if data0.get("q") is not None and data0["q"].shape[1] <= 8:
        ax_j.legend(fontsize=7, ncol=2)

    # TCP velocity magnitude (smoothness)
    ax_v = fig.add_subplot(2, 2, 4)
    for label, data in logs.items():
        tcp = np.stack([data["tcp_x"], data["tcp_y"], data["tcp_z"]], axis=1)
        dt = np.diff(data["t"])
        dt[dt <= 0] = np.nan
        vel = np.linalg.norm(np.diff(tcp, axis=0), axis=1) / dt
        ax_v.plot(data["t"][1:], vel, label=f"{label} |v|", lw=0.9)
    ax_v.set_xlabel("time (s)")
    ax_v.set_ylabel("TCP speed (m/s)")
    ax_v.set_title("TCP speed (continuity check)")
    ax_v.grid(True, alpha=0.3)
    ax_v.legend()

    fig.tight_layout()
    if save is not None:
        fig.savefig(save, dpi=150)
        print(f"Saved plot: {save}")
    if show:
        plt.show()
    else:
        plt.close(fig)

    # Per-arm joint grid if bimanual
    if n_arms >= 2:
        fig2, axes = plt.subplots(2, 4, figsize=(16, 6), sharex=True)
        fig2.suptitle("Per-joint signals (q solid, ref dashed)")
        for row, (label, data) in enumerate(logs.items()):
            if "q" not in data:
                continue
            for j in range(min(4, data["q"].shape[1])):
                ax = axes[row, j]
                ax.plot(data["t"], data["q"][:, j], lw=0.9)
                ax.plot(data["t"], data["ref"][:, j], "--", alpha=0.6, lw=0.7)
                ax.set_title(f"{label} j{j}")
                ax.grid(True, alpha=0.3)
        for ax in axes[-1]:
            ax.set_xlabel("time (s)")
        fig2.tight_layout()
        if save is not None:
            joint_png = save.with_name(save.stem + "_joints" + save.suffix)
            fig2.savefig(joint_png, dpi=150)
            print(f"Saved plot: {joint_png}")
        if show:
            plt.show()
        else:
            plt.close(fig2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot OpenArm Quest teleop trajectory logs")
    parser.add_argument(
        "command",
        nargs="?",
        default="plot",
        choices=("plot",),
        help="only 'plot' is needed; recording is done by the shell wrapper",
    )
    parser.add_argument("--dir", required=True, type=Path, help="directory with *_tcp.csv logs")
    parser.add_argument("--save", type=Path, default=None, help="save PNG instead of only showing")
    parser.add_argument("--no-show", action="store_true", help="do not open GUI windows")
    args = parser.parse_args()

    if args.command == "plot":
        plot_logs(args.dir, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
