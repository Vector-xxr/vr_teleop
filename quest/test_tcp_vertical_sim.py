#!/usr/bin/env python3
"""Headless regression: continuous right-arm panel up must move hand_tcp vertically."""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INPLACE = ROOT / "openarm_teleop" / "openarm_inplace"
BINARY = INPLACE / "build" / "quest_cartesian_control"
XML = ROOT / "openarm_teleop" / "openarm_mujoco" / "v1" / "scene_table.xml"
URDF = (
    ROOT
    / "openarm_teleop"
    / "openarm_description"
    / "assets"
    / "robot"
    / "openarm_v1.0"
    / "urdf"
    / "example"
    / "v1.urdf"
)


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_for_text(path: Path, needle: str, proc: subprocess.Popen[str], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            text = path.read_text(errors="replace") if path.exists() else ""
            raise RuntimeError(f"controller exited {proc.returncode} before ready\n{text[-4000:]}")
        if path.exists() and needle in path.read_text(errors="replace"):
            return
        time.sleep(0.05)
    text = path.read_text(errors="replace") if path.exists() else ""
    raise TimeoutError(f"timed out waiting for {needle!r}\n{text[-4000:]}")


def send_jog(port: int, command: str) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(command.encode("utf-8"), ("127.0.0.1", port))


def robust_endpoint(rows: list[dict[str, str]], first: bool) -> tuple[float, float, float]:
    # Average a short stationary window to reject one-tick startup/stop noise.
    n = max(10, min(200, len(rows) // 10))
    window = rows[:n] if first else rows[-n:]
    return tuple(
        sum(float(row[f"tcp_{axis}"]) for row in window) / len(window)
        for axis in ("x", "y", "z")
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.5)
    parser.add_argument("--speed", type=float, default=0.03, help="panel speed, m/s")
    parser.add_argument("--max-ratio", type=float, default=0.05)
    parser.add_argument("--min-dz", type=float, default=0.075)
    parser.add_argument("--no-build", action="store_true")
    parser.add_argument("--keep", action="store_true", help="keep temporary logs")
    args = parser.parse_args()

    if not args.no_build:
        subprocess.run(
            ["cmake", "--build", str(INPLACE / "build"), "--target", "quest_cartesian_control", "-j2"],
            check=True,
        )
    for path in (BINARY, XML, URDF):
        if not path.exists():
            raise FileNotFoundError(path)

    run_dir = Path(tempfile.mkdtemp(prefix="openarm_tcp_vertical_"))
    stdout_path = run_dir / "controller.log"
    traj_dir = run_dir / "traj"
    jog_port, record_port = free_udp_port(), free_udp_port()
    quest_left, quest_right = free_udp_port(), free_udp_port()

    env = os.environ.copy()
    env.update(
        {
            "OPENARM_MUJOCO_HEADLESS": "1",
            "OPENARM_TELEOP_TRAJ_LOG": str(traj_dir),
            "OPENARM_HM_LOG_DIR": str(run_dir / "hm"),
            "OPENARM_HM_IK_TCP": "1",
            "OPENARM_HM_PANEL_BYPASS_SOFT": "1",
            "OPENARM_HM_PANEL_ORI_SCALE": "0.85",
            "OPENARM_HM_DEBUG_LOG": "0",
            "OPENARM_QUEST_JOG_SPEED": str(args.speed),
        }
    )
    cmd = [
        str(BINARY),
        "--sim",
        "--bimanual",
        "--xml",
        str(XML),
        "--urdf",
        str(URDF),
        "--quest-port-left",
        str(quest_left),
        "--quest-port-right",
        str(quest_right),
        "--quest-ui",
        "--quest-ui-record-port",
        str(record_port),
        "--quest-ui-jog-port",
        str(jog_port),
        "--teleop-mode",
        "human_mimic",
    ]

    proc: subprocess.Popen[str] | None = None
    try:
        with stdout_path.open("w", encoding="utf-8") as output:
            proc = subprocess.Popen(
                cmd,
                cwd=INPLACE,
                env=env,
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
            )
            wait_for_text(stdout_path, "tip IK frame=TCP", proc, timeout=30.0)
            wait_for_text(stdout_path, "jog UDP", proc, timeout=10.0)
            time.sleep(0.5)
            send_jog(jog_port, "hold,right,up")
            time.sleep(args.duration)
            send_jog(jog_port, "release,right,up")
            time.sleep(0.5)
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5.0)

        csv_path = traj_dir / "right_tcp.csv"
        if not csv_path.exists():
            raise RuntimeError(f"missing HM trajectory log: {csv_path}")
        with csv_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        if len(rows) < 100:
            raise RuntimeError(f"too few trajectory samples: {len(rows)}")

        p0 = robust_endpoint(rows, first=True)
        p1 = robust_endpoint(rows, first=False)
        delta = tuple(b - a for a, b in zip(p0, p1))
        dxy = (delta[0] ** 2 + delta[1] ** 2) ** 0.5
        dz = delta[2]
        ratio = dxy / max(abs(dz), 1e-9)
        passed = dz >= args.min_dz and ratio <= args.max_ratio
        print(
            f"delta_xyz=[{delta[0]:.6f}, {delta[1]:.6f}, {delta[2]:.6f}] m "
            f"|dxy|/|dz|={ratio:.6f} samples={len(rows)}"
        )
        print("PASS" if passed else "FAIL")
        if not passed:
            print(f"logs kept at: {run_dir}")
            return 1
        return 0
    finally:
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5.0)
        if args.keep:
            print(f"logs kept at: {run_dir}")
        elif run_dir.exists():
            shutil.rmtree(run_dir)


if __name__ == "__main__":
    raise SystemExit(main())
