#!/usr/bin/env python3
"""KER → MuJoCo teleop bridge (no dora).

Reads filtered joint angles from OpenArm KER over USB and writes them into
the local OpenArm bimanual MuJoCo scene for visual alignment checks.

Uses conda env ``openarm_teleop1`` (mujoco already installed) + vendored
``KER/vendor/openarm_ker`` (only needs ``pyusb``).

Example:
  /home/lan/miniconda3/envs/openarm_teleop1/bin/python \\
    /home/lan/openarm_teleop1/KER/ker_mujoco_teleop.py
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / "vendor"
if str(VENDOR) not in sys.path:
    sys.path.insert(0, str(VENDOR))

import mujoco
import mujoco.viewer
from openarm_ker.ker_stream import KERStream, CMD_STANDBY, CMD_STREAM


DEFAULT_XML = Path(
    os.environ.get(
        "OPENARM_MUJOCO_SCENE",
        str(
            ROOT.parent
            / "openarm_teleop"
            / "openarm_mujoco"
            / "v1"
            / "scene_table.xml"
        ),
    )
)



def map_range(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    lo, hi = (in_min, in_max) if in_min < in_max else (in_max, in_min)
    x = min(max(x, lo), hi)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def _arm7_from_ker(
    ch7: list[float], *, swap_j67: bool, negate_j6: bool
) -> np.ndarray:
    """KER 7 arm channels (deg) → OpenArm joint1..7 (rad).

    MuJoCo side is always written as joint1←[0] … joint7←[6] (not reversed).
    Some KER builds appear to emit wrist channels swapped vs OpenArm J6/J7;
    ``swap_j67`` exchanges those two before deg→rad.
    ``negate_j6`` flips OpenArm joint6 (wrist pitch / tip up-down) sign.
    """
    a = list(ch7)
    if swap_j67:
        a[5], a[6] = a[6], a[5]  # KER ch6/ch7 ↔ OpenArm joint6/joint7
    q = np.deg2rad(np.asarray(a, dtype=np.float64))
    if negate_j6:
        q[5] = -q[5]
    return q


def ker_angles_to_qpos(
    raw_deg: list[float],
    *,
    swap_j67: bool = True,
    negate_j6: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """Match dora-openarm-ker layout, with optional wrist channel fixes."""
    filtered = list(raw_deg)
    grip_r_deg = map_range(filtered[7], 0.0, -60.0, -60.0, 10.0)
    grip_l_deg = map_range(filtered[15], 0.0, 60.0, 60.0, -10.0)

    right = np.concatenate(
        [
            _arm7_from_ker(
                filtered[:7], swap_j67=swap_j67, negate_j6=negate_j6
            ),
            [np.deg2rad(grip_r_deg)],
        ]
    )
    left = np.concatenate(
        [
            _arm7_from_ker(
                filtered[8:15], swap_j67=swap_j67, negate_j6=negate_j6
            ),
            [np.deg2rad(grip_l_deg)],
        ]
    )
    return left, right


def grip_rad_to_finger(grip_rad: float, side: str) -> float:
    """Map KER gripper motor angle (rad) → MuJoCo finger slide [0, 0.044].

    Heuristic only — tune if finger direction looks inverted.
    """
    # Right KER grip mapped ~[-60°, +10°]; left ~[+60°, -10°] after map_range.
    # Treat more-closed as smaller finger opening.
    if side == "right":
        opened = map_range(np.rad2deg(grip_rad), -60.0, 10.0, 0.0, 0.044)
    else:
        opened = map_range(np.rad2deg(grip_rad), 60.0, -10.0, 0.0, 0.044)
    return float(np.clip(opened, 0.0, 0.044))


class ArmAddrs:
    def __init__(self, model: mujoco.MjModel, side: str):
        prefix = f"openarm_{side}_"
        self.arm = [
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}joint{i}")
            ]
            for i in range(1, 8)
        ]
        self.finger1 = model.jnt_qposadr[
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}finger_joint1"
            )
        ]
        self.finger2 = model.jnt_qposadr[
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}finger_joint2"
            )
        ]


def apply_arm(data: mujoco.MjData, addrs: ArmAddrs, q8: np.ndarray, side: str):
    for i in range(7):
        data.qpos[addrs.arm[i]] = q8[i]
    finger = grip_rad_to_finger(float(q8[7]), side)
    data.qpos[addrs.finger1] = finger
    data.qpos[addrs.finger2] = finger


def parse_args():
    p = argparse.ArgumentParser(description="KER USB → MuJoCo OpenArm teleop")
    p.add_argument("--xml", type=Path, default=DEFAULT_XML, help="MuJoCo scene XML")
    p.add_argument("--hz", type=float, default=100.0, help="Viewer/control rate")
    p.add_argument(
        "--print-every",
        type=float,
        default=1.0,
        help="Seconds between status prints (0=off)",
    )
    p.add_argument(
        "--dummy",
        action="store_true",
        help="No KER: animate sine joints to verify MuJoCo mapping only",
    )
    p.add_argument(
        "--swap-j67",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Swap KER wrist channels onto OpenArm joint6/joint7 (default: on).",
    )
    p.add_argument(
        "--negate-j6",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Negate OpenArm joint6 (tip up/down) (default: off).",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    xml = args.xml.expanduser().resolve()
    if not xml.is_file():
        print(f"[ker-sim] XML not found: {xml}", file=sys.stderr)
        return 2

    print(f"[ker-sim] loading {xml}")
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    left_addr = ArmAddrs(model, "left")
    right_addr = ArmAddrs(model, "right")

    stream = None
    if not args.dummy:
        print("[ker-sim] connecting KER USB 303a:4002 ...")
        stream = KERStream()
        stream.connect()
        stream.send_command(CMD_STREAM)
        print(f"[ker-sim] KER metadata: {stream.metadata}")
        print(
            f"[ker-sim] swap_j67={args.swap_j67} negate_j6={args.negate_j6}"
        )
    else:
        print("[ker-sim] DUMMY mode (no USB)")

    dt = 1.0 / max(args.hz, 1.0)
    last_print = 0.0
    t0 = time.time()
    n = 0

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                loop_t = time.time()
                if args.dummy:
                    t = loop_t - t0
                    left = np.zeros(8)
                    right = np.zeros(8)
                    left[0] = 0.3 * np.sin(t)
                    right[0] = 0.3 * np.sin(t + 0.5)
                    left[7] = np.deg2rad(0.0)
                    right[7] = np.deg2rad(0.0)
                else:
                    pkt = stream.recv()
                    if pkt is None or "angles" not in pkt:
                        time.sleep(0.001)
                        viewer.sync()
                        continue
                    left, right = ker_angles_to_qpos(
                        pkt["angles"],
                        swap_j67=args.swap_j67,
                        negate_j6=args.negate_j6,
                    )

                apply_arm(data, left_addr, left, "left")
                apply_arm(data, right_addr, right, "right")
                mujoco.mj_forward(model, data)
                viewer.sync()
                n += 1

                now = time.time()
                if args.print_every > 0 and now - last_print >= args.print_every:
                    print(
                        f"[ker-sim] n={n} "
                        f"Lj1={left[0]:+.3f} Rj1={right[0]:+.3f} "
                        f"Lg={left[7]:+.3f} Rg={right[7]:+.3f} "
                        f"Lf={data.qpos[left_addr.finger1]:.4f} "
                        f"Rf={data.qpos[right_addr.finger1]:.4f}"
                    )
                    last_print = now

                sleep = dt - (time.time() - loop_t)
                if sleep > 0:
                    time.sleep(sleep)
    finally:
        if stream is not None:
            try:
                stream.send_command(CMD_STANDBY)
            except Exception:
                pass
            stream.close()
            print("[ker-sim] KER disconnected")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
