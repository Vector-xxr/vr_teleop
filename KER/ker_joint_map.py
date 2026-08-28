"""Shared KER → OpenArm joint mapping (validated in MuJoCo)."""

from __future__ import annotations

import numpy as np


def map_range(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    lo, hi = (in_min, in_max) if in_min < in_max else (in_max, in_min)
    x = min(max(x, lo), hi)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def arm7_from_ker(
    ch7: list[float],
    *,
    swap_j67: bool = True,
    negate_j6: bool = False,
    negate_j7: bool = False,
) -> np.ndarray:
    """KER 7 arm channels (deg) → OpenArm joint1..7 (rad).

    With swap_j67: OpenArm j6 ← KER ch7, OpenArm j7 ← KER ch6.
    Tip up/down on hardware has been the j7 path after swap (negate_j7),
    not j6 — toggling only negate_j6 left tip direction unchanged.
    """
    a = list(ch7)
    if swap_j67:
        a[5], a[6] = a[6], a[5]
    q = np.deg2rad(np.asarray(a, dtype=np.float64))
    if negate_j6:
        q[5] = -q[5]
    if negate_j7:
        q[6] = -q[6]
    return q


def ker_angles_to_qpos8(
    raw_deg: list[float],
    *,
    swap_j67: bool = True,
    negate_j6: bool = False,
    negate_j7: bool = False,
    invert_left_grip: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (left_q8, right_q8): joints 1–7 + gripper (rad).

    Gripper (deg → later rad):
      right: KER 0(open)→-60, -60(close)→+10
      left:  KER 0(open)→-60, +60(close)→+10 when invert_left_grip
             (Enactic default was opposite and closes at rest on this HW)
    """
    filtered = list(raw_deg)
    grip_r_deg = map_range(filtered[7], 0.0, -60.0, -60.0, 10.0)
    if invert_left_grip:
        # Same open/close sense as right follower on this hardware.
        grip_l_deg = map_range(filtered[15], 0.0, 60.0, -60.0, 10.0)
    else:
        grip_l_deg = map_range(filtered[15], 0.0, 60.0, 60.0, -10.0)

    right = np.concatenate(
        [
            arm7_from_ker(
                filtered[:7],
                swap_j67=swap_j67,
                negate_j6=negate_j6,
                negate_j7=negate_j7,
            ),
            [np.deg2rad(grip_r_deg)],
        ]
    )
    left = np.concatenate(
        [
            arm7_from_ker(
                filtered[8:15],
                swap_j67=swap_j67,
                negate_j6=negate_j6,
                negate_j7=negate_j7,
            ),
            [np.deg2rad(grip_l_deg)],
        ]
    )
    return left, right
