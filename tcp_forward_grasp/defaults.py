"""Defaults for horizontal forward grasp after red-ball XY align.

Default approach is along the *robot base* frame (not TCP-Z, not base-Z descent).
Distance is a manual fixed length (AprilTag auto-distance not used yet).
"""

from __future__ import annotations

import os
from pathlib import Path

from . import paths

LEFT_CAN = os.environ.get("OPENARM_LEFT_FOLLOWER_CAN", "follower_left")
RIGHT_CAN = os.environ.get("OPENARM_RIGHT_FOLLOWER_CAN", "follower_right")
SUDO_PASSWORD = os.environ.get("OPENARM_SUDO_PASSWORD", "knowin")

# Manual fixed approach distance after vision XY align on the red ball.
GRASP_APPROACH_M = float(os.environ.get("OPENARM_GRASP_APPROACH_M", os.environ.get("OPENARM_TCP_APPROACH_M", "0.065")))

# base = robot base axes (what you want as "forward"); tcp = EE frame columns.
APPROACH_FRAME = os.environ.get("OPENARM_APPROACH_FRAME", "base").lower()  # base|tcp
# For base frame: x|y|z are base axes. Existing retreat uses base -X, so "forward"
# toward the tray is often base +X; flip with APPROACH_SIGN=-1 if reversed on hardware.
APPROACH_AXIS = os.environ.get("OPENARM_APPROACH_AXIS", os.environ.get("OPENARM_TCP_APPROACH_AXIS", "x")).lower()
APPROACH_SIGN = float(os.environ.get("OPENARM_APPROACH_SIGN", os.environ.get("OPENARM_TCP_APPROACH_SIGN", "1.0")))

# Legacy aliases used by earlier TCP-only knobs
TCP_APPROACH_AXIS = APPROACH_AXIS
TCP_APPROACH_SIGN = APPROACH_SIGN

CAMERA_INDEX = int(os.environ.get("OPENARM_LEFT_CAMERA_INDEX", "6"))


def resolve_path(env_key: str, fallback: Path) -> Path:
    raw = os.environ.get(env_key, "").strip()
    if raw:
        return Path(raw).expanduser().resolve()
    return fallback.resolve()
