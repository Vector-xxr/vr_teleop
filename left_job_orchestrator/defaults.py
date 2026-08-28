"""Orchestrator defaults — grasp params come from tcp_forward_grasp."""

from __future__ import annotations

import os
from pathlib import Path

from . import paths

# TCP job API (client <-> this service). Replay ports come from align/flow defaults.
JOB_LISTEN_PORT = int(os.environ.get("OPENARM_SKEWER_JOB_LISTEN_PORT", "47925"))
# Aux push thread: status heartbeat on subscribed TCP connections.
SUBSCRIBE_HEARTBEAT_S = float(os.environ.get("OPENARM_LEFT_JOB_SUBSCRIBE_HEARTBEAT_S", "2"))
# Shared secret for JSON API (HMAC + token). Default: openarm-job-api (auth ON).
API_TOKEN = os.environ.get("OPENARM_LEFT_JOB_API_TOKEN", "").strip()

LEFT_CAN = os.environ.get("OPENARM_LEFT_FOLLOWER_CAN", "follower_left")
RIGHT_CAN = os.environ.get("OPENARM_RIGHT_FOLLOWER_CAN", "follower_right")
SUDO_PASSWORD = os.environ.get("OPENARM_SUDO_PASSWORD", "")

CAMERA_BACKEND = "opencv"
CAMERA_INDEX = int(os.environ.get("OPENARM_LEFT_CAMERA_INDEX", "6"))

# Thin overlay on top of tcp_forward_grasp.flow_loader.FLOW_CLI_DEFAULTS
ORCH_OVERRIDES: dict = {
    "execute": True,
    "dual_replay": True,
    "arm_side": "left_arm",
    "config": paths.LEFT_CONFIG,
    "steady_replay": paths.STEADY_CSV,
    "observe_replay": paths.DEFAULT_BEGIN_CSV,
    "right_follower_can": RIGHT_CAN,
    "left_follower_can": LEFT_CAN,
    "follower_can": LEFT_CAN,
    "sudo_password": SUDO_PASSWORD,
    "camera_backend": CAMERA_BACKEND,
    "camera_index": CAMERA_INDEX,
    "gripper_gate_file": paths.DEFAULT_GRIPPER_GATE_JSON,
    # Headless service default (override with OPENARM_LEFT_JOB_PREVIEW=1).
    "preview": False,
    "no_preview": True,
    "output_dir": paths.OUTPUT_ROOT,
}


def resolve_jobs_root() -> Path:
    env = os.environ.get("OPENARM_SKEWER_JOBS_ROOT", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    return paths.RECORDINGS_ROOT.resolve()


def resolve_steady_csv() -> Path:
    env = os.environ.get("OPENARM_STARTUP_CSV", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    return paths.STEADY_CSV.resolve()


def resolve_observe_csv() -> Path:
    """Grasp-start / begin pose CSV (legacy name: observe)."""
    env = (
        os.environ.get("OPENARM_LEFT_BEGIN_CSV", "").strip()
        or os.environ.get("OPENARM_LEFT_OBSERVE_CSV", "").strip()
    )
    if env:
        return Path(env).expanduser().resolve()
    return paths.DEFAULT_BEGIN_CSV.resolve()


def want_preview() -> bool:
    return os.environ.get("OPENARM_LEFT_JOB_PREVIEW", "").strip() in {"1", "true", "TRUE", "yes"}
