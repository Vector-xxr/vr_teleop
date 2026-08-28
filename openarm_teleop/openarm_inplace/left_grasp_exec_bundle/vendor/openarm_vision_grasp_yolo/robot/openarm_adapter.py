"""OpenArm robot-side helpers for the vision grasp pipeline.

This module defines the right-arm frame convention, URDF path, joint naming,
and dense replay CSV writers consumed by `openarm_inplace` replay control.
It intentionally keeps Cartesian planning out of Python: poses are converted to
7-DOF joint waypoints through `scripts/solve_openarm_fk_ik.py`, then written as replay
CSV for the C++ SocketCAN controller.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Protocol, Sequence

import numpy as np

from .transforms import load_matrix4x4, validate_T


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = (
    WORKSPACE_ROOT
    / "openarm_description"
    / "assets"
    / "robot"
    / "openarm_v1.0"
    / "urdf"
    / "example"
    / "v1.urdf"
)


@dataclass(frozen=True)
class OpenArmFrames:
    """OpenArm frame convention used by this minimal project."""

    arm_side: str = "right_arm"
    base_frame: str = "openarm_body_link0"
    ee_frame: str = "auto"
    urdf_path: Path = DEFAULT_URDF

    def __post_init__(self) -> None:
        arm_side = "left_arm" if self.arm_side == "left_arm" else "right_arm"
        object.__setattr__(self, "arm_side", arm_side)
        if self.ee_frame in {"", "auto"}:
            side = "left" if arm_side == "left_arm" else "right"
            object.__setattr__(self, "ee_frame", f"openarm_{side}_hand_tcp")

    @property
    def joint_names(self) -> list[str]:
        side = "left" if self.arm_side == "left_arm" else "right"
        return [f"openarm_{side}_joint{i}" for i in range(1, 8)]


class PoseProvider(Protocol):
    def get_T_base_ee(self) -> np.ndarray:
        """Return current base->ee transform in meters."""


class IKSolver(Protocol):
    def solve(self, T_base_tcp: np.ndarray, seed_joints: list[float] | None = None) -> list[float]:
        """Return 7 joint positions in radians."""


class JsonPoseProvider:
    """Offline capture pose. Use this before live OpenArm FK is ready."""

    def __init__(self, path: Path, key: str = "T_base_ee") -> None:
        self.path = path
        self.key = key

    def get_T_base_ee(self) -> np.ndarray:
        return load_matrix4x4(self.path, self.key)


class NotImplementedFK:
    """Placeholder for live OpenArm FK from follower joint feedback."""

    def __init__(self, frames: OpenArmFrames) -> None:
        self.frames = frames

    def get_T_base_ee(self) -> np.ndarray:
        raise NotImplementedError(
            "OpenArm live FK is not wired yet.\n"
            "Need: follower 7 joint angles -> KDL/MoveIt FK -> "
            f"{self.frames.base_frame} -> {self.frames.ee_frame}.\n"
            "Temporary workaround: dump T_base_ee.json and use JsonPoseProvider."
        )


class NotImplementedIK:
    """Placeholder so we never pretend Cartesian motion is available."""

    def __init__(self, frames: OpenArmFrames) -> None:
        self.frames = frames

    def solve(self, T_base_tcp: np.ndarray, seed_joints: list[float] | None = None) -> list[float]:
        validate_T(T_base_tcp, "T_base_tcp")
        raise NotImplementedError(
            "OpenArm IK is not wired yet.\n"
            f"Need IK for base={self.frames.base_frame}, ee={self.frames.ee_frame}, "
            f"joints={self.frames.joint_names}.\n"
            "Recommended: MoveIt / TRAC-IK / KDL numerical IK."
        )


def write_single_arm_replay_csv(
    path: Path,
    joint_waypoints: Iterable[list[float]],
    gripper: float = 0.0,
    dt: float = 0.02,
    sample_period: float = 0.02,
    segment_durations: Sequence[float] | None = None,
) -> Path:
    """Write dense 8-DOF CSV for openarm_inplace unilateral_control --replay.

    CsvReplay samples rows as discrete references. Therefore each segment must be
    explicitly interpolated in the CSV; writing only start/end rows would create a
    step jump at the final timestamp.
    """

    path.parent.mkdir(parents=True, exist_ok=True)
    waypoints = list(joint_waypoints)
    if dt <= 0.0:
        raise RuntimeError(f"Replay segment duration must be positive, got {dt}")
    if sample_period <= 0.0:
        raise RuntimeError(f"Replay sample period must be positive, got {sample_period}")
    if segment_durations is not None and len(segment_durations) != max(0, len(waypoints) - 1):
        raise RuntimeError(
            f"Need {max(0, len(waypoints) - 1)} segment durations, got {len(segment_durations)}"
        )
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        header = ["time"]
        for i in range(8):
            header.extend([f"q{i}", f"dq{i}"])
        writer.writerow(header)
        if not waypoints:
            return path
        for q7 in waypoints:
            if len(q7) != 7:
                raise RuntimeError(f"Need 7 joints, got {len(q7)}")
        time_s = 0.0

        def write_row(t: float, q7: list[float], dq7: list[float]) -> None:
            row = [f"{t:.6f}"]
            for q, dq in zip([*q7, gripper], [*dq7, 0.0]):
                row.extend([f"{float(q):.8f}", f"{float(dq):.8f}"])
            writer.writerow(row)

        write_row(time_s, [float(v) for v in waypoints[0]], [0.0] * 7)
        durations = list(segment_durations) if segment_durations is not None else [dt] * max(0, len(waypoints) - 1)
        for segment_duration, start, goal in zip(durations, waypoints[:-1], waypoints[1:]):
            if segment_duration <= 0.0:
                raise RuntimeError(f"Replay segment duration must be positive, got {segment_duration}")
            start_q = [float(v) for v in start]
            goal_q = [float(v) for v in goal]
            delta = [b - a for a, b in zip(start_q, goal_q)]
            steps = max(1, int(round(segment_duration / sample_period)))
            for step in range(1, steps + 1):
                u = step / steps
                # Quintic smoothstep: zero velocity/acceleration at boundaries.
                s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
                ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
                q = [a + s * d for a, d in zip(start_q, delta)]
                dq = [(ds_du / segment_duration) * d for d in delta]
                time_s += segment_duration / steps
                write_row(time_s, q, dq)
    return path


# What changed vs Dobot
DOBOT_VS_OPENARM = {
    "GetPose / get_tcp_matrix": "follower joints + FK -> T_base_ee",
    "MovJ / MovL(x,y,z,rx,ry,rz)": "IK(T_base_tcp) -> q1..q7 -> openarm_inplace replay/UDP",
    "Tool / User frame": "OpenArmFrames + T_grasp_tcp + hand-eye T_ee_cam",
    "workspace mm": "workspace meters + joint limits",
}
