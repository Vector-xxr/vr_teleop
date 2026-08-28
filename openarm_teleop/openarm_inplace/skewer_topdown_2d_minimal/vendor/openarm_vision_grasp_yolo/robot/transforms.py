"""Small JSON and homogeneous-transform utilities for OpenArm vision grasping.

All transforms are 4x4 matrices in meters. Validation is intentionally strict so
bad hand-eye, FK, or vision outputs fail before an unsafe replay is generated.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    def _round(value: Any) -> Any:
        if isinstance(value, np.ndarray):
            return _round(value.tolist())
        if isinstance(value, dict):
            return {k: _round(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [_round(v) for v in value]
        if isinstance(value, (bool, np.bool_)):
            return bool(value)
        if isinstance(value, (float, np.floating)):
            return None if not np.isfinite(float(value)) else round(float(value), 8)
        if isinstance(value, (int, np.integer)):
            return int(value)
        return value

    with path.open("w", encoding="utf-8") as f:
        json.dump(_round(data), f, indent=2, ensure_ascii=False)


def validate_T(T: np.ndarray, name: str = "T") -> None:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise RuntimeError(f"{name} is not 4x4: {T.shape}")
    if not np.all(np.isfinite(T)):
        raise RuntimeError(f"{name} contains NaN/Inf")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise RuntimeError(f"{name} last row must be [0,0,0,1]")
    R = T[:3, :3]
    det = float(np.linalg.det(R))
    ortho = float(np.linalg.norm(R.T @ R - np.eye(3)))
    if abs(det - 1.0) > 1e-3 or ortho > 1e-2:
        raise RuntimeError(f"{name} rotation invalid: det={det:.6f}, ortho={ortho:.6g}")


def load_matrix4x4(path: Path, key: str) -> np.ndarray:
    data = load_json(path)
    if key not in data:
        raise RuntimeError(f"{path} missing key: {key}")
    T = np.asarray(data[key], dtype=np.float64)
    validate_T(T, key)
    return T


def save_matrix4x4(path: Path, key: str, T: np.ndarray, extra: dict[str, Any] | None = None) -> None:
    validate_T(T, key)
    payload = dict(extra or {})
    payload[key] = np.asarray(T, dtype=np.float64).tolist()
    save_json(path, payload)


def transform_cam_grasp_to_base(
    T_base_ee_capture: np.ndarray,
    T_ee_cam: np.ndarray,
    T_cam_grasp: np.ndarray,
) -> np.ndarray:
    """Same formula as Dobot project: T_base = T_base_ee @ T_ee_cam @ T_cam."""
    validate_T(T_base_ee_capture, "T_base_ee_capture")
    validate_T(T_ee_cam, "T_ee_cam")
    validate_T(T_cam_grasp, "T_cam_grasp")
    return T_base_ee_capture @ T_ee_cam @ T_cam_grasp


def rotation_error_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    R_delta = R1.T @ R2
    cos_angle = float(np.clip((np.trace(R_delta) - 1.0) * 0.5, -1.0, 1.0))
    return float(math.degrees(math.acos(cos_angle)))


def pose_delta(T_from: np.ndarray, T_to: np.ndarray) -> dict[str, Any]:
    validate_T(T_from, "T_from")
    validate_T(T_to, "T_to")
    return {
        "delta_xyz_m": (T_to[:3, 3] - T_from[:3, 3]).tolist(),
        "translation_distance_m": float(np.linalg.norm(T_to[:3, 3] - T_from[:3, 3])),
        "rotation_error_deg": rotation_error_deg(T_from[:3, :3], T_to[:3, :3]),
    }
