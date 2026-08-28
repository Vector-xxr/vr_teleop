"""Camera grasp to OpenArm TCP target conversion.

The main formula is:
  T_base_tcp = T_base_ee @ T_ee_cam @ T_cam_grasp @ T_grasp_tcp

`safe_lift_m` creates a safe pregrasp target by lifting the visual grasp target
along OpenArm base Z. The final physical descent is handled later by
`scripts/run_safe_pregrasp_pipeline.py`.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from .openarm_adapter import OpenArmFrames
from .transforms import (
    load_json,
    load_matrix4x4,
    pose_delta,
    save_json,
    save_matrix4x4,
    transform_cam_grasp_to_base,
    validate_T,
)


def normalize(v: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm > 1e-12:
        return v / norm
    if fallback is not None:
        return normalize(np.asarray(fallback, dtype=np.float64))
    raise RuntimeError("Cannot normalize near-zero vector")


def should_level_small_object_grasp(mode: str, object_class: str | None) -> bool:
    mode = str(mode or "auto").strip().lower()
    if mode in {"off", "false", "no", "none"}:
        return False
    if mode in {"on", "true", "yes", "always"}:
        return True
    if mode != "auto":
        raise RuntimeError(f"Unknown level-small-object-grasp mode: {mode}")
    return str(object_class or "").strip().lower() in {"cake", "cup"}


def level_front_grasp_in_base(T_base_grasp: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    """
    Keep the grasp point, make the approach axis horizontal, and keep gripper roll level.

    Visual grasp frame convention used here:
      +Z = approach direction
      +Y = gripper vertical/up direction after leveling
      +X = jaw-width direction, horizontal and perpendicular to approach
    """
    T = np.asarray(T_base_grasp, dtype=np.float64).copy()
    base_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    old_R = np.asarray(T[:3, :3], dtype=np.float64)
    old_z = normalize(old_R[:, 2], np.array([1.0, 0.0, 0.0], dtype=np.float64))
    old_z_vertical = float(np.dot(old_z, base_up))
    z_axis = old_z - base_up * old_z_vertical
    if float(np.linalg.norm(z_axis)) < 1e-8:
        old_x = normalize(old_R[:, 0], np.array([1.0, 0.0, 0.0], dtype=np.float64))
        z_axis = old_x - base_up * float(np.dot(old_x, base_up))
    z_axis = normalize(z_axis, np.array([1.0, 0.0, 0.0], dtype=np.float64))
    y_axis = base_up
    x_axis = normalize(np.cross(y_axis, z_axis), np.array([0.0, 1.0, 0.0], dtype=np.float64))
    z_axis = normalize(np.cross(x_axis, y_axis), z_axis)
    T[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    meta = {
        "old_approach_axis": old_z.tolist(),
        "new_approach_axis": z_axis.tolist(),
        "old_approach_vertical_component": old_z_vertical,
        "new_approach_vertical_component": float(np.dot(z_axis, base_up)),
        "new_gripper_vertical_axis": y_axis.tolist(),
    }
    return T, meta


def load_grasp_poses(path: Path) -> dict[str, np.ndarray]:
    data = load_json(path)
    if "poses" in data and isinstance(data["poses"], dict):
        raw = data["poses"]
    elif "grasp_poses" in data and isinstance(data["grasp_poses"], dict):
        raw = data["grasp_poses"]
    elif "poses" in data and isinstance(data["poses"], list):
        raw = {}
        for item in data["poses"]:
            if not isinstance(item, dict) or "id" not in item:
                continue
            pose_id = str(item["id"])
            matrix = item.get("T_cam_pose") or item.get("T") or item.get("matrix") or item.get("transform")
            if matrix is not None:
                raw[pose_id] = matrix
    elif "grasp_poses" in data and isinstance(data["grasp_poses"], list):
        raw = {}
        for item in data["grasp_poses"]:
            if not isinstance(item, dict) or "id" not in item:
                continue
            pose_id = str(item["id"])
            matrix = item.get("T_cam_pose") or item.get("T") or item.get("matrix") or item.get("transform")
            if matrix is not None:
                raw[pose_id] = matrix
    else:
        raw = {k: v for k, v in data.items() if isinstance(v, list)}

    poses: dict[str, np.ndarray] = {}
    for pose_id, value in raw.items():
        matrix = value.get("T_cam_pose") if isinstance(value, dict) else value
        if matrix is None and isinstance(value, dict):
            matrix = value.get("T")
        if matrix is None and isinstance(value, dict):
            matrix = value.get("matrix") or value.get("transform")
        if matrix is None:
            continue
        T = np.asarray(matrix, dtype=np.float64)
        validate_T(T, pose_id)
        poses[pose_id] = T
    if not poses:
        raise RuntimeError(f"No grasp poses in {path}")
    return poses


def infer_pregrasp_id(poses: dict[str, np.ndarray], grasp_id: str) -> str:
    if grasp_id.startswith("grasp"):
        candidate = grasp_id.replace("grasp", "pregrasp", 1)
        if candidate in poses:
            return candidate
    fallback = f"pre{grasp_id}"
    if fallback in poses:
        return fallback
    pregrasp_ids = sorted(name for name in poses if "pregrasp" in name.lower())
    if len(pregrasp_ids) == 1:
        return pregrasp_ids[0]
    raise RuntimeError(f"Cannot infer pregrasp for {grasp_id}. Available: {sorted(poses)}")


def build_targets(
    T_base_ee_capture: np.ndarray,
    T_ee_cam: np.ndarray,
    poses_cam: dict[str, np.ndarray],
    grasp_id: str,
    safe_lift_m: float,
    T_grasp_tcp: np.ndarray | None = None,
    front_standoff_m: float = 0.15,
    object_class: str | None = None,
    level_small_object_grasp: str = "auto",
) -> dict[str, Any]:
    """Same transform chain as Dobot RobotController.execute_grasp, OpenArm frames."""

    if grasp_id not in poses_cam:
        raise RuntimeError(f"Missing {grasp_id}. Available: {sorted(poses_cam)}")
    pregrasp_id = infer_pregrasp_id(poses_cam, grasp_id)
    T_cam_pregrasp = poses_cam[pregrasp_id]
    T_cam_grasp = poses_cam[grasp_id]

    T_base_pregrasp = transform_cam_grasp_to_base(T_base_ee_capture, T_ee_cam, T_cam_pregrasp)
    T_base_grasp = transform_cam_grasp_to_base(T_base_ee_capture, T_ee_cam, T_cam_grasp)
    level_applied = should_level_small_object_grasp(level_small_object_grasp, object_class)
    level_meta: dict[str, Any] = {
        "mode": str(level_small_object_grasp),
        "object_class": str(object_class or ""),
        "applied": bool(level_applied),
    }
    if level_applied:
        T_base_grasp, extra = level_front_grasp_in_base(T_base_grasp)
        level_meta.update(extra)
        # Preserve the template pregrasp standoff direction/length, but level its
        # orientation to match the final grasp. The front approach target below
        # is still generated from --front-standoff-m and remains the executed
        # front-mode approach pose.
        pregrasp_offset = T_base_pregrasp[:3, 3] - transform_cam_grasp_to_base(
            T_base_ee_capture,
            T_ee_cam,
            T_cam_grasp,
        )[:3, 3]
        T_base_pregrasp = T_base_grasp.copy()
        T_base_pregrasp[:3, 3] = T_base_grasp[:3, 3] + pregrasp_offset
    T_base_safe = T_base_pregrasp.copy()
    T_base_safe[2, 3] += safe_lift_m

    if T_grasp_tcp is None:
        T_grasp_tcp = np.eye(4, dtype=np.float64)
    validate_T(T_grasp_tcp, "T_grasp_tcp")

    # Dobot: matrix_to_pose(T_base_stage @ T_grasp_tcp) then MovJ
    # OpenArm: keep T_base_*_tcp here, later IK -> joints
    T_base_safe_tcp = T_base_safe @ T_grasp_tcp
    T_base_pregrasp_tcp = T_base_pregrasp @ T_grasp_tcp
    T_base_grasp_tcp = T_base_grasp @ T_grasp_tcp

    # Front approach: stand farther back than template pregrasp so opening
    # the gripper does not hit the object. Back off along -approach (+Z of grasp).
    approach_dir = np.asarray(T_base_grasp_tcp[:3, 2], dtype=np.float64)
    approach_norm = float(np.linalg.norm(approach_dir))
    if approach_norm < 1e-12:
        raise RuntimeError("Invalid grasp approach axis for front standoff")
    approach_dir = approach_dir / approach_norm
    standoff = max(0.0, float(front_standoff_m))
    T_base_front_approach_tcp = T_base_grasp_tcp.copy()
    T_base_front_approach_tcp[:3, 3] = T_base_grasp_tcp[:3, 3] - approach_dir * standoff

    return {
        "robot": "openarm",
        "transform_formula": (
            "T_base_grasp = T_base_ee_capture @ T_ee_cam @ T_cam_grasp; "
            "T_base_tcp = T_base_grasp_frame @ T_grasp_tcp"
        ),
        "dobot_difference": (
            "Dobot would call MovJ(matrix_to_pose(T_base_tcp)). "
            "OpenArm must IK(T_base_tcp) then send joint references."
        ),
        "grasp_id": grasp_id,
        "pregrasp_id": pregrasp_id,
        "object_class": str(object_class or ""),
        "level_small_object_grasp": level_meta,
        "safe_lift_m": safe_lift_m,
        "front_standoff_m": standoff,
        "T_base_ee_capture": T_base_ee_capture.tolist(),
        "T_ee_cam": T_ee_cam.tolist(),
        "T_cam_pregrasp": T_cam_pregrasp.tolist(),
        "T_cam_grasp": T_cam_grasp.tolist(),
        "T_grasp_tcp": T_grasp_tcp.tolist(),
        "T_base_safe_pregrasp_tcp": T_base_safe_tcp.tolist(),
        "T_base_pregrasp_tcp": T_base_pregrasp_tcp.tolist(),
        "T_base_front_approach_tcp": T_base_front_approach_tcp.tolist(),
        "T_base_grasp_tcp": T_base_grasp_tcp.tolist(),
        "delta_safe_to_grasp": pose_delta(T_base_safe_tcp, T_base_grasp_tcp),
        "delta_pregrasp_to_grasp": pose_delta(T_base_pregrasp_tcp, T_base_grasp_tcp),
        "delta_front_approach_to_grasp": pose_delta(T_base_front_approach_tcp, T_base_grasp_tcp),
        "next_step": (
            "front mode: IK T_base_front_approach_tcp first (standoff before object), "
            "open gripper, then extend to T_base_grasp_tcp. "
            "above mode: IK T_base_safe_pregrasp_tcp first, then descend along base Z."
        ),
    }


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Convert camera grasps to OpenArm base TCP targets.")
    p.add_argument("--grasps-cam", type=Path, required=True)
    p.add_argument("--handeye", type=Path, required=True)
    p.add_argument("--base-ee", type=Path, required=True)
    p.add_argument("--grasp-id", default="grasp_side_0")
    p.add_argument("--safe-lift-m", type=float, default=0.08)
    p.add_argument(
        "--front-standoff-m",
        type=float,
        default=0.15,
        help="Front approach distance before grasp along -approach axis (meter). "
        "Larger = open gripper farther from object. Template pregrasp is only ~0.08 m.",
    )
    p.add_argument("--grasp-tcp", type=Path, default=None)
    p.add_argument("--object-class", default="", help="Detected object class; cake/cup enable small-object leveling in auto mode.")
    p.add_argument(
        "--level-small-object-grasp",
        choices=("auto", "always", "off"),
        default="auto",
        help="auto=level cake/cup front grasps in OpenArm base frame; always=force; off=keep visual grasp orientation.",
    )
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--arm-side", choices=("right_arm", "left_arm"), default="right_arm")
    p.add_argument("--base-frame", default="openarm_body_link0")
    p.add_argument(
        "--ee-frame",
        default="auto",
        help="OpenArm TCP frame; default auto -> openarm_{left|right}_hand_tcp from --arm-side.",
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    T_ee_cam = load_matrix4x4(args.handeye, "T_ee_cam")
    T_base_ee = load_matrix4x4(args.base_ee, "T_base_ee")
    poses = load_grasp_poses(args.grasps_cam)
    T_grasp_tcp = load_matrix4x4(args.grasp_tcp, "T_grasp_tcp") if args.grasp_tcp else None
    if args.grasp_tcp is not None:
        tcp_meta = load_json(args.grasp_tcp)
        declared = str(tcp_meta.get("arm_side", "") or "").strip().lower()
        if declared in {"left", "left_arm", "right", "right_arm"}:
            declared_side = "left_arm" if declared in {"left", "left_arm"} else "right_arm"
            if declared_side != args.arm_side:
                raise RuntimeError(
                    f"--grasp-tcp {args.grasp_tcp} is for {declared_side}, but --arm-side={args.arm_side}"
                )
        if tcp_meta.get("calibrated") is False:
            print(f"[openarm][WARN] grasp_tcp {args.grasp_tcp} calibrated=false")

    payload = build_targets(
        T_base_ee_capture=T_base_ee,
        T_ee_cam=T_ee_cam,
        poses_cam=poses,
        grasp_id=args.grasp_id,
        safe_lift_m=args.safe_lift_m,
        T_grasp_tcp=T_grasp_tcp,
        front_standoff_m=args.front_standoff_m,
        object_class=args.object_class,
        level_small_object_grasp=args.level_small_object_grasp,
    )
    frames = OpenArmFrames(
        arm_side=args.arm_side,
        base_frame=args.base_frame,
        ee_frame=args.ee_frame,
    )
    payload["openarm_frames"] = {
        "arm_side": frames.arm_side,
        "base_frame": frames.base_frame,
        "ee_frame": frames.ee_frame,
        "joint_names": frames.joint_names,
        "urdf_path": str(frames.urdf_path),
    }
    save_json(args.output, payload)
    out_dir = args.output.parent
    save_matrix4x4(
        out_dir / "T_base_safe_pregrasp_tcp.json",
        "T_base_safe_pregrasp_tcp",
        np.asarray(payload["T_base_safe_pregrasp_tcp"]),
    )
    save_matrix4x4(
        out_dir / "T_base_pregrasp_tcp.json",
        "T_base_pregrasp_tcp",
        np.asarray(payload["T_base_pregrasp_tcp"]),
    )
    save_matrix4x4(
        out_dir / "T_base_front_approach_tcp.json",
        "T_base_front_approach_tcp",
        np.asarray(payload["T_base_front_approach_tcp"]),
        extra={
            "front_standoff_m": payload["front_standoff_m"],
            "definition": "grasp pose backed off along -approach for open-gripper approach",
        },
    )
    save_matrix4x4(
        out_dir / "T_base_grasp_tcp.json",
        "T_base_grasp_tcp",
        np.asarray(payload["T_base_grasp_tcp"]),
    )
    print(f"[openarm] wrote {args.output}")
    print(f"[openarm] safe pregrasp: {out_dir / 'T_base_safe_pregrasp_tcp.json'}")
    print(f"[openarm] template pregrasp: {out_dir / 'T_base_pregrasp_tcp.json'}")
    print(
        f"[openarm] front approach (standoff={payload['front_standoff_m']:.3f}m): "
        f"{out_dir / 'T_base_front_approach_tcp.json'}"
    )
    front_xyz = np.asarray(payload["T_base_front_approach_tcp"], dtype=np.float64)[:3, 3]
    grasp_xyz = np.asarray(payload["T_base_grasp_tcp"], dtype=np.float64)[:3, 3]
    delta_xyz = grasp_xyz - front_xyz
    print(
        "[openarm][target] "
        f"front_xyz=[{front_xyz[0]:+.4f}, {front_xyz[1]:+.4f}, {front_xyz[2]:+.4f}] "
        f"grasp_xyz=[{grasp_xyz[0]:+.4f}, {grasp_xyz[1]:+.4f}, {grasp_xyz[2]:+.4f}] "
        f"front_to_grasp=[{delta_xyz[0]:+.4f}, {delta_xyz[1]:+.4f}, {delta_xyz[2]:+.4f}]"
    )
    print(f"[openarm] grasp: {out_dir / 'T_base_grasp_tcp.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
