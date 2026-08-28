#!/usr/bin/env python3
"""
OpenArm KDL FK/IK wrapper used by the grasp pipeline.

Default FK usage:

  cd /home/wenjiajie/code/openarm_yaocao/openarm_teleop-openarm_teleop/openarm_vision_grasp_yolo
  conda activate anygrasp

  python scripts/solve_openarm_fk_ik.py \
    --auto-build \
    --joints <run_dir>/current_capture_joints.json \
    --joint-source follower \
    --output-dir runs/debug_fk

Default IK usage:

  python scripts/solve_openarm_fk_ik.py \
    --joints <run_dir>/current_lift_seed_joints.json \
    --joint-source plain \
    --target <run_dir>/T_base_safe_pregrasp_tcp.json \
    --target-key T_base_safe_pregrasp_tcp \
    --output-dir runs/debug_ik \
    --replay-duration 22.0 \
    --replay-sample-period 0.01

Inputs:
  --joints reads either follower/leader joint JSON or plain {"arm": [...]}.
  --target is optional; without it the script only writes FK.

Outputs:
  T_base_ee_from_seed.json, kdl_fk_ik_result.json, and when IK succeeds,
  ik_solution_joints.json plus safe_pregrasp_replay.csv.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot.openarm_adapter import OpenArmFrames, write_single_arm_replay_csv  # noqa: E402
from robot.transforms import load_matrix4x4, pose_delta, save_matrix4x4  # noqa: E402


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def load_seed_joints(path: Path, source: str) -> list[float]:
    data = load_json(path)
    if source == "follower":
        joints = data.get("follower", {}).get("arm")
    elif source == "leader":
        joints = data.get("leader", {}).get("arm")
    else:
        joints = data.get("arm") or data.get("joints") or data.get("seed_joints")
    if not isinstance(joints, list) or len(joints) != 7:
        raise RuntimeError(f"{path} does not contain 7 {source} arm joints")
    return [float(v) for v in joints]


def candidate_ik_seeds(primary: list[float]) -> list[list[float]]:
    """
    Build alternate IK seeds. KDL Newton-Raphson is local; a reachable pose often
    fails from one seed and succeeds from another configuration.
    """
    seeds: list[list[float]] = [list(primary)]
    # Keep elbow/shoulder, flip wrist yaw-like joints.
    for idx in (4, 5, 6):
        flipped = list(primary)
        flipped[idx] = -float(primary[idx])
        seeds.append(flipped)
    # Nudge major joints toward a more outstretched / tucked posture.
    for delta in (-0.4, 0.4, -0.8, 0.8):
        for idx in (1, 3):
            nudged = list(primary)
            nudged[idx] = float(primary[idx]) + delta
            seeds.append(nudged)
    # Small random-ish deterministic dither around primary (no RNG dependency).
    pattern = (
        (0.15, -0.10, 0.20, -0.25, 0.15, -0.20, 0.10),
        (-0.20, 0.15, -0.15, 0.30, -0.10, 0.25, -0.15),
        (0.05, 0.25, -0.30, 0.10, -0.25, 0.10, 0.20),
        (-0.10, -0.20, 0.25, -0.35, 0.20, -0.15, -0.10),
    )
    for offsets in pattern:
        seeds.append([float(primary[i]) + float(offsets[i]) for i in range(7)])
    # Deduplicate while preserving order.
    unique: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for seed in seeds:
        key = tuple(round(v, 5) for v in seed)
        if key in seen:
            continue
        seen.add(key)
        unique.append(seed)
    return unique


def run_kdl_tool(
    tool: Path,
    *,
    frames: OpenArmFrames,
    seed: list[float],
    output: Path,
    target_csv: str | None,
    eps: float,
    max_iter: int,
) -> int:
    command = [
        str(tool),
        "--urdf",
        str(frames.urdf_path),
        "--base",
        frames.base_frame,
        "--ee",
        frames.ee_frame,
        "--seed",
        ",".join(f"{v:.12g}" for v in seed),
        "--output",
        str(output),
        "--eps",
        str(eps),
        "--max-iter",
        str(max_iter),
    ]
    if target_csv is not None:
        command.extend(["--target", target_csv])
    return subprocess.run(command).returncode


def matrix_to_csv16(T: list[list[float]]) -> str:
    return ",".join(f"{float(T[r][c]):.12g}" for r in range(4) for c in range(4))


def ensure_tool_built(build_dir: Path, tool: Path, auto_build: bool) -> None:
    if tool.exists():
        return
    if not auto_build:
        raise RuntimeError(f"KDL tool not built: {tool}. Re-run with --auto-build.")
    build_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(["cmake", "-S", str(ROOT), "-B", str(build_dir)], check=True)
    subprocess.run(["cmake", "--build", str(build_dir), "-j2", "--target", "kdl_fk_ik_tool"], check=True)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="OpenArm KDL FK/IK wrapper.")
    parser.add_argument(
        "--joints",
        type=Path,
        default=ROOT / "examples" / "right_observe_joint_snapshot_20260709_135028.json",
        help="JSON containing follower.arm 7 joint values.",
    )
    parser.add_argument("--joint-source", choices=("follower", "leader", "plain"), default="follower")
    parser.add_argument("--target", type=Path, default=None, help="JSON target matrix, e.g. T_base_safe_pregrasp_tcp.json")
    parser.add_argument("--target-key", default="T_base_safe_pregrasp_tcp")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "kdl")
    parser.add_argument("--arm-side", choices=("right_arm", "left_arm"), default="right_arm")
    parser.add_argument("--base-frame", default="openarm_body_link0")
    parser.add_argument("--ee-frame", default="auto")
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--auto-build", action="store_true", help="Configure/build the C++ KDL tool if needed.")
    parser.add_argument("--eps", type=float, default=1e-5)
    parser.add_argument("--max-iter", type=int, default=500)
    parser.add_argument(
        "--multi-seed",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="On IK failure, retry with alternate joint seeds (default on).",
    )
    parser.add_argument(
        "--max-seed-attempts",
        type=int,
        default=12,
        help="Maximum seed attempts when --multi-seed is enabled.",
    )
    parser.add_argument("--alt-seed", type=Path, default=None, help="Optional extra seed JSON with arm:[7].")
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--gripper", type=float, default=0.0)
    parser.add_argument("--replay-duration", type=float, default=20.0, help="Seconds for seed -> IK replay segment.")
    parser.add_argument("--replay-sample-period", type=float, default=0.01, help="Dense replay sample period in seconds.")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    frames = OpenArmFrames(
        arm_side=args.arm_side,
        base_frame=args.base_frame,
        ee_frame=args.ee_frame,
        urdf_path=args.urdf or OpenArmFrames().urdf_path,
    )
    build_dir = ROOT / "build"
    tool = build_dir / "kdl_fk_ik_tool"
    ensure_tool_built(build_dir, tool, args.auto_build)

    seed = load_seed_joints(args.joints, args.joint_source)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    kdl_result = args.output_dir / "kdl_fk_ik_result.json"

    target_T = None
    target_csv = None
    if args.target is not None:
        target_T = load_matrix4x4(args.target, args.target_key)
        target_csv = matrix_to_csv16(target_T.tolist())

    seeds = [seed]
    if args.alt_seed is not None:
        seeds.append(load_seed_joints(args.alt_seed, "plain"))
    if args.target is not None and args.multi_seed:
        seeds = candidate_ik_seeds(seed)
        if args.alt_seed is not None:
            seeds.insert(1, load_seed_joints(args.alt_seed, "plain"))
        seeds = seeds[: max(1, int(args.max_seed_attempts))]

    proc_code = 2
    result: dict[str, Any] = {}
    used_seed = seed
    for attempt, trial_seed in enumerate(seeds):
        attempt_out = kdl_result if attempt == 0 else (args.output_dir / f"kdl_fk_ik_result_seed{attempt:02d}.json")
        proc_code = run_kdl_tool(
            tool,
            frames=frames,
            seed=trial_seed,
            output=attempt_out,
            target_csv=target_csv,
            eps=args.eps,
            max_iter=args.max_iter,
        )
        if proc_code not in (0, 2):
            return proc_code
        result = load_json(attempt_out)
        if args.target is None or result.get("ik_success"):
            used_seed = trial_seed
            if attempt > 0:
                # Promote successful alternate seed result to the canonical path.
                save_json(kdl_result, result)
                print(f"[openarm kdl] IK succeeded with alternate seed #{attempt}")
            break
        if attempt == 0 and len(seeds) > 1:
            print(
                f"[openarm kdl] primary seed failed (code={result.get('ik_return_code')}); "
                f"retrying up to {len(seeds) - 1} alternate seeds"
            )

    save_matrix4x4(
        args.output_dir / "T_base_ee_from_seed.json",
        "T_base_ee",
        result["T_base_ee_fk"],
        extra={
            "source": "KDL FK from OpenArm follower joints",
            "base_frame": frames.base_frame,
            "ee_frame": frames.ee_frame,
            "joint_source": str(args.joints),
            "seed_joints": used_seed,
        },
    )

    if args.target is not None and result.get("ik_success"):
        q = [float(v) for v in result["ik_solution_joints"]]
        save_json(
            args.output_dir / "ik_solution_joints.json",
            {
                "arm_side": frames.arm_side,
                "joint_names": frames.joint_names,
                "seed_joints": used_seed,
                "primary_seed_joints": seed,
                "ik_solution_joints": q,
                "ik_delta_from_seed": result.get("ik_delta_from_seed"),
                "target": str(args.target),
                "target_key": args.target_key,
            },
        )
        replay_csv = args.replay_csv or (args.output_dir / "safe_pregrasp_replay.csv")
        write_single_arm_replay_csv(
            replay_csv,
            [seed, q],
            gripper=args.gripper,
            dt=args.replay_duration,
            sample_period=args.replay_sample_period,
        )
        fk_check_result = args.output_dir / "kdl_ik_solution_fk_result.json"
        fk_proc = run_kdl_tool(
            tool,
            frames=frames,
            seed=q,
            output=fk_check_result,
            target_csv=None,
            eps=args.eps,
            max_iter=args.max_iter,
        )
        if fk_proc == 0 and target_T is not None:
            fk_check = load_json(fk_check_result)
            save_matrix4x4(
                args.output_dir / "T_base_ee_from_ik_solution.json",
                "T_base_ee",
                fk_check["T_base_ee_fk"],
                extra={
                    "source": "KDL FK from IK solution joints",
                    "target": str(args.target),
                    "target_key": args.target_key,
                },
            )
            T_ik_fk = load_matrix4x4(args.output_dir / "T_base_ee_from_ik_solution.json", "T_base_ee")
            error = pose_delta(target_T, T_ik_fk)
            save_json(
                args.output_dir / "ik_target_error.json",
                {
                    "target": str(args.target),
                    "target_key": args.target_key,
                    "translation_error_m": error["translation_distance_m"],
                    "rotation_error_deg": error["rotation_error_deg"],
                    "delta_xyz_m": error["delta_xyz_m"],
                    "note": (
                        "If this is near zero, IK/FK is tracking the requested target; "
                        "upstream transform/calibration is the likely error source."
                    ),
                },
            )
            print(
                "[openarm kdl] IK FK check: "
                f"translation_error={error['translation_distance_m']:.6f}m "
                f"rotation_error={error['rotation_error_deg']:.3f}deg"
            )
        print(f"[openarm kdl] wrote replay CSV: {replay_csv}")
    elif args.target is not None:
        print(
            "[openarm kdl] IK failed after "
            f"{len(seeds)} seed(s). Pose may still be reachable with another configuration; "
            "try moving object closer, another grasp candidate, or larger --ik-max-iter."
        )

    print(f"[openarm kdl] FK output: {args.output_dir / 'T_base_ee_from_seed.json'}")
    print(f"[openarm kdl] raw KDL result: {kdl_result}")
    return 0 if (args.target is None or result.get("ik_success")) else 2


if __name__ == "__main__":
    raise SystemExit(main())
