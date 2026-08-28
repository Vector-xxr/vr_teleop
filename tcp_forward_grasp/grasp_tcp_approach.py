"""Fixed-distance forward grasp after LR-only red-ball align.

Default approach: robot *base* frame axis (not TCP-Z, not base-Z descent).
After close: vertical lift (+base Z), then return to the pre-approach aligned pose.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


def axis_index(name: str) -> int:
    key = str(name).strip().lower()
    mapping = {"x": 0, "y": 1, "z": 2, "0": 0, "1": 1, "2": 2}
    if key not in mapping:
        raise ValueError(f"approach axis must be x|y|z, got {name!r}")
    return mapping[key]


def base_approach_direction(axis: str, sign: float) -> np.ndarray:
    idx = axis_index(axis)
    direction = np.zeros(3, dtype=np.float64)
    direction[idx] = 1.0 if float(sign) >= 0.0 else -1.0
    return direction


def tcp_approach_direction(T_base_ee: np.ndarray, axis: str, sign: float) -> np.ndarray:
    idx = axis_index(axis)
    col = np.asarray(T_base_ee[:3, idx], dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(col))
    if n < 1e-9:
        raise RuntimeError("TCP approach axis column has near-zero length")
    return (float(sign) * col) / n


def resolve_approach_direction(T_aligned: np.ndarray, args) -> tuple[np.ndarray, str, str, float]:
    frame = str(getattr(args, "approach_frame", "base")).strip().lower()
    axis = str(
        getattr(args, "approach_axis", getattr(args, "tcp_approach_axis", "x"))
    ).strip().lower()
    sign = float(getattr(args, "approach_sign", getattr(args, "tcp_approach_sign", 1.0)))
    if frame in {"base", "world", "robot"}:
        return base_approach_direction(axis, sign), "base", axis, sign
    if frame in {"tcp", "ee", "tool"}:
        return tcp_approach_direction(T_aligned, axis, sign), "tcp", axis, sign
    raise ValueError(f"approach_frame must be base|tcp, got {frame!r}")


def _lift_ik(calib, pre_lift_T, q_seed, run_dir, args, requested_lift_m: float, min_lift_m: float):
    lift_candidates: list[float] = []
    for value in (
        requested_lift_m,
        requested_lift_m * 0.75,
        requested_lift_m * 0.50,
        requested_lift_m * 0.25,
        min_lift_m,
    ):
        lift_m = round(float(value), 4)
        if lift_m < min_lift_m or lift_m <= 0.0 or lift_m in lift_candidates:
            continue
        lift_candidates.append(lift_m)
    if not lift_candidates:
        raise RuntimeError("no positive post-grasp lift candidate")

    last_err: Exception | None = None
    for lift_m in lift_candidates:
        candidate_T = pre_lift_T.copy()
        candidate_T[2, 3] += lift_m
        try:
            candidate_q = calib.solve_ik(
                candidate_T,
                q_seed,
                run_dir,
                f"grasp_lift_{int(round(lift_m * 1000.0)):03d}mm",
                args,
            )
        except Exception as exc:  # noqa: BLE001
            last_err = exc
            print(f"[tcp-grasp][WARN] lift IK failed at {lift_m*1000:.0f}mm", flush=True)
            continue
        return candidate_q, candidate_T, lift_m
    raise RuntimeError("post-grasp lift IK failed for all heights") from last_err


def execute_grasp_sequence_tcp_approach(
    *,
    q_aligned: list[float],
    T_aligned: np.ndarray,
    server_proc,
    run_dir: Path,
    args,
    core: Any,
) -> tuple[list[float], np.ndarray, float]:
    """Open -> approach -> close -> vertical lift -> return to pre-approach pose."""
    calib = core.calib
    args.last_grasp_reverse_return_waypoints = []

    approach_m = float(
        getattr(args, "grasp_approach_m", getattr(args, "grasp_descent_m", 0.065))
    )
    if approach_m <= 0.0:
        raise RuntimeError("grasp_approach_m must be > 0 (manual fixed approach distance)")

    direction, frame, axis, sign = resolve_approach_direction(T_aligned, args)
    approach_T = T_aligned.copy()
    approach_T[:3, 3] = np.asarray(T_aligned[:3, 3], dtype=np.float64) + approach_m * direction
    q_approach = calib.solve_ik(approach_T, q_aligned, run_dir, "grasp_forward_approach", args)

    requested_lift_m = float(args.post_grasp_lift_m)
    min_lift_m = max(0.0, float(getattr(args, "min_post_grasp_lift_m", 0.0)))
    q_lift, lift_T, actual_lift_m = _lift_ik(
        calib, approach_T, q_approach, run_dir, args, requested_lift_m, min_lift_m
    )

    # Return to the pose *before* forward approach (after LR align), gripper closed.
    return_mode = str(getattr(args, "post_grasp_return", "aligned")).strip().lower()
    if return_mode in {"aligned", "previous", "pre_approach", "wait"}:
        q_return = list(q_aligned)
        return_T = T_aligned.copy()
        return_label = "return_pre_approach_aligned"
    else:
        # Legacy: retreat along base -X after lift.
        retreat_m = max(0.0, float(getattr(args, "post_grasp_retreat_mm", 0.0)) * 0.001)
        return_T = lift_T.copy()
        return_T[0, 3] -= retreat_m
        q_return = calib.solve_ik(return_T, q_lift, run_dir, "grasp_retreat_x_minus", args)
        return_label = "retreat_base_x_minus"

    max_delta = max(
        max(abs(a - b) for a, b in zip(q_aligned, q_approach)),
        max(abs(a - b) for a, b in zip(q_approach, q_lift)),
        max(abs(a - b) for a, b in zip(q_lift, q_return)),
    )
    max_allow = float(getattr(args, "max_grasp_joint_delta_rad", 2.0))
    if max_delta > max_allow:
        raise RuntimeError(
            f"grasp joint delta too large: {max_delta:.3f} rad > {max_allow}. "
            "Reduce --grasp-approach-m / --post-grasp-lift-m."
        )

    approach_duration = float(
        getattr(args, "grasp_approach_duration", getattr(args, "grasp_descent_duration", 3.5))
    )
    return_duration = float(
        getattr(args, "post_grasp_return_duration", getattr(args, "post_grasp_retreat_duration", 1.5))
    )

    replay_csv = run_dir / "grasp_replays" / "forward_lift_return.csv"
    waypoints = [
        (q_aligned, float(args.gripper_current_after_align)),
        (q_aligned, float(args.gripper_open)),
        (q_approach, float(args.gripper_open)),
        (q_approach, float(args.gripper_open)),
        (q_approach, float(args.gripper_close)),
        (q_approach, float(args.gripper_close)),
        (q_lift, float(args.gripper_close)),
        (q_return, float(args.gripper_close)),
    ]
    durations = [
        float(getattr(args, "gripper_open_duration", 0.4)),
        approach_duration,
        float(args.pre_close_hold_duration),
        float(args.gripper_close_duration),
        float(args.gripper_close_hold_duration),
        float(args.post_grasp_lift_duration),
        return_duration,
    ]

    args.last_grasp_reverse_return_waypoints = []
    core.write_replay_csv_8d(
        replay_csv,
        waypoints,
        durations,
        float(getattr(args, "replay_sample_period", 0.01)),
    )
    core.save_json(
        run_dir / "tcp_grasp_plan.json",
        {
            "mode": "forward_lift_return_aligned",
            "target": "rightmost_red_horizontal_align",
            "grasp_approach_m": approach_m,
            "approach_frame": frame,
            "approach_axis": axis,
            "approach_sign": sign,
            "approach_direction_base": direction.tolist(),
            "post_grasp_lift_m": float(actual_lift_m),
            "post_grasp_return": return_mode,
            "return_label": return_label,
            "q_aligned": list(q_aligned),
            "q_approach": list(q_approach),
            "q_lift": list(q_lift),
            "q_return": list(q_return),
            "T_base_ee_aligned": calib.matrix_to_list(T_aligned),
            "T_base_ee_approach": calib.matrix_to_list(approach_T),
            "T_base_ee_lift": calib.matrix_to_list(lift_T),
            "T_base_ee_return": calib.matrix_to_list(return_T),
            "replay_csv": str(replay_csv),
        },
    )
    print(
        "[tcp-grasp] open -> approach "
        f"{approach_m*1000:.0f}mm ({frame} {sign:+.0f}{axis}) -> close -> "
        f"lift {actual_lift_m*1000:.0f}mm -> {return_label}",
        flush=True,
    )
    core.play_full_replay_on_server(server_proc, replay_csv, args, label="tcp-grasp")
    return list(q_return), return_T, float(args.gripper_close)
