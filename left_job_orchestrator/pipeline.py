"""Numbered job pipeline.

Files named actionN.csv and visual_graspN.txt are executed by trailing number.
The legacy unnumbered visual_grasp.txt remains between action1 and action2.

Motor / gripper config comes from the dual replay server started with
openarm_vr_compat_env.sh (same as VR teleop) — this module does not change gains.
"""

from __future__ import annotations

import json
import re
import sys
from argparse import Namespace
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from . import paths
from .defaults import resolve_observe_csv
from .flow_loader import build_flow_args, load_flow_module

_ACTION_RE = re.compile(r"^action(\d+)\.csv$", re.IGNORECASE)
_VISUAL_RE = re.compile(r"^visual_gr(?:a|i)sp(\d+)\.txt$", re.IGNORECASE)
_LEGACY_VISUAL_RE = re.compile(r"^visual_gr(?:a|i)sp\.txt$", re.IGNORECASE)


@dataclass(frozen=True)
class JobStep:
    kind: str
    number: int
    path: Path
    legacy_between: bool = False

    @property
    def label(self) -> str:
        return self.path.stem

    @property
    def sort_key(self) -> tuple[int, int]:
        # visual_graspN runs immediately before actionN. Legacy visual_grasp.txt
        # is represented as visual step 2, preserving action1 -> grasp -> action2.
        return (self.number, 0 if self.kind == "visual_grasp" else 1)


def load_job_overrides(job_folder: Path) -> dict[str, Any]:
    path = job_folder / "job.json"
    if not path.is_file():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"job.json must be an object: {path}")
    return data


def discover_job_steps(job_folder: Path) -> list[JobStep]:
    steps: list[JobStep] = []
    numbered: dict[tuple[str, int], Path] = {}
    for path in job_folder.iterdir():
        if not path.is_file():
            continue
        action_match = _ACTION_RE.fullmatch(path.name)
        visual_match = _VISUAL_RE.fullmatch(path.name)
        legacy_visual = _LEGACY_VISUAL_RE.fullmatch(path.name)
        if action_match:
            number = int(action_match.group(1))
            kind = "action"
            legacy_between = False
        elif visual_match:
            number = int(visual_match.group(1))
            kind = "visual_grasp"
            legacy_between = False
        elif legacy_visual:
            key = ("visual_grasp", 2)
            if key in numbered:
                raise ValueError(
                    f"duplicate visual_grasp step 2: {numbered[key].name}, {path.name}"
                )
            numbered[key] = path
            steps.append(
                JobStep("visual_grasp", 2, path.resolve(), legacy_between=True)
            )
            continue
        else:
            continue
        if number < 1:
            raise ValueError(f"step number must be >= 1: {path.name}")
        key = (kind, number)
        if key in numbered:
            raise ValueError(
                f"duplicate {kind} step {number}: {numbered[key].name}, {path.name}"
            )
        numbered[key] = path
        steps.append(JobStep(kind, number, path.resolve(), legacy_between))
    steps.sort(key=lambda step: step.sort_key)
    return steps


def preflight_job(job_folder: Path) -> list[JobStep]:
    steps = discover_job_steps(job_folder)
    actions = [step for step in steps if step.kind == "action"]
    if not actions:
        raise FileNotFoundError(f"no actionN.csv found in {job_folder}")
    if actions[0].number != 1:
        raise FileNotFoundError(f"missing action1.csv in {job_folder}")
    for index, step in enumerate(steps):
        if step.kind != "visual_grasp":
            continue
        if index == 0 or steps[index - 1].kind != "action":
            raise ValueError(f"{step.path.name} must follow an actionN.csv")
        if index + 1 < len(steps) and steps[index + 1].kind != "action":
            raise ValueError(f"{step.path.name} must be followed by an actionN.csv")
    return steps


def find_visual_grasp_marker(job_folder: Path) -> Path | None:
    """Return the first visual-grasp step, or None."""
    return next(
        (step.path for step in discover_job_steps(job_folder) if step.kind == "visual_grasp"),
        None,
    )


def resolve_job_begin(job_folder: Path, overrides: dict[str, Any]) -> Path:
    raw = (
        overrides.get("begin_csv")
        or overrides.get("begin_replay")
        or overrides.get("observe_replay")
        or overrides.get("observe_csv")
    )
    if raw:
        p = Path(str(raw)).expanduser()
        if not p.is_absolute():
            p = (job_folder / p).resolve()
        else:
            p = p.resolve()
        if not p.is_file():
            raise FileNotFoundError(f"begin/observe CSV not found: {p}")
        return p
    default = resolve_observe_csv()
    if not default.is_file():
        raise FileNotFoundError(f"default begin.csv missing: {default}")
    return default


def _raise_if_aborted(abort_check: Callable[[], bool] | None) -> None:
    if abort_check is not None and abort_check():
        raise RuntimeError("abort requested")


def play_action_on_server(
    server_proc,
    args: Namespace,
    run_dir: Path,
    action: Path,
    *,
    abort_check: Callable[[], bool] | None = None,
) -> None:
    """Play one numbered action on the shared dual replay server."""
    flow = load_flow_module()
    core = flow.core
    replay = flow.path_arg(action, action.name)
    _raise_if_aborted(abort_check)
    print(f"[pipeline] play {action.stem}: {replay}", flush=True)
    core.play_full_replay_on_server(
        server_proc,
        replay,
        args,
        label=action.stem,
        side=None,
        feedback_dir=run_dir / "feedback_dumps",
    )
    _raise_if_aborted(abort_check)


def run_visual_grasp_step(
    server_proc,
    args: Namespace,
    run_dir: Path,
    marker: Path,
    previous_action: Path,
    next_action: Path | None,
    begin_csv: Path,
    *,
    abort_check: Callable[[], bool] | None = None,
    skip_next_on_grasp_fail: bool = True,
) -> bool:
    """Run one visual-grasp event without replaying adjacent action files."""
    from tcp_forward_grasp import flow as grasp_flow

    flow = load_flow_module()
    core = flow.core
    previous = flow.path_arg(previous_action, previous_action.name)
    begin = flow.path_arg(begin_csv, "begin.csv")
    _raise_if_aborted(abort_check)

    gripper_open = float(
        getattr(args, "gripper_open", None) or grasp_flow.read_begin_gripper(begin)
    )
    args.gripper_open = gripper_open
    args.gripper = gripper_open
    q_obs, _g_obs_csv, _T_obs = grasp_flow.prepare_observe_state(
        args, run_dir, begin
    )
    g_obs = float(gripper_open)
    q_start, g_start = core.read_recorded_active_replay_last_q(previous, args)
    print(
        f"[pipeline] {marker.stem}: quintic {previous_action.stem}-end -> begin "
        f"({float(args.pre_to_observe_duration):.1f}s)",
        flush=True,
    )
    core.execute_joint_transition(
        name=f"{marker.stem}_to_begin",
        label=f"{marker.stem}-pre",
        q_start=list(q_start),
        gripper_start=float(g_start),
        q_goal=list(q_obs),
        gripper_goal=g_obs,
        duration_s=float(args.pre_to_observe_duration),
        server_proc=server_proc,
        run_dir=run_dir,
        args=args,
    )
    _raise_if_aborted(abort_check)
    T_obs = grasp_flow.calib.solve_fk(q_obs, run_dir, args)
    grasp_flow.log_pose_state(f"{marker.stem}-begin", q_obs, g_obs, T_obs)

    q_end = list(q_obs)
    g_end = g_obs
    try:
        q_end, _T_end, g_end = grasp_flow.run_lr_forward_close(
            server_proc,
            args,
            run_dir,
            q_start=list(q_obs),
            T_start=T_obs,
            gripper_start=g_obs,
            lr_only=bool(getattr(args, "lr_only", False)),
        )
    except Exception as exc:
        if skip_next_on_grasp_fail:
            raise RuntimeError(
                f"{marker.name} failed; skipped remaining steps: {exc}"
            ) from exc
        print(f"[WARN] {marker.name} failed; continuing sequence: {exc}", flush=True)

    _raise_if_aborted(abort_check)
    if bool(getattr(args, "lr_only", False)):
        print(f"[pipeline] {marker.stem} completed in LR-only mode", flush=True)
        return False
    if next_action is not None:
        following = flow.path_arg(next_action, next_action.name)
        q_next, g_next = core.read_recorded_active_replay_first_q(following, args)
        print(
            f"[pipeline] {marker.stem}: quintic grasp-end -> {next_action.stem}-start "
            f"({float(args.post_replay_transition_duration):.1f}s)",
            flush=True,
        )
        core.execute_joint_transition(
            name=f"{marker.stem}_to_{next_action.stem}",
            label=f"{marker.stem}-post",
            q_start=list(q_end),
            gripper_start=float(g_end),
            q_goal=list(q_next),
            gripper_goal=float(g_next),
            duration_s=float(args.post_replay_transition_duration),
            server_proc=server_proc,
            run_dir=run_dir,
            args=args,
        )
    return True


def run_job(
    server_proc,
    base_args: Namespace,
    jobs_root: Path,
    job_id: str,
    *,
    abort_check: Callable[[], bool] | None = None,
    skip_action2_on_grasp_fail: bool = True,
    output_root: Path | None = None,
) -> Path:
    """Execute one job on the shared dual replay server.

    Returns the per-job run_dir used for logs/artifacts.
    """
    root = str(paths.VR_CONTROL_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)

    flow = load_flow_module()
    job_folder = paths.job_dir(jobs_root, job_id)
    if not job_folder.is_dir():
        raise FileNotFoundError(f"job folder not found: {job_folder}")

    steps = preflight_job(job_folder)
    actions = [step.path for step in steps if step.kind == "action"]
    first_action = actions[0]
    last_action = actions[-1]
    overrides = load_job_overrides(job_folder)
    visual_steps = [step for step in steps if step.kind == "visual_grasp"]
    want_grasp = bool(visual_steps)

    out_root = (output_root or paths.OUTPUT_ROOT).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    run_dir = out_root / (
        f"job{paths.normalize_job_id(job_id)}_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    skip_keys = {
        "observe_replay",
        "observe_csv",
        "begin_csv",
        "begin_replay",
    }
    args = build_flow_args(
        overrides={k: v for k, v in overrides.items() if k not in skip_keys},
        output_dir=out_root,
    )
    for key in (
        "replay_command_port",
        "replay_status_port",
        "command_port",
        "status_port",
        "left_follower_can",
        "right_follower_can",
        "follower_can",
        "sudo_password",
    ):
        if hasattr(base_args, key) and getattr(base_args, key) is not None:
            setattr(args, key, getattr(base_args, key))

    args.execute = True
    args.pre_replay = first_action
    args.post_replay = last_action

    begin_csv: Path | None = None
    if want_grasp:
        begin_csv = resolve_job_begin(job_folder, overrides)
        args.observe_replay = begin_csv

    flow.save_run_plan(
        run_dir,
        args,
        steady_replay=Path(getattr(base_args, "steady_replay", paths.STEADY_CSV)),
        pre_replay=first_action,
        observe_replay=begin_csv or first_action,
        post_replay=last_action,
    )
    flow.core.save_json(
        run_dir / "mode.json",
        {
            "mode": "numbered_job_sequence",
            "visual_grasp": want_grasp,
            "steps": [
                {
                    "number": step.number,
                    "kind": step.kind,
                    "path": str(step.path),
                    "legacy_between": step.legacy_between,
                }
                for step in steps
            ],
            "grasp": (
                "tcp_forward_grasp.run_lr_forward_close" if want_grasp else "none"
            ),
            "begin_csv": str(begin_csv) if begin_csv else "",
            "gripper_gate_file": str(getattr(args, "gripper_gate_file", "")),
        },
    )

    print(f"[pipeline] job={job_id} dir={job_folder}", flush=True)
    print(
        "[pipeline] sequence=" + " -> ".join(step.label for step in steps),
        flush=True,
    )
    if begin_csv is not None:
        print(f"[pipeline] visual begin={begin_csv}", flush=True)

    previous_action: Path | None = None
    for index, step in enumerate(steps):
        _raise_if_aborted(abort_check)
        if step.kind == "action":
            play_action_on_server(
                server_proc,
                args,
                run_dir,
                step.path,
                abort_check=abort_check,
            )
            previous_action = step.path
            continue

        assert begin_csv is not None
        assert previous_action is not None
        next_action = (
            steps[index + 1].path
            if index + 1 < len(steps) and steps[index + 1].kind == "action"
            else None
        )
        should_continue = run_visual_grasp_step(
            server_proc,
            args,
            run_dir,
            step.path,
            previous_action,
            next_action,
            begin_csv,
            abort_check=abort_check,
            skip_next_on_grasp_fail=skip_action2_on_grasp_fail,
        )
        if not should_continue:
            break

    flow.core.save_json(
        run_dir / "final_hold.json",
        {
            "mode": "numbered_job_sequence",
            "last_completed_action": str(previous_action) if previous_action else "",
            "sequence": [step.label for step in steps],
        },
    )
    print("[pipeline] completed numbered job sequence", flush=True)
    return run_dir
