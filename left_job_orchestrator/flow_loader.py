"""Build flow args via tcp_forward_grasp (same grasp stack as ./tcp_forward_grasp/run_once.sh)."""

from __future__ import annotations

import sys
from argparse import Namespace
from pathlib import Path
from typing import Any

from . import defaults, paths


def _ensure_vr_control_on_path() -> None:
    root = str(paths.VR_CONTROL_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)


def load_flow_module():
    """dual_left_visual_flow (server lifecycle) via tcp_forward_grasp loader."""
    _ensure_vr_control_on_path()
    from tcp_forward_grasp.flow_loader import load_flow_module as _load

    return _load()


def build_flow_args(
    *,
    overrides: dict[str, Any] | None = None,
    output_dir: Path | None = None,
) -> Namespace:
    """Namespace with tcp_forward_grasp defaults + orchestrator overlays."""
    _ensure_vr_control_on_path()
    from tcp_forward_grasp.flow_loader import build_flow_args as _build

    merged: dict[str, Any] = dict(defaults.ORCH_OVERRIDES)
    if defaults.want_preview():
        merged["preview"] = True
        merged["no_preview"] = False
    if output_dir is not None:
        merged["output_dir"] = output_dir
    if overrides:
        merged.update(overrides)
        # job.json may still say observe_replay; keep as begin pose.
        if "begin_csv" in overrides and "observe_replay" not in overrides:
            merged["observe_replay"] = overrides["begin_csv"]
        if "begin_replay" in overrides and "observe_replay" not in overrides:
            merged["observe_replay"] = overrides["begin_replay"]
    return _build(overrides=merged, output_dir=output_dir or merged.get("output_dir"))
