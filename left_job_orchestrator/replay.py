"""Thin wrappers around dual_left_visual_flow server lifecycle (via tcp_forward_grasp)."""

from __future__ import annotations

from argparse import Namespace
from pathlib import Path

from .flow_loader import load_flow_module


def start_server_and_steady(args: Namespace, run_dir: Path, steady_csv: Path):
    flow = load_flow_module()
    return flow.start_server_and_steady(args, run_dir, steady_csv)


def shutdown_server(reason: str = "service shutdown") -> None:
    flow = load_flow_module()
    flow.cleanup_running_processes(reason)


def install_cleanup_handlers() -> None:
    flow = load_flow_module()
    flow.install_cleanup_handlers()
