"""Ensure dual_replay writes motor torque/temp CSV into the run directory.

run_once uses the skewer vendor openarm_inplace tree (required for approach_on /
tcp_vel visual servo). That vendor binary now supports --motor-log; we force
OPENARM_REPLAY_MOTOR_LOG into the run dir and verify the CSV appears after ready.

Do NOT point OPENARM_DUAL_BUILD_DIR at the main-tree build: main lacks TcpApproachStream.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Callable

from .motor_temp_monitor import configure_motor_log_for_run, start_monitor_for_run


def _force_motor_log_env(csv_path: Path) -> None:
    os.environ["OPENARM_REPLAY_MOTOR_LOG"] = str(csv_path)
    os.environ.setdefault("OPENARM_REPLAY_MOTOR_LOG_HZ", "100")
    os.environ.setdefault("OPENARM_REPLAY_TEMP_PRINT", "1")
    os.environ.setdefault("OPENARM_REPLAY_TEMP_PRINT_S", "2")
    # Keep vendor build (approach stream). Drop any prior main-tree override.
    os.environ.pop("OPENARM_DUAL_BUILD_DIR", None)


def patch_start_replay_server(flow_module: Any, run_dir: Path) -> Path:
    """Wrap flow.calib.start_replay_server so motor CSV lands in run_dir."""
    csv_path = configure_motor_log_for_run(run_dir)
    _force_motor_log_env(csv_path)
    calib = flow_module.calib
    original: Callable = calib.start_replay_server

    def _wrapped(initial_csv: Path, args: Any):
        _force_motor_log_env(csv_path)
        print(
            f"[motor-temp] forcing OPENARM_REPLAY_MOTOR_LOG={csv_path} "
            f"hz={os.environ.get('OPENARM_REPLAY_MOTOR_LOG_HZ')} "
            f"(vendor dual_replay with approach stream)",
            flush=True,
        )
        proc = original(initial_csv, args)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if csv_path.is_file() and csv_path.stat().st_size > 0:
                print(
                    f"[motor-temp] CSV ok ({csv_path.stat().st_size} bytes) -> {csv_path}",
                    flush=True,
                )
                return proc
            time.sleep(0.2)
        print(
            f"[motor-temp][WARN] motor CSV not created after server start: {csv_path}\n"
            f"  Check server stdout for [motor-log][ERROR] / unknown option --motor-log.\n"
            f"  Rebuild vendor: skewer_topdown_2d_minimal/vendor/openarm_inplace/build_current",
            flush=True,
        )
        return proc

    calib.start_replay_server = _wrapped  # type: ignore[method-assign]
    return csv_path


def begin_temp_logging(flow_module: Any, run_dir: Path):
    """Patch server start + start live monitor. Returns (monitor, csv_path)."""
    csv_path = patch_start_replay_server(flow_module, run_dir)
    mon = start_monitor_for_run(run_dir)
    return mon, csv_path
