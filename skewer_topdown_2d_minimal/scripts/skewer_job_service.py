#!/usr/bin/env python3
"""Persistent skewer job service: startup once, then run jobs on command.

Lifecycle:
  1) Start one dual_replay_control server (shared CAN / control process).
  2) Play recordings/tasks/steady_action_on_table.csv once.
  3) Idle and listen for UDP job commands.
  4) On "1"/"2"/"3" (or run|1 / job1), execute
     recordings/job<id>/action1.csv -> 2D grasp -> action2.csv.

Default job folders (under vr_control):
  openarm_teleop/openarm_inplace/recordings/job1/
  openarm_teleop/openarm_inplace/recordings/job2/

Safety:
  - Only one job at a time (busy reject).
  - Preflight: required CSVs must exist and parse.
  - Abort / safety / load_failed stops the current job and refuses next until clear.
  - External align failures skip action2 by default.
  - Graceful shutdown sends UDP shutdown to the replay server.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def default_recordings_root() -> Path:
    """Canonical field recordings tree under vr_control."""
    return (
        ROOT.parent
        / "openarm_teleop"
        / "openarm_inplace"
        / "recordings"
    ).resolve()


def normalize_job_id(raw: str) -> str:
    """Map '1' / 'job1' / 'JOB1' -> '1' (numeric slot id)."""
    text = raw.strip()
    low = text.lower()
    if low.startswith("job"):
        text = text[3:]
    return text.strip()


def job_folder_name(job_id: str) -> str:
    """Map slot id '1' -> folder name 'job1'."""
    jid = normalize_job_id(job_id)
    if jid.isdigit():
        return f"job{jid}"
    return jid


def resolve_job_dir(jobs_root: Path, job_id: str) -> Path:
    return (jobs_root / job_folder_name(job_id)).resolve()


@dataclass
class ServiceConfig:
    jobs_root: Path  # recordings/ containing job1/, job2/, ...
    openarm_dir: Path
    align_script: Path
    runtime_root: Path = field(default_factory=lambda: ROOT / "jobs" / "_runtime")
    command_port: int = 47920  # dual replay control
    status_port: int = 47921
    job_listen_port: int = 47925  # this service API
    left_can: str = "follower_left"
    right_can: str = "follower_right"
    camera_backend: str = "opencv"
    camera_index: int = 16
    settle_s: float = 0.5
    ready_timeout_s: float = 240.0
    sudo_password: str = ""
    skip_action2_on_grasp_fail: bool = True
    max_move_mm: float = 180.0
    xy_correction_scale: float = 1.0
    grasp_descent_m: float = 0.04
    grasp_descent_duration: float = 5.0
    pre_close_hold_duration: float = 0.6
    gripper_close_duration: float = 1.2
    gripper_close_hold_duration: float = 1.2
    post_grasp_lift_m: float = 0.10
    post_grasp_lift_duration: float = 3.0
    place_duration: float = 3.0
    gripper_open: float = -0.5
    gripper_close: float = 0.1
    black_v_max: int = 80
    min_area_px: int = 250
    target_selection: str = "skewer"
    track_roi_px: float = 90.0
    auto_start_delay_s: float = 1.0
    quick_detect_preview_s: float = 0.25


@dataclass
class ServiceState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    busy: bool = False
    current_job: str | None = None
    last_result: str = "idle"
    last_error: str = ""
    startup_done: bool = False
    abort_requested: bool = False
    server_proc: subprocess.Popen | None = None
    stop_service: bool = False


def log(msg: str) -> None:
    print(f"[skewer-job] {msg}", flush=True)


def csv_duration_s(csv_path: Path) -> float:
    duration = 0.0
    with csv_path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        for row in csv.DictReader(f):
            if row and row.get("time"):
                try:
                    duration = float(row["time"])
                except ValueError:
                    continue
    return duration


def first_nonempty_row(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        for row in csv.DictReader(f):
            if row:
                return row
    raise RuntimeError(f"empty CSV: {path}")


def last_nonempty_row(path: Path) -> dict[str, str]:
    last = None
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        for row in csv.DictReader(f):
            if row:
                last = row
    if last is None:
        raise RuntimeError(f"empty CSV: {path}")
    return last


def right_q8(row: dict[str, str]) -> list[float]:
    return [float(row[f"q{i}"]) for i in range(8, 16)]


def write_observe_and_place(
    action1: Path, action2: Path, observe_out: Path, place_q_out: Path
) -> list[float]:
    observe_q = right_q8(last_nonempty_row(action1))
    place_q = right_q8(first_nonempty_row(action2))
    observe_out.parent.mkdir(parents=True, exist_ok=True)
    with observe_out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", *[f"q{i}" for i in range(8)]])
        writer.writerow(["0.000000", *[f"{v:.9f}" for v in observe_q]])
    place_q_out.write_text(",".join(f"{v:.9f}" for v in place_q[:7]) + "\n", encoding="utf-8")
    return place_q[:7]


def load_job_overrides(job_dir: Path) -> dict[str, Any]:
    path = job_dir / "job.json"
    if not path.is_file():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"job.json must be an object: {path}")
    return data


def discover_jobs(jobs_root: Path) -> list[str]:
    """Return slot ids ('1','2',...) for recordings/jobN that have both CSVs."""
    jobs: list[str] = []
    if not jobs_root.is_dir():
        return jobs
    for child in sorted(jobs_root.iterdir(), key=lambda p: p.name):
        if not child.is_dir():
            continue
        name = child.name
        if not name.lower().startswith("job"):
            continue
        slot = name[3:]
        if not slot.isdigit():
            continue
        if (child / "action1.csv").is_file() and (child / "action2.csv").is_file():
            jobs.append(slot)
    return jobs


def preflight_job(job_dir: Path) -> tuple[Path, Path]:
    a1 = (job_dir / "action1.csv").resolve()
    a2 = (job_dir / "action2.csv").resolve()
    if not a1.is_file():
        raise FileNotFoundError(f"missing action1.csv in {job_dir}")
    if not a2.is_file():
        raise FileNotFoundError(f"missing action2.csv in {job_dir}")
    # Force parse + dual right-arm columns.
    _ = right_q8(last_nonempty_row(a1))
    _ = right_q8(first_nonempty_row(a2))
    return a1, a2


class ReplayClient:
    def __init__(self, cfg: ServiceConfig, state: ServiceState):
        self.cfg = cfg
        self.state = state

    def wait_ready(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", self.cfg.status_port))
        sock.settimeout(0.2)
        deadline = time.time() + self.cfg.ready_timeout_s
        try:
            while time.time() < deadline:
                if self.state.abort_requested or self.state.stop_service:
                    raise RuntimeError("aborted while waiting for replay server ready")
                try:
                    data, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                text = data.decode("utf-8", errors="replace")
                log(f"[status] {text}")
                if text.startswith(("ready", "motor_log")):
                    return
                if text.startswith(("load_failed", "safety", "shutdown_done")):
                    raise RuntimeError(text)
            raise TimeoutError("timeout waiting for dual replay server ready")
        finally:
            sock.close()

    def play_and_wait(self, csv_path: Path, label: str, side: str = "") -> None:
        if self.state.abort_requested:
            raise RuntimeError("abort requested")
        duration = csv_duration_s(csv_path)
        timeout_s = max(20.0, duration + 20.0 + self.cfg.settle_s)
        status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        status.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        status.bind(("127.0.0.1", self.cfg.status_port))
        status.settimeout(0.2)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        parts = ["play", str(csv_path.resolve()), "0", label]
        if side:
            parts.extend([side, "1.0"])
        message = "|".join(parts).encode("utf-8")
        try:
            deadline = time.time() + timeout_s
            next_send = 0.0
            acknowledged = False
            while time.time() < deadline:
                if self.state.abort_requested or self.state.stop_service:
                    raise RuntimeError("abort requested during play")
                now = time.time()
                if not acknowledged and now >= next_send:
                    log(f"UDP {message.decode()} -> :{self.cfg.command_port}")
                    sender.sendto(message, ("127.0.0.1", self.cfg.command_port))
                    next_send = now + 1.0
                try:
                    data, _ = status.recvfrom(4096)
                except socket.timeout:
                    continue
                text = data.decode("utf-8", errors="replace")
                log(f"[status] {text}")
                if text.startswith(("accepted", "started")):
                    acknowledged = True
                if text.startswith("finished"):
                    time.sleep(self.cfg.settle_s)
                    return
                if text.startswith(("load_failed", "safety", "shutdown_done")):
                    raise RuntimeError(text)
            raise TimeoutError(f"timeout waiting for {label} to finish")
        finally:
            sender.close()
            status.close()

    def shutdown_server(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(b"shutdown", ("127.0.0.1", self.cfg.command_port))
            sock.close()
        except OSError:
            pass
        proc = self.state.server_proc
        if proc is None:
            return
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()


def start_replay_server(cfg: ServiceConfig, state: ServiceState, boot_csv: Path) -> None:
    script = cfg.openarm_dir / "script" / "run_cpp_real_replay_both.sh"
    if not script.is_file():
        raise FileNotFoundError(script)
    env = os.environ.copy()
    env["OPENARM_TORQUE_PANEL"] = "0"
    env["OPENARM_LEFT_FOLLOWER_CAN"] = cfg.left_can
    env["OPENARM_RIGHT_FOLLOWER_CAN"] = cfg.right_can
    if cfg.sudo_password:
        env["OPENARM_SUDO_PASSWORD"] = cfg.sudo_password
    cmd = [
        str(script),
        str(boot_csv),
        "--command-port",
        str(cfg.command_port),
        "--status-port",
        str(cfg.status_port),
    ]
    log(f"CAN: left={cfg.left_can} right={cfg.right_can}")
    log(f"starting dual replay server: {' '.join(cmd)}")
    state.server_proc = subprocess.Popen(cmd, env=env, cwd=str(cfg.openarm_dir))
    time.sleep(0.5)
    if state.server_proc.poll() is not None:
        raise RuntimeError("dual replay server exited during startup")


def run_grasp(
    cfg: ServiceConfig,
    state: ServiceState,
    observe_replay: Path,
    place_q: list[float],
    overrides: dict[str, Any],
) -> None:
    if state.abort_requested:
        raise RuntimeError("abort requested before grasp")
    camera_backend = str(overrides.get("camera_backend", cfg.camera_backend))
    camera_index = int(overrides.get("camera_index", cfg.camera_index))
    descent = float(overrides.get("grasp_descent_m", cfg.grasp_descent_m))
    args = [
        sys.executable,
        str(cfg.align_script),
        "--observe-replay",
        str(observe_replay),
        "--quick-grasp-on-d",
        "--auto-once",
        "--exit-after-auto-once",
        "--auto-start-delay-s",
        str(float(overrides.get("auto_start_delay_s", cfg.auto_start_delay_s))),
        "--auto-detect-retry-s",
        "1.0",
        "--no-quick-align-angle",
        "--max-move-mm",
        str(float(overrides.get("max_move_mm", cfg.max_move_mm))),
        "--xy-correction-scale",
        str(float(overrides.get("xy_correction_scale", cfg.xy_correction_scale))),
        "--grasp-descent-m",
        str(descent),
        "--grasp-descent-duration",
        str(float(overrides.get("grasp_descent_duration", cfg.grasp_descent_duration))),
        "--pre-close-hold-duration",
        str(cfg.pre_close_hold_duration),
        "--gripper-close-duration",
        str(cfg.gripper_close_duration),
        "--gripper-close-hold-duration",
        str(cfg.gripper_close_hold_duration),
        "--post-grasp-lift-m",
        str(float(overrides.get("post_grasp_lift_m", cfg.post_grasp_lift_m))),
        "--post-grasp-lift-duration",
        str(cfg.post_grasp_lift_duration),
        "--place-q",
        ",".join(f"{v:.9f}" for v in place_q),
        "--place-duration",
        str(cfg.place_duration),
        "--gripper-open",
        str(cfg.gripper_open),
        "--gripper-close",
        str(cfg.gripper_close),
        "--quick-detect-preview-s",
        str(cfg.quick_detect_preview_s),
        "--execute",
        "--external-replay-server",
        "--replay-command-format",
        "dual",
        "--replay-command-side",
        "right",
        "--replay-command-port",
        str(cfg.command_port),
        "--no-auto-replay-command-port",
        "--follower-can",
        cfg.right_can,
        "--follower-only",
        "--camera-backend",
        camera_backend,
        "--camera-index",
        str(camera_index),
        "--black-v-max",
        str(int(overrides.get("black_v_max", cfg.black_v_max))),
        "--min-area-px",
        str(int(overrides.get("min_area_px", cfg.min_area_px))),
        "--target-selection",
        str(overrides.get("target_selection", cfg.target_selection)),
        "--track-roi-px",
        str(float(overrides.get("track_roi_px", cfg.track_roi_px))),
        "--no-track-fallback-to-full",
    ]
    if cfg.sudo_password:
        args.extend(["--sudo-password", cfg.sudo_password])
    log("running 2D grasp align_once_move.py")
    proc = subprocess.Popen(args, cwd=str(ROOT))
    while True:
        if state.abort_requested or state.stop_service:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
            raise RuntimeError("abort requested during grasp")
        ret = proc.poll()
        if ret is not None:
            if ret != 0:
                raise RuntimeError(f"2D grasp failed with exit code {ret}")
            return
        time.sleep(0.2)


def execute_job(cfg: ServiceConfig, state: ServiceState, job_id: str) -> None:
    job_dir = resolve_job_dir(cfg.jobs_root, job_id)
    if not job_dir.is_dir():
        raise FileNotFoundError(f"job folder not found: {job_dir}")
    action1, action2 = preflight_job(job_dir)
    overrides = load_job_overrides(job_dir)
    runtime = cfg.runtime_root / normalize_job_id(job_id)
    runtime.mkdir(parents=True, exist_ok=True)
    observe = runtime / "observe_from_action1_last_right.csv"
    place_file = runtime / "place_q_from_action2_start_right.txt"
    place_q = write_observe_and_place(action1, action2, observe, place_file)

    client = ReplayClient(cfg, state)
    log(f"job {job_id}: dir={job_dir}")
    log(f"job {job_id}: play action1 ({action1})")
    client.play_and_wait(action1, f"job-{job_id}-action1")
    try:
        log(f"job {job_id}: 2D grasp")
        run_grasp(cfg, state, observe, place_q, overrides)
    except Exception as exc:
        if cfg.skip_action2_on_grasp_fail:
            raise RuntimeError(f"grasp failed; skipped action2: {exc}") from exc
        log(f"[WARN] grasp failed but continue_action2 enabled: {exc}")
    log(f"job {job_id}: play action2 ({action2})")
    client.play_and_wait(action2, f"job-{job_id}-action2")


def status_payload(cfg: ServiceConfig, state: ServiceState) -> str:
    jobs = ",".join(discover_jobs(cfg.jobs_root)) or "-"
    with state.lock:
        return (
            f"ok|busy={int(state.busy)}|job={state.current_job or '-'}|"
            f"startup_done={int(state.startup_done)}|last={state.last_result}|"
            f"error={state.last_error or '-'}|jobs={jobs}"
        )


def handle_command(cfg: ServiceConfig, state: ServiceState, text: str, addr: tuple) -> str:
    cmd = text.strip()
    if not cmd:
        return "error|empty"
    low = cmd.lower()

    if low in {"status", "list"}:
        return status_payload(cfg, state)
    if low in {"shutdown", "quit", "exit"}:
        state.stop_service = True
        state.abort_requested = True
        return "ok|shutdown_requested"
    if low in {"abort", "stop"}:
        state.abort_requested = True
        return "ok|abort_requested"

    job_id = None
    if low.startswith("run|"):
        job_id = normalize_job_id(cmd.split("|", 1)[1])
    elif low.startswith("run "):
        job_id = normalize_job_id(cmd.split(None, 1)[1])
    else:
        # bare id: "1" / "2" / "job1"
        job_id = normalize_job_id(cmd)

    if not job_id:
        return "error|bad_command"

    job_dir = resolve_job_dir(cfg.jobs_root, job_id)
    if not job_dir.is_dir():
        return f"error|unknown_job|{job_folder_name(job_id)}|{job_dir}"
    if not (job_dir / "action1.csv").is_file() or not (job_dir / "action2.csv").is_file():
        return f"error|missing_csv|{job_dir}"

    with state.lock:
        if state.busy:
            return f"busy|job={state.current_job}"
        if not state.startup_done:
            return "error|startup_not_done"
        state.busy = True
        state.current_job = job_id
        state.abort_requested = False
        state.last_result = "running"
        state.last_error = ""

    def worker() -> None:
        try:
            execute_job(cfg, state, job_id)
            with state.lock:
                state.last_result = "success"
                state.last_error = ""
            log(f"job {job_id}: success")
        except Exception as exc:  # noqa: BLE001 - surface to client/logs
            with state.lock:
                state.last_result = "failed"
                state.last_error = str(exc)
            log(f"job {job_id}: FAILED: {exc}")
        finally:
            with state.lock:
                state.busy = False
                state.current_job = None

    threading.Thread(target=worker, name=f"job-{job_id}", daemon=True).start()
    return f"accepted|job={job_id}|from={addr[0]}:{addr[1]}"


def resolve_startup_csv(cfg: ServiceConfig) -> Path:
    """Use field steady CSV under recordings/tasks (same as old pipeline)."""
    candidates: list[Path] = []
    env = os.environ.get("OPENARM_STARTUP_CSV", "").strip()
    if env:
        candidates.append(Path(env).expanduser())

    # Canonical field path (run_steady_then_visual_grasp / Quest startup):
    #   <vr_control>/openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv
    vr_control_root = ROOT.parent  # .../skewer_topdown_2d_minimal -> .../vr_control
    candidates.extend(
        [
            vr_control_root
            / "openarm_teleop"
            / "openarm_inplace"
            / "recordings"
            / "tasks"
            / "steady_action_on_table.csv",
            ROOT
            / "vendor"
            / "openarm_inplace"
            / "recordings"
            / "tasks"
            / "steady_action_on_table.csv",
            # Optional packaging override / root copy fallbacks.
            ROOT / "jobs" / "startup" / "action_start.csv",
            vr_control_root / "steady_action_on_table.csv",
        ]
    )

    tried: list[str] = []
    for raw in candidates:
        path = raw.expanduser().resolve()
        tried.append(str(path))
        if path.is_file():
            log(f"startup action: {path}")
            return path

    raise FileNotFoundError(
        "missing startup/steady CSV. Tried:\n  - "
        + "\n  - ".join(tried)
        + "\nPlace steady_action_on_table.csv under "
        "openarm_teleop/openarm_inplace/recordings/tasks/ "
        "(or set OPENARM_STARTUP_CSV / jobs/startup/action_start.csv)."
    )


def serve(cfg: ServiceConfig) -> int:
    state = ServiceState()
    cfg.runtime_root.mkdir(parents=True, exist_ok=True)
    log(f"jobs root (recordings): {cfg.jobs_root}")
    log(f"runtime root: {cfg.runtime_root}")
    startup_csv = resolve_startup_csv(cfg)

    # Boot server with startup CSV so initial transition has a valid dual file.
    start_replay_server(cfg, state, startup_csv)
    client = ReplayClient(cfg, state)

    def _on_signal(signum: int, _frame: Any) -> None:
        log(f"signal {signum}: shutting down")
        state.stop_service = True
        state.abort_requested = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        client.wait_ready()
        log("playing startup action once")
        client.play_and_wait(startup_csv, "startup")
        state.startup_done = True
        log(
            "ready. Send UDP to "
            f":{cfg.job_listen_port}  commands: 1 | 2 | 3 | status | abort | shutdown"
        )
        log(f"available jobs: {discover_jobs(cfg.jobs_root)}")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", cfg.job_listen_port))
        sock.settimeout(0.5)
        try:
            while not state.stop_service:
                # Detect dead control process.
                if state.server_proc is not None and state.server_proc.poll() is not None:
                    raise RuntimeError(
                        f"dual replay server exited unexpectedly code={state.server_proc.returncode}"
                    )
                try:
                    data, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                text = data.decode("utf-8", errors="replace").strip()
                log(f"cmd from {addr}: {text}")
                try:
                    reply = handle_command(cfg, state, text, addr)
                except Exception as exc:  # noqa: BLE001
                    reply = f"error|{exc}"
                sock.sendto(reply.encode("utf-8"), addr)
                log(f"reply -> {addr}: {reply}")
        finally:
            sock.close()
    finally:
        log("cleanup: shutdown replay server")
        client.shutdown_server()
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Persistent skewer action1->2D->action2 job service")
    p.add_argument(
        "--jobs-root",
        type=Path,
        default=default_recordings_root(),
        help="Directory containing job1/, job2/, ... (default: openarm_inplace/recordings)",
    )
    p.add_argument(
        "--runtime-root",
        type=Path,
        default=ROOT / "jobs" / "_runtime",
        help="Writable dir for observe/place bridge files",
    )
    p.add_argument(
        "--openarm-dir",
        type=Path,
        default=ROOT / "vendor" / "openarm_inplace",
    )
    p.add_argument(
        "--align-script",
        type=Path,
        default=ROOT / "scripts" / "align_once_move.py",
    )
    p.add_argument("--command-port", type=int, default=47920)
    p.add_argument("--status-port", type=int, default=47921)
    p.add_argument("--job-listen-port", type=int, default=47925)
    p.add_argument(
        "--left-can",
        default=os.environ.get("OPENARM_LEFT_FOLLOWER_CAN", "follower_left"),
    )
    p.add_argument(
        "--right-can",
        default=os.environ.get("OPENARM_RIGHT_FOLLOWER_CAN", "follower_right"),
    )
    p.add_argument("--camera-backend", default=os.environ.get("OPENARM_RIGHT_CAMERA_BACKEND", "opencv"))
    p.add_argument("--camera-index", type=int, default=int(os.environ.get("OPENARM_RIGHT_CAMERA_INDEX", "16")))
    p.add_argument("--sudo-password", default=os.environ.get("OPENARM_SUDO_PASSWORD", ""))
    p.add_argument("--grasp-descent-m", type=float, default=0.04)
    p.add_argument(
        "--continue-action2-on-grasp-fail",
        action="store_true",
        help="By default skip action2 if 2D grasp fails.",
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    cfg = ServiceConfig(
        jobs_root=args.jobs_root.expanduser().resolve(),
        openarm_dir=args.openarm_dir.expanduser().resolve(),
        align_script=args.align_script.expanduser().resolve(),
        runtime_root=args.runtime_root.expanduser().resolve(),
        command_port=args.command_port,
        status_port=args.status_port,
        job_listen_port=args.job_listen_port,
        left_can=args.left_can,
        right_can=args.right_can,
        camera_backend=args.camera_backend,
        camera_index=args.camera_index,
        sudo_password=args.sudo_password,
        grasp_descent_m=args.grasp_descent_m,
        skip_action2_on_grasp_fail=not args.continue_action2_on_grasp_fail,
    )
    return serve(cfg)


if __name__ == "__main__":
    raise SystemExit(main())
