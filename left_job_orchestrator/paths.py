"""Canonical paths under vr_control for the left-arm job orchestrator."""

from __future__ import annotations

from pathlib import Path

ORCH_DIR = Path(__file__).resolve().parent
VR_CONTROL_ROOT = ORCH_DIR.parent

OPENARM_INPLACE = VR_CONTROL_ROOT / "openarm_teleop" / "openarm_inplace"
SKEWER_LEFT = OPENARM_INPLACE / "skewer_topdown_2d_minimal"
FLOW_SCRIPT = SKEWER_LEFT / "scripts" / "dual_left_visual_flow.py"

RECORDINGS_ROOT = OPENARM_INPLACE / "recordings"
TASKS_DIR = RECORDINGS_ROOT / "tasks"
STEADY_CSV = TASKS_DIR / "steady_action_on_table.csv"

LEFT_BUNDLE = OPENARM_INPLACE / "left_grasp_exec_bundle"
LEFT_CONFIG = LEFT_BUNDLE / "config" / "skewer_align_config_left.json"
# Grasp start pose for tcp_forward_grasp (replaces old observe CSV).
DEFAULT_BEGIN_CSV = LEFT_BUNDLE / "begin.csv"
DEFAULT_OBSERVE_CSV = DEFAULT_BEGIN_CSV

TCP_FORWARD_GRASP = VR_CONTROL_ROOT / "tcp_forward_grasp"
DEFAULT_GRIPPER_GATE_JSON = TCP_FORWARD_GRASP / "config" / "gripper_gate_left.json"

RUNTIME_ROOT = ORCH_DIR / "runtime"
OUTPUT_ROOT = ORCH_DIR / "outputs"


def job_dir(jobs_root: Path, job_id: str) -> Path:
    jid = job_id.strip()
    low = jid.lower()
    if low.startswith("job"):
        jid = jid[3:]
    jid = jid.strip()
    if jid.isdigit():
        return (jobs_root / f"job{jid}").resolve()
    return (jobs_root / jid).resolve()


def normalize_job_id(raw: str) -> str:
    text = raw.strip()
    low = text.lower()
    if low.startswith("job"):
        text = text[3:]
    return text.strip()


def discover_jobs(jobs_root: Path) -> list[str]:
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
        if (child / "action1.csv").is_file():
            jobs.append(slot)
    return jobs
