"""Paths for the TCP-forward grasp module."""

from __future__ import annotations

from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
VR_CONTROL_ROOT = MODULE_DIR.parent

OPENARM_INPLACE = VR_CONTROL_ROOT / "openarm_teleop" / "openarm_inplace"
SKEWER_LEFT = OPENARM_INPLACE / "skewer_topdown_2d_minimal"
FLOW_SCRIPT = SKEWER_LEFT / "scripts" / "dual_left_visual_flow.py"

LEFT_BUNDLE = OPENARM_INPLACE / "left_grasp_exec_bundle"
LEFT_CONFIG = LEFT_BUNDLE / "config" / "skewer_align_config_left.json"
# Recorded left-arm approach / wait motion played before vision align.
DEFAULT_BEGIN_CSV = LEFT_BUNDLE / "begin.csv"
DEFAULT_OBSERVE_CSV = DEFAULT_BEGIN_CSV

RECORDINGS_ROOT = OPENARM_INPLACE / "recordings"
STEADY_CSV = RECORDINGS_ROOT / "tasks" / "steady_action_on_table.csv"
DEFAULT_ACTION1 = RECORDINGS_ROOT / "tasks" / "action1.csv"
DEFAULT_ACTION2 = RECORDINGS_ROOT / "tasks" / "action2.csv"

OUTPUT_ROOT = MODULE_DIR / "outputs"
DEFAULT_GRIPPER_GATE_JSON = MODULE_DIR / "config" / "gripper_gate_left.json"
