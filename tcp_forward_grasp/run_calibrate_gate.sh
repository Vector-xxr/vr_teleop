#!/usr/bin/env bash
# Calibrate fixed gripper-opening box (white gripper). Save JSON for grasp reuse.
# Prefer: gripper OPEN at approach scale, white plate NOT in view.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_CONTROL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

PYTHON_BIN="${OPENARM_PYTHON:-}"
if [[ -z "${PYTHON_BIN}" ]]; then
  if [[ -x "${HOME}/miniconda3/envs/openarm_teleop1/bin/python" ]]; then
    PYTHON_BIN="${HOME}/miniconda3/envs/openarm_teleop1/bin/python"
  elif [[ -x "/home/knowin/miniconda3/envs/openarm_teleop1/bin/python" ]]; then
    PYTHON_BIN="/home/knowin/miniconda3/envs/openarm_teleop1/bin/python"
  else
    PYTHON_BIN="python3"
  fi
fi

export PYTHONPATH="${VR_CONTROL_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

echo "[calib-gate] python=${PYTHON_BIN}"
cd "${VR_CONTROL_DIR}"
exec "${PYTHON_BIN}" -m tcp_forward_grasp.calibrate_gripper_gate "$@"
