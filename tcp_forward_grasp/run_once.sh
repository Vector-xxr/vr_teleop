#!/usr/bin/env bash
# Default: steady -> action1 -> quintic to begin -> grasp -> quintic -> action2
# Skip prep:  ./run_once.sh --skip-steady
# Grasp only: ./run_once.sh --grasp-only
# Debug LR:   ./run_once.sh --lr-only
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

echo "[tcp-forward] python=${PYTHON_BIN}"
echo "[tcp-forward] default=steady->action1->quintic->grasp->quintic->action2"
echo "[tcp-forward] motor temps: live [motor-temp] lines + run_dir/motor_torque_temp.csv @ 100Hz + motor_temps_live.log"
# Optional: OPENARM_REPLAY_TEMP_PRINT_S=1  OPENARM_REPLAY_MOTOR_LOG_HZ=100

cd "${VR_CONTROL_DIR}"
exec "${PYTHON_BIN}" -m tcp_forward_grasp.flow "$@"
