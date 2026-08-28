#!/usr/bin/env bash
# Record TCP/joint trajectories during Quest teleop, then plot continuity/smoothness.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
INPLACE_DIR="${REPO_ROOT}/openarm_teleop/openarm_inplace"
ENV_NAME="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"

MODE="sim"
if [[ "${1:-}" == "--real" ]]; then
  MODE="real"
  shift
elif [[ "${1:-}" == "--sim" ]]; then
  MODE="sim"
  shift
fi

OUT_DIR="${1:-/tmp/openarm_teleop_traj_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${OUT_DIR}"
export OPENARM_TELEOP_TRAJ_LOG="${OUT_DIR}"

PLOT=1
if [[ "${PLOT_ON_EXIT:-1}" == "0" ]]; then
  PLOT=0
fi

cleanup() {
  local code=$?
  echo ""
  echo "[traj-test] stopped (exit=${code})"
  if [[ "${PLOT}" == "1" ]] && compgen -G "${OUT_DIR}/*_tcp.csv" >/dev/null; then
    echo "[traj-test] plotting logs in ${OUT_DIR} ..."
    if command -v conda >/dev/null 2>&1; then
      # shellcheck disable=SC1091
      source "$(conda info --base)/etc/profile.d/conda.sh"
      conda activate "${ENV_NAME}" 2>/dev/null || true
    fi
    python3 "${ROOT_DIR}/test_teleop_trajectory.py" plot --dir "${OUT_DIR}" || true
  else
    echo "[traj-test] no CSV logs found under ${OUT_DIR}"
  fi
}
trap cleanup EXIT INT TERM

echo "=============================================="
echo " OpenArm teleop trajectory test"
echo " Log dir : ${OUT_DIR}"
echo " Mode    : ${MODE}"
echo "----------------------------------------------"
echo " 1) Quest: open https://<LAN-IP>:8443/client.html"
echo " 2) Start AR Tracking, hold Grip and move"
echo " 3) Ctrl+C here when done -> auto plot"
echo "=============================================="

# Build if needed
if [[ ! -x "${INPLACE_DIR}/build/quest_cartesian_control" ]]; then
  echo "[traj-test] building quest_cartesian_control..."
  cmake -S "${INPLACE_DIR}" -B "${INPLACE_DIR}/build"
  cmake --build "${INPLACE_DIR}/build" --target quest_cartesian_control -j"$(nproc)"
fi

if [[ "${MODE}" == "sim" ]]; then
  "${ROOT_DIR}/run_openarm_cartesian_bimanual_sim.sh"
else
  "${ROOT_DIR}/run_openarm_cartesian_bimanual.sh"
fi
