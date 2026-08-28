#!/usr/bin/env bash
# Start left-arm job orchestrator (steady once -> idle -> jobN on TCP :47925).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_CONTROL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# systemd does not inherit an interactive shell's ROS environment. The vendor
# replay binary links ROS libraries (for example librcutils.so), so source the
# setup in the actual ExecStart process rather than only in ExecStartPre.
ROS_SETUP="${OPENARM_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
if [[ -f "${ROS_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
else
  echo "[left-job][ERROR] ROS setup not found: ${ROS_SETUP}" >&2
  echo "[left-job][HINT] Set OPENARM_ROS_SETUP in env.systemd." >&2
  exit 1
fi

PYTHON_BIN="${OPENARM_PYTHON:-}"
if [[ -z "${PYTHON_BIN}" ]]; then
  if [[ -x "${HOME}/miniconda3/envs/openarm_teleop1/bin/python" ]]; then
    PYTHON_BIN="${HOME}/miniconda3/envs/openarm_teleop1/bin/python"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
  fi
fi
if [[ -z "${PYTHON_BIN}" || ! -x "${PYTHON_BIN}" ]]; then
  echo "[left-job][ERROR] Python not found. Set OPENARM_PYTHON to an executable path." >&2
  exit 1
fi

if [[ -z "${OPENARM_SUDO_PASSWORD:-}" || "${OPENARM_SUDO_PASSWORD}" == "CHANGE_ME" ]]; then
  echo "[left-job][ERROR] OPENARM_SUDO_PASSWORD is not configured." >&2
  echo "[left-job][HINT] Edit left_job_orchestrator/env.systemd for systemd startup." >&2
  exit 1
fi

export PYTHONPATH="${VR_CONTROL_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

JOBS_ROOT="${OPENARM_SKEWER_JOBS_ROOT:-${VR_CONTROL_DIR}/openarm_teleop/openarm_inplace/recordings}"
JOB_LISTEN_PORT="${OPENARM_SKEWER_JOB_LISTEN_PORT:-47925}"

ARGS=(
  --jobs-root "${JOBS_ROOT}"
  --job-listen-port "${JOB_LISTEN_PORT}"
)

echo "[left-job] python=${PYTHON_BIN}"
echo "[left-job] ros-setup=${ROS_SETUP}"
echo "[left-job] jobs-root=${JOBS_ROOT}"
echo "[left-job] listen=:${JOB_LISTEN_PORT}"
echo "[left-job] expect: ${JOBS_ROOT}/jobN/action1.csv (plus optional numbered steps)"

cd "${VR_CONTROL_DIR}"
exec "${PYTHON_BIN}" -m left_job_orchestrator.service "${ARGS[@]}" "$@"
