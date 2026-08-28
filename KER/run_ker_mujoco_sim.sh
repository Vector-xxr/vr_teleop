#!/usr/bin/env bash
# KER → MuJoCo visual teleop (no dora).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/env.sh"
PY="${OPENARM_PYTHON}"

if [[ ! -x "${PY}" ]] && ! command -v "${PY}" >/dev/null 2>&1; then
  echo "[ker-sim] python not found: ${PY}" >&2
  exit 1
fi

EXTRA=()
if [[ -n "${OPENARM_MUJOCO_SCENE:-}" ]]; then
  EXTRA+=(--xml "${OPENARM_MUJOCO_SCENE}")
fi

exec "${PY}" "${ROOT}/ker_mujoco_teleop.py" "${EXTRA[@]}" "$@"
