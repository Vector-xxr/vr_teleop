#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"

if ! command -v conda >/dev/null 2>&1; then
  if [[ -x "${HOME}/miniconda3/bin/conda" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
  else
    echo "conda not found. Install Miniconda first."
    exit 1
  fi
fi

if conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  echo "Updating existing conda env: ${ENV_NAME}"
  conda env update -n "${ENV_NAME}" -f "${ROOT_DIR}/environment.yml" --prune
else
  echo "Creating conda env: ${ENV_NAME}"
  conda env create -f "${ROOT_DIR}/environment.yml"
fi

echo "Quest conda env ready: ${ENV_NAME}"
echo "Activate with: conda activate ${ENV_NAME}"
