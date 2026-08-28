#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/env.sh"
exec "${OPENARM_PYTHON}" "${ROOT}/ker_record_panel.py" "$@"
