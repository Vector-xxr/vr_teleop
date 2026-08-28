#!/usr/bin/env bash
# Disable and remove openarm-left-job.service from systemd.
set -euo pipefail

UNIT_DST="/etc/systemd/system/openarm-left-job.service"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_CONTROL="$(cd "${ROOT}/.." && pwd)"
ENV_DST="${VR_CONTROL}/left_job_orchestrator/env.systemd"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 1
fi

systemctl disable --now openarm-left-job.service 2>/dev/null || true
rm -f "${UNIT_DST}"
systemctl daemon-reload
echo "Removed ${UNIT_DST}"
if [[ -f "${ENV_DST}" ]]; then
  echo "Left in place (secrets): ${ENV_DST}"
  echo "Delete manually if you want: rm ${ENV_DST}"
fi
