#!/usr/bin/env bash
# Bring up CAN-FD and run KER → real OpenArm teleop (+ record panel).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/env.sh"
PY="${OPENARM_PYTHON}"
PASS="${OPENARM_SUDO_PASSWORD:-}"

run_sudo() {
  if [[ -n "${PASS}" ]]; then
    printf '%s\n' "${PASS}" | sudo -S -p "" "$@"
  else
    sudo "$@"
  fi
}

configure_can() {
  local bitrate="${OPENARM_CAN_BITRATE:-1000000}"
  local dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"
  for iface in "$@"; do
    echo "[ker-real] configuring ${iface} CAN-FD ${bitrate}/${dbitrate}"
    run_sudo ip link set "${iface}" down || true
    run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    run_sudo ip link set "${iface}" up
  done
}

if lsusb -d 303a:4002 >/dev/null 2>&1; then
  BUS=$(lsusb -d 303a:4002 | awk '{print $2}')
  DEV=$(lsusb -d 303a:4002 | awk '{print $4}' | tr -d ':')
  run_sudo chmod a+rw "/dev/bus/usb/${BUS}/${DEV}" || true
fi

RIGHT_CAN="${OPENARM_RIGHT_CAN}"
LEFT_CAN="${OPENARM_LEFT_CAN}"
if [[ "${OPENARM_SKIP_CAN_CONFIG:-0}" != "1" ]]; then
  configure_can "${RIGHT_CAN}" "${LEFT_CAN}"
fi
ip -br link | grep -E 'can[0-9]' || true

pkill -9 -f 'ker_real_teleop.py' 2>/dev/null || true
pkill -9 -f 'ker_record_panel.py' 2>/dev/null || true
sleep 0.5

if [[ ! -x "${PY}" ]] && ! command -v "${PY}" >/dev/null 2>&1; then
  echo "[ker-real] python not found: ${PY} (set OPENARM_PYTHON)" >&2
  exit 1
fi

exec "${PY}" -u "${ROOT}/ker_real_teleop.py" \
  --right-can "${RIGHT_CAN}" \
  --left-can "${LEFT_CAN}" \
  --swap-j67 \
  --no-negate-j6 \
  --negate-j7 \
  --invert-left-grip \
  --panel \
  "$@"
