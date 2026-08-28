#!/usr/bin/env bash
# One-shot HW align for this public machine (follower_* + KER USB).
# Usage: export OPENARM_SUDO_PASSWORD='...'; ./align_hw_once.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/env.sh"
PASS="${OPENARM_SUDO_PASSWORD:-}"
run_sudo() {
  if [[ -n "${PASS}" ]]; then printf '%s\n' "${PASS}" | sudo -S -p '' "$@"; else sudo "$@"; fi
}
bitrate="${OPENARM_CAN_BITRATE:-1000000}"
dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"
for iface in "${OPENARM_RIGHT_CAN}" "${OPENARM_LEFT_CAN}"; do
  echo "[align] CAN-FD ${iface} ${bitrate}/${dbitrate}"
  run_sudo ip link set "${iface}" down || true
  run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
  run_sudo ip link set "${iface}" up
done
ip -br link | grep -E 'follower|can' || true
if lsusb -d 303a:4002 >/dev/null 2>&1; then
  BUS=$(lsusb -d 303a:4002 | awk '{print $2}')
  DEV=$(lsusb -d 303a:4002 | awk '{print $4}' | tr -d ':')
  run_sudo chmod a+rw "/dev/bus/usb/${BUS}/${DEV}"
  echo "[align] KER USB rw: /dev/bus/usb/${BUS}/${DEV}"
else
  echo "[align][WARN] KER USB 303a:4002 not plugged in"
fi
echo "[align] done. Start teleop: ./run_ker_real_teleop.sh"
