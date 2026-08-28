#!/usr/bin/env bash
# Wait for udev/USB-CAN enumeration. Read-only: does not configure or raise CAN.
set -euo pipefail

TIMEOUT_S="${OPENARM_CAN_WAIT_TIMEOUT_S:-60}"
INTERVAL_S="${OPENARM_CAN_WAIT_INTERVAL_S:-1}"
LEFT_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-follower_left}"
RIGHT_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-follower_right}"

command -v ip >/dev/null 2>&1 || {
  echo "[can-wait][ERROR] ip command not found (install iproute2)" >&2
  exit 1
}

start="${SECONDS}"
while true; do
  missing=()
  for iface in "${LEFT_CAN}" "${RIGHT_CAN}"; do
    if ! ip link show "${iface}" >/dev/null 2>&1; then
      missing+=("${iface}")
    fi
  done

  if [[ "${#missing[@]}" -eq 0 ]]; then
    echo "[can-wait] interfaces ready: ${LEFT_CAN}, ${RIGHT_CAN}"
    exit 0
  fi

  if (( SECONDS - start >= TIMEOUT_S )); then
    echo "[can-wait][ERROR] timed out after ${TIMEOUT_S}s; missing: ${missing[*]}" >&2
    exit 1
  fi
  echo "[can-wait] waiting for: ${missing[*]}"
  sleep "${INTERVAL_S}"
done
