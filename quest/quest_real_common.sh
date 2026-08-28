#!/usr/bin/env bash
# Shared helpers for Quest Cartesian real-hardware launch scripts.

quest_activate_conda() {
  local env_name="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"
  if command -v conda >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
  elif [[ -x "${HOME}/miniconda3/bin/conda" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
  else
    echo "[quest][ERROR] conda not found. Run ${QUEST_DIR}/setup_venv.sh first." >&2
    exit 1
  fi
  if ! conda env list | awk '{print $1}' | grep -qx "${env_name}"; then
    echo "[quest][ERROR] Conda env '${env_name}' not found. Run: ${QUEST_DIR}/setup_venv.sh" >&2
    exit 1
  fi
  conda activate "${env_name}"
}

quest_run_sudo() {
  if sudo -n true >/dev/null 2>&1; then
    sudo "$@"
  elif [[ -n "${OPENARM_SUDO_PASSWORD:-}" ]]; then
    printf '%s\n' "${OPENARM_SUDO_PASSWORD}" | sudo -S -p "" "$@"
  else
    echo "[quest] sudo required to configure CAN interfaces..."
    sudo "$@"
  fi
}

quest_configure_can_interfaces() {
  if [[ "${OPENARM_SKIP_CAN_CONFIG:-0}" == "1" ]]; then
    return
  fi
  if [[ "${OPENARM_AUTO_CONFIG_CAN:-1}" != "1" ]]; then
    return
  fi

  local bitrate="${OPENARM_CAN_BITRATE:-1000000}"
  local dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"
  local configured=" "

  for iface in "$@"; do
    if [[ "${configured}" == *" ${iface} "* ]]; then
      continue
    fi
    configured="${configured}${iface} "

    if ! ip link show "${iface}" >/dev/null 2>&1; then
      echo "[quest][ERROR] SocketCAN interface '${iface}' not found." >&2
      echo "[quest][HINT] ip link show type can" >&2
      exit 1
    fi

    echo "[quest] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
    quest_run_sudo ip link set "${iface}" down
    quest_run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    quest_run_sudo ip link set "${iface}" up
  done
}

quest_check_can_interface() {
  local iface="$1"
  if ! command -v ip >/dev/null 2>&1; then
    return
  fi
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    echo "[quest][ERROR] SocketCAN interface '${iface}' not found." >&2
    exit 1
  fi
  if ! ip link show "${iface}" | grep -q "state UP"; then
    echo "[quest][ERROR] SocketCAN interface '${iface}' is not UP." >&2
    echo "[quest][HINT] export OPENARM_AUTO_CONFIG_CAN=1 or bring it up manually." >&2
    exit 1
  fi
}

quest_port_in_use() {
  local port="$1"
  # Prefer process check: ss/lsof may miss listeners in some environments.
  if pgrep -af '[q]uest_bridge\.py' 2>/dev/null | grep -qE -- "--http-port[ =]${port}([[:space:]]|$)"; then
    return 0
  fi
  if command -v ss >/dev/null 2>&1; then
    if ss -ltn 2>/dev/null | grep -qE ":${port}([^0-9]|$)"; then
      return 0
    fi
  fi
  if command -v lsof >/dev/null 2>&1; then
    if lsof -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1; then
      return 0
    fi
  fi
  # Last resort: try binding (fails if something else holds the port).
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$port" <<'PY' >/dev/null 2>&1
import socket, sys
port = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
except OSError:
    sys.exit(0)  # in use
finally:
    s.close()
sys.exit(1)  # free
PY
    return $?
  fi
  return 1
}

quest_stop_stale_control() {
  if pgrep -f '[q]uest_cartesian_control' >/dev/null 2>&1; then
    echo "[quest] stopping stale quest_cartesian_control..."
    pkill -TERM -f '[q]uest_cartesian_control' 2>/dev/null || true
    sleep 0.3
    pkill -KILL -f '[q]uest_cartesian_control' 2>/dev/null || true
  fi
}

quest_build_control() {
  echo "[quest] Building quest_cartesian_control..."
  cmake -S "${INPLACE_DIR}" -B "${INPLACE_DIR}/build"
  cmake --build "${INPLACE_DIR}/build" --target quest_cartesian_control -j"$(nproc)"
}

quest_wait_pid() {
  local pid="$1"
  local timeout_s="${2:-8}"
  local waited=0
  while kill -0 "${pid}" 2>/dev/null; do
    if awk -v w="${waited}" -v t="${timeout_s}" 'BEGIN { exit !(w >= t) }'; then
      return 1
    fi
    sleep 0.1
    waited="$(awk -v w="${waited}" 'BEGIN { printf "%.1f", w + 0.1 }')"
  done
  wait "${pid}" 2>/dev/null || true
  return 0
}

quest_kill_pid() {
  local pid="$1"
  local name="$2"
  local grace_s="${3:-8}"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi
  echo "[quest] stopping ${name} (pid=${pid})..."
  kill -TERM "${pid}" 2>/dev/null || true
  if ! quest_wait_pid "${pid}" "${grace_s}"; then
    echo "[quest] ${name} still running after ${grace_s}s; SIGKILL"
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

quest_cleanup() {
  # Prevent re-entrancy when EXIT fires after INT/TERM trap.
  if [[ "${QUEST_CLEANUP_DONE:-0}" == "1" ]]; then
    return 0
  fi
  QUEST_CLEANUP_DONE=1

  local control_grace="${OPENARM_QUEST_SHUTDOWN_WAIT:-15}"
  local panel_grace=3

  quest_kill_pid "${RECORD_PANEL_PID:-}" "record panel" "${panel_grace}"
  RECORD_PANEL_PID=""
  quest_kill_pid "${JOG_PANEL_PID:-}" "jog panel" "${panel_grace}"
  JOG_PANEL_PID=""
  # Control may be in Kd damping (~6s); allow longer than panels.
  quest_kill_pid "${CONTROL_PID:-}" "quest_cartesian_control" "${control_grace}"
  CONTROL_PID=""
  if [[ "${OWNED_BRIDGE:-0}" -eq 1 ]]; then
    quest_kill_pid "${BRIDGE_PID:-}" "quest_bridge" 5
    BRIDGE_PID=""
  fi
  # Belt-and-suspenders for orphans (e.g. pipeline/tee / hung join).
  quest_stop_stale_control
}

quest_start_ui_panels() {
  if [[ "${OPENARM_QUEST_UI:-1}" != "1" ]]; then
    echo "[quest] UI panels disabled (OPENARM_QUEST_UI=0)"
    return
  fi
  local record_port="${OPENARM_QUEST_UI_RECORD_PORT:-47930}"
  local jog_port="${OPENARM_QUEST_UI_JOG_PORT:-47931}"
  # Panel displays mm/s; controller uses OPENARM_QUEST_JOG_SPEED (m/s, default 0.02).
  local jog_speed_mms
  jog_speed_mms="$(python3 -c 'import os; print(float(os.environ.get("OPENARM_QUEST_JOG_SPEED","0.02"))*1000.0)')"
  # Ensure GUI children (ffplay) can talk to the desktop session.
  if [[ -z "${DISPLAY:-}" ]]; then
    export DISPLAY=:0
  fi
  if [[ -z "${XAUTHORITY:-}" ]]; then
    for _xa in /run/user/"${UID:-1000}"/.mutter-Xwaylandauth.* \
               /run/user/"${UID:-1000}"/gdm/Xauthority \
               "${HOME}/.Xauthority"; do
      if [[ -f "${_xa}" ]]; then
        export XAUTHORITY="${_xa}"
        break
      fi
    done
  fi
  export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-x11}"
  export SDL_AUDIODRIVER="${SDL_AUDIODRIVER:-dummy}"

  echo "[quest] Launching action-record + TCP-jog panels (hold-to-jog ${jog_speed_mms} mm/s)..."
  echo "[quest] Camera auto-preview: OPENARM_QUEST_RECORD_CAMS=${OPENARM_QUEST_RECORD_CAMS:-8,16}"
  echo "[quest]   DISPLAY=${DISPLAY}  log=/tmp/openarm_cam_preview.log"
  echo "[quest]   disable auto: OPENARM_QUEST_RECORD_CAM_AUTOSTART=0"
  python3 "${QUEST_DIR}/quest_action_record_panel.py" --port "${record_port}" &
  RECORD_PANEL_PID=$!
  python3 "${QUEST_DIR}/quest_tcp_jog_panel.py" --port "${jog_port}" --speed "${jog_speed_mms}" &
  JOG_PANEL_PID=$!
  sleep 0.3
  if ! kill -0 "${RECORD_PANEL_PID}" 2>/dev/null; then
    echo "[quest][WARN] record panel exited immediately" >&2
  fi
}

quest_start_bridge() {
  local bimanual_flag=()
  if [[ "${QUEST_BIMANUAL:-0}" == "1" ]]; then
    bimanual_flag=(--udp-left-port "${QUEST_PORT_LEFT}" --udp-right-port "${QUEST_PORT_RIGHT}")
  else
    bimanual_flag=(--single-arm --udp-port "${QUEST_PORT_RIGHT}")
  fi

  if quest_port_in_use "${HTTP_PORT}"; then
    echo "[quest] Quest bridge already listening on :${HTTP_PORT}, reusing it"
    echo "        (stop with: pkill -f quest_bridge.py)"
    OWNED_BRIDGE=0
    BRIDGE_PID=""
    return
  fi

  echo "[quest] Launching Quest bridge (HTTPS)..."
  python "${QUEST_DIR}/quest_bridge.py" \
    --host 0.0.0.0 \
    --http-port "${HTTP_PORT}" \
    "${bimanual_flag[@]}" &
  BRIDGE_PID=$!
  OWNED_BRIDGE=1
  sleep 1
  if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "[quest][ERROR] Quest bridge exited immediately (port :${HTTP_PORT} busy?)." >&2
    echo "[quest][HINT] Check: pgrep -af quest_bridge.py ; ss -ltn | grep ${HTTP_PORT}" >&2
    echo "[quest][HINT] Or stop the old one: pkill -f quest_bridge.py" >&2
    OWNED_BRIDGE=0
    BRIDGE_PID=""
    return 1
  fi
}

quest_print_quest_url() {
  local lan_ip=""
  if command -v hostname >/dev/null 2>&1; then
    lan_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  if [[ -z "${lan_ip}" ]]; then
    lan_ip="<LAN-IP>"
  fi
  echo "[quest] Open on Quest browser: https://${lan_ip}:${HTTP_PORT}/client.html"
}

# Per-run human_mimic / teleop status logs (file, not terminal spam).
# Override with OPENARM_HM_LOG_DIR=/path before launch.
quest_prepare_hm_logs() {
  local inplace_dir="${1:-${INPLACE_DIR:-.}}"
  if [[ -z "${OPENARM_HM_LOG_DIR:-}" ]]; then
    OPENARM_HM_LOG_DIR="${inplace_dir}/recordings/hm_logs/quest_$(date +%Y%m%d_%H%M%S)"
  fi
  mkdir -p "${OPENARM_HM_LOG_DIR}"
  export OPENARM_HM_LOG_DIR
  echo "[quest] HM logs -> ${OPENARM_HM_LOG_DIR}"
  echo "[quest]   left_hm.log / right_hm.log / status.log (console: OPENARM_HM_LOG_CONSOLE=1 / OPENARM_HM_STATUS_CONSOLE=1)"
}
