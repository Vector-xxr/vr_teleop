#!/usr/bin/env bash
# Install openarm-left-job.service; enable/start only when explicitly requested.
# Portable: derives repo root / user from this script's location.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_CONTROL="$(cd "${ROOT}/.." && pwd)"
UNIT_SRC="${ROOT}/openarm-left-job.service"
UNIT_DST="/etc/systemd/system/openarm-left-job.service"
ENV_EXAMPLE="${ROOT}/env.systemd.example"
ENV_DST="${VR_CONTROL}/left_job_orchestrator/env.systemd"
RUN_SCRIPT="${VR_CONTROL}/left_job_orchestrator/run_service.sh"
WAIT_CAN_SCRIPT="${ROOT}/wait_for_can.sh"
CHECK_ENV_SCRIPT="${ROOT}/check_service_env.sh"
START_NOW=0
ENABLE_BOOT=0
FORCE_ENV=0
DRY_RUN=0
USER_OVERRIDE=""
PYTHON_OVERRIDE=""

# Prefer the invoking user when run via sudo (not root).
RUN_USER="${SUDO_USER:-${USER:-knowin}}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-start) START_NOW=0; shift ;; # retained for compatibility
    --enable) ENABLE_BOOT=1; shift ;;
    --start) START_NOW=1; ENABLE_BOOT=1; shift ;;
    --force-env) FORCE_ENV=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --user)
      [[ $# -ge 2 && -n "$2" ]] || {
        echo "--user requires a username" >&2
        exit 2
      }
      USER_OVERRIDE="$2"
      shift 2
      ;;
    --python)
      [[ $# -ge 2 && -x "$2" ]] || {
        echo "--python requires an executable Python path" >&2
        exit 2
      }
      PYTHON_OVERRIDE="$2"
      shift 2
      ;;
    -h|--help)
      cat <<EOF
Usage: sudo $0 [--enable] [--start] [--force-env] [--user USER] [--python PATH]

  Installs systemd unit for left_job_orchestrator on this machine.
  Paths are taken from the repo that contains this script.

  --enable     enable at boot, but do not start now
  --start      start immediately after install (arms may move)
  --force-env  overwrite left_job_orchestrator/env.systemd from example
  --user USER  service account (default: sudo caller / repo owner)
  --python PATH Python interpreter for the service
  --dry-run    render and validate only; do not write system files
EOF
      exit 0
      ;;
    *)
      echo "Unknown arg: $1 (try --help)" >&2
      exit 1
      ;;
  esac
done

if [[ -n "${USER_OVERRIDE}" ]]; then
  RUN_USER="${USER_OVERRIDE}"
elif [[ "${RUN_USER}" == "root" ]]; then
  RUN_USER="$(stat -c '%U' "${VR_CONTROL}" 2>/dev/null || echo root)"
fi
if [[ "${RUN_USER}" == "root" ]]; then
  echo "Refusing to run the robot service as root." >&2
  echo "Run via sudo from the normal account, or pass --user <account>." >&2
  exit 1
fi
RUN_GROUP="$(id -gn "${RUN_USER}" 2>/dev/null || echo "${RUN_USER}")"

if [[ "${DRY_RUN}" -ne 1 && "$(id -u)" -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 1
fi

if [[ ! -f "${UNIT_SRC}" ]]; then
  echo "missing ${UNIT_SRC}" >&2
  exit 1
fi
if [[ ! -x "${RUN_SCRIPT}" ]]; then
  echo "missing or non-executable run script: ${RUN_SCRIPT}" >&2
  exit 1
fi
if [[ ! -x "${WAIT_CAN_SCRIPT}" ]]; then
  echo "missing or non-executable CAN wait script: ${WAIT_CAN_SCRIPT}" >&2
  exit 1
fi
if [[ ! -r "${CHECK_ENV_SCRIPT}" ]]; then
  echo "missing or unreadable service environment check: ${CHECK_ENV_SCRIPT}" >&2
  exit 1
fi
if ! id "${RUN_USER}" >/dev/null 2>&1; then
  echo "User '${RUN_USER}' does not exist on this machine." >&2
  echo "Create it, or chown the repo to an existing user, then re-run." >&2
  exit 1
fi

# Ensure env file exists (do not clobber secrets unless --force-env).
if [[ "${DRY_RUN}" -ne 1 && ( ! -f "${ENV_DST}" || "${FORCE_ENV}" -eq 1 ) ]]; then
  if [[ ! -f "${ENV_EXAMPLE}" ]]; then
    echo "missing ${ENV_EXAMPLE}" >&2
    exit 1
  fi
  install -o "${RUN_USER}" -g "${RUN_GROUP}" -m 600 "${ENV_EXAMPLE}" "${ENV_DST}"
  echo "Wrote ${ENV_DST} (edit OPENARM_SUDO_PASSWORD before relying on CAN)."
elif [[ "${DRY_RUN}" -ne 1 ]]; then
  echo "Keeping existing ${ENV_DST}"
fi

# Pin the service interpreter so migration never falls back to another user's
# hard-coded conda path or system Python.
if [[ "${DRY_RUN}" -ne 1 ]]; then
  USER_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
  DETECTED_PYTHON="${PYTHON_OVERRIDE}"
  if [[ -z "${DETECTED_PYTHON}" && -x "${USER_HOME}/miniconda3/envs/openarm_teleop1/bin/python" ]]; then
    DETECTED_PYTHON="${USER_HOME}/miniconda3/envs/openarm_teleop1/bin/python"
  fi
  EXISTING_PYTHON="$(awk -F= '$1=="OPENARM_PYTHON" {print substr($0, index($0, "=") + 1); exit}' "${ENV_DST}")"
  if [[ -z "${DETECTED_PYTHON}" ]]; then
    DETECTED_PYTHON="${EXISTING_PYTHON}"
  fi
  if [[ -z "${DETECTED_PYTHON}" || ! -x "${DETECTED_PYTHON}" ]]; then
    echo "No usable openarm_teleop1 Python found for ${RUN_USER}." >&2
    echo "Re-run with: sudo $0 --python /absolute/path/to/python" >&2
    exit 1
  fi
  TMP_ENV="$(mktemp)"
  awk -v value="${DETECTED_PYTHON}" '
    BEGIN { done=0 }
    /^OPENARM_PYTHON=/ {
      if (!done) print "OPENARM_PYTHON=" value
      done=1
      next
    }
    { print }
    END { if (!done) print "OPENARM_PYTHON=" value }
  ' "${ENV_DST}" > "${TMP_ENV}"
  install -o "${RUN_USER}" -g "${RUN_GROUP}" -m 600 "${TMP_ENV}" "${ENV_DST}"
  rm -f "${TMP_ENV}"
  echo "Pinned OPENARM_PYTHON=${DETECTED_PYTHON}"
fi

env_value() {
  local key="$1"
  awk -v key="${key}" '
    $0 ~ "^[[:space:]]*" key "=" {
      value=$0
      sub("^[[:space:]]*" key "=", "", value)
      sub(/^[[:space:]]+/, "", value)
      sub(/[[:space:]]+$/, "", value)
      if ((substr(value,1,1) == "\"" && substr(value,length(value),1) == "\"") ||
          (substr(value,1,1) == "\047" && substr(value,length(value),1) == "\047")) {
        value=substr(value,2,length(value)-2)
      }
      print value
      exit
    }
  ' "${ENV_DST}"
}

validate_enable_environment() {
  local password python_bin ros_distro ros_setup
  password="$(env_value OPENARM_SUDO_PASSWORD)"
  if [[ -z "${password}" || "${password}" == "CHANGE_ME" ]]; then
    echo "Refusing --enable/--start: configure OPENARM_SUDO_PASSWORD in ${ENV_DST}." >&2
    return 1
  fi

  python_bin="$(env_value OPENARM_PYTHON)"
  if [[ -z "${python_bin}" ]] || ! sudo -u "${RUN_USER}" test -x "${python_bin}"; then
    echo "Refusing --enable/--start: OPENARM_PYTHON is not executable by ${RUN_USER}." >&2
    return 1
  fi

  ros_distro="$(env_value ROS_DISTRO)"
  ros_distro="${ros_distro:-humble}"
  ros_setup="$(env_value OPENARM_ROS_SETUP)"
  ros_setup="${ros_setup:-/opt/ros/${ros_distro}/setup.bash}"
  if ! sudo -u "${RUN_USER}" test -r "${ros_setup}"; then
    echo "Refusing --enable/--start: ROS setup is not readable: ${ros_setup}" >&2
    echo "Set OPENARM_ROS_SETUP in ${ENV_DST}." >&2
    return 1
  fi

  echo "Checking builds and ROS runtime links as ${RUN_USER} (no hardware access)..."
  sudo -u "${RUN_USER}" -H \
    env OPENARM_PYTHON="${python_bin}" OPENARM_ROS_SETUP="${ros_setup}" \
    /usr/bin/bash -c '
      set -euo pipefail
      set +u
      source "${OPENARM_ROS_SETUP}"
      set -u
      exec "$1" --check
    ' bash "${VR_CONTROL}/migration_build/build_all.sh"
  echo "Environment check passed (password value was not printed)."
}

# Runtime directories must be writable by the service user even if the repo was
# copied/extracted by root.
if [[ "${DRY_RUN}" -ne 1 ]]; then
  install -d -o "${RUN_USER}" -g "${RUN_GROUP}" -m 775 \
    "${VR_CONTROL}/left_job_orchestrator/runtime" \
    "${VR_CONTROL}/left_job_orchestrator/outputs" \
    "${VR_CONTROL}/tcp_forward_grasp/outputs"
fi

# Render unit with this machine's paths/user (template may still say knowin).
TMP_UNIT="$(mktemp "${TMPDIR:-/tmp}/openarm-left-job.XXXXXX.service")"
sed \
  -e "s|/home/knowin/Desktop/vr_control|${VR_CONTROL}|g" \
  -e "s|^User=.*|User=${RUN_USER}|" \
  -e "s|^Group=.*|Group=${RUN_GROUP}|" \
  "${UNIT_SRC}" > "${TMP_UNIT}"

# Sanity: every path in the unit that we care about must exist.
for key in WorkingDirectory EnvironmentFile ExecStartPre ExecStart; do
  val="$(awk -F= -v k="$key" '$1==k {print $2; exit}' "${TMP_UNIT}" | sed 's/^-//')"
  path="${val%% *}"
  if [[ -n "${path}" && ! -e "${path}" && "${key}" != "EnvironmentFile" ]]; then
    echo "Rendered ${key} path does not exist: ${path}" >&2
    rm -f "${TMP_UNIT}"
    exit 1
  fi
done
if command -v systemd-analyze >/dev/null 2>&1; then
  systemd-analyze verify "${TMP_UNIT}"
fi

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "DRY RUN OK (no files installed)"
  echo "  repo=${VR_CONTROL}"
  echo "  user/group=${RUN_USER}/${RUN_GROUP}"
  echo "  unit=${UNIT_DST}"
  echo "  env=${ENV_DST}"
  echo "--- rendered unit ---"
  cat "${TMP_UNIT}"
  rm -f "${TMP_UNIT}"
  exit 0
fi

install -m 644 "${TMP_UNIT}" "${UNIT_DST}"
rm -f "${TMP_UNIT}"

systemctl daemon-reload
if [[ "${ENABLE_BOOT}" -eq 1 ]]; then
  if ! validate_enable_environment; then
    systemctl disable openarm-left-job.service >/dev/null 2>&1 || true
    echo "Service was left disabled because enable/start validation failed." >&2
    exit 1
  fi
  systemctl enable openarm-left-job.service
fi

echo "Installed: ${UNIT_DST}"
echo "  WorkingDirectory=${VR_CONTROL}"
echo "  User=${RUN_USER}"
echo "  EnvironmentFile=${ENV_DST}"

if [[ "${START_NOW}" -eq 1 ]]; then
  echo "Starting service now (arms will move after steady)..."
  systemctl reset-failed openarm-left-job.service || true
  systemctl start openarm-left-job.service
  systemctl --no-pager --full status openarm-left-job.service || true
else
  if [[ "${ENABLE_BOOT}" -eq 1 ]]; then
    echo "Enabled on boot; not started now."
  else
    echo "Installed but did not change enablement or start the service (safe default)."
    echo "Edit ${ENV_DST}, verify builds/CAN, then re-run: sudo $0 --enable"
  fi
fi

echo "Logs: journalctl -u openarm-left-job -f"
