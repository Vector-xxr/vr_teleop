#!/usr/bin/env bash
# Recreate missing /dev/video* nodes when drivers are loaded but udev skipped mknod.
# Safe to re-run. Needs root.
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "[fix-v4l] need root: sudo $0" >&2
  exit 1
fi

n=0
for d in /sys/class/video4linux/video*; do
  [[ -e "$d/dev" ]] || continue
  name="$(basename "$d")"
  majmin="$(cat "$d/dev")"
  maj="${majmin%%:*}"
  min="${majmin##*:}"
  target="/dev/${name}"
  if [[ -e "${target}" ]]; then
    continue
  fi
  mknod "${target}" c "${maj}" "${min}"
  chmod 666 "${target}" || true
  echo "[fix-v4l] created ${target} (${maj}:${min}) name=$(cat "$d/name" 2>/dev/null || true)"
  n=$((n + 1))
done

if [[ "${n}" -eq 0 ]]; then
  echo "[fix-v4l] nothing to create (nodes already present, or no video4linux in sysfs)"
else
  echo "[fix-v4l] created ${n} node(s)"
fi

echo "[fix-v4l] now:"
ls -la /dev/video8 /dev/video16 2>&1 || ls -la /dev/video* 2>&1 | head -20
