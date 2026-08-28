#!/usr/bin/env bash
# Gripper chain diagnostic for HUMAN_MIMIC mode.
#
# Runs the bimanual sim in human_mimic mode and prints ONLY the gripper-relevant
# lines so you can watch the A/B/X/Y button chain light up stage by stage:
#
#   STAGE 1  [quest-bridge] ... btn_close=1 / btn_open=1
#            -> the WebXR client (client.html) is sending the face button.
#            -> if this stays 0 while you press A/B/X/Y, the problem is the
#               Quest client / gamepad button index (see client.html readFaceButtons).
#
#   STAGE 2  [*-human-mimic] ... btnClose=1 / btnOpen=1
#            -> the button reached the controller (bridge UDP -> receiver -> sample).
#            -> if STAGE 1 shows 1 but this stays 0, the UDP port routing or field
#               parsing is wrong (quest_pose_receiver.cpp fields[10]/[11]).
#
#   STAGE 3  [*] [human-mimic] gripper closing/opening/stop (...)
#            grip_mot= (0=stopped 1=opening 2=closing)   grip_tgt= (target pos)
#            -> the state machine fired and the target is moving.
#            -> if STAGE 2 shows 1 but you never see this, the state machine is
#               stuck (already at a limit, or recenter combo A+B swallowed it).
#
#   STAGE 4  the gripper visibly opens/closes in the MuJoCo window.
#            -> if STAGE 3 moves grip_tgt but the sim gripper does not move, the
#               problem is downstream (send_gripper / model finger joint).
#
# Full unfiltered output is tee'd to /tmp/gripper_chain.log for later inspection.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG=/tmp/gripper_chain.log

echo "[gripper-test] starting sim in human_mimic mode; press A/B (right) and X/Y (left)."
echo "[gripper-test] full log -> ${LOG}"
echo "[gripper-test] watching stages: bridge btn_* -> controller btnClose/Open -> grip_mot/grip_tgt"
echo

export OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic"

# Line-buffer so matches appear immediately; keep the MuJoCo window too.
stdbuf -oL -eL "${ROOT_DIR}/run_openarm_cartesian_bimanual_sim.sh" 2>&1 \
  | tee "${LOG}" \
  | stdbuf -oL grep --line-buffered -E \
      'btn_close=[1-9]|btn_open=[1-9]|btnClose=1|btnOpen=1|grip_mot=[1-9]|gripper (closing|opening|stop)|\[human-mimic\] gripper' \
  || true
