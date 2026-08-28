#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

python scripts/align_once_move.py \
  --arm-side left_arm \
  --observe-replay outputs/left_observe_record_20260728_161523/real_follower_left_01.csv \
  --quick-grasp-on-d \
  --auto-once \
  --after-grasp-action lift-only \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale 0.73 \
  --grasp-descent-m 0.065 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.03 \
  --post-grasp-lift-duration 1.5 \
  --post-grasp-retreat-mm 100 \
  --post-grasp-retreat-duration 1.5 \
  --post-grasp-retreat-order after-lift \
  --max-grasp-joint-delta-rad 1.3 \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --quick-detect-preview-s 0.25 \
  --execute \
  --follower-can can1 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --target-selection red \
  --red-h-low 12 \
  --red-h-high 168 \
  --red-v-min 25 \
  --red-s-min 70 \
  --red-min-area-px 35 \
  --red-max-area-px 5000 \
  --red-roi-left 0.0 \
  --red-roi-right 1.0 \
  --red-roi-top 0.05 \
  --red-roi-bottom 0.90 \
  --red-max-target-dist-px 300 \
  --no-track-target
