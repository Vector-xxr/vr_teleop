# Dual Replay + Left Visual Grasp Full Flow

This command runs:

1. dual-arm steady replay
2. dual-arm action1 replay
3. left arm directly interpolates from action1 last row to the left observe replay last row
4. wait 1s, then auto-detect with camera index 6
5. left-arm-only XY align, descend, close, lift, and retreat
6. left arm interpolates to action2 first row
7. dual-arm action2 replay

Important code checks:

- `steady-replay`, `pre-replay`, and `post-replay` are sent with `side=None`, so they are dual-arm actions.
- `pre-to-observe`, `auto-xy`, `grasp`, and `post-prep` use the active side. This script forces `arm_side=left_arm`, and `active_replay_side(args)` resolves to `left`.
- Each replay call waits for the replay server completion path before returning. The next action is called only after the previous function returns.
- The script forces `auto_once=True`, `auto_cycle=False`, `preview=False`, and starts detection after `--auto-start-delay-s`.
- Visual grasp parameters match the tested left-arm command below.
- If any Python exception, Ctrl+C, or SIGTERM happens after the replay server starts, the script calls cleanup and stops the running replay process.

```bash
cd /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/skewer_topdown_2d_minimal

/home/knowin/miniconda3/envs/openarm_teleop1/bin/python scripts/dual_left_visual_flow.py \
  --execute \
  --dual-replay \
  --arm-side left_arm \
  --config /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/config/skewer_align_config_left.json \
  --steady-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv \
  --pre-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/action1.csv \
  --observe-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/outputs/left_observe_record_20260728_161523/real_follower_left_01.csv \
  --post-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/action2.csv \
  --right-follower-can follower_right \
  --left-follower-can follower_left \
  --sudo-password knowin \
  --server-startup-s 2.0 \
  --camera-backend opencv \
  --camera-index 6 \
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
  --no-track-target \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --initial-transition-duration 1.2 \
  --pre-to-observe-duration 5.0 \
  --post-replay-transition-duration 5.0 \
  --max-replay-final-error-rad 0.3491 \
  --verify-replay-feedback \
  --abort-on-replay-feedback-stall \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale-x 0.50 \
  --xy-correction-scale-y 1.10 \
  --quick-grasp-on-d \
  --auto-start-delay-s 0.5 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --quick-detect-preview-s 0.25 \
  --no-quick-verify-after-xy \
  --grasp-descent-m 0.065 \
  --grasp-descent-duration 3.5 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.06 \
  --post-grasp-lift-duration 1.5 \
  --post-grasp-retreat-mm 100 \
  --post-grasp-retreat-duration 1.5 \
  --post-grasp-retreat-order after-lift \
  --return-observe-duration 3.0 \
  --max-grasp-joint-delta-rad 2.0 \
  --no-preview
```
