# Left Grasp Execution Bundle

This folder keeps the same relative layout expected by the left-arm grasp command.

## Entry Point

Run from this folder:

```bash
conda activate anygrasp
bash RUN_LEFT_GRASP_COMMAND.sh
```

## Included Runtime Files

- `scripts/align_once_move.py`
- `scripts/auto_calibrate_pixel_to_motion.py`
- `run_skewer_2d_servo.py`
- `config/skewer_align_config_left.json`
- `outputs/left_observe_record_20260728_161523/real_follower_left_01.csv`
- `vendor/openarm_inplace/`
- `vendor/openarm_vision_grasp_yolo/`
- `vendor/openarm_description/`
- `vendor/openarm_can/`
- `vendor/template_library/`

## Notes

- The command uses `--arm-side left_arm` and `--follower-can can1`.
- The current left-arm config is set to the linear calibration model.
- The post-grasp sequence is: close gripper, lift 30mm, retreat base -X 100mm, then stop.
