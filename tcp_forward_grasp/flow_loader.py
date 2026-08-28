"""Load dual_left_visual_flow and build field-default argparse.Namespace."""

from __future__ import annotations

import importlib.util
import sys
from argparse import Namespace
from pathlib import Path
from typing import Any

from . import defaults, paths

# Same visual/motion defaults as the tested left dual_left_visual_flow command.
FLOW_CLI_DEFAULTS: dict[str, Any] = {
    "execute": True,
    "dual_replay": True,
    "arm_side": "left_arm",
    "config": paths.LEFT_CONFIG,
    "steady_replay": paths.STEADY_CSV,
    "observe_replay": paths.DEFAULT_BEGIN_CSV,
    "pre_replay": paths.DEFAULT_ACTION1,
    "post_replay": paths.DEFAULT_ACTION2,
    "right_follower_can": defaults.RIGHT_CAN,
    "left_follower_can": defaults.LEFT_CAN,
    "follower_can": defaults.LEFT_CAN,
    "sudo_password": defaults.SUDO_PASSWORD,
    "server_startup_s": 2.0,
    "camera_backend": "opencv",
    "camera_index": defaults.CAMERA_INDEX,
    # Red balls on white plate (black background); prefer rightmost; LR then forward.
    "target_selection": "red",
    "red_fallback": False,
    "white_s_max": 80,
    "white_v_min": 130,
    "white_min_area_frac": 0.03,
    "white_dilate_px": 14,
    "tray_above_target_margin_px": 30.0,
    "tray_max_y_frac": 0.70,
    "tray_min_white_frac": 0.18,
    "tray_white_inner_r": 4,
    "tray_white_outer_r": 28,
    "tray_row_y_tol_px": 70.0,
    "red_h_low": 15,
    "red_h_high": 165,
    "red_v_min": 30,
    "red_s_min": 45,
    "red_min_area_px": 50,
    "red_max_area_px": 25000,
    "red_roi_left": 0.0,
    "red_roi_right": 1.0,
    "red_roi_top": 0.02,
    "red_roi_bottom": 0.72,
    "red_max_target_dist_px": 500,
    # Whole red skewer handle (beads + sprinkle barrel), not a single ball.
    "red_handle_mode": True,
    "red_handle_close_px": 18,
    "tray_max_bbox_aspect": 6.0,
    "track_max_bbox_aspect": 8.0,
    # LR align (image-u only), then forward+LR until ball in fingers.
    "xy_correction_scale": 0.75,
    "xy_correction_scale_x": 0.75,
    "xy_correction_scale_y": 0.85,
    "lr_align_passes": 20,
    "lr_align_done_err_px": 8.0,
    "forward_align_passes": 80,
    "forward_step_mm": 8.0,
    "forward_near_step_mm": 5.0,
    "forward_crawl_step_mm": 3.0,
    "forward_near_overlap_frac": 0.05,
    "forward_max_openloop_streak": 3,
    "forward_motion_duration": 0.35,
    "forward_settle_s": 0.02,
    "forward_max_move_mm": 18.0,
    "forward_transition_s": 0.12,
    "approach_speed_mm_s": 25.0,
    "approach_lat_gain_mm_s_per_px": 0.5,
    "approach_accel_mm_s2": 200.0,
    "approach_cmd_timeout_s": 0.15,
    "approach_loop_hz": 25.0,
    "approach_max_s": 25.0,
    "gripper_enter_confirm_frames": 2,
    "xy_align_passes": 30,
    "xy_align_done_err_px": 8.0,
    "xy_align_motion_duration": 2.2,
    "xy_align_settle_s": 0.22,
    "max_move_mm": 50.0,
    "motion_duration": 2.2,
    "settle_s": 0.22,
    "auto_detect_timeout_s": 0.0,
    "track_target": True,
    "track_max_jump_px": 200.0,
    "track_after_motion_jump_px": 360.0,
    "track_left_slack_px": 55.0,
    "track_resnap_right_px": 40.0,
    "track_resnap_y_band_px": 90.0,
    "track_area_ratio_min": 0.15,
    "track_area_ratio_max": 6.0,
    "approach_lift_z_mm": 24.0,
    "lr_only": False,
    "align_only": False,
    "gripper_open": -0.462158,
    "gripper_close": 0.1,
    "gripper_black_v_max": 45,
    "gripper_black_s_max": 90,
    "gripper_roi_top_frac": 0.45,
    "gripper_roi_side_frac": 0.05,
    "gripper_min_area_px": 1200.0,
    "gripper_min_height_px": 40.0,
    "gripper_min_aspect_h_w": 1.15,
    "gripper_gate_half_width_px": 55.0,
    "gripper_gate_tip_y_frac": 0.58,
    "gripper_gate_min_width_px": 35.0,
    "gripper_gate_max_width_px": 180.0,
    "gripper_min_tip_y_frac": 0.38,
    "gripper_gate_require_fingers": True,
    "gripper_gate_stable_samples": 15,
    "gripper_gate_stable_window": 18,
    "gripper_gate_stable_spread_px": 8.0,
    "gripper_gate_stable_timeout_s": 5.0,
    "gripper_enter_margin_px": 2.0,
    "gripper_enter_y_slack_px": 8.0,
    # Handle mode: close when >=60% of red bbox is inside the gripper gate.
    "gripper_enter_frac": 0.60,
    "gripper_touch_margin_px": 8.0,
    "gripper_enter_past_tip_px": 8.0,
    # 0 = auto half of min(bbox_w, bbox_h); set px to force.
    "gripper_enter_top_half_px": 0.0,
    "gripper_gate_file": paths.DEFAULT_GRIPPER_GATE_JSON,
    # Gate tip = tip-to-tip grasp face (foremost white + small palm inset).
    "calib_tip_ahead_of_green_px": 85.0,
    "calib_tip_min_ahead_of_green_px": 25.0,
    "calib_tip_inset_px": 6.0,
    "calib_finger_tip_percentile": 5.0,
    "calib_finger_above_green_px": 130.0,
    "calib_finger_roi_top_frac": 0.28,
    "calib_finger_search_half_px": 160.0,
    "calib_finger_s_max": 100,
    "calib_finger_v_min": 130,
    "calib_finger_min_area_px": 400.0,
    "initial_transition_duration": 1.2,
    "pre_to_observe_duration": 3.0,
    "post_replay_transition_duration": 3.0,
    "max_replay_final_error_rad": 0.3491,
    "verify_replay_feedback": True,
    "abort_on_replay_feedback_stall": True,
    "max_step_joint_delta_rad": 0.85,
    "quick_grasp_on_d": True,
    "auto_start_delay_s": 0.25,
    "auto_detect_retry_s": 0.6,
    "quick_align_angle": False,
    "quick_detect_preview_s": 0.25,
    "quick_verify_after_xy": False,
    # Kept for compatibility with shared parsers; this module uses grasp_approach_m instead of Z descent.
    "grasp_descent_m": defaults.GRASP_APPROACH_M,
    "grasp_descent_duration": 3.5,
    "grasp_approach_m": defaults.GRASP_APPROACH_M,
    "grasp_approach_duration": 3.5,
    "approach_frame": defaults.APPROACH_FRAME,
    "approach_axis": defaults.APPROACH_AXIS,
    "approach_sign": defaults.APPROACH_SIGN,
    "tcp_approach_axis": defaults.APPROACH_AXIS,
    "tcp_approach_sign": defaults.APPROACH_SIGN,
    "pre_close_hold_duration": 0.4,
    "gripper_close_duration": 0.9,
    "gripper_close_hold_duration": 0.5,
    "post_grasp_lift_m": 0.09,
    "post_grasp_lift_duration": 1.0,
    "post_grasp_retreat_mm": 140.0,
    "post_grasp_retreat_duration": 1.0,
    "post_grasp_retreat_order": "after-lift",
    "post_grasp_return": "aligned",
    "post_grasp_return_duration": 1.5,
    "return_observe_duration": 3.0,
    "max_grasp_joint_delta_rad": 2.0,
    "preview": True,
    "no_preview": False,
    "preview_max_width": 960,
    "auto_once": True,
    "auto_cycle": False,
    "output_dir": paths.OUTPUT_ROOT,
}


def load_flow_module():
    script = paths.FLOW_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(script)
    name = "tcp_forward_grasp_dual_left_visual_flow"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {script}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    scripts_dir = str(script.parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    skewer_dir = str(paths.SKEWER_LEFT)
    if skewer_dir not in sys.path:
        sys.path.insert(0, skewer_dir)
    spec.loader.exec_module(module)
    return module


def build_flow_args(
    *,
    overrides: dict[str, Any] | None = None,
    output_dir: Path | None = None,
) -> Namespace:
    flow = load_flow_module()
    parser = flow.build_arg_parser()
    args = parser.parse_args([])
    merged: dict[str, Any] = dict(FLOW_CLI_DEFAULTS)
    if output_dir is not None:
        merged["output_dir"] = output_dir
    if overrides:
        merged.update(overrides)
    for key, value in merged.items():
        setattr(args, key, value)
    if merged.get("no_preview", False):
        args.preview = False
        args.no_preview = True
    else:
        args.preview = True
        args.no_preview = False
    flow.apply_runtime_defaults(args)
    # apply_runtime_defaults may force no_preview; restore CLI intent afterward in flow.main.
    if not merged.get("no_preview", False):
        args.preview = True
        args.no_preview = False
    args.preview_max_width = int(merged.get("preview_max_width", 960))
    flow.core.apply_config_defaults(args)
    if not getattr(args, "pixel_to_base_mm", None):
        raise RuntimeError(f"pixel_to_base_mm missing; check {paths.LEFT_CONFIG}")
    # Ensure TCP knobs survive apply_config_defaults
    args.grasp_approach_m = float(
        merged.get("grasp_approach_m", defaults.GRASP_APPROACH_M)
    )
    args.grasp_approach_duration = float(merged.get("grasp_approach_duration", 3.5))
    args.approach_frame = str(merged.get("approach_frame", defaults.APPROACH_FRAME))
    args.approach_axis = str(merged.get("approach_axis", defaults.APPROACH_AXIS))
    args.approach_sign = float(merged.get("approach_sign", defaults.APPROACH_SIGN))
    args.tcp_approach_axis = args.approach_axis
    args.tcp_approach_sign = args.approach_sign
    args.post_grasp_return = str(merged.get("post_grasp_return", "aligned"))
    args.post_grasp_return_duration = float(merged.get("post_grasp_return_duration", 1.5))
    # Keep intended XY gains / pacing even if config/parser defaults overwrite them.
    args.xy_correction_scale = float(merged.get("xy_correction_scale", 0.75))
    args.xy_correction_scale_x = float(merged.get("xy_correction_scale_x", 0.75))
    args.xy_correction_scale_y = float(merged.get("xy_correction_scale_y", 0.85))
    args.xy_align_passes = int(merged.get("xy_align_passes", 30))
    args.xy_align_done_err_px = float(merged.get("xy_align_done_err_px", 8.0))
    args.xy_align_motion_duration = float(merged.get("xy_align_motion_duration", 2.2))
    args.xy_align_settle_s = float(merged.get("xy_align_settle_s", 0.22))
    args.motion_duration = float(merged.get("motion_duration", args.xy_align_motion_duration))
    args.settle_s = float(merged.get("settle_s", args.xy_align_settle_s))
    args.max_move_mm = float(merged.get("max_move_mm", 50.0))
    args.auto_detect_timeout_s = float(merged.get("auto_detect_timeout_s", 0.0))
    args.lr_align_passes = int(merged.get("lr_align_passes", 20))
    args.lr_align_done_err_px = float(merged.get("lr_align_done_err_px", 8.0))
    args.forward_align_passes = int(merged.get("forward_align_passes", 80))
    args.forward_step_mm = float(merged.get("forward_step_mm", 8.0))
    args.lr_only = bool(merged.get("lr_only", False))
    args.approach_lift_z_mm = float(merged.get("approach_lift_z_mm", 24.0))
    args.post_grasp_lift_m = float(merged.get("post_grasp_lift_m", 0.09))
    args.post_grasp_retreat_mm = float(merged.get("post_grasp_retreat_mm", 140.0))
    args.post_grasp_lift_duration = float(merged.get("post_grasp_lift_duration", 1.0))
    args.post_grasp_retreat_duration = float(merged.get("post_grasp_retreat_duration", 1.0))
    args.forward_near_step_mm = float(merged.get("forward_near_step_mm", 5.0))
    args.forward_crawl_step_mm = float(merged.get("forward_crawl_step_mm", 3.0))
    args.forward_near_overlap_frac = float(merged.get("forward_near_overlap_frac", 0.05))
    args.forward_max_openloop_streak = int(merged.get("forward_max_openloop_streak", 3))
    args.gripper_enter_confirm_frames = int(merged.get("gripper_enter_confirm_frames", 2))
    args.gripper_enter_frac = float(merged.get("gripper_enter_frac", 0.60))
    args.gripper_touch_margin_px = float(merged.get("gripper_touch_margin_px", 8.0))
    args.gripper_enter_past_tip_px = float(
        merged.get("gripper_enter_past_tip_px", 8.0)
    )
    args.gripper_enter_top_half_px = float(merged.get("gripper_enter_top_half_px", 0.0))
    args.red_handle_mode = bool(merged.get("red_handle_mode", True))
    args.red_handle_close_px = int(merged.get("red_handle_close_px", 18))
    args.tray_max_bbox_aspect = float(merged.get("tray_max_bbox_aspect", 6.0))
    args.track_max_bbox_aspect = float(merged.get("track_max_bbox_aspect", 8.0))
    args.red_max_area_px = float(merged.get("red_max_area_px", 25000.0))
    args.gripper_gate_file = merged.get("gripper_gate_file", paths.DEFAULT_GRIPPER_GATE_JSON)
    args.forward_motion_duration = float(merged.get("forward_motion_duration", 0.35))
    args.forward_settle_s = float(merged.get("forward_settle_s", 0.02))
    args.forward_max_move_mm = float(merged.get("forward_max_move_mm", 18.0))
    args.forward_transition_s = float(merged.get("forward_transition_s", 0.12))
    args.approach_speed_mm_s = float(merged.get("approach_speed_mm_s", 25.0))
    args.approach_lat_gain_mm_s_per_px = float(
        merged.get("approach_lat_gain_mm_s_per_px", 0.5)
    )
    args.approach_accel_mm_s2 = float(merged.get("approach_accel_mm_s2", 200.0))
    args.approach_cmd_timeout_s = float(merged.get("approach_cmd_timeout_s", 0.15))
    args.approach_loop_hz = float(merged.get("approach_loop_hz", 25.0))
    args.approach_max_s = float(merged.get("approach_max_s", 25.0))
    args.track_left_slack_px = float(merged.get("track_left_slack_px", 55.0))
    args.track_resnap_right_px = float(merged.get("track_resnap_right_px", 40.0))
    args.track_resnap_y_band_px = float(merged.get("track_resnap_y_band_px", 90.0))
    args.pre_to_observe_duration = float(merged.get("pre_to_observe_duration", 3.0))
    args.post_replay_transition_duration = float(
        merged.get("post_replay_transition_duration", 3.0)
    )
    args.gripper_close_duration = float(merged.get("gripper_close_duration", 0.9))
    args.gripper_close_hold_duration = float(merged.get("gripper_close_hold_duration", 0.5))
    args.auto_start_delay_s = float(merged.get("auto_start_delay_s", 0.25))
    args.gripper_black_v_max = int(merged.get("gripper_black_v_max", 45))
    args.gripper_min_area_px = float(merged.get("gripper_min_area_px", 1200.0))
    args.gripper_gate_stable_spread_px = float(merged.get("gripper_gate_stable_spread_px", 8.0))
    args.gripper_gate_stable_samples = int(merged.get("gripper_gate_stable_samples", 15))
    return args
