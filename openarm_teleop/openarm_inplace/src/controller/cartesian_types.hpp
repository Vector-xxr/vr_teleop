#pragma once

#include <controller/reference_smoother.hpp>

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

struct QuestPoseSample {
    int64_t timestamp_ms = 0;           // source timestamp from Quest bridge
    int64_t receive_timestamp_ns = 0;   // steady_clock receive time (ns)
    uint64_t sequence_id = 0;
    bool grip_active = false;
    double trigger = 0.0;
    // Face buttons: right A/B, left X/Y (mapped to close/open in controller).
    bool button_close = false;
    bool button_open = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
};

struct CartesianGains {
    Eigen::Matrix<double, 6, 1> kp =
        (Eigen::Matrix<double, 6, 1>() << 80.0, 80.0, 80.0, 40.0, 40.0, 40.0).finished();
    Eigen::Matrix<double, 6, 1> kd =
        (Eigen::Matrix<double, 6, 1>() << 8.0, 8.0, 8.0, 4.0, 4.0, 4.0).finished();
};

enum class OrientationMode {
    HOLD_CLUTCH_ORIENTATION = 0,  // position teleop only; TCP rotation locked at clutch
    VR_ORIENTATION = 1,           // full 6-DOF: VR hand pose (xyz + rot) -> TCP target
    FREE_ORIENTATION = 2,         // position only; orientation not commanded
};

enum class NullspaceNominalMode {
    CLUTCH_POSE = 0,
    TABLE_READY_POSE = 1,
};

// Velocity-level DLS IK (primary teleop path).
struct DampedLeastSquaresConfig {
    bool enabled = true;
    double kp_pos = 4.0;                 // 1/s
    double kp_ori = 2.0;                 // 1/s
    double max_linear_speed_mps = 0.60;  // m/s
    double max_angular_speed_radps = 1.5;
    double lambda = 0.08;                // DLS damping
    double max_joint_velocity_radps = 1.5;
    // Per-joint overrides; empty => use max_joint_velocity_radps for all.
    std::vector<double> max_joint_velocity_radps_vec;
    bool use_nullspace = false;
    double nullspace_gain = 0.2;
    NullspaceNominalMode nullspace_nominal_mode = NullspaceNominalMode::CLUTCH_POSE;
    std::vector<double> table_ready_pose;  // used when TABLE_READY_POSE
    // Per-joint nullspace weights (size dof). Empty => all 1.0. Prefer higher elbow weight.
    std::vector<double> nullspace_joint_weights;
    // DLS task mobility weights (size dof). Higher => prefer that joint for Cartesian twist.
    // DLS mobility weights (array idx 0..6 = motors j0..j6; j7 gripper via trigger).
    std::vector<double> task_joint_weights;
    // Soft joint limits: start scaling velocity when within this fraction of travel to hard limit.
    double soft_limit_margin_frac = 0.12;
    // Tracking: |q_ref - q_meas| soft clamp.
    double max_tracking_error_rad = 0.25;
    // Emergency step = emergency_step_scale * qdot_max * dt (not a fixed rad step).
    double emergency_step_scale = 2.0;
    bool debug_log = true;
    double debug_log_hz = 5.0;
    bool csv_log = false;
    std::string csv_log_path;
};

struct CartesianSafetyLimits {
    // Soft age: keep last T_target, do not extrapolate VR.
    double quest_stale_sec = 0.050;
    // Hard age: freeze teleop, q_dot=0, q_ref=q_meas; require re-clutch on recovery.
    double quest_timeout_sec = 0.150;
    double max_position_jump_m = 0.05;
    double max_rotation_jump_rad = 0.52;
    double max_dq_rad_per_sec = 1.5;
    // Legacy fixed step (position-IK path / emergency fallback only). Prefer dls.emergency_step_scale.
    double max_joint_step_rad = 0.02;
    double max_joint_jump_rad = 0.50;
    double max_ik_position_error_m = 0.06;
    double max_ik_rotation_error_rad = 0.70;
    double sigma_min_threshold = 0.05;
    double lambda0 = 0.05;
    double lambda_max = 0.5;
    double fatal_lock_timeout_sec = 1.0;
    double grip_debounce_sec = 0.05;
    double min_clutch_hold_sec = 0.12;
    int ik_fail_hold_count = 50;
    int ik_fail_estop_count = 500;
    double degrade_scale = 0.3;
};

struct CartesianWorkspaceLimits {
    double min_z = 0.12;
    double max_z = 0.85;
    double max_radius = 0.60;
    double max_teleop_offset_m = 0.45;
};

// Human virtual-skeleton redundancy resolution.
// The redundant DOF (elbow swivel) is pinned to a deterministic function of the VR hand pose
// derived from a simple 2-link human arm model. This makes the hand->jointspace map
// path-independent (returnable) and keeps configurations human-like (avoids singular/odd poses).
struct HumanSkeletonConfig {
    bool enabled = false;
    // Human arm segment lengths (metres). Rough adult defaults; tunable.
    double upper_arm_len_m = 0.30;
    double forearm_len_m = 0.28;
    // Shoulder position offset relative to the VR hand at clutch time, in VR/world axes
    // (Quest world: +X right, +Y up, +Z toward user / backward). Shoulder sits below+behind hand.
    Eigen::Vector3d shoulder_offset_vr = Eigen::Vector3d(-0.05, -0.30, 0.20);
    // Gravity-down reference in each frame (defines the elbow-droop direction / swivel zero).
    Eigen::Vector3d down_vr = Eigen::Vector3d(0.0, -1.0, 0.0);
    Eigen::Vector3d down_robot = Eigen::Vector3d(0.0, 0.0, -1.0);
    // Additional swivel bias applied on top of the natural droop (rad). +/- opens elbow out/in.
    double swivel_bias_rad = 0.0;
    // Sign to reconcile human vs robot swivel convention (flip if elbow drives the wrong way).
    double swivel_sign = 1.0;
    // How strongly the elbow swivel follows the TCP orientation (rigid forearm<->hand coupling).
    // 1.0 = elbow fully slaved to gripper orientation (returnable, human-like);
    // 0.0 = elbow held at the fixed droop target regardless of orientation (legacy behaviour).
    double orientation_coupling = 1.0;
    // Robot chain segment indices used as shoulder / elbow anchors (auto if < 0).
    int shoulder_segment = -1;
    int elbow_segment = -1;
    // Convergence gain and cap for the null-space swivel controller.
    double swivel_gain = 2.0;      // 1/s
    double max_swivel_rate = 2.0;  // rad/s along self-motion
    bool debug_log = true;
};

struct CartesianControllerConfig {
    double pos_scale = 1.0;
    double rot_scale = 1.0;
    // Legacy flag: maps to OrientationMode::HOLD_CLUTCH_ORIENTATION when true.
    bool position_only_teleop = false;
    OrientationMode orientation_mode = OrientationMode::HOLD_CLUTCH_ORIENTATION;
    bool direct_ik_control = false;  // legacy: skip VR/joint smoothers (kept for A/B)
    bool mirror_translation_y = false;
    double gripper_open = 0.0;
    double gripper_closed = 0.04;
    // SLERP on VR hand quaternion (0 = off, 1 = raw). Reduces tracking jitter.
    double vr_orientation_filter_alpha = 0.32;
    // Weight on the VR angular-velocity feedforward (Δrot/dt). Lower = calmer, less jittery,
    // relies on kp_ori feedback for tracking. 1.0 = full feedforward (snappy but amplifies noise).
    double vr_orientation_ff_gain = 1.0;
    // Human wrist range-of-motion clamp: cap TCP orientation deviation from the clutch pose,
    // per TCP body axis (rad). Prevents anatomically impossible wrist flips / over-rotation.
    bool wrist_rom_enabled = false;
    double wrist_rom_limit_x_rad = 1.57;
    double wrist_rom_limit_y_rad = 1.57;
    double wrist_rom_limit_z_rad = 1.57;
    // TCP-frame orientation signs after VR grip -> TCP axis permutation (±1 each, independent).
    double vr_tcp_pitch_sign = -1.0;  // j5 up/down (after M conjugation)
    double vr_tcp_yaw_sign = 1.0;    // j6 left/right — conjugation M handles axis; ±1 only if needed
    double vr_tcp_roll_sign = 1.0;
    CartesianGains gains;
    CartesianSafetyLimits safety;
    CartesianWorkspaceLimits workspace;
    SmoothingConfig smoothing;
    DampedLeastSquaresConfig dls;
    HumanSkeletonConfig skeleton;
};

enum class CartesianControlState {
    Idle,
    Active,
    Degraded,
    Hold,
    Estop,
};
