#pragma once

#include <controller/cartesian_types.hpp>
#include <controller/dynamics.hpp>
#include <controller/pose_kalman_filter.hpp>
#include <controller/reference_smoother.hpp>
#include <robot_state.hpp>

#include <chrono>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

class CartesianController {
public:
    explicit CartesianController(Dynamics* dynamics, CartesianControllerConfig config = {},
                                 std::string log_prefix = {});

    bool step(const std::vector<JointState>& current_joints, const QuestPoseSample& raw_pose,
              double dt, std::vector<JointState>* joint_reference);

    void reset();
    void seed_hold_reference(const std::vector<JointState>& joints);
    CartesianControlState state() const { return state_; }
    bool consume_fatal_error(std::string* message);
    double tcp_target_offset_norm() const { return (p_des_target_ - robot_ref_pos_).norm(); }
    double teleop_offset_limit_m() const { return config_.workspace.max_teleop_offset_m; }
    bool teleop_offset_saturated() const { return teleop_offset_saturated_; }
    bool workspace_radius_saturated() const { return workspace_radius_saturated_; }
    Eigen::Vector3d tcp_target_position() const { return p_des_target_; }
    double rotation_target_offset_rad() const;
    bool quest_timeout_active() const { return quest_timeout_active_; }
    OrientationMode effective_orientation_mode() const;
    bool orientation_follow_enabled() const { return !holds_orientation(); }

private:
    struct DlsDebugSnapshot {
        double quest_data_age_ms = -1.0;
        double controller_dt = 0.0;
        double position_error_norm = 0.0;
        double orientation_error_norm = 0.0;
        double linear_velocity_cmd_norm = 0.0;
        double angular_velocity_cmd_norm = 0.0;
        double vr_delta_angle_rad = 0.0;
        Eigen::Vector3d vr_omega_robot = Eigen::Vector3d::Zero();
        Eigen::Vector3d vr_delta_axis_robot = Eigen::Vector3d::Zero();
        double sigma_min = 0.0;
        double lambda = 0.0;
        double qdot_raw_max = 0.0;
        double qdot_limited_max = 0.0;
        double delta_q_ref_max = 0.0;
        double tracking_error_max = 0.0;
        bool soft_limit_active = false;
        bool quest_timeout_active = false;
        Eigen::Vector3d p_cur = Eigen::Vector3d::Zero();
        Eigen::Vector3d p_des = Eigen::Vector3d::Zero();
        Eigen::Vector3d e_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d e_ori = Eigen::Vector3d::Zero();
        Eigen::VectorXd q_dot_raw;
        Eigen::VectorXd q_dot_lim;
        std::vector<double> q_ref;
    };

    Dynamics* dynamics_ = nullptr;
    CartesianControllerConfig config_;
    std::string log_prefix_;
    PoseKalmanFilter pose_filter_;
    QuestPoseSample filtered_vr_pose_;
    CartesianControlState state_ = CartesianControlState::Idle;

    bool clutch_active_ = false;
    Eigen::Vector3d hand_ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond hand_ref_rot_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d robot_ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond robot_ref_rot_ = Eigen::Quaterniond::Identity();

    std::vector<JointState> last_safe_reference_;
    std::vector<double> last_ik_q_;
    std::vector<double> q_ref_;  // continuous internal joint reference (DLS path)
    bool q_ref_valid_ = false;
    Eigen::Vector3d last_solved_cart_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond last_solved_cart_rot_ = Eigen::Quaterniond::Identity();
    int ik_fail_count_ = 0;
    std::chrono::steady_clock::time_point last_pose_time_;
    int64_t last_vr_timestamp_ms_ = -1;
    uint64_t last_vr_sequence_id_ = 0;
    int64_t last_filter_measurement_ms_ = -1;
    bool last_raw_grip_active_ = false;
    bool last_ingested_grip_active_ = false;
    double last_ingested_trigger_ = 0.0;
    bool last_ingested_button_close_ = false;
    bool last_ingested_button_open_ = false;
    Eigen::Vector3d last_ingested_position_ =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    Eigen::Vector3d p_des_target_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_des_target_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d last_ik_fail_reset_pos_ = Eigen::Vector3d::Zero();
    double gripper_target_ = 0.0;
    bool just_activated_clutch_ = false;
    double locked_elapsed_sec_ = 0.0;
    std::string fatal_error_;
    bool fatal_announced_ = false;
    bool runtime_position_only_ = false;
    bool grip_debounce_initialized_ = false;
    bool debounced_grip_active_ = false;
    double grip_pending_elapsed_sec_ = 0.0;
    bool grip_pending_value_ = false;
    double clutch_active_elapsed_sec_ = 0.0;
    Eigen::Quaterniond last_target_quat_ = Eigen::Quaterniond::Identity();
    bool last_target_quat_valid_ = false;
    std::vector<double> q_nominal_;
    std::chrono::steady_clock::time_point last_dls_debug_time_;
    uint64_t dls_debug_tick_ = 0;
    bool quest_timeout_active_ = false;
    bool quest_stale_active_ = false;
    bool teleop_offset_saturated_ = false;
    bool workspace_radius_saturated_ = false;
    bool require_reclutch_ = false;
    bool target_frozen_ = false;
    std::chrono::steady_clock::time_point last_dls_error_log_time_;
    std::chrono::steady_clock::time_point last_workspace_warn_time_;
    Eigen::Quaterniond last_vr_hand_quat_ = Eigen::Quaterniond::Identity();
    bool last_vr_hand_quat_valid_ = false;
    Eigen::Quaterniond filtered_hand_ori_ = Eigen::Quaterniond::Identity();
    bool filtered_hand_ori_valid_ = false;
    Eigen::Vector3d omega_des_robot_ = Eigen::Vector3d::Zero();
    double last_vr_delta_angle_rad_ = 0.0;
    Eigen::Vector3d last_vr_delta_axis_robot_ = Eigen::Vector3d::Zero();
    std::ofstream dls_csv_;
    bool dls_csv_header_written_ = false;

    CartesianPoseSmoother cartesian_smoother_;
    JointReferenceSmoother joint_smoother_;
    JointReferenceOutputFilter joint_output_filter_;
    ScalarSmoother gripper_smoother_;

    bool ingest_vr_sample(const QuestPoseSample& raw_pose);
    void update_filtered_vr_pose(const QuestPoseSample& raw_pose, double dt);
    void update_cartesian_target_from_vr(const QuestPoseSample& filtered_pose);
    bool compute_joint_reference(const std::vector<JointState>& current_joints, double dt,
                                 const Eigen::Vector3d& p_des, const Eigen::Quaterniond& q_des,
                                 double gripper_pos, std::vector<JointState>* joint_reference);
    bool compute_joint_reference_dls(const std::vector<JointState>& current_joints, double dt,
                                     const Eigen::Vector3d& p_des, const Eigen::Quaterniond& q_des,
                                     double gripper_pos, double quest_age_ms,
                                     std::vector<JointState>* joint_reference);
    bool compute_joint_reference_position_ik(const std::vector<JointState>& current_joints,
                                             double dt, const Eigen::Vector3d& p_des,
                                             const Eigen::Quaterniond& q_des, double gripper_pos,
                                             std::vector<JointState>* joint_reference);
    bool apply_gripper_reference(size_t arm_dof, size_t total_dof, double gripper_pos, double dt,
                                 std::vector<JointState>* joint_reference);

    static Eigen::Vector3d rotation_error_vector(const Eigen::Quaterniond& q_current,
                                                 const Eigen::Quaterniond& q_desired);
    static Eigen::Quaterniond align_quaternion_sign(const Eigen::Quaterniond& reference,
                                                    Eigen::Quaterniond candidate);
    Eigen::VectorXd compute_nullspace_velocity(const double* q_meas) const;
    void maybe_log_dls_debug(const DlsDebugSnapshot& snap);
    void maybe_log_dls_csv(const DlsDebugSnapshot& snap);
    void sync_q_ref_to_meas(const std::vector<JointState>& current_joints);
    void apply_soft_joint_limits(const std::vector<double>& q_meas, Eigen::VectorXd* q_dot) const;
    void clamp_tracking_error(const std::vector<double>& q_meas, std::vector<double>* q_ref) const;
    double joint_velocity_limit(size_t joint_index) const;
    double quest_data_age_sec(const QuestPoseSample& sample) const;
    void handle_quest_timeout_recovery();
    void hold_meas_reference(const std::vector<JointState>& current_joints,
                             double gripper_pos, double dt,
                             std::vector<JointState>* joint_reference);

    Eigen::Matrix3d vr_to_robot_axes() const;
    Eigen::Vector3d map_vr_delta_to_robot(const Eigen::Vector3d& delta_vr) const;
    Eigen::Quaterniond map_vr_delta_rot_to_robot(const Eigen::Quaterniond& delta_vr) const;
    // Clamp a relative TCP rotation (from clutch pose) to human wrist ROM, per body axis.
    Eigen::Quaterniond clamp_wrist_rom(const Eigen::Quaterniond& delta_robot) const;
    void update_vr_orientation_rate(const Eigen::Quaterniond& q_hand, double dt,
                                      const Eigen::Quaterniond& r_tcp_current);
    Eigen::Vector3d clamp_workspace(const Eigen::Vector3d& position) const;
    Eigen::Vector3d clamp_teleop_offset(const Eigen::Vector3d& position) const;
    bool cartesian_target_unchanged(const Eigen::Vector3d& p_des,
                                    const Eigen::Quaterniond& q_des) const;
    bool ik_solution_usable(const std::vector<double>& q, const Eigen::Vector3d& p_des) const;
    void refine_ik_orientation(const std::vector<double>& q_seed, const Eigen::Quaterniond& q_des,
                               double dt, std::vector<double>* q_out);
    void activate_clutch(const std::vector<JointState>& current_joints,
                         const QuestPoseSample& raw_pose);
    void deactivate_clutch(const std::vector<JointState>* current_joints);
    bool quest_watchdog_ok(const QuestPoseSample& raw_pose) const;
    CartesianControlState state_after_ik_failure() const;
    void record_fatal_error(const std::string& message);
    void update_locked_timer(double dt, bool grip_now);
    bool update_debounced_grip(bool grip_now, double dt);
    bool holds_orientation() const;

    enum class GripperMotion { Stopped, Opening, Closing };

    GripperMotion gripper_motion_ = GripperMotion::Stopped;
    double gripper_hold_pos_ = 0.0;
    bool gripper_hold_initialized_ = false;
    bool last_button_close_ = false;
    bool last_button_open_ = false;
    bool last_trigger_orientation_lock_ = false;
    bool trigger_lock_rot_valid_ = false;
    Eigen::Quaterniond trigger_lock_rot_ = Eigen::Quaterniond::Identity();

    Eigen::Quaterniond measured_tcp_orientation(const std::vector<JointState>& joints) const;
    bool trigger_orientation_lock_active(const QuestPoseSample& pose) const;
    void apply_trigger_orientation_lock(const std::vector<JointState>& current_joints,
                                        const QuestPoseSample& teleop_pose, bool grip_now);

    // --- Human virtual-skeleton redundancy resolution ---
    Eigen::Vector3d skeleton_shoulder_vr_ = Eigen::Vector3d::Zero();
    bool skeleton_calibrated_ = false;
    int skeleton_shoulder_seg_ = -1;
    int skeleton_elbow_seg_ = -1;
    double last_human_swivel_rad_ = 0.0;
    double last_robot_swivel_rad_ = 0.0;
    // TCP-local direction (unit, wrist->elbow) captured at clutch so the elbow droops at the
    // neutral gripper pose and rigidly follows TCP orientation thereafter.
    Eigen::Vector3d skeleton_forearm_axis_tcp_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    bool skeleton_forearm_axis_valid_ = false;
    std::chrono::steady_clock::time_point last_skeleton_log_time_;

    void calibrate_skeleton_shoulder(const QuestPoseSample& clutch_pose,
                                     const std::vector<double>& q_clutch);
    // Orthonormal in-plane basis (ref = gravity-down projected perp to axis, side = axis x ref).
    bool swivel_basis(const Eigen::Vector3d& axis, Eigen::Vector3d& ref, Eigen::Vector3d& side) const;
    // Desired swivel angle of the human elbow (rad) about the shoulder->hand axis, from the VR hand.
    bool human_swivel_target(const Eigen::Vector3d& hand_vr, double* swivel_out) const;
    // Target robot swivel (rad) implied by the commanded TCP orientation (rigid forearm coupling).
    bool robot_swivel_target(const std::vector<double>& q_meas, double* swivel_out) const;
    // Robot swivel angle (rad) of the elbow about the shoulder->TCP axis for a given joint vector.
    bool robot_swivel_angle(const std::vector<double>& q, double* swivel_out) const;
    // Null-space velocity that drives the robot swivel toward the human target (deterministic).
    Eigen::VectorXd compute_skeleton_nullspace_velocity(const std::vector<double>& q_meas,
                                                        double sigma_min);
    void update_gripper_from_buttons(const QuestPoseSample& raw_pose,
                                     const std::vector<JointState>& current_joints, double dt);
    void apply_gripper_to_reference(const std::vector<JointState>& current_joints, double dt,
                                    std::vector<JointState>* joint_reference);
};
