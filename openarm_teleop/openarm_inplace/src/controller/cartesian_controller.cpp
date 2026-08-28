#include <controller/cartesian_controller.hpp>

#include <openarm_constants.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace {

constexpr double kCartesianPosDeadbandM = 5e-4;
constexpr double kCartesianRotDeadbandRad = 5e-3;

double wrap_to_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

}  // namespace

CartesianController::CartesianController(Dynamics* dynamics, CartesianControllerConfig config,
                                         std::string log_prefix)
    : dynamics_(dynamics),
      config_(std::move(config)),
      log_prefix_(std::move(log_prefix)),
      cartesian_smoother_(config_.smoothing),
      joint_smoother_(config_.smoothing),
      joint_output_filter_(config_.smoothing),
      gripper_smoother_(config_.smoothing.max_gripper_speed) {
    if (config_.position_only_teleop &&
        config_.orientation_mode == OrientationMode::VR_ORIENTATION) {
        config_.orientation_mode = OrientationMode::HOLD_CLUTCH_ORIENTATION;
    }
    if (config_.dls.csv_log && !config_.dls.csv_log_path.empty()) {
        dls_csv_.open(config_.dls.csv_log_path, std::ios::out | std::ios::trunc);
    }
}

OrientationMode CartesianController::effective_orientation_mode() const {
    if (config_.position_only_teleop || runtime_position_only_) {
        return OrientationMode::HOLD_CLUTCH_ORIENTATION;
    }
    return config_.orientation_mode;
}

bool CartesianController::holds_orientation() const {
    return effective_orientation_mode() == OrientationMode::HOLD_CLUTCH_ORIENTATION;
}

void CartesianController::reset() {
    state_ = CartesianControlState::Idle;
    clutch_active_ = false;
    pose_filter_.reset();
    ik_fail_count_ = 0;
    last_safe_reference_.clear();
    last_ik_q_.clear();
    q_ref_.clear();
    q_ref_valid_ = false;
    last_vr_timestamp_ms_ = -1;
    last_vr_sequence_id_ = 0;
    last_raw_grip_active_ = false;
    last_ingested_grip_active_ = false;
    last_ingested_trigger_ = 0.0;
    last_ingested_button_close_ = false;
    last_ingested_button_open_ = false;
    last_ingested_position_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    just_activated_clutch_ = false;
    locked_elapsed_sec_ = 0.0;
    fatal_error_.clear();
    fatal_announced_ = false;
    debounced_grip_active_ = false;
    grip_debounce_initialized_ = false;
    grip_pending_elapsed_sec_ = 0.0;
    grip_pending_value_ = false;
    clutch_active_elapsed_sec_ = 0.0;
    runtime_position_only_ = false;
    filtered_vr_pose_ = QuestPoseSample{};
    last_filter_measurement_ms_ = -1;
    last_target_quat_valid_ = false;
    q_nominal_.clear();
    dls_debug_tick_ = 0;
    quest_timeout_active_ = false;
    quest_stale_active_ = false;
    require_reclutch_ = false;
    target_frozen_ = false;
    gripper_motion_ = GripperMotion::Stopped;
    gripper_hold_initialized_ = false;
    last_button_close_ = false;
    last_button_open_ = false;
    last_trigger_orientation_lock_ = false;
    trigger_lock_rot_valid_ = false;
}

void CartesianController::seed_hold_reference(const std::vector<JointState>& joints) {
    last_safe_reference_ = joints;
    sync_q_ref_to_meas(joints);
}

void CartesianController::sync_q_ref_to_meas(const std::vector<JointState>& current_joints) {
    const size_t arm_dof = dynamics_ != nullptr ? dynamics_->GetDof() : 0;
    q_ref_.assign(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof && i < current_joints.size(); ++i) {
        q_ref_[i] = current_joints[i].position;
    }
    q_ref_valid_ = !q_ref_.empty();
    last_ik_q_ = q_ref_;
}

void CartesianController::deactivate_clutch(const std::vector<JointState>* current_joints) {
    clutch_active_ = false;
    state_ = CartesianControlState::Idle;
    last_ik_q_.clear();
    q_ref_valid_ = false;
    locked_elapsed_sec_ = 0.0;
    just_activated_clutch_ = false;
    clutch_active_elapsed_sec_ = 0.0;
    runtime_position_only_ = false;
    target_frozen_ = false;
    last_vr_hand_quat_valid_ = false;
    filtered_hand_ori_valid_ = false;
    omega_des_robot_.setZero();
    last_vr_delta_angle_rad_ = 0.0;
    last_vr_delta_axis_robot_.setZero();
    trigger_lock_rot_valid_ = false;
    last_trigger_orientation_lock_ = false;

    // Freeze internal target to measured pose so re-grip does not jump from stale q_ref.
    if (current_joints != nullptr && dynamics_ != nullptr && !current_joints->empty()) {
        sync_q_ref_to_meas(*current_joints);
        const size_t arm_dof = dynamics_->GetDof();
        std::vector<double> q_meas(arm_dof, 0.0);
        for (size_t i = 0; i < arm_dof && i < current_joints->size(); ++i) {
            q_meas[i] = (*current_joints)[i].position;
        }
        Eigen::Matrix3d r_meas = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_meas = Eigen::Vector3d::Zero();
        dynamics_->GetEECordinate(q_meas.data(), r_meas, p_meas);
        p_des_target_ = p_meas;
        q_des_target_ = Eigen::Quaterniond(r_meas).normalized();
        last_solved_cart_pos_ = p_meas;
        last_solved_cart_rot_ = q_des_target_;
        cartesian_smoother_.reset(p_meas, q_des_target_);
    }
}

bool CartesianController::update_debounced_grip(bool grip_now, double dt) {
    if (!grip_debounce_initialized_) {
        debounced_grip_active_ = grip_now;
        grip_pending_value_ = grip_now;
        grip_pending_elapsed_sec_ = 0.0;
        grip_debounce_initialized_ = true;
        return debounced_grip_active_;
    }

    if (grip_now == debounced_grip_active_) {
        grip_pending_value_ = grip_now;
        grip_pending_elapsed_sec_ = 0.0;
        return debounced_grip_active_;
    }

    if (grip_now != grip_pending_value_) {
        grip_pending_value_ = grip_now;
        grip_pending_elapsed_sec_ = 0.0;
    } else {
        grip_pending_elapsed_sec_ += dt;
    }

    if (grip_pending_elapsed_sec_ >= config_.safety.grip_debounce_sec) {
        debounced_grip_active_ = grip_pending_value_;
        grip_pending_elapsed_sec_ = 0.0;
    }
    return debounced_grip_active_;
}

bool CartesianController::consume_fatal_error(std::string* message) {
    if (fatal_error_.empty() || fatal_announced_) {
        return false;
    }
    fatal_announced_ = true;
    if (message != nullptr) {
        *message = fatal_error_;
    }
    return true;
}

void CartesianController::record_fatal_error(const std::string& message) {
    if (!fatal_error_.empty()) {
        return;
    }
    fatal_error_ = message;
    std::cerr << log_prefix_ << "[cartesian][FATAL] " << message << std::endl;
}

void CartesianController::update_locked_timer(double dt, bool grip_now) {
    if (!clutch_active_ || !grip_now ||
        (state_ != CartesianControlState::Hold && state_ != CartesianControlState::Estop)) {
        locked_elapsed_sec_ = 0.0;
        return;
    }

    locked_elapsed_sec_ += dt;
    if (state_ == CartesianControlState::Estop) {
        Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
        Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
        if (!last_ik_q_.empty()) {
            dynamics_->GetEECordinate(last_ik_q_.data(), r_curr, p_curr);
        } else {
            p_curr = robot_ref_pos_;
        }
        const double dist = (p_curr - p_des_target_).norm();
        const double reachable_dist =
            std::max(config_.safety.max_ik_position_error_m * 3.0,
                     config_.workspace.max_teleop_offset_m * 0.5);
        if (dist <= reachable_dist) {
            // Stay in VR orientation mode; do not fall back to position-only teleop.
            ik_fail_count_ = 0;
            state_ = CartesianControlState::Active;
            locked_elapsed_sec_ = 0.0;
            return;
        }
        locked_elapsed_sec_ += dt;
        if (locked_elapsed_sec_ >= config_.safety.fatal_lock_timeout_sec) {
            record_fatal_error("IK failed repeatedly (estop). TCP dist to target=" +
                               std::to_string(dist) +
                               " m. Target may be outside arm reach. "
                               "Release Grip and restart teleop.");
        }
        return;
    }
    if (locked_elapsed_sec_ >= config_.safety.fatal_lock_timeout_sec) {
        record_fatal_error("Cartesian control locked in hold while Grip is pressed "
                           "(ik_fail_count=" +
                           std::to_string(ik_fail_count_) + "). Release Grip and restart teleop.");
    }
}

double CartesianController::quest_data_age_sec(const QuestPoseSample& sample) const {
    if (!sample.valid || sample.receive_timestamp_ns <= 0) {
        if (!filtered_vr_pose_.valid || last_pose_time_.time_since_epoch().count() == 0) {
            return std::numeric_limits<double>::infinity();
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_pose_time_)
            .count();
    }
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    return std::max(0.0, static_cast<double>(now_ns - sample.receive_timestamp_ns) * 1e-9);
}

bool CartesianController::quest_watchdog_ok(const QuestPoseSample& raw_pose) const {
    const double age = quest_data_age_sec(raw_pose);
    return std::isfinite(age) && age < config_.safety.quest_timeout_sec;
}

void CartesianController::handle_quest_timeout_recovery() {
    if (!quest_timeout_active_) {
        return;
    }
    // Fresh data arrived after hard timeout: require re-clutch; do not reuse old refs.
    require_reclutch_ = true;
    if (clutch_active_) {
        std::cout << log_prefix_
                  << "[cartesian] Quest data recovered after timeout; release and re-grip to "
                     "re-clutch"
                  << std::endl;
        deactivate_clutch(nullptr);
    }
    quest_timeout_active_ = false;
    target_frozen_ = false;
}

CartesianControlState CartesianController::state_after_ik_failure() const {
    if (ik_fail_count_ >= config_.safety.ik_fail_estop_count) {
        return CartesianControlState::Estop;
    }
    if (ik_fail_count_ >= config_.safety.ik_fail_hold_count) {
        return CartesianControlState::Hold;
    }
    return CartesianControlState::Active;
}

Eigen::Matrix3d CartesianController::vr_to_robot_axes() const {
    // Same linear map as translation:
    //   robot_x = -vr_z,  robot_y = -vr_x,  robot_z = +vr_y
    // VR: +X right, +Y up, +Z backward. Robot base: +X forward, +Z up.
    Eigen::Matrix3d M;
    M << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    if (config_.mirror_translation_y) {
        M.row(1) *= -1.0;
    }
    return M;
}

Eigen::Quaterniond CartesianController::map_vr_delta_rot_to_robot(
    const Eigen::Quaterniond& delta_vr) const {
    // M conjugation: VR grip axes -> robot TCP axes (v9 left/right was correct here).
    // Per-axis ±1 signs applied in TCP body frame afterward (independent pitch vs yaw).
    const Eigen::Matrix3d M = vr_to_robot_axes();
    const Eigen::Matrix3d dR = M * delta_vr.normalized().toRotationMatrix() * M.transpose();
    Eigen::AngleAxisd aa(Eigen::Quaterniond(dR).normalized());
    double angle = aa.angle();
    if (!std::isfinite(angle) || angle < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    Eigen::Vector3d rv_body = aa.axis() * angle;
    rv_body.x() *= config_.vr_tcp_roll_sign;
    rv_body.y() *= config_.vr_tcp_pitch_sign;
    rv_body.z() *= config_.vr_tcp_yaw_sign;
    angle = rv_body.norm();
    if (angle < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rv_body / angle)).normalized();
}

Eigen::Quaterniond CartesianController::clamp_wrist_rom(
    const Eigen::Quaterniond& delta_robot) const {
    if (!config_.wrist_rom_enabled) {
        return delta_robot;
    }
    Eigen::AngleAxisd aa(delta_robot.normalized());
    double angle = aa.angle();
    if (!std::isfinite(angle) || angle < 1e-9) {
        return delta_robot;
    }
    // Rotation vector in the clutch TCP body frame; clamp each component to human wrist ROM.
    Eigen::Vector3d rv = aa.axis() * angle;
    rv.x() = std::clamp(rv.x(), -config_.wrist_rom_limit_x_rad, config_.wrist_rom_limit_x_rad);
    rv.y() = std::clamp(rv.y(), -config_.wrist_rom_limit_y_rad, config_.wrist_rom_limit_y_rad);
    rv.z() = std::clamp(rv.z(), -config_.wrist_rom_limit_z_rad, config_.wrist_rom_limit_z_rad);
    const double n = rv.norm();
    if (n < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(n, rv / n)).normalized();
}

void CartesianController::update_vr_orientation_rate(const Eigen::Quaterniond& q_hand,
                                                     double dt,
                                                     const Eigen::Quaterniond& r_tcp_current) {
    omega_des_robot_.setZero();
    last_vr_delta_angle_rad_ = 0.0;
    last_vr_delta_axis_robot_.setZero();
    if (dt <= 0.0) {
        last_vr_hand_quat_ = q_hand.normalized();
        last_vr_hand_quat_valid_ = true;
        return;
    }

    if (!last_vr_hand_quat_valid_) {
        last_vr_hand_quat_ = q_hand.normalized();
        last_vr_hand_quat_valid_ = true;
        return;
    }

    Eigen::Quaterniond delta_vr =
        (last_vr_hand_quat_.inverse() * q_hand.normalized()).normalized();
    if (delta_vr.w() < 0.0) {
        delta_vr.coeffs() = -delta_vr.coeffs();
    }
    if (config_.rot_scale != 1.0) {
        Eigen::AngleAxisd axis_angle(delta_vr.normalized());
        double angle = axis_angle.angle();
        if (!std::isfinite(angle)) {
            angle = 0.0;
        }
        delta_vr = Eigen::Quaterniond(
            Eigen::AngleAxisd(angle * config_.rot_scale, axis_angle.axis()));
    }

    const Eigen::Quaterniond delta_robot = map_vr_delta_rot_to_robot(delta_vr);
    Eigen::AngleAxisd axis_angle(delta_robot.normalized());
    double angle = axis_angle.angle();
    if (!std::isfinite(angle) || angle < 1e-9) {
        last_vr_hand_quat_ = q_hand.normalized();
        return;
    }

    last_vr_delta_angle_rad_ = angle;
    last_vr_delta_axis_robot_ = axis_angle.axis();
    const Eigen::Vector3d rotvec_body = axis_angle.axis() * angle;
    omega_des_robot_ = r_tcp_current.normalized() * (rotvec_body / dt);
    const double wmax = config_.dls.max_angular_speed_radps;
    const double wnorm = omega_des_robot_.norm();
    if (wnorm > wmax && wnorm > 1e-9) {
        omega_des_robot_ *= wmax / wnorm;
    }

    last_vr_hand_quat_ = q_hand.normalized();
    last_vr_hand_quat_valid_ = true;
}

Eigen::Vector3d CartesianController::map_vr_delta_to_robot(const Eigen::Vector3d& delta_vr) const {
    return (vr_to_robot_axes() * delta_vr) * config_.pos_scale;
}

double CartesianController::rotation_target_offset_rad() const {
    const Eigen::Quaterniond q_err = robot_ref_rot_.inverse() * q_des_target_.normalized();
    Eigen::AngleAxisd axis_angle(q_err.normalized());
    double angle = axis_angle.angle();
    if (!std::isfinite(angle)) {
        angle = 0.0;
    }
    return angle;
}

Eigen::Vector3d CartesianController::clamp_workspace(const Eigen::Vector3d& position) const {
    Eigen::Vector3d clamped = position;
    clamped.z() = std::clamp(clamped.z(), config_.workspace.min_z, config_.workspace.max_z);

    const double radius = clamped.head<2>().norm();
    if (radius > config_.workspace.max_radius) {
        clamped.head<2>() *= config_.workspace.max_radius / radius;
    }
    return clamped;
}

Eigen::Vector3d CartesianController::clamp_teleop_offset(const Eigen::Vector3d& position) const {
    const Eigen::Vector3d offset = position - robot_ref_pos_;
    const double max_offset = config_.workspace.max_teleop_offset_m;
    if (offset.norm() <= max_offset) {
        return position;
    }
    return robot_ref_pos_ + offset.normalized() * max_offset;
}

bool CartesianController::cartesian_target_unchanged(const Eigen::Vector3d& p_des,
                                                     const Eigen::Quaterniond& q_des) const {
    if (last_ik_q_.empty()) {
        return false;
    }
    if ((p_des - last_solved_cart_pos_).norm() > kCartesianPosDeadbandM) {
        return false;
    }
    if (holds_orientation()) {
        return true;
    }
    const Eigen::Quaterniond q_err =
        last_solved_cart_rot_.inverse() * q_des.normalized();
    Eigen::AngleAxisd axis_angle(q_err.normalized());
    double angle = axis_angle.angle();
    if (!std::isfinite(angle)) {
        angle = 0.0;
    }
    return angle <= kCartesianRotDeadbandRad;
}

bool CartesianController::ik_solution_usable(const std::vector<double>& q,
                                             const Eigen::Vector3d& p_des) const {
    Eigen::Matrix3d r_fk = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_fk = Eigen::Vector3d::Zero();
    dynamics_->GetEECordinate(q.data(), r_fk, p_fk);
    return (p_fk - p_des).norm() <= config_.safety.max_ik_position_error_m;
}

void CartesianController::refine_ik_orientation(const std::vector<double>& q_seed,
                                                const Eigen::Quaterniond& q_des, double dt,
                                                std::vector<double>* q_out) {
    if (q_out == nullptr || holds_orientation()) {
        return;
    }
    dynamics_->RefineIkOrientation(q_seed.data(), q_des, dt, q_out->data());
}

void CartesianController::activate_clutch(const std::vector<JointState>& current_joints,
                                          const QuestPoseSample& raw_pose) {
    hand_ref_pos_ = raw_pose.position;
    hand_ref_rot_ = raw_pose.orientation.normalized();

    const size_t arm_dof = dynamics_->GetDof();
    std::vector<double> q_curr(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof && i < current_joints.size(); ++i) {
        q_curr[i] = current_joints[i].position;
    }

    Eigen::Matrix3d r_robot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_robot = Eigen::Vector3d::Zero();
    dynamics_->GetEECordinate(q_curr.data(), r_robot, p_robot);

    robot_ref_pos_ = p_robot;
    robot_ref_rot_ = Eigen::Quaterniond(r_robot).normalized();
    clutch_active_ = true;
    ik_fail_count_ = 0;
    runtime_position_only_ = false;
    locked_elapsed_sec_ = 0.0;
    state_ = CartesianControlState::Active;
    last_safe_reference_ = current_joints;
    last_ik_q_ = q_curr;
    sync_q_ref_to_meas(current_joints);
    last_solved_cart_pos_ = p_robot;
    last_solved_cart_rot_ = robot_ref_rot_;

    cartesian_smoother_.reset(p_robot, robot_ref_rot_);
    p_des_target_ = p_robot;
    q_des_target_ = robot_ref_rot_;  // HOLD_CLUTCH: R_des fixed at clutch pose
    last_ik_fail_reset_pos_ = p_robot;
    require_reclutch_ = false;
    target_frozen_ = false;
    quest_timeout_active_ = false;
    quest_stale_active_ = false;

    std::vector<double> dq0(arm_dof, 0.0);
    joint_smoother_.reset(q_curr, dq0);
    joint_output_filter_.reset(q_curr);
    gripper_smoother_.reset(current_joints.size() > arm_dof ? current_joints[arm_dof].position
                                                            : config_.gripper_open);
    // Nullspace bias follows the current clutch pose so each re-grip re-anchors the arm.
    if (config_.dls.nullspace_nominal_mode == NullspaceNominalMode::TABLE_READY_POSE &&
        config_.dls.table_ready_pose.size() == arm_dof) {
        q_nominal_ = config_.dls.table_ready_pose;
    } else {
        q_nominal_ = q_curr;
    }
    last_target_quat_ = robot_ref_rot_;
    last_target_quat_valid_ = true;
    last_vr_hand_quat_ = hand_ref_rot_;
    last_vr_hand_quat_valid_ = true;
    filtered_hand_ori_ = hand_ref_rot_;
    filtered_hand_ori_valid_ = true;
    omega_des_robot_.setZero();
    last_vr_delta_angle_rad_ = 0.0;
    last_vr_delta_axis_robot_.setZero();
    calibrate_skeleton_shoulder(raw_pose, q_curr);
    last_human_swivel_rad_ = config_.skeleton.swivel_bias_rad;
    std::cout << log_prefix_ << "[cartesian] clutch activated (rising edge): hand_ref=["
              << hand_ref_pos_.transpose() << "] tcp_ref=[" << p_robot.transpose()
              << "] ori_mode="
              << (holds_orientation() ? "HOLD_CLUTCH_ORIENTATION" : "VR_ORIENTATION")
              << " ori_map=M_conj" << std::endl;
}

bool CartesianController::ingest_vr_sample(const QuestPoseSample& raw_pose) {
    if (!raw_pose.valid) {
        return false;
    }
    const bool grip_changed = raw_pose.grip_active != last_ingested_grip_active_;
    const bool trigger_changed =
        std::abs(raw_pose.trigger - last_ingested_trigger_) > 1e-6;
    const bool button_changed =
        raw_pose.button_close != last_ingested_button_close_ ||
        raw_pose.button_open != last_ingested_button_open_;
    const bool position_changed =
        !last_ingested_position_.allFinite() ||
        (raw_pose.position - last_ingested_position_).norm() > 1e-5;
    const bool seq_changed = raw_pose.sequence_id != 0 && raw_pose.sequence_id != last_vr_sequence_id_;
    if (raw_pose.timestamp_ms == last_vr_timestamp_ms_ && !grip_changed && !trigger_changed &&
        !button_changed && !position_changed && !seq_changed) {
        return false;
    }

    last_vr_timestamp_ms_ = raw_pose.timestamp_ms;
    last_vr_sequence_id_ = raw_pose.sequence_id;
    last_ingested_grip_active_ = raw_pose.grip_active;
    last_ingested_trigger_ = raw_pose.trigger;
    last_ingested_button_close_ = raw_pose.button_close;
    last_ingested_button_open_ = raw_pose.button_open;
    last_ingested_position_ = raw_pose.position;
    last_pose_time_ = std::chrono::steady_clock::now();
    return true;
}

void CartesianController::update_filtered_vr_pose(const QuestPoseSample& raw_pose, double dt) {
    if (dt <= 0.0) {
        return;
    }

    QuestPoseSample filtered;
    const bool new_measurement =
        raw_pose.valid && raw_pose.timestamp_ms != last_filter_measurement_ms_;

    if (new_measurement) {
        // Quest sample interval — independent of controller dt.
        double filter_dt = 1.0 / 90.0;
        if (last_filter_measurement_ms_ >= 0) {
            filter_dt = (raw_pose.timestamp_ms - last_filter_measurement_ms_) * 0.001;
            filter_dt = std::clamp(filter_dt, 1.0 / 120.0, 0.2);
        }
        if (!pose_filter_.update(raw_pose, filter_dt, &filtered)) {
            return;
        }
        last_filter_measurement_ms_ = raw_pose.timestamp_ms;
        last_pose_time_ = std::chrono::steady_clock::now();
    } else if (raw_pose.valid && filtered_vr_pose_.valid) {
        // Do not extrapolate VR pose between Quest frames for teleop target.
        filtered = filtered_vr_pose_;
        filtered.grip_active = raw_pose.grip_active;
        filtered.trigger = raw_pose.trigger;
        filtered.button_close = raw_pose.button_close;
        filtered.button_open = raw_pose.button_open;
        filtered.timestamp_ms = raw_pose.timestamp_ms;
        filtered.receive_timestamp_ns = raw_pose.receive_timestamp_ns;
        filtered.sequence_id = raw_pose.sequence_id;
    } else if (filtered_vr_pose_.valid) {
        filtered = filtered_vr_pose_;
    } else if (raw_pose.valid) {
        if (!pose_filter_.update(raw_pose, std::max(dt, 1.0 / 120.0), &filtered)) {
            return;
        }
        last_filter_measurement_ms_ = raw_pose.timestamp_ms;
        last_pose_time_ = std::chrono::steady_clock::now();
    } else {
        return;
    }

    filtered_vr_pose_ = filtered;
    filtered_vr_pose_.valid = true;
    if (raw_pose.valid) {
        filtered_vr_pose_.grip_active = raw_pose.grip_active;
        filtered_vr_pose_.trigger = raw_pose.trigger;
        filtered_vr_pose_.button_close = raw_pose.button_close;
        filtered_vr_pose_.button_open = raw_pose.button_open;
        filtered_vr_pose_.timestamp_ms = raw_pose.timestamp_ms;
        filtered_vr_pose_.receive_timestamp_ns = raw_pose.receive_timestamp_ns;
        filtered_vr_pose_.sequence_id = raw_pose.sequence_id;
    }
}

void CartesianController::update_cartesian_target_from_vr(const QuestPoseSample& filtered_pose) {
    if (target_frozen_) {
        // Stale Quest: keep last T_target, do not extrapolate.
        return;
    }

    const Eigen::Vector3d delta_hand_body =
        hand_ref_rot_.inverse() * (filtered_pose.position - hand_ref_pos_);
    // Body-frame delta: pure controller rotation does not inject a position target.
    // World-frame delta (old) made tilt look like arm translation.
    const Eigen::Vector3d delta_robot = map_vr_delta_to_robot(delta_hand_body);
    const Eigen::Vector3d p_unclamped = robot_ref_pos_ + delta_robot;
    teleop_offset_saturated_ =
        (p_unclamped - robot_ref_pos_).norm() >
        config_.workspace.max_teleop_offset_m * 0.999;
    workspace_radius_saturated_ =
        p_unclamped.head<2>().norm() > config_.workspace.max_radius * 0.999;
    p_des_target_ = clamp_workspace(clamp_teleop_offset(p_unclamped));
    if (teleop_offset_saturated_ || workspace_radius_saturated_) {
        const auto now = std::chrono::steady_clock::now();
        if (last_workspace_warn_time_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_workspace_warn_time_).count() >= 1.0) {
            last_workspace_warn_time_ = now;
            std::cout << log_prefix_ << "[cartesian] workspace limit: off="
                      << (p_unclamped - robot_ref_pos_).norm() << "/"
                      << config_.workspace.max_teleop_offset_m << "m xy_r="
                      << p_unclamped.head<2>().norm() << "/"
                      << config_.workspace.max_radius << "m (VR hand can move further; "
                         "TCP target clamped — arm looks bent but cannot extend)"
                      << std::endl;
        }
    }

    if ((p_des_target_ - last_ik_fail_reset_pos_).norm() > 0.01) {
        ik_fail_count_ = 0;
        last_ik_fail_reset_pos_ = p_des_target_;
    }

    if (holds_orientation() ||
        effective_orientation_mode() == OrientationMode::FREE_ORIENTATION) {
        // HOLD: R_des fixed at clutch. FREE: orientation not commanded via VR.
        q_des_target_ = robot_ref_rot_;
    } else {
        // VR_ORIENTATION: hand delta in grip frame -> TCP-frame rotvec -> R_des on clutch body.
        Eigen::Quaterniond q_hand = filtered_pose.orientation.normalized();
        q_hand = align_quaternion_sign(hand_ref_rot_, q_hand);
        Eigen::Quaterniond delta_vr = (hand_ref_rot_.inverse() * q_hand).normalized();
        if (delta_vr.w() < 0.0) {
            delta_vr.coeffs() = -delta_vr.coeffs();
        }
        if (config_.rot_scale != 1.0) {
            Eigen::AngleAxisd axis_angle(delta_vr.normalized());
            double angle = axis_angle.angle();
            if (!std::isfinite(angle)) {
                angle = 0.0;
            }
            delta_vr = Eigen::Quaterniond(
                Eigen::AngleAxisd(angle * config_.rot_scale, axis_angle.axis()));
        }
        const Eigen::Quaterniond delta_robot = clamp_wrist_rom(map_vr_delta_rot_to_robot(delta_vr));
        Eigen::Quaterniond q_des = (robot_ref_rot_ * delta_robot).normalized();
        if (last_target_quat_valid_) {
            q_des = align_quaternion_sign(last_target_quat_, q_des);
        }
        q_des_target_ = q_des;
        last_target_quat_ = q_des_target_;
        last_target_quat_valid_ = true;
    }
    cartesian_smoother_.set_target(p_des_target_, q_des_target_);
}

Eigen::Quaterniond CartesianController::measured_tcp_orientation(
    const std::vector<JointState>& joints) const {
    if (dynamics_ == nullptr) {
        return Eigen::Quaterniond::Identity();
    }
    const size_t arm_dof = dynamics_->GetDof();
    std::vector<double> q_meas(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof && i < joints.size(); ++i) {
        q_meas[i] = joints[i].position;
    }
    Eigen::Matrix3d r_tcp = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_tmp = Eigen::Vector3d::Zero();
    dynamics_->GetEECordinate(q_meas.data(), r_tcp, p_tmp);
    return Eigen::Quaterniond(r_tcp).normalized();
}

bool CartesianController::trigger_orientation_lock_active(const QuestPoseSample& pose) const {
    return pose.valid && pose.trigger >= 0.5;
}

void CartesianController::apply_trigger_orientation_lock(
    const std::vector<JointState>& current_joints, const QuestPoseSample& teleop_pose,
    bool grip_now) {
    if (!clutch_active_ || !grip_now ||
        effective_orientation_mode() != OrientationMode::VR_ORIENTATION || holds_orientation()) {
        last_trigger_orientation_lock_ = false;
        return;
    }

    const bool lock_now = trigger_orientation_lock_active(teleop_pose);
    if (lock_now && !last_trigger_orientation_lock_) {
        trigger_lock_rot_ = measured_tcp_orientation(current_joints);
        trigger_lock_rot_valid_ = true;
        std::cout << log_prefix_ << "[cartesian] trigger held: TCP orientation locked (position "
                     "teleop only)"
                  << std::endl;
    } else if (!lock_now && last_trigger_orientation_lock_) {
        std::cout << log_prefix_ << "[cartesian] trigger released: VR orientation teleop resumed"
                  << std::endl;
    }
    last_trigger_orientation_lock_ = lock_now;

    if (!lock_now || !trigger_lock_rot_valid_) {
        return;
    }

    q_des_target_ = trigger_lock_rot_;
    last_target_quat_ = q_des_target_;
    last_target_quat_valid_ = true;
    omega_des_robot_.setZero();
    last_vr_delta_angle_rad_ = 0.0;
    last_vr_delta_axis_robot_.setZero();
    cartesian_smoother_.set_target(p_des_target_, q_des_target_);
}

bool CartesianController::apply_gripper_reference(size_t arm_dof, size_t total_dof,
                                                double gripper_pos, double dt,
                                                std::vector<JointState>* joint_reference) {
    if (total_dof <= arm_dof || joint_reference == nullptr) {
        return true;
    }
    if (config_.direct_ik_control) {
        (*joint_reference)[arm_dof].position = gripper_pos;
        (*joint_reference)[arm_dof].velocity = 0.0;
        (*joint_reference)[arm_dof].effort = 0.0;
        return true;
    }
    gripper_smoother_.set_target(gripper_pos);
    (*joint_reference)[arm_dof].position = gripper_smoother_.step(dt);
    (*joint_reference)[arm_dof].velocity = 0.0;
    (*joint_reference)[arm_dof].effort = 0.0;
    return true;
}

void CartesianController::update_gripper_from_buttons(
    const QuestPoseSample& raw_pose, const std::vector<JointState>& current_joints, double dt) {
    const size_t arm_dof = dynamics_ != nullptr ? dynamics_->GetDof() : 0;
    if (arm_dof == 0 || current_joints.size() <= arm_dof || dt <= 0.0) {
        return;
    }

    const double q_meas = current_joints[arm_dof].position;
    constexpr double kGripperTol = 0.0005;
    const double speed = config_.smoothing.max_gripper_speed;

    if (!gripper_hold_initialized_) {
        gripper_hold_pos_ = q_meas;
        gripper_target_ = q_meas;
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_initialized_ = true;
        gripper_smoother_.reset(q_meas);
    }

    if (gripper_motion_ == GripperMotion::Opening &&
        q_meas <= config_.gripper_open + kGripperTol) {
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_pos_ = config_.gripper_open;
        gripper_target_ = config_.gripper_open;
    } else if (gripper_motion_ == GripperMotion::Closing &&
               q_meas >= config_.gripper_closed - kGripperTol) {
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_pos_ = config_.gripper_closed;
        gripper_target_ = config_.gripper_closed;
    }

    if (raw_pose.valid) {
        last_button_close_ = raw_pose.button_close;
        last_button_open_ = raw_pose.button_open;

        // Hold-to-move: A/X held → close, B/Y held → open, release → freeze.
        const bool want_close = raw_pose.button_close && !raw_pose.button_open;
        const bool want_open = raw_pose.button_open && !raw_pose.button_close;
        const GripperMotion prev = gripper_motion_;

        if (want_close) {
            if (q_meas < config_.gripper_closed - kGripperTol) {
                if (prev != GripperMotion::Closing) {
                    gripper_motion_ = GripperMotion::Closing;
                    std::cout << log_prefix_ << "[cartesian] gripper closing (hold A/X)"
                              << std::endl;
                }
            } else {
                gripper_motion_ = GripperMotion::Stopped;
                gripper_hold_pos_ = config_.gripper_closed;
                gripper_target_ = config_.gripper_closed;
            }
        } else if (want_open) {
            if (q_meas > config_.gripper_open + kGripperTol) {
                if (prev != GripperMotion::Opening) {
                    gripper_motion_ = GripperMotion::Opening;
                    std::cout << log_prefix_ << "[cartesian] gripper opening (hold B/Y)"
                              << std::endl;
                }
            } else {
                gripper_motion_ = GripperMotion::Stopped;
                gripper_hold_pos_ = config_.gripper_open;
                gripper_target_ = config_.gripper_open;
            }
        } else if (prev == GripperMotion::Closing || prev == GripperMotion::Opening) {
            gripper_motion_ = GripperMotion::Stopped;
            gripper_hold_pos_ = q_meas;
            gripper_target_ = q_meas;
            std::cout << log_prefix_ << "[cartesian] gripper hold (release) pos=" << q_meas
                      << std::endl;
        }
    }

    switch (gripper_motion_) {
        case GripperMotion::Stopped:
            gripper_target_ = gripper_hold_pos_;
            break;
        case GripperMotion::Opening:
            gripper_target_ = std::max(config_.gripper_open, gripper_target_ - speed * dt);
            break;
        case GripperMotion::Closing:
            gripper_target_ = std::min(config_.gripper_closed, gripper_target_ + speed * dt);
            break;
    }
    gripper_smoother_.set_target(gripper_target_);
}

void CartesianController::apply_gripper_to_reference(
    const std::vector<JointState>& current_joints, double dt,
    std::vector<JointState>* joint_reference) {
    if (joint_reference == nullptr || dynamics_ == nullptr) {
        return;
    }
    const size_t arm_dof = dynamics_->GetDof();
    apply_gripper_reference(arm_dof, joint_reference->size(), gripper_target_, dt, joint_reference);
    last_safe_reference_ = *joint_reference;
}

bool CartesianController::compute_joint_reference(
    const std::vector<JointState>& current_joints, double dt, const Eigen::Vector3d& p_des,
    const Eigen::Quaterniond& q_des, double gripper_pos, std::vector<JointState>* joint_reference) {
    if (config_.dls.enabled) {
        return compute_joint_reference_dls(current_joints, dt, p_des, q_des, gripper_pos, -1.0,
                                           joint_reference);
    }
    return compute_joint_reference_position_ik(current_joints, dt, p_des, q_des, gripper_pos,
                                               joint_reference);
}

Eigen::Quaterniond CartesianController::align_quaternion_sign(const Eigen::Quaterniond& reference,
                                                              Eigen::Quaterniond candidate) {
    candidate.normalize();
    if (reference.coeffs().dot(candidate.coeffs()) < 0.0) {
        candidate.coeffs() = -candidate.coeffs();
    }
    return candidate.normalized();
}

Eigen::Vector3d CartesianController::rotation_error_vector(const Eigen::Quaterniond& q_current,
                                                          const Eigen::Quaterniond& q_desired) {
    // Base-frame rotation error: e_ori = log(R_des * R_cur^T).
    // Must match KDL geometric Jacobian (ω expressed in base), not body-frame q_cur^{-1}*q_des.
    Eigen::Quaterniond q_err =
        (q_desired.normalized() * q_current.normalized().inverse()).normalized();
    if (q_err.w() < 0.0) {
        q_err.coeffs() = -q_err.coeffs();
    }
    Eigen::AngleAxisd aa(q_err);
    double angle = aa.angle();
    if (!std::isfinite(angle) || angle < 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    return aa.axis() * angle;
}

double CartesianController::joint_velocity_limit(size_t joint_index) const {
    if (joint_index < config_.dls.max_joint_velocity_radps_vec.size()) {
        return config_.dls.max_joint_velocity_radps_vec[joint_index];
    }
    return config_.dls.max_joint_velocity_radps;
}

Eigen::VectorXd CartesianController::compute_nullspace_velocity(const double* q_meas) const {
    const size_t dof = dynamics_->GetDof();
    Eigen::VectorXd q_dot_null = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof));
    if (!config_.dls.use_nullspace || q_nominal_.size() != dof || q_meas == nullptr) {
        return q_dot_null;
    }

    Eigen::MatrixXd N;
    dynamics_->GetNullSpace(q_meas, N);
    Eigen::VectorXd q_err(static_cast<Eigen::Index>(dof));
    for (size_t i = 0; i < dof; ++i) {
        double w = 1.0;
        if (i < config_.dls.nullspace_joint_weights.size()) {
            w = config_.dls.nullspace_joint_weights[i];
        }
        q_err[static_cast<Eigen::Index>(i)] = w * (q_nominal_[i] - q_meas[i]);
    }
    q_dot_null = N * (config_.dls.nullspace_gain * q_err);
    for (Eigen::Index i = 0; i < q_dot_null.size(); ++i) {
        const double vmax = joint_velocity_limit(static_cast<size_t>(i));
        q_dot_null[i] = std::clamp(q_dot_null[i], -vmax, vmax);
    }
    return q_dot_null;
}

void CartesianController::calibrate_skeleton_shoulder(const QuestPoseSample& clutch_pose,
                                                      const std::vector<double>& q_clutch) {
    if (!config_.skeleton.enabled) {
        skeleton_calibrated_ = false;
        return;
    }
    skeleton_shoulder_vr_ = clutch_pose.position + config_.skeleton.shoulder_offset_vr;
    const int nseg = dynamics_ != nullptr ? static_cast<int>(dynamics_->GetNrOfSegments()) : 0;
    skeleton_shoulder_seg_ = config_.skeleton.shoulder_segment >= 0
                                 ? config_.skeleton.shoulder_segment
                                 : std::max(1, static_cast<int>(std::lround(0.25 * nseg)));
    skeleton_elbow_seg_ = config_.skeleton.elbow_segment >= 0
                              ? config_.skeleton.elbow_segment
                              : std::max(2, static_cast<int>(std::lround(0.60 * nseg)));
    skeleton_shoulder_seg_ = std::clamp(skeleton_shoulder_seg_, 0, std::max(0, nseg));
    skeleton_elbow_seg_ = std::clamp(skeleton_elbow_seg_, 0, std::max(0, nseg));
    skeleton_calibrated_ = true;

    // Capture the TCP-local forearm axis so that at the clutch (neutral) gripper pose the elbow
    // droops straight down, and thereafter the elbow rigidly follows the TCP orientation. This
    // makes the hand->config map single-valued (returnable) and anatomically coupled.
    skeleton_forearm_axis_valid_ = false;
    if (dynamics_ != nullptr && q_clutch.size() == dynamics_->GetDof()) {
        Eigen::Matrix3d r_tmp = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_sh = Eigen::Vector3d::Zero();
        Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
        if (dynamics_->GetSegmentCordinate(q_clutch.data(), skeleton_shoulder_seg_, r_tmp, p_sh)) {
            dynamics_->GetEECordinate(q_clutch.data(), r_tmp, p_tcp);
            Eigen::Vector3d ref;
            Eigen::Vector3d side;
            if (swivel_basis(p_tcp - p_sh, ref, side)) {
                // ref is the base-frame droop direction; express it in the TCP frame at clutch.
                skeleton_forearm_axis_tcp_ = (robot_ref_rot_.normalized().conjugate() * ref).normalized();
                skeleton_forearm_axis_valid_ = true;
            }
        }
    }
    std::cout << log_prefix_ << "[skeleton] calibrated: shoulder_vr=["
              << skeleton_shoulder_vr_.transpose() << "] seg(sh=" << skeleton_shoulder_seg_
              << ",el=" << skeleton_elbow_seg_ << "/" << nseg << ") Lu="
              << config_.skeleton.upper_arm_len_m << " Lf=" << config_.skeleton.forearm_len_m
              << " fa_tcp=[" << skeleton_forearm_axis_tcp_.transpose()
              << "] coupling=" << config_.skeleton.orientation_coupling << std::endl;
}

bool CartesianController::human_swivel_target(const Eigen::Vector3d& hand_vr,
                                              double* swivel_out) const {
    if (swivel_out == nullptr || !skeleton_calibrated_) {
        return false;
    }
    // 2-link human arm: shoulder -> elbow -> hand. Natural posture = elbow droops along the
    // gravity-down direction projected perpendicular to the shoulder->hand axis. The swivel angle
    // measured from that down-reference is therefore the configured bias (0 = pure droop).
    const Eigen::Vector3d s_to_h = hand_vr - skeleton_shoulder_vr_;
    const double L = s_to_h.norm();
    if (!(L > 1e-4)) {
        return false;
    }
    // (reachability check only; result is the droop swivel angle)
    *swivel_out = config_.skeleton.swivel_bias_rad;
    return true;
}

bool CartesianController::swivel_basis(const Eigen::Vector3d& axis, Eigen::Vector3d& ref,
                                      Eigen::Vector3d& side) const {
    const double axis_norm = axis.norm();
    if (!(axis_norm > 1e-6)) {
        return false;
    }
    const Eigen::Vector3d a = axis / axis_norm;
    const Eigen::Vector3d down = config_.skeleton.down_robot;
    ref = down - down.dot(a) * a;
    if (ref.norm() < 1e-4) {
        // Arm nearly aligned with gravity: pick an arbitrary perpendicular reference.
        const Eigen::Vector3d alt =
            std::abs(a.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        ref = alt - alt.dot(a) * a;
    }
    ref.normalize();
    side = a.cross(ref).normalized();
    return true;
}

bool CartesianController::robot_swivel_angle(const std::vector<double>& q,
                                             double* swivel_out) const {
    if (swivel_out == nullptr || dynamics_ == nullptr || !skeleton_calibrated_) {
        return false;
    }
    Eigen::Matrix3d r_tmp = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_sh = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_el = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
    if (!dynamics_->GetSegmentCordinate(q.data(), skeleton_shoulder_seg_, r_tmp, p_sh) ||
        !dynamics_->GetSegmentCordinate(q.data(), skeleton_elbow_seg_, r_tmp, p_el)) {
        return false;
    }
    dynamics_->GetEECordinate(q.data(), r_tmp, p_tcp);

    const Eigen::Vector3d axis = p_tcp - p_sh;
    Eigen::Vector3d ref;
    Eigen::Vector3d side;
    if (!swivel_basis(axis, ref, side)) {
        return false;
    }
    const Eigen::Vector3d a = axis.normalized();
    Eigen::Vector3d e_off = (p_el - p_sh);
    e_off -= e_off.dot(a) * a;
    if (e_off.norm() < 1e-5) {
        return false;
    }
    *swivel_out = std::atan2(e_off.dot(side), e_off.dot(ref));
    return true;
}

bool CartesianController::robot_swivel_target(const std::vector<double>& q_meas,
                                             double* swivel_out) const {
    if (swivel_out == nullptr || dynamics_ == nullptr || !skeleton_calibrated_) {
        return false;
    }
    // Fall back to the fixed droop bias when orientation coupling is disabled.
    if (config_.skeleton.orientation_coupling <= 1e-6 || !skeleton_forearm_axis_valid_) {
        *swivel_out = config_.skeleton.swivel_bias_rad;
        return true;
    }
    Eigen::Matrix3d r_tmp = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_sh = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
    if (!dynamics_->GetSegmentCordinate(q_meas.data(), skeleton_shoulder_seg_, r_tmp, p_sh)) {
        return false;
    }
    dynamics_->GetEECordinate(q_meas.data(), r_tmp, p_tcp);

    const Eigen::Vector3d axis = p_tcp - p_sh;
    Eigen::Vector3d ref;
    Eigen::Vector3d side;
    if (!swivel_basis(axis, ref, side)) {
        return false;
    }
    const Eigen::Vector3d a = axis.normalized();
    // Rigid forearm: the elbow direction is the TCP-local forearm axis rotated by the commanded
    // TCP orientation, projected onto the swivel plane. This makes the elbow follow the gripper.
    Eigen::Vector3d f_des = q_des_target_.normalized() * skeleton_forearm_axis_tcp_;
    f_des -= f_des.dot(a) * a;
    if (f_des.norm() < 1e-4) {
        // Forearm axis nearly parallel to arm axis (near elbow singularity): hold droop.
        *swivel_out = config_.skeleton.swivel_bias_rad;
        return true;
    }
    const double psi_ori = std::atan2(f_des.dot(side), f_des.dot(ref));
    *swivel_out = config_.skeleton.swivel_sign * config_.skeleton.orientation_coupling * psi_ori +
                  config_.skeleton.swivel_bias_rad;
    return true;
}

Eigen::VectorXd CartesianController::compute_skeleton_nullspace_velocity(
    const std::vector<double>& q_meas, double sigma_min) {
    const size_t dof = dynamics_->GetDof();
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof));
    if (!config_.skeleton.enabled || !skeleton_calibrated_ || q_meas.size() != dof) {
        return dq;
    }
    // Near a singularity the null-space is ill-conditioned; self-motion leaks into the TCP and the
    // swivel servo thrashes (observed as the end-effector "spinning" on its own). Stand down.
    if (sigma_min < 0.02) {
        return dq;
    }

    double psi_r = 0.0;
    if (!robot_swivel_angle(q_meas, &psi_r)) {
        return dq;
    }
    last_robot_swivel_rad_ = psi_r;
    double psi_des = config_.skeleton.swivel_bias_rad;
    if (!robot_swivel_target(q_meas, &psi_des)) {
        return dq;
    }
    last_human_swivel_rad_ = psi_des;  // logged as psi_h (now the orientation-coupled target)
    const double err = wrap_to_pi(psi_des - psi_r);
    // Deadband: don't creep the arm to fix negligible swivel error (looks like self-drift).
    if (std::abs(err) < 0.05) {
        return dq;
    }

    Eigen::MatrixXd N;
    dynamics_->GetNullSpace(q_meas.data(), N);
    if (N.rows() != static_cast<Eigen::Index>(dof) || N.cols() < 1) {
        return dq;
    }
    Eigen::VectorXd n_unit = N.col(0);
    const double n_norm = n_unit.norm();
    if (!(n_norm > 1e-6)) {
        return dq;
    }
    n_unit /= n_norm;

    // Numerically measure how swivel changes along the self-motion direction (deterministic sign).
    const double eps = 1e-3;
    std::vector<double> q_pert(q_meas);
    for (size_t i = 0; i < dof; ++i) {
        q_pert[i] += eps * n_unit[static_cast<Eigen::Index>(i)];
    }
    double psi_r_pert = 0.0;
    if (!robot_swivel_angle(q_pert, &psi_r_pert)) {
        return dq;
    }
    const double sens = wrap_to_pi(psi_r_pert - psi_r) / eps;
    if (std::abs(sens) < 0.02) {
        return dq;  // self-motion barely affects swivel (degenerate/near-singular) — do nothing
    }

    double rate = config_.skeleton.swivel_gain * err / sens;
    rate = std::clamp(rate, -config_.skeleton.max_swivel_rate, config_.skeleton.max_swivel_rate);
    dq = n_unit * rate;
    for (Eigen::Index i = 0; i < dq.size(); ++i) {
        const double vmax = joint_velocity_limit(static_cast<size_t>(i));
        dq[i] = std::clamp(dq[i], -vmax, vmax);
    }
    return dq;
}

void CartesianController::apply_soft_joint_limits(const std::vector<double>& q_meas,
                                                  Eigen::VectorXd* q_dot) const {
    if (q_dot == nullptr) {
        return;
    }
    std::vector<double> q_min;
    std::vector<double> q_max;
    if (!dynamics_->GetJointPositionLimits(&q_min, &q_max) || q_min.size() != q_meas.size()) {
        return;
    }
    const double margin_frac = std::clamp(config_.dls.soft_limit_margin_frac, 0.01, 0.45);
    for (size_t i = 0; i < q_meas.size() && static_cast<Eigen::Index>(i) < q_dot->size(); ++i) {
        const double span = q_max[i] - q_min[i];
        if (!(span > 1e-6)) {
            continue;
        }
        const double soft = span * margin_frac;
        const double q = q_meas[i];
        double& dq = (*q_dot)[static_cast<Eigen::Index>(i)];
        // Near upper limit: scale down positive velocity.
        if (dq > 0.0) {
            const double room = q_max[i] - q;
            if (room <= 0.0) {
                dq = 0.0;
            } else if (room < soft) {
                dq *= room / soft;
            }
        }
        // Near lower limit: scale down negative velocity.
        if (dq < 0.0) {
            const double room = q - q_min[i];
            if (room <= 0.0) {
                dq = 0.0;
            } else if (room < soft) {
                dq *= room / soft;
            }
        }
    }
}

void CartesianController::clamp_tracking_error(const std::vector<double>& q_meas,
                                               std::vector<double>* q_ref) const {
    if (q_ref == nullptr) {
        return;
    }
    const double max_err = config_.dls.max_tracking_error_rad;
    for (size_t i = 0; i < q_ref->size() && i < q_meas.size(); ++i) {
        const double lo = q_meas[i] - max_err;
        const double hi = q_meas[i] + max_err;
        (*q_ref)[i] = std::clamp((*q_ref)[i], lo, hi);
    }
}

void CartesianController::hold_meas_reference(const std::vector<JointState>& current_joints,
                                              double gripper_pos, double dt,
                                              std::vector<JointState>* joint_reference) {
    sync_q_ref_to_meas(current_joints);
    const size_t arm_dof = dynamics_->GetDof();
    const size_t total_dof = current_joints.size();
    joint_reference->resize(total_dof);
    for (size_t i = 0; i < arm_dof && i < total_dof; ++i) {
        (*joint_reference)[i].position = q_ref_[i];
        (*joint_reference)[i].velocity = 0.0;
        (*joint_reference)[i].effort = 0.0;
    }
    apply_gripper_reference(arm_dof, total_dof, gripper_pos, dt, joint_reference);
    last_safe_reference_ = *joint_reference;
}

void CartesianController::maybe_log_dls_debug(const DlsDebugSnapshot& snap) {
    if (!config_.dls.debug_log) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const double period = 1.0 / std::max(config_.dls.debug_log_hz, 0.1);
    if (last_dls_debug_time_.time_since_epoch().count() != 0) {
        const double elapsed =
            std::chrono::duration<double>(now - last_dls_debug_time_).count();
        if (elapsed < period) {
            return;
        }
    }
    last_dls_debug_time_ = now;
    ++dls_debug_tick_;

    std::ostringstream oss;
    oss << log_prefix_ << "[dls] #" << dls_debug_tick_
        << " age_ms=" << snap.quest_data_age_ms << " dt=" << snap.controller_dt
        << " ori=" << (holds_orientation() ? "HOLD" : "VR")
        << " rot_off=" << rotation_target_offset_rad()
        << " |e_pos|=" << snap.position_error_norm << " |e_ori|=" << snap.orientation_error_norm
        << " |v|=" << snap.linear_velocity_cmd_norm << " |w|=" << snap.angular_velocity_cmd_norm
        << " vr_da=" << snap.vr_delta_angle_rad << " |vr_w|=" << snap.vr_omega_robot.norm()
        << " vr_axis=[" << snap.vr_delta_axis_robot.transpose() << "]"
        << " sigma_min=" << snap.sigma_min << " lambda=" << snap.lambda
        << " qdot_raw_max=" << snap.qdot_raw_max << " qdot_lim_max=" << snap.qdot_limited_max
        << " dq_ref_max=" << snap.delta_q_ref_max << " track_err_max=" << snap.tracking_error_max
        << " soft_lim=" << (snap.soft_limit_active ? 1 : 0)
        << " quest_to=" << (snap.quest_timeout_active ? 1 : 0) << " p_cur=["
        << snap.p_cur.transpose() << "] p_tgt=[" << snap.p_des.transpose() << "] q_ref=[";
    for (size_t i = 0; i < snap.q_ref.size(); ++i) {
        if (i) {
            oss << ' ';
        }
        oss << snap.q_ref[i];
    }
    oss << ']';
    if (config_.skeleton.enabled) {
        oss << " skel[psi_des=" << last_human_swivel_rad_ << " psi_r=" << last_robot_swivel_rad_
            << " err=" << wrap_to_pi(last_human_swivel_rad_ - last_robot_swivel_rad_) << "]";
    }
    std::cout << oss.str() << std::endl;
    maybe_log_dls_csv(snap);
}

void CartesianController::maybe_log_dls_csv(const DlsDebugSnapshot& snap) {
    if (!config_.dls.csv_log || !dls_csv_.is_open()) {
        return;
    }
    if (!dls_csv_header_written_) {
        dls_csv_ << "tick,quest_data_age_ms,controller_dt,position_error_norm,"
                    "orientation_error_norm,linear_velocity_cmd_norm,angular_velocity_cmd_norm,"
                    "sigma_min,lambda,qdot_raw_max,qdot_limited_max,delta_q_ref_max,"
                    "tracking_error_max,soft_limit_active,quest_timeout_active\n";
        dls_csv_header_written_ = true;
    }
    dls_csv_ << dls_debug_tick_ << ',' << snap.quest_data_age_ms << ',' << snap.controller_dt << ','
             << snap.position_error_norm << ',' << snap.orientation_error_norm << ','
             << snap.linear_velocity_cmd_norm << ',' << snap.angular_velocity_cmd_norm << ','
             << snap.sigma_min << ',' << snap.lambda << ',' << snap.qdot_raw_max << ','
             << snap.qdot_limited_max << ',' << snap.delta_q_ref_max << ','
             << snap.tracking_error_max << ',' << (snap.soft_limit_active ? 1 : 0) << ','
             << (snap.quest_timeout_active ? 1 : 0) << '\n';
}

bool CartesianController::compute_joint_reference_dls(
    const std::vector<JointState>& current_joints, double dt, const Eigen::Vector3d& p_des,
    const Eigen::Quaterniond& q_des, double gripper_pos, double quest_age_ms,
    std::vector<JointState>* joint_reference) {
    const size_t arm_dof = dynamics_->GetDof();
    const size_t total_dof = current_joints.size();
    if (joint_reference == nullptr || dt <= 0.0) {
        return false;
    }

    std::vector<double> q_meas(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof && i < total_dof; ++i) {
        q_meas[i] = current_joints[i].position;
    }

    if (!q_ref_valid_ || q_ref_.size() != arm_dof) {
        sync_q_ref_to_meas(current_joints);
    }

    // Hard Quest timeout: freeze output to measured pose.
    if (quest_timeout_active_) {
        hold_meas_reference(current_joints, gripper_pos, dt, joint_reference);
        state_ = CartesianControlState::Hold;
        DlsDebugSnapshot snap;
        snap.quest_data_age_ms = quest_age_ms;
        snap.controller_dt = dt;
        snap.quest_timeout_active = true;
        snap.q_ref = q_ref_;
        maybe_log_dls_debug(snap);
        return true;
    }

    Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
    dynamics_->GetEECordinate(q_meas.data(), r_curr, p_curr);

    Eigen::Quaterniond q_des_aligned = q_des.normalized();
    if (last_target_quat_valid_) {
        q_des_aligned = align_quaternion_sign(last_target_quat_, q_des_aligned);
    }
    last_target_quat_ = q_des_aligned;
    last_target_quat_valid_ = true;

    // Always compute full 6D error. HOLD_CLUTCH keeps R_des = R_robot_ref so e_ori
    // actively regulates orientation back to the clutch pose (not zeroed).
    const Eigen::Vector3d e_pos = p_des - p_curr;
    const Eigen::Vector3d e_ori =
        rotation_error_vector(Eigen::Quaterniond(r_curr), q_des_aligned);

    Eigen::Vector3d v_lin = config_.dls.kp_pos * e_pos;
    Eigen::Vector3d v_ang = config_.dls.kp_ori * e_ori;
    if (!holds_orientation() &&
        effective_orientation_mode() == OrientationMode::VR_ORIENTATION) {
        // Unified 6-DOF: VR hand twist rate + pose error (not split pos/ori solves).
        // Feedforward gain < 1 tames the Δrot/dt jitter amplification (calmer wrist).
        v_ang += config_.vr_orientation_ff_gain * omega_des_robot_;
    }
    if (effective_orientation_mode() == OrientationMode::FREE_ORIENTATION) {
        v_ang.setZero();
    }
    const double v_norm = v_lin.norm();
    if (v_norm > config_.dls.max_linear_speed_mps && v_norm > 1e-9) {
        v_lin *= config_.dls.max_linear_speed_mps / v_norm;
    }
    const double w_norm = v_ang.norm();
    if (w_norm > config_.dls.max_angular_speed_radps && w_norm > 1e-9) {
        v_ang *= config_.dls.max_angular_speed_radps / w_norm;
    }

    double sigma_min = 0.0;
    Eigen::VectorXd q_dot_raw;
    Eigen::VectorXd twist(6);
    twist.head<3>() = v_lin;
    twist.tail<3>() = v_ang;
    const std::vector<double>* task_w =
        config_.dls.task_joint_weights.empty() ? nullptr : &config_.dls.task_joint_weights;
    if (!dynamics_->SolveDlsIk(q_meas.data(), twist, config_.dls.lambda, &q_dot_raw, &sigma_min,
                               task_w) ||
        !q_dot_raw.allFinite()) {
        ++ik_fail_count_;
        const auto now = std::chrono::steady_clock::now();
        if (last_dls_error_log_time_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_dls_error_log_time_).count() >= 0.2) {
            last_dls_error_log_time_ = now;
            std::cerr << log_prefix_
                      << "[dls] solve failed / non-finite; resetting q_ref to q_meas"
                      << std::endl;
        }
        hold_meas_reference(current_joints, gripper_pos, dt, joint_reference);
        state_ = state_after_ik_failure();
        return state_ != CartesianControlState::Active;
    }

    Eigen::VectorXd q_dot = q_dot_raw;
    if (config_.skeleton.enabled) {
        // Deterministic human-skeleton swivel resolution replaces the fixed-nominal null-space pull.
        q_dot += compute_skeleton_nullspace_velocity(q_meas, sigma_min);
    } else if (config_.dls.use_nullspace) {
        q_dot += compute_nullspace_velocity(q_meas.data());
    }

    // Primary limit: per-joint velocity.
    Eigen::VectorXd q_dot_lim = q_dot;
    for (Eigen::Index i = 0; i < q_dot_lim.size(); ++i) {
        const double vmax = joint_velocity_limit(static_cast<size_t>(i));
        q_dot_lim[i] = std::clamp(q_dot_lim[i], -vmax, vmax);
    }

    bool soft_limit_active = false;
    {
        Eigen::VectorXd before = q_dot_lim;
        apply_soft_joint_limits(q_meas, &q_dot_lim);
        soft_limit_active = (before - q_dot_lim).norm() > 1e-9;
    }

    // Integrate internal reference (not q_meas + q_dot*dt each cycle).
    std::vector<double> q_ref_next = q_ref_;
    double delta_q_ref_max = 0.0;
    for (size_t i = 0; i < arm_dof; ++i) {
        const double vmax = joint_velocity_limit(i);
        const double dq = q_dot_lim[static_cast<Eigen::Index>(i)] * dt;
        // Emergency protection only: allow up to emergency_step_scale * qdot_max * dt.
        const double emerg = config_.dls.emergency_step_scale * vmax * dt;
        const double dq_clamped = std::clamp(dq, -emerg, emerg);
        q_ref_next[i] += dq_clamped;
        delta_q_ref_max = std::max(delta_q_ref_max, std::abs(dq_clamped));
    }
    dynamics_->ClampJointPositions(q_ref_next.data());
    clamp_tracking_error(q_meas, &q_ref_next);

    double tracking_error_max = 0.0;
    for (size_t i = 0; i < arm_dof; ++i) {
        tracking_error_max = std::max(tracking_error_max, std::abs(q_ref_next[i] - q_meas[i]));
    }

    // If tracking error still huge after clamp, resync (e.g. external disturbance).
    if (tracking_error_max > config_.dls.max_tracking_error_rad * 1.01) {
        sync_q_ref_to_meas(current_joints);
        q_ref_next = q_ref_;
        tracking_error_max = 0.0;
    }

    q_ref_ = q_ref_next;
    q_ref_valid_ = true;

    joint_reference->resize(total_dof);
    for (size_t i = 0; i < arm_dof; ++i) {
        (*joint_reference)[i].position = q_ref_[i];
        (*joint_reference)[i].velocity = q_dot_lim[static_cast<Eigen::Index>(i)];
        (*joint_reference)[i].effort = 0.0;
    }
    apply_gripper_reference(arm_dof, total_dof, gripper_pos, dt, joint_reference);

    DlsDebugSnapshot snap;
    snap.quest_data_age_ms = quest_age_ms;
    snap.controller_dt = dt;
    snap.position_error_norm = e_pos.norm();
    snap.orientation_error_norm = e_ori.norm();
    snap.linear_velocity_cmd_norm = v_lin.norm();
    snap.angular_velocity_cmd_norm = v_ang.norm();
    snap.vr_delta_angle_rad = last_vr_delta_angle_rad_;
    snap.vr_omega_robot = omega_des_robot_;
    snap.vr_delta_axis_robot = last_vr_delta_axis_robot_;
    snap.sigma_min = sigma_min;
    snap.lambda = config_.dls.lambda;
    snap.qdot_raw_max = q_dot_raw.cwiseAbs().maxCoeff();
    snap.qdot_limited_max = q_dot_lim.cwiseAbs().maxCoeff();
    snap.delta_q_ref_max = delta_q_ref_max;
    snap.tracking_error_max = tracking_error_max;
    snap.soft_limit_active = soft_limit_active;
    snap.quest_timeout_active = quest_timeout_active_;
    snap.p_cur = p_curr;
    snap.p_des = p_des;
    snap.e_pos = e_pos;
    snap.e_ori = e_ori;
    snap.q_dot_raw = q_dot_raw;
    snap.q_dot_lim = q_dot_lim;
    snap.q_ref = q_ref_;
    maybe_log_dls_debug(snap);

    last_ik_q_ = q_ref_;
    last_solved_cart_pos_ = p_des;
    last_solved_cart_rot_ = q_des_aligned;
    last_safe_reference_ = *joint_reference;
    ik_fail_count_ = 0;
    state_ = CartesianControlState::Active;
    return true;
}

bool CartesianController::compute_joint_reference_position_ik(
    const std::vector<JointState>& current_joints, double dt, const Eigen::Vector3d& p_des,
    const Eigen::Quaterniond& q_des, double gripper_pos, std::vector<JointState>* joint_reference) {
    const size_t arm_dof = dynamics_->GetDof();
    const size_t total_dof = current_joints.size();
    if (joint_reference == nullptr || last_ik_q_.size() != arm_dof) {
        return false;
    }

    std::vector<double> q_curr(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof && i < total_dof; ++i) {
        q_curr[i] = current_joints[i].position;
    }

    if (!config_.direct_ik_control && cartesian_target_unchanged(p_des, q_des)) {
        *joint_reference = last_safe_reference_;
        joint_reference->resize(total_dof);
        apply_gripper_reference(arm_dof, total_dof, gripper_pos, dt, joint_reference);
        last_safe_reference_ = *joint_reference;
        state_ = CartesianControlState::Active;
        return true;
    }

    std::vector<double> q_target(arm_dof, 0.0);
    bool ik_ok = dynamics_->SolvePosIk(q_curr.data(), q_des.normalized().toRotationMatrix(), p_des,
                                      q_target.data());
    if (!ik_ok) {
        ik_ok = dynamics_->SolvePosIk(last_ik_q_.data(), q_des.normalized().toRotationMatrix(),
                                     p_des, q_target.data());
    }
    if (!ik_ok || !ik_solution_usable(q_target, p_des)) {
        ik_ok = dynamics_->SolvePosIkPosition(q_curr.data(), p_des, q_target.data());
    }
    if (!ik_ok) {
        ik_ok = dynamics_->SolvePosIkPosition(last_ik_q_.data(), p_des, q_target.data());
    }
    if (!ik_ok && ik_solution_usable(q_curr, p_des)) {
        q_target = q_curr;
        ik_ok = true;
    }
    if (!ik_ok) {
        Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
        dynamics_->GetEECordinate(q_curr.data(), r_curr, p_curr);
        const double dist_before = (p_curr - p_des).norm();
        Eigen::VectorXd v_cart(6);
        v_cart.setZero();
        v_cart.head<3>() = (p_des - p_curr) / std::max(dt, 1e-3);
        if (!holds_orientation()) {
            const Eigen::Quaterniond q_curr_q(r_curr);
            const Eigen::Quaterniond q_err = q_curr_q.inverse() * q_des.normalized();
            Eigen::AngleAxisd axis_angle(q_err.normalized());
            double angle = axis_angle.angle();
            if (!std::isfinite(angle)) {
                angle = 0.0;
            }
            if (angle > 1e-6) {
                v_cart.tail<3>() = axis_angle.axis() * (angle / std::max(dt, 1e-3));
            }
        }
        Eigen::VectorXd dq_step;
        if (dynamics_->SolveDlsIk(q_curr.data(), v_cart, config_.safety.lambda0, &dq_step)) {
            for (size_t i = 0; i < arm_dof; ++i) {
                q_target[i] = q_curr[i] + dq_step[static_cast<Eigen::Index>(i)] * dt;
            }
            Eigen::Matrix3d r_step = Eigen::Matrix3d::Identity();
            Eigen::Vector3d p_step = Eigen::Vector3d::Zero();
            dynamics_->GetEECordinate(q_target.data(), r_step, p_step);
            const double dist_after = (p_step - p_des).norm();
            ik_ok = dist_after + 1e-4 < dist_before || ik_solution_usable(q_target, p_des);
        }
    }
    if (!ik_ok) {
        ++ik_fail_count_;
        if (ik_fail_count_ <= 3 || ik_fail_count_ % 200 == 0) {
            Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
            Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
            dynamics_->GetEECordinate(q_curr.data(), r_curr, p_curr);
            std::cerr << log_prefix_ << "[cartesian] position IK failed count=" << ik_fail_count_
                      << " dist=" << (p_curr - p_des).norm()
                      << " tcp_curr=[" << p_curr.transpose() << "] target_tcp=["
                      << p_des.transpose() << "]" << std::endl;
        }
        state_ = state_after_ik_failure();
        *joint_reference = last_safe_reference_;
        return state_ != CartesianControlState::Active;
    }

    if (!holds_orientation()) {
        refine_ik_orientation(q_target, q_des, dt, &q_target);
    }

    if (!ik_solution_usable(q_target, p_des)) {
        Eigen::Matrix3d r_chk = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_chk = Eigen::Vector3d::Zero();
        dynamics_->GetEECordinate(q_target.data(), r_chk, p_chk);
        const double pos_err = (p_chk - p_des).norm();
        if (pos_err > config_.safety.max_ik_position_error_m * 2.5) {
            ++ik_fail_count_;
            if (ik_fail_count_ <= 3 || ik_fail_count_ % 200 == 0) {
                std::cerr << log_prefix_ << "[cartesian] IK refine degraded count="
                          << ik_fail_count_ << " pos_err=" << pos_err << std::endl;
            }
            state_ = state_after_ik_failure();
            *joint_reference = last_safe_reference_;
            return state_ != CartesianControlState::Active;
        }
    }

    // Legacy position-IK path: keep fixed max_joint_step for A/B (not DLS path).
    const double max_step =
        config_.direct_ik_control ? config_.safety.max_joint_step_rad * 4.0
                                  : config_.safety.max_joint_step_rad;
    std::vector<double> dq_target(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof; ++i) {
        const double delta = q_target[i] - last_ik_q_[i];
        const double clamped = std::clamp(delta, -max_step, max_step);
        q_target[i] = last_ik_q_[i] + clamped;
        dq_target[i] = clamped / std::max(dt, 1e-6);
    }

    std::vector<double> q_out;
    std::vector<double> dq_out;
    if (config_.direct_ik_control) {
        q_out = q_target;
        dq_out = dq_target;
    } else {
        joint_smoother_.set_target(q_target, dq_target);
        std::vector<double> q_smooth;
        std::vector<double> dq_smooth;
        joint_smoother_.step(dt, &q_smooth, &dq_smooth);

        if (!joint_output_filter_.initialized()) {
            joint_output_filter_.reset(q_smooth);
        }

        joint_output_filter_.step(dt, q_smooth, &q_out, &dq_out);
        if (q_out.size() != arm_dof) {
            q_out = q_smooth;
            dq_out = dq_smooth;
        }
    }

    joint_reference->resize(total_dof);
    for (size_t i = 0; i < arm_dof; ++i) {
        (*joint_reference)[i].position = q_out[i];
        (*joint_reference)[i].velocity = dq_out[i];
        (*joint_reference)[i].effort = 0.0;
    }
    apply_gripper_reference(arm_dof, total_dof, gripper_pos, dt, joint_reference);

    last_ik_q_ = q_out;
    last_solved_cart_pos_ = p_des;
    last_solved_cart_rot_ = q_des.normalized();
    last_safe_reference_ = *joint_reference;
    ik_fail_count_ = 0;
    state_ = runtime_position_only_ ? CartesianControlState::Degraded : CartesianControlState::Active;
    return true;
}

bool CartesianController::step(const std::vector<JointState>& current_joints,
                               const QuestPoseSample& raw_pose, double dt,
                               std::vector<JointState>* joint_reference) {
    if (joint_reference == nullptr || dynamics_ == nullptr) {
        return false;
    }

    if (!last_safe_reference_.empty()) {
        *joint_reference = last_safe_reference_;
    } else {
        *joint_reference = current_joints;
    }

    // Gripper buttons work even when arm teleop is idle / timed out.
    update_gripper_from_buttons(raw_pose, current_joints, dt);

    if (!config_.direct_ik_control) {
        update_filtered_vr_pose(raw_pose, dt);
    }

    const QuestPoseSample teleop_pose =
        config_.direct_ik_control ? raw_pose
                                  : (filtered_vr_pose_.valid ? filtered_vr_pose_ : raw_pose);
    const QuestPoseSample& age_pose = raw_pose.valid ? raw_pose : teleop_pose;
    const double quest_age = quest_data_age_sec(age_pose);
    const double quest_age_ms = std::isfinite(quest_age) ? quest_age * 1000.0 : -1.0;

    // Soft stale: keep last T_target, do not update from VR.
    quest_stale_active_ =
        std::isfinite(quest_age) && quest_age >= config_.safety.quest_stale_sec &&
        quest_age < config_.safety.quest_timeout_sec;
    target_frozen_ = quest_stale_active_;

    // Hard timeout.
    if (!std::isfinite(quest_age) || quest_age >= config_.safety.quest_timeout_sec) {
        if (!quest_timeout_active_) {
            std::cerr << log_prefix_ << "[cartesian] Quest timeout age_ms=" << quest_age_ms
                      << " — freezing q_ref to q_meas; re-grip required on recovery"
                      << std::endl;
        }
        quest_timeout_active_ = true;
        target_frozen_ = true;
        if (clutch_active_) {
            hold_meas_reference(current_joints, gripper_target_, dt, joint_reference);
            state_ = CartesianControlState::Hold;
            update_locked_timer(dt, last_raw_grip_active_);
        } else {
            state_ = CartesianControlState::Idle;
            apply_gripper_to_reference(current_joints, dt, joint_reference);
        }
        return false;
    }

    if (quest_timeout_active_) {
        handle_quest_timeout_recovery();
    }

    const bool grip_signal = teleop_pose.valid ? teleop_pose.grip_active : raw_pose.grip_active;
    const bool grip_now = config_.direct_ik_control ? grip_signal
                                                      : update_debounced_grip(grip_signal, dt);

    const bool grip_rising = grip_now && !last_raw_grip_active_;
    const bool grip_falling_raw = !grip_now && last_raw_grip_active_;
    const bool grip_falling =
        grip_falling_raw &&
        (!clutch_active_ ||
         (config_.direct_ik_control
              ? false
              : clutch_active_elapsed_sec_ >= config_.safety.min_clutch_hold_sec));
    last_raw_grip_active_ = grip_now;

    if (grip_falling && clutch_active_) {
        std::cout << log_prefix_ << "[cartesian] clutch released" << std::endl;
        deactivate_clutch(&current_joints);
    }

    const bool new_vr = ingest_vr_sample(raw_pose);
    (void)new_vr;

    if (require_reclutch_ && !clutch_active_) {
        if (!(grip_now && grip_rising)) {
            state_ = CartesianControlState::Idle;
            apply_gripper_to_reference(current_joints, dt, joint_reference);
            return true;
        }
    }

    // Every grip rising edge re-anchors: current TCP + current VR hand become the new zero.
    if (grip_now && grip_rising) {
        QuestPoseSample clutch_pose = teleop_pose;
        if (config_.vr_orientation_filter_alpha > 0.0 && clutch_pose.valid) {
            filtered_hand_ori_ = clutch_pose.orientation.normalized();
            filtered_hand_ori_valid_ = true;
            clutch_pose.orientation = filtered_hand_ori_;
        }
        const bool re_clutch = clutch_active_;
        activate_clutch(current_joints, clutch_pose);
        just_activated_clutch_ = true;
        clutch_active_elapsed_sec_ = 0.0;
        if (re_clutch) {
            std::cout << log_prefix_ << "[cartesian] clutch re-anchored (grip rising edge)"
                      << std::endl;
        }
    }

    if (clutch_active_) {
        clutch_active_elapsed_sec_ += dt;
    }

    if (!clutch_active_) {
        state_ = CartesianControlState::Idle;
        apply_gripper_to_reference(current_joints, dt, joint_reference);
        return true;
    }

    // Update T_target only from fresh (non-stale) Quest samples; DLS reuses latest target.
    if (grip_now && !just_activated_clutch_) {
        QuestPoseSample vr_pose = teleop_pose;
        if (config_.vr_orientation_filter_alpha > 0.0 && vr_pose.valid) {
            Eigen::Quaterniond q_meas = vr_pose.orientation.normalized();
            if (!filtered_hand_ori_valid_) {
                filtered_hand_ori_ = q_meas;
                filtered_hand_ori_valid_ = true;
            } else {
                filtered_hand_ori_ =
                    filtered_hand_ori_
                        .slerp(config_.vr_orientation_filter_alpha, q_meas)
                        .normalized();
            }
            vr_pose.orientation = filtered_hand_ori_;
        }
        if (!target_frozen_) {
            vr_pose.orientation = vr_pose.orientation.normalized();
            update_cartesian_target_from_vr(vr_pose);
        }
        // Skeleton swivel target is now derived from the commanded TCP orientation inside
        // compute_skeleton_nullspace_velocity() (rigid forearm coupling), so nothing to do here.
        apply_trigger_orientation_lock(current_joints, teleop_pose, grip_now);
        if (!holds_orientation() &&
            effective_orientation_mode() == OrientationMode::VR_ORIENTATION &&
            !last_trigger_orientation_lock_) {
            const size_t arm_dof = dynamics_->GetDof();
            std::vector<double> q_meas(arm_dof, 0.0);
            for (size_t i = 0; i < arm_dof && i < current_joints.size(); ++i) {
                q_meas[i] = current_joints[i].position;
            }
            Eigen::Matrix3d r_tcp = Eigen::Matrix3d::Identity();
            Eigen::Vector3d p_tmp = Eigen::Vector3d::Zero();
            dynamics_->GetEECordinate(q_meas.data(), r_tcp, p_tmp);
            Eigen::Quaterniond q_hand = vr_pose.orientation.normalized();
            q_hand = align_quaternion_sign(hand_ref_rot_, q_hand);
            update_vr_orientation_rate(q_hand, dt, Eigen::Quaterniond(r_tcp));
        } else {
            omega_des_robot_.setZero();
            last_vr_delta_angle_rad_ = 0.0;
            last_vr_delta_axis_robot_.setZero();
        }
    }
    just_activated_clutch_ = false;

    Eigen::Vector3d p_des = p_des_target_;
    Eigen::Quaterniond q_des = q_des_target_;
    if (!config_.direct_ik_control) {
        cartesian_smoother_.step(dt, &p_des, &q_des);
    }
    // Enforce HOLD orientation even if smoother drifts.
    if (holds_orientation()) {
        q_des = robot_ref_rot_;
        q_des_target_ = robot_ref_rot_;
    } else if (last_trigger_orientation_lock_ && trigger_lock_rot_valid_) {
        q_des = trigger_lock_rot_;
        q_des_target_ = trigger_lock_rot_;
    }

    bool ok = false;
    if (config_.dls.enabled) {
        ok = compute_joint_reference_dls(current_joints, dt, p_des, q_des, gripper_target_,
                                         quest_age_ms, joint_reference);
    } else {
        ok = compute_joint_reference_position_ik(current_joints, dt, p_des, q_des, gripper_target_,
                                                 joint_reference);
    }
    update_locked_timer(dt, grip_now);
    return ok;
}
