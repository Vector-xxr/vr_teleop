#include "controller/skeleton_proximal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
Eigen::Quaterniond align_sign(const Eigen::Quaterniond& reference, const Eigen::Quaterniond& q) {
    Eigen::Quaterniond out = q.normalized();
    if (reference.normalized().dot(out) < 0.0) {
        out.coeffs() = -out.coeffs();
    }
    return out;
}

// Swing-twist: keep heading (yaw about world-up), drop controller tilt.
Eigen::Quaterniond twist_about(const Eigen::Quaterniond& q_in, const Eigen::Vector3d& axis) {
    const Eigen::Quaterniond q = q_in.normalized();
    const Eigen::Vector3d a = axis.normalized();
    const Eigen::Vector3d v(q.x(), q.y(), q.z());
    const Eigen::Vector3d proj = v.dot(a) * a;
    Eigen::Quaterniond twist(q.w(), proj.x(), proj.y(), proj.z());
    if (twist.norm() < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    twist.normalize();
    return twist;
}

Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R, const Eigen::Matrix3d& R_des) {
    Eigen::Matrix3d Re = R_des * R.transpose();
    Eigen::AngleAxisd aa(Eigen::Quaterniond(Re).normalized());
    double angle = aa.angle();
    if (!std::isfinite(angle)) {
        return Eigen::Vector3d::Zero();
    }
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    return aa.axis() * angle;
}

}  // namespace

SkeletonProximalController::SkeletonProximalController(Dynamics* dynamics,
                                                       SkeletonProximalConfig config,
                                                       std::string log_prefix)
    : dynamics_(dynamics), config_(std::move(config)), log_prefix_(std::move(log_prefix)) {}

bool SkeletonProximalController::Init() {
    if (dynamics_ == nullptr) {
        return false;
    }
    kin_ = std::make_unique<HumanMimicKinematics>(config_.urdf_path, config_.root_link,
                                                  config_.shoulder_link, config_.elbow_link,
                                                  config_.wrist_link);
    if (!kin_->Init()) {
        std::cerr << log_prefix_ << "[skel-prox] kinematics init failed" << std::endl;
        return false;
    }
    arm_dof_ = kin_->Dof();
    if (arm_dof_ < 7) {
        std::cerr << log_prefix_ << "[skel-prox] expected >=7 arm dof, got " << arm_dof_ << std::endl;
        return false;
    }
    if (!dynamics_->GetJointPositionLimits(&q_min_, &q_max_)) {
        q_min_.clear();
        q_max_.clear();
    }
    q_target_.assign(arm_dof_, 0.0);
    q_nominal_ = config_.q_nominal;
    if (q_nominal_.size() < arm_dof_) {
        q_nominal_.assign(arm_dof_, 0.0);
        if (arm_dof_ > 3) {
            q_nominal_[3] = 1.570796;
        }
    }

    HumanMimicConfig jrg_cfg;
    jrg_cfg.k_joint = config_.k_joint;
    jrg_cfg.qdot_max = config_.qdot_max;
    jrg_cfg.qddot_max = config_.qddot_max;
    jrg_ = std::make_unique<JointReferenceGenerator>(jrg_cfg);
    jrg_->SetJointLimits(q_min_, q_max_);
    jrg_->Reset(q_nominal_);
    q_target_ = q_nominal_;
    gripper_target_ = config_.gripper_open;
    initialized_ = true;
    std::cout << log_prefix_
              << "[skel-prox] ready (proximal elev/azim/bend + distal VR ori)" << std::endl;
    return true;
}

Eigen::Matrix3d SkeletonProximalController::vr_to_robot_axes() const { return config_.vr_axes; }

void SkeletonProximalController::set_skeleton_pose(const SkeletonPoseArm& arm, bool valid,
                                                   double age_ms) {
    pose_arm_ = arm;
    pose_age_ms_ = age_ms;
    pose_valid_ = valid && arm.conf >= config_.pose_min_conf && age_ms <= config_.pose_max_age_ms &&
                  arm.u_upper.norm() > 0.5 && arm.u_fore.norm() > 0.5;
}

void SkeletonProximalController::clamp_q(std::vector<double>* q) const {
    if (q == nullptr || q_min_.empty()) {
        return;
    }
    for (size_t i = 0; i < q->size() && i < q_min_.size() && i < q_max_.size(); ++i) {
        (*q)[i] = std::clamp((*q)[i], q_min_[i], q_max_[i]);
    }
}

void SkeletonProximalController::slew_segment(std::vector<double>* q,
                                              const std::vector<double>& q_goal, size_t i0,
                                              size_t i1, double max_radps, double dt) const {
    if (q == nullptr) {
        return;
    }
    const double step = std::max(0.0, max_radps) * std::max(1e-4, dt);
    for (size_t i = i0; i < i1 && i < q->size() && i < q_goal.size(); ++i) {
        const double dq = std::clamp(q_goal[i] - (*q)[i], -step, step);
        (*q)[i] += dq;
    }
}

bool SkeletonProximalController::tcp_pose(const std::vector<double>& q, Eigen::Vector3d* p,
                                          Eigen::Matrix3d* R) const {
    Eigen::Vector3d p_w;
    Eigen::Matrix3d R_h;
    Eigen::MatrixXd Jp, Jw;
    if (!kin_->WristState(q, &p_w, &R_h, &Jp, &Jw)) {
        return false;
    }
    if (p != nullptr) {
        *p = p_w + R_h * config_.wrist_to_tcp;
    }
    if (R != nullptr) {
        *R = R_h;
    }
    return true;
}

Eigen::Quaterniond SkeletonProximalController::filter_orientation(const Eigen::Quaterniond& raw,
                                                                  double dt) {
    Eigen::Quaterniond q = raw.normalized();
    if (!ori_filt_valid_) {
        q_filt_ = q;
        ori_filt_valid_ = true;
        return q_filt_;
    }
    if (q_filt_.coeffs().dot(q.coeffs()) < 0.0) {
        q.coeffs() = -q.coeffs();
    }
    const double alpha = std::clamp(8.0 * dt, 0.0, 1.0);
    q_filt_ = q_filt_.slerp(alpha, q).normalized();
    return q_filt_;
}

Eigen::Matrix3d SkeletonProximalController::compute_hand_target(const QuestPoseSample& sample,
                                                                double dt) {
    // Same gravity-locked orientation map as HumanMimicController (incremental clutch anchors).
    const Eigen::Quaterniond q_hand = filter_orientation(sample.orientation, dt);
    const Eigen::Quaterniond& hand_anchor = hand_ref_rot_;
    const Eigen::Quaterniond& robot_anchor = robot_ref_rot_;

    const Eigen::Vector3d world_up = config_.vr_world_up.normalized();
    const Eigen::Quaterniond q_hand_aligned = align_sign(hand_anchor, q_hand);
    Eigen::Quaterniond delta_world = (q_hand_aligned * hand_anchor.inverse()).normalized();
    if (delta_world.w() < 0.0) {
        delta_world.coeffs() = -delta_world.coeffs();
    }
    if (config_.rot_scale != 1.0) {
        Eigen::AngleAxisd aa(delta_world.normalized());
        double angle = aa.angle();
        if (!std::isfinite(angle)) {
            angle = 0.0;
        }
        delta_world = Eigen::Quaterniond(Eigen::AngleAxisd(angle * config_.rot_scale, aa.axis()));
    }

    const Eigen::Quaterniond heading = twist_about(hand_anchor, world_up);
    const Eigen::Matrix3d W = vr_to_robot_axes() * heading.inverse().toRotationMatrix();
    const Eigen::Matrix3d dR = W * delta_world.toRotationMatrix() * W.transpose();
    const Eigen::Quaterniond delta_robot = Eigen::Quaterniond(dR).normalized();
    return (delta_robot * robot_anchor).normalized().toRotationMatrix();
}

Eigen::Matrix3d SkeletonProximalController::gravity_basis(const Eigen::Vector3d& up_in) const {
    Eigen::Vector3d up = up_in;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    Eigen::Vector3d ref = Eigen::Vector3d::UnitX();
    if (std::abs(up.dot(ref)) > 0.9) {
        ref = Eigen::Vector3d::UnitY();
    }
    Eigen::Vector3d ex = ref - ref.dot(up) * up;
    if (ex.norm() < 1e-9) {
        ex = Eigen::Vector3d::UnitY();
    }
    ex.normalize();
    const Eigen::Vector3d ey = up.cross(ex).normalized();
    Eigen::Matrix3d B;
    B.col(0) = ex;
    B.col(1) = ey;
    B.col(2) = up;
    return B;
}

double SkeletonProximalController::wrap_pi(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

void SkeletonProximalController::dir_to_elev_azim(const Eigen::Vector3d& u_in,
                                                  const Eigen::Matrix3d& B, double* elev,
                                                  double* azim) const {
    Eigen::Vector3d u = u_in;
    if (u.norm() < 1e-9) {
        u = Eigen::Vector3d::UnitX();
    } else {
        u.normalize();
    }
    const Eigen::Vector3d ub = B.transpose() * u;  // (x,y,up)
    const double e = std::asin(std::clamp(ub.z(), -1.0, 1.0));
    const double horiz = std::hypot(ub.x(), ub.y());
    const double a = (horiz < 1e-6) ? 0.0 : std::atan2(ub.y(), ub.x());
    if (elev != nullptr) {
        *elev = e;
    }
    if (azim != nullptr) {
        *azim = a;
    }
}

Eigen::Vector3d SkeletonProximalController::elev_azim_to_dir(double elev, double azim,
                                                            const Eigen::Matrix3d& B) const {
    const double ce = std::cos(elev);
    const double se = std::sin(elev);
    const Eigen::Vector3d ub(ce * std::cos(azim), ce * std::sin(azim), se);
    return (B * ub).normalized();
}

double SkeletonProximalController::elbow_interior_angle(const Eigen::Vector3d& u_upper,
                                                        const Eigen::Vector3d& u_fore) const {
    Eigen::Vector3d u = u_upper;
    Eigen::Vector3d f = u_fore;
    if (u.norm() < 1e-9 || f.norm() < 1e-9) {
        return M_PI_2;
    }
    u.normalize();
    f.normalize();
    const double c = std::clamp((-u).dot(f), -1.0, 1.0);
    return std::acos(c);
}

Eigen::Vector3d SkeletonProximalController::forearm_from_bend_droop(
    const Eigen::Vector3d& u_upper, double bend_rad) const {
    Eigen::Vector3d u = u_upper;
    if (u.norm() < 1e-9) {
        u = Eigen::Vector3d::UnitX();
    } else {
        u.normalize();
    }
    const Eigen::Vector3d down = (-config_.robot_up).normalized();
    // Prefer elbow droop: bend plane contains gravity (classic virtual-skeleton).
    Eigen::Vector3d toward = down - down.dot(u) * u;
    if (toward.norm() < 1e-4) {
        Eigen::Vector3d ref = (std::abs(u.z()) < 0.9) ? Eigen::Vector3d::UnitZ()
                                                      : Eigen::Vector3d::UnitX();
        toward = u.cross(ref);
    }
    toward.normalize();
    // If droop points "wrong way" relative to -u cone, keep geometric outward.
    const double b = std::clamp(bend_rad, 1e-3, M_PI - 1e-3);
    const Eigen::Vector3d neg_u = -u;
    Eigen::Vector3d f = std::cos(b) * neg_u + std::sin(b) * toward;
    f.normalize();
    return f;
}

bool SkeletonProximalController::map_proximal_dirs(Eigen::Vector3d* u_robot,
                                                   Eigen::Vector3d* f_robot) const {
    if (u_robot == nullptr || f_robot == nullptr || !prox_calib_valid_ || !pose_valid_) {
        return false;
    }
    Eigen::Vector3d u_h = pose_arm_.u_upper;
    Eigen::Vector3d f_h = pose_arm_.u_fore;
    if (u_h.norm() < 1e-6) {
        return false;
    }
    u_h.normalize();
    if (f_h.norm() > 1e-6) {
        f_h.normalize();
    }

    const Eigen::Matrix3d Bh = gravity_basis((-config_.skel_down).normalized());
    const Eigen::Matrix3d Br = gravity_basis(config_.robot_up.normalized());

    double elev_h = 0.0;
    double azim_h = 0.0;
    dir_to_elev_azim(u_h, Bh, &elev_h, &azim_h);

    const double elev_r =
        std::clamp(elev_r0_ + config_.prox_elev_gain * (elev_h - elev_h0_),
                   config_.prox_elev_min_rad, config_.prox_elev_max_rad);
    const double azim_r =
        azim_r0_ + config_.prox_azim_gain * wrap_pi(azim_h - azim_h0_);

    Eigen::Vector3d u = elev_azim_to_dir(elev_r, azim_r, Br);

    double bend_des = bend_r0_;
    if (f_h.norm() > 1e-6) {
        const double bend_h = elbow_interior_angle(u_h, f_h);
        bend_des = bend_r0_ + config_.prox_bend_gain * (bend_h - bend_h0_);
    }
    bend_des = std::clamp(bend_des, config_.prox_bend_min_rad, config_.prox_bend_max_rad);
    Eigen::Vector3d f = forearm_from_bend_droop(u, bend_des);

    *u_robot = u;
    *f_robot = f;
    return true;
}

bool SkeletonProximalController::calibrate_human_to_robot(const std::vector<double>& q_meas) {
    Eigen::Vector3d S, E;
    Eigen::MatrixXd JpE;
    if (!kin_->ShoulderPos(q_meas, &S) || !kin_->ElbowPosJac(q_meas, &E, &JpE)) {
        return false;
    }
    Eigen::Vector3d p_w;
    Eigen::Matrix3d R_h;
    Eigen::MatrixXd Jp, Jw;
    if (!kin_->WristState(q_meas, &p_w, &R_h, &Jp, &Jw)) {
        return false;
    }
    L1_ = (E - S).norm();
    L2_ = (p_w - E).norm();
    if (L1_ < 1e-3) {
        L1_ = 0.22;
    }
    if (L2_ < 1e-3) {
        L2_ = 0.22;
    }

    Eigen::Vector3d u_r = (E - S);
    Eigen::Vector3d f_r = (p_w - E);
    if (u_r.norm() < 1e-6 || f_r.norm() < 1e-6) {
        prox_calib_valid_ = false;
        return false;
    }
    u_r.normalize();
    f_r.normalize();
    bend_r0_ = elbow_interior_angle(u_r, f_r);

    const Eigen::Matrix3d Br = gravity_basis(config_.robot_up.normalized());
    dir_to_elev_azim(u_r, Br, &elev_r0_, &azim_r0_);

    if (!pose_valid_) {
        prox_calib_valid_ = false;
        return true;
    }
    Eigen::Vector3d u_h = pose_arm_.u_upper;
    Eigen::Vector3d f_h = pose_arm_.u_fore;
    if (u_h.norm() < 1e-6) {
        prox_calib_valid_ = false;
        return false;
    }
    u_h.normalize();
    const Eigen::Matrix3d Bh = gravity_basis((-config_.skel_down).normalized());
    dir_to_elev_azim(u_h, Bh, &elev_h0_, &azim_h0_);
    if (f_h.norm() > 1e-6) {
        f_h.normalize();
        bend_h0_ = elbow_interior_angle(u_h, f_h);
    } else {
        bend_h0_ = bend_r0_;
    }
    prox_calib_valid_ = true;
    std::cout << log_prefix_ << "[skel-prox] clutch calibrated elev/azim/bend "
              << "(L1=" << L1_ << " elev_h0=" << elev_h0_ << " azim_h0=" << azim_h0_
              << " elev_r0=" << elev_r0_ << " azim_r0=" << azim_r0_
              << " bend_h0=" << bend_h0_ << " bend_r0=" << bend_r0_
              << " conf=" << pose_arm_.conf << ")" << std::endl;
    return true;
}

bool SkeletonProximalController::solve_proximal(const Eigen::Vector3d& S, double dt) {
    Eigen::Vector3d u;
    Eigen::Vector3d f;
    if (!map_proximal_dirs(&u, &f)) {
        return false;
    }
    {
        const Eigen::Matrix3d Br = gravity_basis(config_.robot_up.normalized());
        dir_to_elev_azim(u, Br, &dbg_elev_des_, &dbg_azim_des_);
        dbg_bend_des_ = elbow_interior_angle(u, f);
    }
    const Eigen::Vector3d E_des = S + L1_ * u;

    std::vector<double> q = q_target_;
    const double sw_e = std::sqrt(std::max(0.0, config_.w_prox_elbow));
    const double sw_f = std::sqrt(std::max(0.0, config_.w_prox_fore));
    const double sw_c = std::sqrt(std::max(0.0, config_.w_cont_prox));
    constexpr int kProx = 4;
    const int m = 3 + 3 + kProx;
    int iters = 0;
    for (int it = 0; it < config_.prox_ik_iters; ++it) {
        iters = it + 1;
        Eigen::Vector3d E;
        Eigen::MatrixXd JpE;
        Eigen::Vector3d p_w;
        Eigen::Matrix3d R_h;
        Eigen::MatrixXd JpW, Jw;
        if (!kin_->ElbowPosJac(q, &E, &JpE) || !kin_->WristState(q, &p_w, &R_h, &JpW, &Jw)) {
            return false;
        }
        Eigen::Vector3d f_cur = p_w - E;
        if (f_cur.norm() < 1e-6) {
            break;
        }
        f_cur.normalize();

        Eigen::VectorXd r(m);
        Eigen::MatrixXd J(m, kProx);
        J.setZero();
        r.segment<3>(0) = sw_e * (E - E_des);
        J.block(0, 0, 3, kProx) = sw_e * JpE.leftCols(kProx);

        const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - f * f.transpose();
        r.segment<3>(3) = sw_f * P * (f_cur - f);
        const Eigen::MatrixXd Jf = (JpW - JpE) / std::max(1e-3, L2_);
        J.block(3, 0, 3, kProx) = sw_f * P * Jf.leftCols(kProx);

        for (int i = 0; i < kProx; ++i) {
            r(6 + i) = sw_c * (q[i] - q_target_[i]);
            J(6 + i, i) = sw_c;
        }

        Eigen::MatrixXd H = J.transpose() * J;
        H.diagonal().array() += config_.ik_lambda * config_.ik_lambda;
        Eigen::VectorXd dq = H.ldlt().solve(-J.transpose() * r);
        if (!dq.allFinite()) {
            break;
        }
        for (int i = 0; i < kProx; ++i) {
            dq(i) = std::clamp(dq(i), -config_.ik_max_step, config_.ik_max_step);
            q[i] += dq(i);
        }
        clamp_q(&q);
        if (dq.norm() < config_.ik_tol) {
            break;
        }
    }
    dbg_prox_iters_ = iters;
    Eigen::Vector3d E2;
    Eigen::MatrixXd JpE2;
    if (kin_->ElbowPosJac(q, &E2, &JpE2)) {
        dbg_e_elbow_ = (E2 - E_des).norm();
    }
    slew_segment(&q_target_, q, 0, 4, config_.q_slew_prox_radps, dt);
    clamp_q(&q_target_);
    return true;
}

bool SkeletonProximalController::solve_distal(const Eigen::Matrix3d& R_hand_des, double dt) {
    // Orientation-only on q4–q6. Proximal q0–q3 already fixed by SEW; wrist xyz is
    // not a task (follows from elbow + forearm geometry once proximal is right).
    std::vector<double> q = q_target_;
    const double sw_o = std::sqrt(std::max(0.0, config_.w_hand_ori));
    const double sw_c = std::sqrt(std::max(0.0, config_.w_cont_dist));
    constexpr int kDist = 3;
    const int i0 = 4;
    const int m = 3 + kDist;
    int iters = 0;
    for (int it = 0; it < config_.dist_ik_iters; ++it) {
        iters = it + 1;
        Eigen::Vector3d p_w;
        Eigen::Matrix3d R_h;
        Eigen::MatrixXd Jp, Jw;
        if (!kin_->WristState(q, &p_w, &R_h, &Jp, &Jw)) {
            return false;
        }
        const Eigen::Vector3d e_o = orientation_error(R_h, R_hand_des);
        Eigen::VectorXd r(m);
        Eigen::MatrixXd J(m, kDist);
        J.setZero();
        r.segment<3>(0) = sw_o * e_o;
        J.block(0, 0, 3, kDist) = -sw_o * Jw.block(0, i0, 3, kDist);
        for (int i = 0; i < kDist; ++i) {
            r(3 + i) = sw_c * (q[i0 + i] - q_target_[i0 + i]);
            J(3 + i, i) = sw_c;
        }
        Eigen::MatrixXd H = J.transpose() * J;
        H.diagonal().array() += config_.ik_lambda * config_.ik_lambda;
        Eigen::VectorXd dq = H.ldlt().solve(-J.transpose() * r);
        if (!dq.allFinite()) {
            break;
        }
        for (int i = 0; i < kDist; ++i) {
            dq(i) = std::clamp(dq(i), -config_.ik_max_step, config_.ik_max_step);
            q[i0 + i] += dq(i);
        }
        clamp_q(&q);
        if (dq.norm() < config_.ik_tol) {
            break;
        }
    }
    dbg_dist_iters_ = iters;
    {
        Eigen::Vector3d p_w;
        Eigen::Matrix3d R_h;
        Eigen::MatrixXd Jp, Jw;
        if (kin_->WristState(q, &p_w, &R_h, &Jp, &Jw)) {
            dbg_e_ori_ = orientation_error(R_h, R_hand_des).norm();
        }
    }
    slew_segment(&q_target_, q, 4, 7, config_.q_slew_dist_radps, dt);
    clamp_q(&q_target_);
    return true;
}

void SkeletonProximalController::activate_clutch(const std::vector<double>& q_meas,
                                                 const QuestPoseSample& pose) {
    q_target_ = q_meas;
    clamp_q(&q_target_);
    jrg_->Reset(q_target_);
    hand_ref_rot_ = pose.orientation.normalized();
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    if (tcp_pose(q_target_, &p, &R)) {
        robot_ref_rot_ = Eigen::Quaterniond(R).normalized();
    }
    ori_filt_valid_ = false;
    filter_orientation(pose.orientation, 0.01);
    calibrate_human_to_robot(q_target_);
    clutch_active_ = true;
}

void SkeletonProximalController::deactivate_clutch() {
    clutch_active_ = false;
    prox_calib_valid_ = false;
}

void SkeletonProximalController::update_gripper_from_buttons(
    const QuestPoseSample& sample, const std::vector<JointState>& current_joints, double dt) {
    if (current_joints.size() > arm_dof_) {
        gripper_meas_ = current_joints[arm_dof_].position;
    }
    last_button_close_ = sample.button_close;
    last_button_open_ = sample.button_open;

    const double lo = std::min(config_.gripper_open, config_.gripper_closed);
    const double hi = std::max(config_.gripper_open, config_.gripper_closed);
    // Hold-to-move: A/X held → close, B/Y held → open, release → HOLD in place.
    const bool want_close = sample.button_close && !sample.button_open;
    const bool want_open = sample.button_open && !sample.button_close;
    if (want_close) {
        gripper_motion_ = GripperMotion::CLOSING;
    } else if (want_open) {
        gripper_motion_ = GripperMotion::OPENING;
    } else {
        gripper_motion_ = GripperMotion::HOLD;
    }

    const double step = std::max(0.0, config_.gripper_speed) * std::max(0.0, dt);
    if (gripper_motion_ == GripperMotion::CLOSING) {
        const double dir = (config_.gripper_closed >= config_.gripper_open) ? 1.0 : -1.0;
        gripper_target_ += dir * step;
        if ((dir > 0 && gripper_target_ >= config_.gripper_closed) ||
            (dir < 0 && gripper_target_ <= config_.gripper_closed)) {
            gripper_target_ = config_.gripper_closed;
            gripper_motion_ = GripperMotion::HOLD;
        }
    } else if (gripper_motion_ == GripperMotion::OPENING) {
        const double dir = (config_.gripper_open >= config_.gripper_closed) ? 1.0 : -1.0;
        gripper_target_ += dir * step;
        if ((dir > 0 && gripper_target_ >= config_.gripper_open) ||
            (dir < 0 && gripper_target_ <= config_.gripper_open)) {
            gripper_target_ = config_.gripper_open;
            gripper_motion_ = GripperMotion::HOLD;
        }
    }
    gripper_target_ = std::clamp(gripper_target_, lo, hi);
}

void SkeletonProximalController::ramp_arm_reset(double dt) {
    const double step = 1.2 * std::max(1e-4, dt);  // ~70 deg/s
    for (size_t i = 0; i < arm_dof_; ++i) {
        const double target = (i < q_nominal_.size()) ? q_nominal_[i] : 0.0;
        if (q_target_[i] > target) {
            q_target_[i] = std::max(target, q_target_[i] - step);
        } else {
            q_target_[i] = std::min(target, q_target_[i] + step);
        }
    }
}

void SkeletonProximalController::fill_reference(const std::vector<JointState>& current_joints,
                                                const std::vector<double>& q_arm,
                                                std::vector<JointState>* reference) const {
    if (reference == nullptr) {
        return;
    }
    *reference = current_joints;
    for (size_t i = 0; i < arm_dof_ && i < reference->size(); ++i) {
        (*reference)[i].position = q_arm[i];
        (*reference)[i].velocity = 0.0;
        (*reference)[i].effort = 0.0;
    }
    if (reference->size() > arm_dof_) {
        (*reference)[arm_dof_].position = gripper_target_;
        (*reference)[arm_dof_].velocity = 0.0;
        (*reference)[arm_dof_].effort = 0.0;
    }
}

void SkeletonProximalController::maybe_log(const char* tag) {
    if (!config_.debug_log) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const double period = 1.0 / std::max(config_.debug_log_hz, 0.1);
    if (last_log_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - last_log_time_).count() < period) {
        return;
    }
    last_log_time_ = now;
    ++log_tick_;
    std::cout << log_prefix_ << "[skel-prox][" << tag << "] #" << log_tick_
              << " pose=" << (pose_valid_ ? 1 : 0) << " conf=" << pose_arm_.conf
              << " e_elb=" << dbg_e_elbow_ << " e_ori=" << dbg_e_ori_
              << " elev=" << dbg_elev_des_ << " azim=" << dbg_azim_des_
              << " bend=" << dbg_bend_des_ << " it_p=" << dbg_prox_iters_
              << " it_d=" << dbg_dist_iters_;
    if (q_target_.size() >= 7) {
        std::cout << " q0=" << q_target_[0] << " q1=" << q_target_[1] << " q3=" << q_target_[3]
                  << " q4=" << q_target_[4] << " q5=" << q_target_[5] << " q6=" << q_target_[6];
    }
    std::cout << std::endl;
}

bool SkeletonProximalController::step(const std::vector<JointState>& current_joints,
                                      const QuestPoseSample& sample, double dt,
                                      std::vector<JointState>* joint_reference) {
    if (!initialized_ || current_joints.size() < arm_dof_) {
        return false;
    }
    std::vector<double> q_meas(arm_dof_, 0.0);
    for (size_t i = 0; i < arm_dof_; ++i) {
        q_meas[i] = current_joints[i].position;
    }
    update_gripper_from_buttons(sample, current_joints, dt);

    const auto now = std::chrono::steady_clock::now();
    const int64_t now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    double age_sec = std::numeric_limits<double>::infinity();
    if (sample.valid && sample.receive_timestamp_ns > 0) {
        age_sec = static_cast<double>(now_ns - sample.receive_timestamp_ns) * 1e-9;
    }
    quest_timeout_active_ = !sample.valid || age_sec > config_.quest_timeout_sec;

    if (sample.trigger > 0.5 && !last_trigger_) {
        // rising edge reserved; reset while held below
    }
    const bool trigger_held = sample.trigger > 0.5;
    last_trigger_ = trigger_held;

    if (trigger_held) {
        if (clutch_active_) {
            deactivate_clutch();
        }
        ramp_arm_reset(dt);
        const auto& q_ref = jrg_->Step(q_target_, dt);
        fill_reference(current_joints, q_ref, joint_reference);
        maybe_log("reset");
        last_grip_ = sample.grip_active;
        return true;
    }

    if (quest_timeout_active_) {
        if (clutch_active_) {
            deactivate_clutch();
        }
        const auto& q_ref = jrg_->Step(q_target_, dt);
        fill_reference(current_joints, q_ref, joint_reference);
        maybe_log("timeout");
        last_grip_ = sample.grip_active;
        return true;
    }

    if (sample.grip_active && !last_grip_) {
        activate_clutch(q_meas, sample);
    } else if (!sample.grip_active && last_grip_) {
        deactivate_clutch();
    }
    last_grip_ = sample.grip_active;

    if (!clutch_active_) {
        const auto& q_ref = jrg_->Step(q_target_, dt);
        fill_reference(current_joints, q_ref, joint_reference);
        maybe_log("idle");
        return true;
    }

    // Active teleop: proximal from SEW; distal orientation-only (no wrist xyz task).
    Eigen::Vector3d S = Eigen::Vector3d::Zero();
    if (!kin_->ShoulderPos(q_meas, &S)) {
        S.setZero();
    }
    if (pose_valid_) {
        (void)solve_proximal(S, dt);
    }
    // else hold proximal from last q_target_

    const Eigen::Matrix3d R_hand_des = compute_hand_target(sample, dt);
    (void)solve_distal(R_hand_des, dt);

    const auto& q_ref = jrg_->Step(q_target_, dt);
    fill_reference(current_joints, q_ref, joint_reference);
    maybe_log("act");
    return true;
}
