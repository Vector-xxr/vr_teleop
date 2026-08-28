#include "controller/human_mimic_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;

Eigen::Quaterniond align_sign(const Eigen::Quaterniond& reference, const Eigen::Quaterniond& q) {
    Eigen::Quaterniond out = q.normalized();
    if (reference.normalized().dot(out) < 0.0) {
        out.coeffs() = -out.coeffs();
    }
    return out;
}

bool all_finite(const std::vector<double>& q) {
    for (double v : q) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return true;
}

double wrap_to_pi(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

// Elbow interior angle [0,π] implied by reach distance (law of cosines). Unique for
// fixed L1,L2,d — obtuse when the arm is open / extending.
double elbow_flex_from_reach(double d, double L1, double L2) {
    if (L1 < 1e-6 || L2 < 1e-6) {
        return M_PI;
    }
    const double c = std::clamp((L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2), -1.0, 1.0);
    return std::acos(c);
}

// Swing-twist decomposition: return the component of q that rotates about `axis`
// (the "twist"), discarding the perpendicular tilt (the "swing"). Used to keep
// only the controller's HEADING (yaw about world-up) while dropping how it is
// tilted in the hand, so a world-vertical yaw stays gravity-locked.
Eigen::Quaterniond twist_about(const Eigen::Quaterniond& q_in, const Eigen::Vector3d& axis) {
    const Eigen::Quaterniond q = q_in.normalized();
    const Eigen::Vector3d a = axis.normalized();
    const Eigen::Vector3d v(q.x(), q.y(), q.z());
    const Eigen::Vector3d proj = v.dot(a) * a;  // vector part projected onto axis
    Eigen::Quaterniond twist(q.w(), proj.x(), proj.y(), proj.z());
    if (twist.norm() < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    twist.normalize();
    return twist;
}

// Among {phi, phi+π}, pick the branch consistent with continuity + anatomy + reach.
// Vision depth flips land on the antipode; a reaching human never suddenly folds
// to the mirrored “acute” silhouette — convert back to the open/prior branch.
double disambiguate_elbow_phi(double phi_obs, bool has_anchor, double phi_anchor, double stretch,
                              bool extending, double flex_rad, VirtualArmSkeleton* skeleton,
                              const Eigen::Vector3d& shoulder, const Eigen::Vector3d& wrist,
                              const HumanMimicConfig& cfg, int* dbg_flipped) {
    if (dbg_flipped) {
        *dbg_flipped = 0;
    }
    if (!cfg.elbow_phi_disambiguate || skeleton == nullptr) {
        return wrap_to_pi(phi_obs);
    }

    const double candidates[2] = {wrap_to_pi(phi_obs), wrap_to_pi(phi_obs + M_PI)};
    const Eigen::Vector3d out_u = cfg.outward_dir.normalized();
    const Eigen::Vector3d down_u = cfg.downward_dir.normalized();
    const Eigen::Vector3d prefer_raw = cfg.w_outward * out_u + cfg.w_downward * down_u;

    const Eigen::Vector3d sw0 = wrist - shoulder;
    const double dn0 = sw0.norm();
    const double elev = (dn0 > 1e-6) ? std::clamp(sw0.z() / dn0, 0.0, 1.0) : 0.0;
    // When elevating, do NOT score toward gravity droop — that is what keeps the
    // arms pinned low with facing grippers / raised hands.
    const double droop_w = 1.0 - elev * std::max(0.0, cfg.elevation_droop_fade);

    const bool reach_lock = extending || stretch >= cfg.elbow_phi_reach_lock_stretch ||
                            flex_rad >= cfg.elbow_phi_obtuse_flex_rad;

    double best = candidates[0];
    double best_score = -1e100;
    for (double cand : candidates) {
        double score = 0.0;

        if (has_anchor) {
            const double d = std::abs(wrap_to_pi(cand - phi_anchor));
            score += 2.0 * std::cos(d);  // prefer continuous branch
            if (reach_lock && d > 0.5 * M_PI) {
                score -= 20.0;
            } else if (reach_lock) {
                score += 3.0 * (1.0 - d / M_PI);
            }
        }

        const ElbowTarget et = skeleton->compute_elbow_target_from_phi(
            shoulder, wrist, skeleton->L1(), skeleton->L2(), cand);
        if (!et.degenerate && et.bend_dir.norm() > 1e-6 && dn0 > 1e-6) {
            const Eigen::Vector3d axis = sw0 / dn0;
            Eigen::Vector3d pref = prefer_raw - axis * axis.dot(prefer_raw);
            if (pref.norm() > 1e-6) {
                score += 1.5 * et.bend_dir.dot(pref.normalized());
            }
            Eigen::Vector3d down_p = down_u - axis * axis.dot(down_u);
            if (droop_w > 1e-3 && down_p.norm() > 1e-6) {
                score += droop_w * et.bend_dir.dot(down_p.normalized());
            }
        }

        if (score > best_score) {
            best_score = score;
            best = cand;
        }
    }

    if (dbg_flipped && std::abs(wrap_to_pi(best - phi_obs)) > 0.5 * M_PI) {
        *dbg_flipped = 1;
    }
    return best;
}
}  // namespace

HumanMimicController::HumanMimicController(Dynamics* dynamics, HumanMimicConfig config,
                                          std::string log_prefix)
    : dynamics_(dynamics), config_(std::move(config)), log_prefix_(std::move(log_prefix)) {}

bool HumanMimicController::Init() {
    if (dynamics_ == nullptr) {
        std::cerr << log_prefix_ << "[human-mimic] null dynamics" << std::endl;
        return false;
    }
    arm_dof_ = dynamics_->GetDof();

    kin_ = std::make_unique<HumanMimicKinematics>(config_.urdf_path, config_.root_link,
                                                  config_.shoulder_link, config_.elbow_link,
                                                  config_.wrist_link);
    if (!kin_->Init()) {
        return false;
    }
    if (kin_->Dof() != arm_dof_) {
        std::cerr << log_prefix_ << "[human-mimic] wrist chain dof (" << kin_->Dof()
                  << ") != dynamics dof (" << arm_dof_ << ")" << std::endl;
        return false;
    }

    if (!dynamics_->GetJointPositionLimits(&q_min_, &q_max_)) {
        std::cout << log_prefix_ << "[human-mimic] joint limits unavailable; IK unconstrained"
                  << std::endl;
        q_min_.clear();
        q_max_.clear();
    }
    // Optional per-joint IK limit overrides (finite entries replace the URDF limit).
    for (size_t i = 0; i < q_min_.size() && i < config_.q_min_override.size(); ++i) {
        if (std::isfinite(config_.q_min_override[i])) {
            std::cout << log_prefix_ << "[human-mimic] override q_min[" << i << "] " << q_min_[i]
                      << " -> " << config_.q_min_override[i] << std::endl;
            q_min_[i] = config_.q_min_override[i];
        }
    }
    for (size_t i = 0; i < q_max_.size() && i < config_.q_max_override.size(); ++i) {
        if (std::isfinite(config_.q_max_override[i])) {
            std::cout << log_prefix_ << "[human-mimic] override q_max[" << i << "] " << q_max_[i]
                      << " -> " << config_.q_max_override[i] << std::endl;
            q_max_[i] = config_.q_max_override[i];
        }
    }

    q_nominal_ = config_.q_nominal;

    skeleton_ = std::make_unique<VirtualArmSkeleton>(config_);
    ik_ = std::make_unique<HumanMimicIkSolver>(kin_.get(), config_);
    if (!q_min_.empty()) {
        ik_->SetJointLimits(q_min_, q_max_);
    }
    jrg_ = std::make_unique<JointReferenceGenerator>(config_);
    if (!q_min_.empty()) {
        jrg_->SetJointLimits(q_min_, q_max_);
    }

    initialized_ = true;
    const char* ik_backend =
        config_.ik_backend == HumanMimicIkBackend::DIFF_QP ? "diff_qp" : "lm";
    std::cout << log_prefix_ << "[human-mimic] ready (arm_dof=" << arm_dof_ << ", ori_map="
              << (config_.orientation_mapping_mode == OrientationMappingMode::ABSOLUTE_CALIBRATED
                      ? "absolute_calibrated"
                      : "incremental_clutch")
              << ", ik=" << ik_backend << ", elbow=xyz+virtual_phi)" << std::endl;
    return true;
}

Eigen::Matrix3d HumanMimicController::vr_to_robot_axes() const {
    Eigen::Matrix3d M;
    M << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    if (config_.mirror_translation_y) {
        M.row(1) *= -1.0;
    }
    return M;
}

Eigen::Vector3d HumanMimicController::map_vr_delta_pos(const Eigen::Vector3d& delta_vr) const {
    return (vr_to_robot_axes() * delta_vr) * config_.pos_scale;
}

Eigen::Quaterniond HumanMimicController::map_vr_delta_rot(const Eigen::Quaterniond& delta_vr) const {
    // Rotate the rotation AXIS from the VR base frame into the robot base frame with the
    // same proper rotation used for translation (det(M)=+1, VR up -> robot up). No
    // per-axis sign patches: those reflect the rotation vector (improper) and are what
    // scrambled yaw into twist. If a whole axis ends up inverted, fix it inside M.
    const Eigen::Matrix3d M = vr_to_robot_axes();
    const Eigen::Matrix3d dR = M * delta_vr.normalized().toRotationMatrix() * M.transpose();
    return Eigen::Quaterniond(dR).normalized();
}

bool HumanMimicController::tcp_pose(const std::vector<double>& q, Eigen::Vector3d* p,
                                    Eigen::Matrix3d* R) const {
    if (dynamics_ == nullptr || q.size() < arm_dof_) {
        return false;
    }
    dynamics_->GetEECordinate(q.data(), *R, *p);
    return true;
}

Eigen::Vector3d HumanMimicController::panel_jog_up_axis() const {
    Eigen::Vector3d u = config_.up_axis;
    if (u.norm() < 1e-9) {
        u = Eigen::Vector3d::UnitZ();
    } else {
        u.normalize();
    }
    return u;
}

Eigen::Vector3d HumanMimicController::panel_jog_forward_axis() const {
    const Eigen::Vector3d u = panel_jog_up_axis();
    Eigen::Vector3d f = config_.forward_axis;
    if (f.norm() < 1e-9) {
        f = Eigen::Vector3d::UnitX();
    }
    f = f - u * u.dot(f);
    if (f.norm() < 1e-9) {
        f = u.unitOrthogonal();
    } else {
        f.normalize();
    }
    return f;
}

Eigen::Vector3d HumanMimicController::panel_jog_left_axis() const {
    // left = up × forward  (right-handed teleop/world frame)
    return panel_jog_up_axis().cross(panel_jog_forward_axis()).normalized();
}

Eigen::Quaterniond HumanMimicController::filter_orientation(const Eigen::Quaterniond& raw_in,
                                                           double dt) {
    Eigen::Quaterniond raw = raw_in.normalized();
    if (!ori_filt_valid_) {
        ori_filt_ = raw;
        ori_filt_valid_ = true;
        return ori_filt_;
    }
    // Quaternion continuity (dot-product sign).
    if (ori_filt_.dot(raw) < 0.0) {
        raw.coeffs() = -raw.coeffs();
    }
    // Deadband: ignore sub-threshold target motion.
    const double deadband = config_.orientation_deadband_deg * kDeg2Rad;
    double ang = 2.0 * std::acos(std::clamp(std::abs(ori_filt_.dot(raw)), 0.0, 1.0));
    if (ang < deadband) {
        return ori_filt_;
    }
    const double tau = std::max(1e-4, config_.tau_orientation);
    const double alpha = 1.0 - std::exp(-std::max(1e-6, dt) / tau);
    ori_filt_ = ori_filt_.slerp(std::clamp(alpha, 0.0, 1.0), raw).normalized();
    return ori_filt_;
}

Eigen::Matrix3d HumanMimicController::compute_hand_target(const QuestPoseSample& sample, double dt) {
    const Eigen::Quaterniond q_hand = filter_orientation(sample.orientation, dt);

    const bool absolute =
        config_.orientation_mapping_mode == OrientationMappingMode::ABSOLUTE_CALIBRATED;
    const Eigen::Quaterniond& hand_anchor = absolute ? hand_ref_rot_abs_ : hand_ref_rot_;
    const Eigen::Quaterniond& robot_anchor = absolute ? robot_ref_rot_abs_ : robot_ref_rot_;

    // GRAVITY-LOCKED orientation mapping.
    //
    // The delta since grip is taken in the VR WORLD frame (gravity referenced):
    //     delta_world = q_hand * hand_anchor^-1
    // A world-vertical yaw therefore has its axis == world-up regardless of how the
    // controller is tilted in the hand.
    //
    // The world->base map W is built from M but with ONLY the heading (yaw about
    // world-up) of the grip anchor, dropping the controller's tilt:
    //     W = M * twist_about(hand_anchor, world_up)^-1
    // This keeps world-up -> robot-up (so a horizontal yaw stays horizontal) while
    // still absorbing which way the user is facing (so the overall push/pull trend
    // is unchanged from the old anchor-based map). The old map used the FULL anchor
    // (M * hand_anchor^-1), so any tilt in how the controller was held leaked into
    // yaw and tilted the gripper.
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
    // Rotate the world-frame delta's AXIS into the robot base frame via W (conjugation),
    // then apply it in the base frame (LEFT multiply) onto the robot anchor.
    const Eigen::Matrix3d dR = W * delta_world.toRotationMatrix() * W.transpose();
    const Eigen::Quaterniond delta_robot = Eigen::Quaterniond(dR).normalized();
    const Eigen::Quaterniond q_tcp_des = (delta_robot * robot_anchor).normalized();

    // Diagnostics: world-frame VR axis (gravity referenced) and mapped robot base axis.
    {
        const Eigen::AngleAxisd aaw(delta_world.normalized());
        const Eigen::AngleAxisd aar(delta_robot.normalized());
        dbg_delta_ang_ = std::isfinite(aar.angle()) ? aar.angle() : 0.0;
        dbg_vr_axis_world_ = (std::isfinite(aaw.angle()) && aaw.angle() > 1e-6)
                                 ? aaw.axis()
                                 : Eigen::Vector3d::Zero();
        dbg_vr_axis_ = dbg_vr_axis_world_;
        dbg_rob_axis_ = (std::isfinite(aar.angle()) && aar.angle() > 1e-6)
                            ? aar.axis()
                            : Eigen::Vector3d::Zero();
    }
    return q_tcp_des.toRotationMatrix();
}

void HumanMimicController::activate_clutch(const std::vector<double>& q_meas,
                                           const QuestPoseSample& pose) {
    // Anti-droop reanchor: pull toward the last COMMANDED pose (capped vs measured)
    // so gravity sink between grips does not accumulate. TCP map + JRG seed from
    // the same q_anchor so the stream rises with the command rather than camping
    // on the drooped measured pose.
    std::vector<double> q_anchor = q_meas;
    if (config_.reanchor_to_commanded && q_target_.size() >= arm_dof_ && all_finite(q_target_)) {
        const double cap = std::max(0.0, config_.max_reanchor_correction_rad);
        for (size_t i = 0; i < arm_dof_ && i < q_anchor.size(); ++i) {
            q_anchor[i] = std::clamp(q_target_[i], q_meas[i] - cap, q_meas[i] + cap);
        }
    }

    // Position is always re-anchored on grip (incremental clutch).
    hand_ref_pos_ = pose.position;
    hand_ref_rot_ = pose.orientation.normalized();

    Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_tcp = Eigen::Matrix3d::Identity();
    const bool tcp_ok = tcp_pose(q_anchor, &p_tcp, &R_tcp);
    if (tcp_ok) {
        robot_ref_pos_ = p_tcp;
        robot_ref_rot_ = Eigen::Quaterniond(R_tcp).normalized();
        // Absolute orientation: lock the hand<->TCP calibration on the FIRST clutch
        // only; later grips re-anchor position but keep this calibration.
        if (!ori_calibrated_) {
            robot_ref_rot_abs_ = robot_ref_rot_;
            hand_ref_rot_abs_ = hand_ref_rot_;
            ori_calibrated_ = true;
        }
    }
    // Reset the orientation filter to the current hand so there is no startup jump.
    ori_filt_ = hand_ref_rot_;
    ori_filt_valid_ = true;

    // Seed/lock skeleton lengths + bend from the reanchor posture.
    Eigen::Vector3d S, E, p_wrist;
    Eigen::Matrix3d R_hand;
    Eigen::MatrixXd Jp, Jw, JpE;
    if (kin_->ShoulderPos(q_anchor, &S) && kin_->ElbowPosJac(q_anchor, &E, &JpE) &&
        kin_->WristState(q_anchor, &p_wrist, &R_hand, &Jp, &Jw)) {
        if (!skel_lengths_locked_ || !config_.lock_segment_lengths) {
            double L1 = config_.L1;
            double L2 = config_.L2;
            if (L1 <= 0.0) {
                L1 = (E - S).norm();
            }
            if (L2 <= 0.0) {
                L2 = (p_wrist - E).norm();
            }
            if (L1 > 0.05 && L2 > 0.05) {
                skeleton_->SetSegmentLengths(L1, L2);
                if (config_.lock_segment_lengths) {
                    skel_lengths_locked_ = true;
                }
            }
        }
        skeleton_->Calibrate(S, E, p_wrist);

        // One-shot tool-frame cal so delta=0 at clutch truly means zero wrist residual.
        if (config_.auto_calibrate_wrist_tcp && !wrist_tcp_calibrated_ && tcp_ok &&
            R_hand.allFinite()) {
            const Eigen::Vector3d offset = R_hand.transpose() * (p_tcp - p_wrist);
            if (offset.norm() > 0.02 && offset.norm() < 0.40) {
                config_.wrist_to_tcp = offset;
                wrist_tcp_calibrated_ = true;
                std::cout << log_prefix_ << "[human-mimic] wrist_to_tcp cal=["
                          << offset.transpose() << "]" << std::endl;
            }
        }
    }
    elbow_cmd_phi_valid_ = false;  // reseed from skeleton on first post-clutch solve
    elbow_hint_used_valid_ = false;
    dbg_d_prev_valid_ = false;

    std::vector<double> q_arm_anchor(arm_dof_, 0.0);
    for (size_t i = 0; i < arm_dof_ && i < q_anchor.size(); ++i) {
        q_arm_anchor[i] = q_anchor[i];
    }
    q_target_ = q_arm_anchor;
    jrg_->Reset(q_arm_anchor);
    clutch_settle_remaining_ = std::max(0, config_.clutch_settle_ticks);
    clutch_active_ = true;
    hold_active_ = false;
    solve_fail_count_ = 0;

    std::cout << log_prefix_ << "[human-mimic] clutch anchored (L1=" << skeleton_->L1()
              << " L2=" << skeleton_->L2()
              << " len_lock=" << (skel_lengths_locked_ ? 1 : 0) << ")" << std::endl;
}

void HumanMimicController::recenter_orientation(const std::vector<double>& q_meas,
                                                const QuestPoseSample& pose) {
    Eigen::Vector3d p_tcp;
    Eigen::Matrix3d R_tcp;
    if (!tcp_pose(q_meas, &p_tcp, &R_tcp)) {
        return;
    }
    hand_ref_rot_abs_ = pose.orientation.normalized();
    robot_ref_rot_abs_ = Eigen::Quaterniond(R_tcp).normalized();
    ori_calibrated_ = true;
    ori_filt_ = pose.orientation.normalized();
    ori_filt_valid_ = true;
    std::cout << log_prefix_ << "[human-mimic] orientation recentered" << std::endl;
}

void HumanMimicController::nudge_tcp_robot(const Eigen::Vector3d& delta_robot) {
    if (!initialized_ || !delta_robot.allFinite() || delta_robot.norm() < 1e-9) {
        return;
    }
    Eigen::Vector3d p_tcp;
    Eigen::Matrix3d R_tcp;
    if (!tcp_pose(q_target_, &p_tcp, &R_tcp) || q_target_.size() < arm_dof_) {
        return;
    }

    const Eigen::Vector3d p_tcp_des = p_tcp + delta_robot;
    if (clutch_active_) {
        // Keep VR-relative map consistent: shift the clutch TCP origin.
        robot_ref_pos_ += delta_robot;
    } else {
        robot_ref_pos_ = p_tcp_des;
    }

    const Eigen::Vector3d p_wrist_des = p_tcp_des - R_tcp * config_.wrist_to_tcp;
    tip_quiet_ = false;
    hold_active_ = false;  // leave frozen hold so no-grip nudge can stream out
    jog_active_ = true;
    jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);

    const int iters = std::max(1, config_.jog_chase_iters);
    const double dt = std::max(0.005, config_.jog_chase_dt);
    const double tol = std::max(1e-4, config_.jog_wrist_tol_m);
    for (int i = 0; i < iters; ++i) {
        tip_quiet_ = false;
        if (!solve_base_skeleton(q_target_, p_wrist_des, R_tcp, dt)) {
            break;
        }
        if (dbg_wrist_err_ <= tol) {
            break;
        }
    }
    jog_active_ = false;

    std::cout << log_prefix_ << "[human-mimic] jog d=[" << delta_robot.transpose()
              << "] e_wrist=" << dbg_wrist_err_ << " clutch=" << (clutch_active_ ? 1 : 0)
              << std::endl;
}

void HumanMimicController::apply_panel_jog_tick(double dt) {
    if (!initialized_ || kin_ == nullptr || q_target_.size() < arm_dof_ || dt <= 0.0) {
        return;
    }
    if (!panel_jog_vel_.allFinite() || panel_jog_vel_.norm() < 1e-12) {
        panel_tcp_des_valid_ = false;
        panel_speed_filt_ = 0.0;
        return;
    }

    // Seed continuous stream from the *commanded* joint reference (motors / JRG).
    if (!panel_tcp_des_valid_) {
        if (jrg_ != nullptr && jrg_->initialized()) {
            q_target_ = jrg_->reference();
        }
        Eigen::Vector3d p0;
        Eigen::Matrix3d R0;
        if (!tcp_pose(q_target_, &p0, &R0)) {
            return;
        }
        panel_tcp_des_ = p0;
        panel_R_des_ = R0;
        panel_tcp_des_valid_ = true;
        panel_speed_filt_ = 0.0;
        hold_active_ = false;
    }

    hold_active_ = false;
    tip_quiet_ = false;
    jog_active_ = true;
    jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);

    // --- Smooth layer: accel-limit tip speed, then integrate a uniform Cartesian ref ---
    const double speed_cmd = panel_jog_vel_.norm();
    const double dv_max = kPanelSpeedAccel * dt;
    panel_speed_filt_ += std::clamp(speed_cmd - panel_speed_filt_, -dv_max, dv_max);
    Eigen::Vector3d v_smooth = Eigen::Vector3d::Zero();
    if (speed_cmd > 1e-9) {
        v_smooth = (panel_jog_vel_ / speed_cmd) * panel_speed_filt_;
    }
    const Eigen::Vector3d dp = v_smooth * dt;
    panel_tcp_des_ += dp;
    if (clutch_active_) {
        robot_ref_pos_ += dp;
    } else {
        robot_ref_pos_ = panel_tcp_des_;
    }

    // Velocity-level DLS: translate TCP at v_smooth while softly holding latch orientation.
    // Avoids absolute whole-arm IK posture snaps that caused tip jitter / upward pops.
    Eigen::Vector3d p_wrist;
    Eigen::Matrix3d R_hand;
    Eigen::MatrixXd Jp, Jw;
    if (!kin_->WristState(q_target_, &p_wrist, &R_hand, &Jp, &Jw) || Jp.rows() != 3 ||
        Jp.cols() != static_cast<int>(arm_dof_)) {
        jog_active_ = false;
        return;
    }
    // With frozen panel_R_des_, TCP and wrist share the same translational Jacobian.
    const Eigen::Vector3d p_tcp = p_wrist + panel_R_des_ * config_.wrist_to_tcp;
    // Uniform velocity feed + soft position correction (track the integrated ref).
    Eigen::Vector3d dx = v_smooth * dt + 0.35 * (panel_tcp_des_ - p_tcp);
    const double dx_max = std::max(1e-4, panel_speed_filt_ * dt * 2.0 + 0.0005);
    if (dx.norm() > dx_max) {
        dx *= dx_max / dx.norm();
    }

    // Soft orientation hold: small angular step toward panel_R_des_.
    Eigen::Vector3d w_des = Eigen::Vector3d::Zero();
    {
        Eigen::Matrix3d Re = panel_R_des_ * R_hand.transpose();
        Eigen::AngleAxisd aa(Eigen::Quaterniond(Re).normalized());
        double ang = aa.angle();
        if (std::isfinite(ang) && aa.axis().allFinite()) {
            if (ang > M_PI) {
                ang -= 2.0 * M_PI;
            }
            w_des = aa.axis() * std::clamp(ang, -0.15, 0.15) * 0.4;  // rad this tick (soft)
        }
    }

    constexpr double kLambda = 0.08;
    Eigen::MatrixXd J(6, static_cast<int>(arm_dof_));
    J.setZero();
    J.topRows(3) = Jp;
    J.bottomRows(3) = Jw;
    Eigen::VectorXd dx6(6);
    dx6.head<3>() = dx;
    dx6.tail<3>() = w_des;

    Eigen::MatrixXd H = J * J.transpose();
    H.diagonal().array() += kLambda * kLambda;
    const Eigen::VectorXd dq = J.transpose() * H.ldlt().solve(dx6);
    if (!dq.allFinite()) {
        jog_active_ = false;
        return;
    }

    const double qdot_cap = std::max(config_.qdot_max, config_.jog_q_slew_radps);
    for (size_t i = 0; i < arm_dof_ && i < static_cast<size_t>(dq.size()); ++i) {
        q_target_[i] += std::clamp(dq(static_cast<int>(i)), -qdot_cap * dt, qdot_cap * dt);
    }
    if (!q_min_.empty() && !q_max_.empty()) {
        for (size_t i = 0; i < arm_dof_ && i < q_min_.size() && i < q_max_.size(); ++i) {
            q_target_[i] = std::clamp(q_target_[i], q_min_[i], q_max_[i]);
        }
    }

    // Continuously emit this as the joint reference — no second JRG lag layer.
    if (jrg_ != nullptr) {
        jrg_->Reset(q_target_);
    }
    jog_active_ = false;
}

bool HumanMimicController::commanded_tcp_pose(Eigen::Vector3d* p, Eigen::Matrix3d* R) const {
    if (jrg_ != nullptr && jrg_->initialized()) {
        return tcp_pose(jrg_->reference(), p, R);
    }
    return tcp_pose(q_target_, p, R);
}

void HumanMimicController::set_orientation_lock(bool on, const Eigen::Matrix3d& R_lock) {
    ori_lock_active_ = on;
    if (on && R_lock.allFinite()) {
        ori_lock_R_ = R_lock;
    }
}

void HumanMimicController::drive_tcp_robot(const Eigen::Vector3d& p_tcp, const Eigen::Matrix3d& R_hand,
                                           double dt) {
    if (!initialized_ || q_target_.size() < arm_dof_ || dt <= 0.0) {
        return;
    }
    if (!p_tcp.allFinite() || !R_hand.allFinite()) {
        return;
    }
    if (jrg_ != nullptr && jrg_->initialized() && (q_target_.size() < arm_dof_)) {
        q_target_ = jrg_->reference();
    } else if (jrg_ != nullptr && jrg_->initialized() && couple_follow_active_) {
        // Keep left seeded from commanded pose when enter couple.
    }
    hold_active_ = false;
    tip_quiet_ = false;
    jog_active_ = true;
    jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);
    robot_ref_pos_ = p_tcp;
    const Eigen::Vector3d p_wrist_des = p_tcp - R_hand * config_.wrist_to_tcp;
    solve_base_skeleton(q_target_, p_wrist_des, R_hand, dt);
    jog_active_ = false;
}

void HumanMimicController::deactivate_clutch() {
    clutch_active_ = false;
    last_trigger_reset_ = false;
    last_recenter_ = false;
    ori_filt_valid_ = false;
    elbow_cmd_phi_valid_ = false;
}

void HumanMimicController::fill_reference(const std::vector<JointState>& current_joints,
                                          const std::vector<double>& q_arm,
                                          std::vector<JointState>* reference) const {
    const size_t total = current_joints.size();
    reference->assign(total, JointState{});
    for (size_t i = 0; i < total; ++i) {
        (*reference)[i].velocity = 0.0;
        (*reference)[i].effort = 0.0;
        if (i < arm_dof_ && i < q_arm.size()) {
            (*reference)[i].position = q_arm[i];
        } else if (i == arm_dof_) {
            // Gripper: driven by the A/X (close), B/Y (open) state machine.
            (*reference)[i].position = gripper_target_;
        } else {
            (*reference)[i].position = current_joints[i].position;
        }
    }
}

void HumanMimicController::update_gripper_from_buttons(const QuestPoseSample& sample,
                                                      const std::vector<JointState>& current_joints,
                                                      double dt) {
    if (current_joints.size() <= arm_dof_ || dt <= 0.0) {
        return;
    }
    const double q_meas = current_joints[arm_dof_].position;
    const double q_vel = current_joints[arm_dof_].velocity;
    gripper_meas_ = q_meas;
    constexpr double kTol = 0.0005;
    const double open_speed = std::max(0.0, config_.gripper_speed);
    const double close_speed = std::max(0.0, config_.gripper_close_speed);
    const double close_dir =
        (config_.gripper_closed >= config_.gripper_open) ? 1.0 : -1.0;

    if (!gripper_hold_init_) {
        gripper_hold_pos_ = q_meas;
        gripper_target_ = q_meas;
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_init_ = true;
        gripper_stall_timer_ = 0.0;
    }

    // Clamp a hold angle onto the open..closed stroke.
    auto clamp_stroke = [&](double q) {
        if (config_.gripper_closed >= config_.gripper_open) {
            return std::clamp(q, config_.gripper_open, config_.gripper_closed);
        }
        return std::clamp(q, config_.gripper_closed, config_.gripper_open);
    };
    // Freeze while closing: keep the more-closed of command vs measured (+ optional
    // preload). Snapping to q_meas alone drops MIT spring force and feels like a release.
    auto freeze_close_hold = [&](double prefer_cmd) {
        double hold = (close_dir >= 0.0) ? std::max(q_meas, prefer_cmd)
                                         : std::min(q_meas, prefer_cmd);
        hold += close_dir * std::max(0.0, config_.gripper_hold_preload);
        hold = clamp_stroke(hold);
        gripper_hold_pos_ = hold;
        gripper_target_ = hold;
    };

    // Auto-stop at the physical open/closed limits.
    if (gripper_motion_ == GripperMotion::Opening && q_meas <= config_.gripper_open + kTol) {
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_pos_ = config_.gripper_open;
        gripper_target_ = config_.gripper_open;
        gripper_stall_timer_ = 0.0;
    } else if (gripper_motion_ == GripperMotion::Closing &&
               q_meas >= config_.gripper_closed - kTol) {
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_pos_ = config_.gripper_closed;
        gripper_target_ = config_.gripper_closed;
        gripper_stall_timer_ = 0.0;
    }

    if (sample.valid) {
        const bool close_rising = sample.button_close && !last_button_close_;
        const bool open_rising = sample.button_open && !last_button_open_;
        last_button_close_ = sample.button_close;
        last_button_open_ = sample.button_open;

        const double close_target = config_.gripper_closed;
        const double open_target = config_.gripper_open;
        const bool not_fully_closed = std::abs(q_meas - close_target) > kTol;
        const bool not_fully_open = std::abs(q_meas - open_target) > kTol;

        // One press = act immediately. Pressing the SAME direction while it is already
        // moving that way toggles it to a stop; pressing the OPPOSITE direction reverses
        // right away (no need for a second press to "cancel" first).
        if (close_rising) {
            if (gripper_motion_ == GripperMotion::Closing) {
                gripper_motion_ = GripperMotion::Stopped;
                freeze_close_hold(gripper_target_);
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_ << "[human-mimic] gripper stop (A/X toggle)" << std::endl;
            } else if (not_fully_closed) {
                gripper_motion_ = GripperMotion::Closing;
                gripper_target_ = q_meas;
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_ << "[human-mimic] gripper closing (A/X)" << std::endl;
            }
        }
        if (open_rising) {
            if (gripper_motion_ == GripperMotion::Opening) {
                gripper_motion_ = GripperMotion::Stopped;
                gripper_hold_pos_ = q_meas;
                gripper_target_ = q_meas;
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_ << "[human-mimic] gripper stop (B/Y toggle)" << std::endl;
            } else if (not_fully_open) {
                gripper_motion_ = GripperMotion::Opening;
                gripper_target_ = q_meas;
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_ << "[human-mimic] gripper opening (B/Y)" << std::endl;
            }
        }
    }

    switch (gripper_motion_) {
        case GripperMotion::Stopped:
            gripper_target_ = gripper_hold_pos_;
            gripper_stall_timer_ = 0.0;
            break;
        case GripperMotion::Opening:
            gripper_target_ =
                std::max(config_.gripper_open, gripper_target_ - open_speed * dt);
            gripper_stall_timer_ = 0.0;
            break;
        case GripperMotion::Closing:
            gripper_target_ =
                std::min(config_.gripper_closed, gripper_target_ + close_speed * dt);
            break;
    }

    // Soft contact: stop advancing stroke, but keep close-side command preload.
    if (config_.gripper_soft_stall && gripper_motion_ == GripperMotion::Closing) {
        const double lag_close = close_dir * (gripper_target_ - q_meas);
        const bool ahead = lag_close >= config_.gripper_stall_error;
        const bool stalled = std::abs(q_vel) <= config_.gripper_stall_velocity;
        if (ahead && stalled) {
            gripper_stall_timer_ += dt;
            if (gripper_stall_timer_ >= config_.gripper_stall_time) {
                gripper_motion_ = GripperMotion::Stopped;
                freeze_close_hold(gripper_target_);
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_
                          << "[human-mimic] gripper soft-stall hold q_cmd=" << gripper_hold_pos_
                          << " q_meas=" << q_meas
                          << " (keep preload; re-press A/X to squeeze more)" << std::endl;
            }
        } else {
            gripper_stall_timer_ = 0.0;
        }
    }
}

void HumanMimicController::ramp_arm_reset(double dt) {
    if (q_target_.size() < arm_dof_) {
        return;
    }
    const double step = std::max(0.0, config_.reset_speed) * dt;
    const int start = std::max(0, config_.reset_first_joint);
    const bool has_nominal = q_nominal_.size() >= arm_dof_;
    for (int i = start; i < static_cast<int>(arm_dof_) && i < static_cast<int>(q_target_.size());
         ++i) {
        // Return to the start posture (q_nominal / TABLE_READY), else neutral.
        const double target = has_nominal ? q_nominal_[i] : 0.0;
        if (q_target_[i] > target) {
            q_target_[i] = std::max(target, q_target_[i] - step);
        } else {
            q_target_[i] = std::min(target, q_target_[i] + step);
        }
    }
}

void HumanMimicController::hold_reference(const std::vector<JointState>& current_joints,
                                          const std::vector<double>& q_hold,
                                          std::vector<JointState>* reference) {
    if (!hold_active_) {
        // Freeze once: prefer the last commanded reference (smooth on release),
        // fall back to the measured pose at startup.
        hold_q_ = jrg_->initialized() ? jrg_->reference() : q_hold;
        jrg_->Reset(hold_q_);
        q_target_ = hold_q_;
        hold_active_ = true;
    }
    fill_reference(current_joints, hold_q_, reference);
}

bool HumanMimicController::solve_base_skeleton(const std::vector<double>& q_meas,
                                              const Eigen::Vector3d& p_wrist_des,
                                              const Eigen::Matrix3d& R_hand_des, double solve_dt) {
    // Prefer commanded joints for the shoulder used by soft-reach / skeleton —
    // building geometry on lagged measured proximal joints is a common reason
    // the tip fights the elbow while the shoulder "doesn't move".
    Eigen::Vector3d S_now;
    const bool use_cmd_skel = q_target_.size() >= arm_dof_ && all_finite(q_target_);
    if (!kin_->ShoulderPos(use_cmd_skel ? q_target_ : q_meas, &S_now)) {
        if (!kin_->ShoulderPos(q_meas, &S_now)) {
            S_now = robot_ref_pos_;
        }
    }

    // Reach normalization (scheme A): allowed wrist distance ceiling rises with
    // downward teleop progress from the clutch anchor. Mid-travel stays bent;
    // workspace bottom allows near-straight; raising lowers the ceiling so the
    // elbow can return to an obtuse bend (fully reversible — no fixed asymptote).
    const double reach_full = skeleton_->L1() + skeleton_->L2();
    Eigen::Vector3d p_wrist_use = p_wrist_des;
    Eigen::Vector3d r_sw = p_wrist_des - S_now;
    const double d_req = r_sw.norm();
    dbg_d_req_ = d_req;
    double reach_progress = 0.0;
    if (config_.reach_soft_enabled && d_req > 1e-6 && reach_full > 1e-3) {
        const double margin_full = std::max(1e-4, config_.reach_soft_margin_m);
        double margin = margin_full;
        if (config_.reach_soft_progress_enabled) {
            const double down_lim = std::max(0.05, config_.max_teleop_offset_down_m);
            // up_off is set in step() before this solve (TCP offset along up_axis).
            double p = 0.0;
            if (dbg_up_off_ < 0.0) {
                p = std::clamp(-dbg_up_off_ / down_lim, 0.0, 1.0);
            }
            // Smoothstep: slow at the start (stay bent), open near the bottom.
            reach_progress = p * p * (3.0 - 2.0 * p);
            const double margin_bent =
                std::max(margin_full, config_.reach_soft_bent_margin_m);
            margin = margin_bent + reach_progress * (margin_full - margin_bent);
        } else {
            // Progress gate off: fixed near-straight ceiling (no mid-travel bend hold).
            reach_progress = 1.0;
        }
        const double d_star = std::max(1e-3, reach_full - margin);

        // Hard ceiling at d_star (moving when progress is on). Raising lowers
        // progress → compresses again → bend returns. No exponential asymptote.
        const double d_out = std::min(d_req, d_star);
        p_wrist_use = S_now + (r_sw / d_req) * d_out;
    }
    dbg_reach_progress_ = reach_progress;

    const SkeletonResult skel = skeleton_->Solve(S_now, p_wrist_use);
    dbg_reach_ = skel.reachable;
    dbg_phi_base_ = skel.phi_base;
    dbg_d_ = (p_wrist_use - S_now).norm();
    dbg_dmax_ = skeleton_->L1() + skeleton_->L2() - config_.reach_margin_m;

    // Stretch from the ROBOT's commanded wrist distance (after soft-norm), not human
    // d_req. Human d_req is usually > L1+L2, which permanently tripped "full stretch"
    // and zeroed the skeleton lock — that is exactly what causes 换逆解.
    const double d_cmd = dbg_d_;
    const double d_star_stretch =
        std::max(1e-3, reach_full - config_.reach_soft_margin_m);
    double stretch = 0.0;
    if (d_star_stretch > 1e-3 && d_cmd > 1e-6) {
        const double start_frac = std::clamp(config_.extension_yield_start, 0.0, 0.99);
        const double start_d = start_frac * d_star_stretch;
        const double span = std::max(1e-6, d_star_stretch - start_d);
        stretch = std::clamp((d_cmd - start_d) / span, 0.0, 1.0);
    }
    dbg_stretch_ = stretch;

    // Tip-priority yield amount (used later for IK weights + shoulder slew).
    tip_yield_ = 0.0;
    const bool jog_mode = jog_active_ || jog_bypass_remaining_ > 0;
    if (jog_mode) {
        // Panel jog owns the tip — do not treat the hand as "quiet" and freeze.
        tip_quiet_ = false;
    }
    if (!jog_mode && config_.tip_priority_enabled) {
        if (d_req > 1e-6) {
            tip_yield_ = std::clamp(
                (d_req - d_cmd) / std::max(1e-3, config_.reach_soft_zone_m), 0.0, 1.0);
        }
        const double werr0 = std::max(0.0, config_.tip_priority_wrist_err_m);
        if (dbg_wrist_err_ > werr0) {
            tip_yield_ = std::max(
                tip_yield_, std::clamp((dbg_wrist_err_ - werr0) / 0.05, 0.0, 1.0));
        }
    }

    // Elbow target: virtual-skeleton phi, optionally biased by HybrIK, then
    // SEMANTICALLY DISAMBIGUATED. Depth flips report an antipodal / “acute”
    // silhouette; while reaching forward a human never does that — pick the
    // continuous/obtuse-consistent branch instead of chasing the wrong one.
    double phi_goal = skel.phi_base;
    bool hint_applied = false;
    const bool use_hint =
        config_.elbow_hint_enabled && elbow_hint_valid_ && skel.h_radius > 1e-3;
    if (use_hint) {
        double phi_hint =
            config_.elbow_hint_sign * elbow_hint_phi_raw_ + config_.elbow_hint_offset_rad;
        // First resolve HybrIK's depth flip vs the virtual skeleton: among
        // {hint, hint+π}, take the one nearer to phi_base (same silhouette family).
        {
            const double d0 = std::abs(wrap_to_pi(phi_hint - skel.phi_base));
            const double d1 = std::abs(wrap_to_pi(phi_hint + M_PI - skel.phi_base));
            if (d1 + 1e-6 < d0) {
                phi_hint = wrap_to_pi(phi_hint + M_PI);
            }
        }
        const double w = std::clamp(config_.elbow_hint_weight, 0.0, 1.0);
        double d_phi = wrap_to_pi(phi_hint - skel.phi_base);
        const double max_d = std::max(0.0, config_.elbow_hint_max_delta_rad);
        d_phi = std::clamp(d_phi, -max_d, max_d);
        phi_goal = skel.phi_base + w * d_phi;
        hint_applied = true;
        elbow_hint_phi_used_ = phi_goal;
        elbow_hint_used_valid_ = true;
    } else if (!elbow_hint_valid_) {
        elbow_hint_used_valid_ = false;
        elbow_hint_phi_used_ = skel.phi_base;
    }

    const double flex = elbow_flex_from_reach(d_cmd, skeleton_->L1(), skeleton_->L2());
    dbg_elbow_flex_ = flex;
    const bool extending = dbg_d_prev_valid_ && (d_cmd > dbg_d_prev_ + 0.002);
    dbg_d_prev_ = d_cmd;
    dbg_d_prev_valid_ = true;

    const double phi_anchor = elbow_cmd_phi_valid_ ? elbow_cmd_phi_ : skel.phi_base;
    int flipped = 0;
    phi_goal = disambiguate_elbow_phi(phi_goal, /*has_anchor=*/true, phi_anchor, stretch,
                                      extending, flex, skeleton_.get(), S_now, skel.wrist_clamped,
                                      config_, &flipped);
    dbg_phi_flip_ = flipped;

    // Near straight: freeze commanded phi. The swivel circle collapses and any
    // small phi / HybrIK jitter turns into a visible "elbow spin" around S→W.
    const bool phi_frozen =
        stretch >= std::clamp(config_.extension_phi_freeze_stretch, 0.0, 1.0) ||
        skel.h_radius < std::max(1e-4, config_.elbow_singularity_h_m);

    if (!elbow_cmd_phi_valid_ || skel.h_radius < 1e-3) {
        elbow_cmd_phi_ = phi_goal;
        elbow_cmd_phi_valid_ = true;
    } else if (phi_frozen) {
        // Keep last commanded phi — do not chase singular observations.
    } else if (config_.elbow_cmd_slew_enabled) {
        double max_step = std::max(0.0, config_.elbow_cmd_slew_radps) * std::max(1e-4, solve_dt);
        if (hint_applied) {
            max_step = std::min(max_step, std::max(0.0, config_.elbow_hint_slew_radps) *
                                              std::max(1e-4, solve_dt));
        }
        max_step = std::min(max_step, std::max(1e-4, config_.elbow_cmd_max_step_rad));
        double step = wrap_to_pi(phi_goal - elbow_cmd_phi_);
        step = std::clamp(step, -max_step, max_step);
        elbow_cmd_phi_ += step;
    } else {
        elbow_cmd_phi_ = phi_goal;
    }

    Eigen::Vector3d elbow_des = skel.elbow_des;
    {
        const ElbowTarget et = skeleton_->compute_elbow_target_from_phi(
            S_now, skel.wrist_clamped, skeleton_->L1(), skeleton_->L2(), elbow_cmd_phi_);
        if (!et.degenerate && et.E.allFinite()) {
            elbow_des = et.E;
            if (!phi_frozen) {
                skeleton_->SetContinuityBend(et.bend_dir);
            }
        }
    }
    if (hint_applied && !phi_frozen) {
        elbow_hint_phi_used_ = elbow_cmd_phi_;
    }

    // Near extension: pin elbow onto the S–W axis so tip orientation (not swivel)
    // owns the distal posture. Always do this when stretched — HybrIK phi is
    // singular here too and otherwise "spins" the forearm.
    // Skip during panel jog: straighten + elbow fight a pure tip translation and
    // can pop the arm upward on a left/right nudge.
    if (config_.extension_straighten_elbow && stretch > 1e-3 && !jog_mode) {
        const Eigen::Vector3d sw = skel.wrist_clamped - S_now;
        const double d = sw.norm();
        if (d > 1e-6) {
            const Eigen::Vector3d n = sw / d;
            const double L1 = skeleton_->L1();
            const double L2 = skeleton_->L2();
            const double a = (L1 * L1 - L2 * L2 + d * d) / (2.0 * d);
            const Eigen::Vector3d elbow_straight = S_now + n * a;
            elbow_des = (1.0 - stretch) * elbow_des + stretch * elbow_straight;
        }
    }

    // Near extension: kill elbow position task, BOOST tip orientation, keep continuity.
    double elbow_w_scale = 1.0;
    double ori_w_scale = 1.0;
    double cont_w_scale = 1.0;
    double nom_w_scale = 1.0;
    if (config_.extension_yield_enabled && stretch > 0.0 && !jog_mode) {
        elbow_w_scale = 1.0 + stretch * (config_.extension_elbow_scale_min - 1.0);
        ori_w_scale = 1.0 + stretch * (config_.extension_ori_scale_min - 1.0);
        cont_w_scale = 1.0 + stretch * (config_.extension_cont_scale_min - 1.0);
        nom_w_scale = 1.0 + stretch * (config_.extension_nom_scale_min - 1.0);
    }

    // Tip-priority: only when NOT near-straight. Near extension tip_yield from
    // soft-reach is chronically high and would strip continuity → free spin.
    if (tip_yield_ > 1e-6 && stretch < 0.5 && !jog_mode) {
        elbow_w_scale *=
            (1.0 - tip_yield_ * (1.0 - std::clamp(config_.tip_priority_elbow_scale_min, 0.0, 1.0)));
        cont_w_scale *=
            (1.0 - tip_yield_ * (1.0 - std::clamp(config_.tip_priority_cont_scale_min, 0.0, 1.0)));
        nom_w_scale *= (1.0 - tip_yield_);
    }
    if (jog_mode) {
        if (panel_stream_active()) {
            // Continuous hold-to-jog: keep posture soft so the first frame does not
            // "pop" the tip (old jog_elbow_scale=0.1 caused the upward jerk).
            elbow_w_scale *= 0.75;
            nom_w_scale *= 0.35;
        } else {
            elbow_w_scale *= std::clamp(config_.jog_elbow_scale, 0.0, 1.0);
            nom_w_scale = 0.0;
        }
    }

    // Single unified whole-arm IK: wrist + hand ori + skeleton elbow.
    const HumanMimicIkResult ik =
        ik_->Solve(q_target_, skel.wrist_clamped, R_hand_des, elbow_des, q_nominal_,
                   elbow_w_scale, ori_w_scale, cont_w_scale, nom_w_scale);
    dbg_ik_ok_ = ik.ok;
    dbg_ik_iters_ = ik.iters;
    dbg_wrist_err_ = ik.wrist_pos_err;
    dbg_ori_err_ = ik.hand_ori_err;
    dbg_elbow_err_ = ik.elbow_pos_err;
    if (ik.ok && all_finite(ik.q)) {
        // Fine-motion hold: when the hand is quiet and the wrist is already close,
        // skip IK polishing — that residual chasing shows up as fingertip jitter.
        // Never hold during/just after panel jog (that was the stick→slip jump).
        if (config_.fine_motion_hold_enabled && !jog_mode && tip_quiet_ &&
            ik.wrist_pos_err <= std::max(1e-4, config_.wrist_hold_err_m)) {
            solve_fail_count_ = 0;
            dbg_hold_prev_ = true;
            if (jog_bypass_remaining_ > 0 && !jog_active_) {
                --jog_bypass_remaining_;
            }
            return true;
        }
        if (jog_active_) {
            // Chase loop: accept IK promptly so a button press finishes the tip move.
            q_target_ = ik.q;
            if (!q_min_.empty() && !q_max_.empty()) {
                for (size_t i = 0; i < arm_dof_ && i < q_min_.size() && i < q_max_.size(); ++i) {
                    q_target_[i] = std::clamp(q_target_[i], q_min_[i], q_max_[i]);
                }
            }
        } else if (config_.q_target_slew_enabled && q_target_.size() >= arm_dof_ &&
                   ik.q.size() >= arm_dof_) {
            const double max_step =
                std::max(0.0, config_.q_target_slew_max_radps) * std::max(1e-4, solve_dt);
            const double jog_step =
                std::max(0.0, config_.jog_q_slew_radps) * std::max(1e-4, solve_dt);
            const double sh_boost =
                1.0 + tip_yield_ * std::max(0.0, config_.tip_priority_shoulder_slew_boost - 1.0);
            const double roll_scale =
                1.0 + stretch * (std::clamp(config_.extension_roll_slew_scale, 0.05, 1.0) - 1.0);
            const double q_db =
                (!jog_mode && tip_quiet_) ? std::max(0.0, config_.q_update_deadband_rad) : 0.0;
            for (size_t i = 0; i < arm_dof_; ++i) {
                double joint_max = jog_mode ? std::max(max_step, jog_step) : max_step;
                if (i < 2 && tip_yield_ > 1e-6 && stretch < 0.5) {
                    joint_max *= sh_boost;
                }
                if (i == 2 && stretch > 1e-3 && !jog_mode) {
                    joint_max *= roll_scale;
                }
                if (i == 3 && !jog_mode) {
                    joint_max = std::min(
                        joint_max,
                        std::max(0.0, config_.elbow_joint_slew_radps) * std::max(1e-4, solve_dt));
                }
                double raw_dq = ik.q[i] - q_target_[i];
                if (q_db > 0.0 && std::abs(raw_dq) < q_db) {
                    continue;
                }
                const double dq = std::clamp(raw_dq, -joint_max, joint_max);
                q_target_[i] += dq;
            }
            if (!q_min_.empty() && !q_max_.empty()) {
                for (size_t i = 0; i < arm_dof_ && i < q_min_.size() && i < q_max_.size(); ++i) {
                    q_target_[i] = std::clamp(q_target_[i], q_min_[i], q_max_[i]);
                }
            }
        } else {
            q_target_ = ik.q;
        }
        solve_fail_count_ = 0;
        dbg_hold_prev_ = false;
        if (jog_bypass_remaining_ > 0 && !jog_active_) {
            --jog_bypass_remaining_;
        }
        return true;
    }
    ++solve_fail_count_;
    dbg_hold_prev_ = true;
    if (jog_bypass_remaining_ > 0 && !jog_active_) {
        --jog_bypass_remaining_;
    }
    return false;
}

bool HumanMimicController::step(const std::vector<JointState>& current_joints,
                               const QuestPoseSample& sample, double dt,
                               std::vector<JointState>* joint_reference) {
    if (!initialized_ || current_joints.size() < arm_dof_) {
        return false;
    }

    std::vector<double> q_meas(arm_dof_, 0.0);
    for (size_t i = 0; i < arm_dof_; ++i) {
        q_meas[i] = current_joints[i].position;
    }

    // Orientation recenter gesture (both face buttons) takes priority over the
    // gripper state machine so a simultaneous press does not toggle the gripper.
    const bool recenter_combo =
        config_.orientation_recenter_button == OrientationRecenterButton::BOTH_FACE_BUTTONS &&
        sample.valid && sample.button_close && sample.button_open;
    const bool recenter_edge = recenter_combo && !last_recenter_;
    if (!recenter_combo) {
        // Gripper is live only when we are not issuing a recenter gesture.
        update_gripper_from_buttons(sample, current_joints, dt);
    } else {
        last_button_close_ = sample.button_close;
        last_button_open_ = sample.button_open;
    }
    last_recenter_ = recenter_combo;

    // Quest freshness.
    const auto now = std::chrono::steady_clock::now();
    const int64_t now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    double age_sec = std::numeric_limits<double>::infinity();
    if (sample.valid && sample.receive_timestamp_ns > 0) {
        age_sec = static_cast<double>(now_ns - sample.receive_timestamp_ns) * 1e-9;
    }
    const bool timed_out = !std::isfinite(age_sec) || age_sec >= config_.quest_timeout_sec;
    const bool stale = std::isfinite(age_sec) && age_sec >= config_.quest_stale_sec && !timed_out;

    const bool panel_jogging = panel_jog_vel_.norm() > 1e-6;
    const bool post_nudge_stream = jog_bypass_remaining_ > 0;

    if (timed_out && !panel_jogging) {
        if (!quest_timeout_active_) {
            std::cerr << log_prefix_ << "[human-mimic] Quest timeout (age="
                      << (std::isfinite(age_sec) ? age_sec : -1.0)
                      << "s) — freezing; re-grip required" << std::endl;
        }
        quest_timeout_active_ = true;
        require_reclutch_ = true;
        if (clutch_active_) {
            deactivate_clutch();
        }
        last_grip_ = false;
        hold_reference(current_joints, q_meas, joint_reference);
        return true;
    }
    if (quest_timeout_active_ && !panel_jogging) {
        quest_timeout_active_ = false;
        require_reclutch_ = true;
    } else if (quest_timeout_active_ && panel_jogging) {
        // Panel jog remains available without Quest; clear sticky timeout lock so we
        // do not re-freeze the arm the moment the button is released.
        quest_timeout_active_ = false;
    }

    const bool grip_now = sample.valid && sample.grip_active;
    const bool grip_rising = grip_now && !last_grip_;
    const bool grip_falling = !grip_now && last_grip_;
    last_grip_ = grip_now;

    if (grip_falling && clutch_active_) {
        std::cout << log_prefix_ << "[human-mimic] clutch released" << std::endl;
        deactivate_clutch();
    }

    if (require_reclutch_ && !clutch_active_ && !panel_jogging && !post_nudge_stream) {
        if (grip_rising) {
            require_reclutch_ = false;
        } else {
            hold_reference(current_joints, q_meas, joint_reference);
            return true;
        }
    }
    if (require_reclutch_ && (panel_jogging || post_nudge_stream)) {
        // Panel motion does not require a re-grip.
        require_reclutch_ = false;
    }

    if (grip_rising) {
        activate_clutch(q_meas, sample);
    }

    // Couple-follow left arm: only stream the last / drive_tcp result (no VR map).
    if (couple_follow_active_ && !panel_jogging) {
        hold_active_ = false;
        const std::vector<double>& q_ref =
            jrg_->Step(q_target_, dt, /*k_scale=*/4.0, /*qdot_scale=*/2.0);
        fill_reference(current_joints, q_ref, joint_reference);
        return true;
    }

    if (!clutch_active_) {
        if (panel_jogging) {
            if (q_target_.size() < arm_dof_) {
                q_target_ = q_meas;
                jrg_->Reset(q_target_);
            }
            apply_panel_jog_tick(dt);
            maybe_log("panel");
            // Velocity-IK already Reset(jrg) each tick → emit that smooth stream directly.
            const std::vector<double>& q_ref =
                (jrg_ != nullptr && jrg_->initialized()) ? jrg_->reference() : q_target_;
            fill_reference(current_joints, q_ref, joint_reference);
            return true;
        }
        if (post_nudge_stream) {
            // Discrete nudge without grip: keep streaming q_target_ until bypass ends.
            hold_active_ = false;
            --jog_bypass_remaining_;
            const std::vector<double>& q_ref = jrg_->Step(q_target_, dt);
            fill_reference(current_joints, q_ref, joint_reference);
            return true;
        }
        hold_reference(current_joints, q_meas, joint_reference);
        return true;
    }

    // Explicit orientation recenter (position anchor unchanged).
    if (recenter_edge && grip_now) {
        recenter_orientation(q_meas, sample);
    }

    // Trigger held = uniform whole-arm return to the start posture; on release the
    // clutch re-anchors at the current pose so teleop resumes without a jump.
    const bool trigger_now =
        sample.valid && grip_now && (sample.trigger >= config_.trigger_threshold);
    const bool trigger_falling = last_trigger_reset_ && !trigger_now;
    last_trigger_reset_ = trigger_now;

    const bool freeze_on_fail = solve_fail_count_ > config_.solve_fail_freeze_count;

    if (trigger_now) {
        ramp_arm_reset(dt);
    } else if (trigger_falling) {
        activate_clutch(q_meas, sample);
    } else if (freeze_on_fail) {
        std::cerr << log_prefix_ << "[human-mimic] repeated IK failure — freezing; re-grip required"
                  << std::endl;
        require_reclutch_ = true;
        deactivate_clutch();
        last_grip_ = false;
        hold_reference(current_joints, q_meas, joint_reference);
        return true;
    } else if (grip_now && !stale && sample.valid) {
        // Pure virtual-skeleton chain: VR target -> compute_hand_target ->
        // VirtualArmSkeleton -> single whole-arm IK -> q_target.
        if (clutch_settle_remaining_ > 0) {
            // Hold commanded pose right after grip so TCP/wrist frame residuals
            // cannot flash a first-frame jump before the map is zeroed.
            --clutch_settle_remaining_;
            if (panel_jogging) {
                apply_panel_jog_tick(dt);
            }
            maybe_log("settle");
        } else {
            // Panel hold stacks with VR by sliding the clutch TCP origin.
            if (panel_jogging) {
                const Eigen::Vector3d dp = panel_jog_vel_ * dt;
                if (dp.allFinite() && dp.norm() > 1e-12) {
                    robot_ref_pos_ += dp;
                    hold_active_ = false;
                }
            }
            Eigen::Matrix3d R_hand_des =
                ori_lock_active_ ? ori_lock_R_ : compute_hand_target(sample, dt);

            // World-frame translation: Quest up stays robot up even if the
            // controller is tilted (body-frame lift looked "diagonal only").
            Eigen::Vector3d delta_vr = sample.position - hand_ref_pos_;
            if (!config_.position_map_world_frame) {
                delta_vr = hand_ref_rot_.inverse() * delta_vr;
            }
            Eigen::Vector3d delta_robot = map_vr_delta_pos(delta_vr);
            // Axis-separated workspace cap. Vertical must NOT share a ball budget with
            // lateral: facing grippers already spend the lateral allowance, and a
            // combined |Y,Z| clamp crushed lift (arm felt locked / sank).
            {
                Eigen::Vector3d f = config_.forward_axis;
                Eigen::Vector3d u = config_.up_axis;
                const double fn = f.norm();
                const double un = u.norm();
                const double back = config_.max_teleop_offset_m;
                const double open = std::max(config_.max_teleop_offset_forward_m, back);
                const double up_lim = std::max(0.05, config_.max_teleop_offset_up_m);
                const double down_lim = std::max(0.05, config_.max_teleop_offset_down_m);
                if (fn > 1e-9 && un > 1e-9) {
                    f /= fn;
                    u /= un;
                    // Keep up orthogonal to forward for a clean 3-axis split.
                    u = (u - f * f.dot(u));
                    if (u.norm() < 1e-9) {
                        u = Eigen::Vector3d::UnitZ();
                    } else {
                        u.normalize();
                    }
                    double s = delta_robot.dot(f);
                    double z = delta_robot.dot(u);
                    Eigen::Vector3d lat = delta_robot - s * f - z * u;
                    const double lat_n = lat.norm();
                    if (lat_n > open && lat_n > 1e-9) {
                        lat *= open / lat_n;
                    }
                    s = std::clamp(s, -back, open);
                    z = std::clamp(z, -down_lim, up_lim);
                    delta_robot = s * f + z * u + lat;
                    dbg_fwd_off_ = s;
                    dbg_up_off_ = z;
                } else {
                    const double off = delta_robot.norm();
                    if (off > open && off > 1e-9) {
                        delta_robot *= open / off;
                    }
                }
            }
            const Eigen::Vector3d p_tcp_des = robot_ref_pos_ + delta_robot;
            const Eigen::Vector3d p_wrist_des = p_tcp_des - R_hand_des * config_.wrist_to_tcp;

            tip_quiet_ = delta_robot.norm() < std::max(1e-4, config_.position_deadband_m);
            solve_base_skeleton(q_meas, p_wrist_des, R_hand_des, dt);
            maybe_log("act");
        }
    } else if (panel_jogging) {
        // Clutch still armed but VR quiet/stale — panel can still drive the tip.
        apply_panel_jog_tick(dt);
        maybe_log("panel");
        const std::vector<double>& q_ref =
            (jrg_ != nullptr && jrg_->initialized()) ? jrg_->reference() : q_target_;
        fill_reference(current_joints, q_ref, joint_reference);
        return true;
    }
    // Stale / idle: keep last q_target_ (jrg continues to converge / hold).

    const std::vector<double>& q_ref = jrg_->Step(q_target_, dt);
    fill_reference(current_joints, q_ref, joint_reference);
    return true;
}

void HumanMimicController::maybe_log(const char* tag) {
    if (!config_.debug_log) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const double period = 1.0 / std::max(config_.debug_log_hz, 0.1);
    if (last_log_time_.time_since_epoch().count() != 0) {
        if (std::chrono::duration<double>(now - last_log_time_).count() < period) {
            return;
        }
    }
    last_log_time_ = now;
    ++log_tick_;

    // Per-arm joint-limit diagnostics.
    double min_margin = std::numeric_limits<double>::infinity();
    int min_joint = -1;
    int at_limit = 0;
    if (q_min_.size() >= arm_dof_ && q_max_.size() >= arm_dof_ && q_target_.size() >= arm_dof_) {
        for (size_t i = 0; i < arm_dof_; ++i) {
            const double m = std::min(q_target_[i] - q_min_[i], q_max_[i] - q_target_[i]);
            if (m < min_margin) {
                min_margin = m;
                min_joint = static_cast<int>(i);
            }
            if (m < 0.05) {
                ++at_limit;
            }
        }
    }

    std::cout << log_prefix_ << "[human-mimic][" << tag << "] #" << log_tick_
              << " ori=" << (config_.orientation_mapping_mode ==
                                     OrientationMappingMode::ABSOLUTE_CALIBRATED
                                 ? "abs"
                                 : "inc")
              << " calib=" << (ori_calibrated_ ? 1 : 0) << " reach=" << (dbg_reach_ ? 1 : 0)
              << " d=" << dbg_d_ << " d_req=" << dbg_d_req_ << " dmax=" << dbg_dmax_
              << " stretch=" << dbg_stretch_ << " reach_p=" << dbg_reach_progress_
              << " fwd_off=" << dbg_fwd_off_ << " up_off=" << dbg_up_off_
              << " ik_ok=" << (dbg_ik_ok_ ? 1 : 0) << " it=" << dbg_ik_iters_
              << " e_wrist=" << dbg_wrist_err_ << " e_ori=" << dbg_ori_err_
              << " e_elb=" << dbg_elbow_err_
              << " lim=" << min_margin << "@j" << min_joint << " at_lim=" << at_limit
              << " hold_prev=" << (dbg_hold_prev_ ? 1 : 0)
              << " phi_base=" << dbg_phi_base_ << " phi_cmd=" << elbow_cmd_phi_
              << " flex=" << dbg_elbow_flex_ << " flip=" << dbg_phi_flip_
              << " tip_y=" << tip_yield_ << " hint_ok=" << (elbow_hint_valid_ ? 1 : 0)
              << " phi_used=" << elbow_hint_phi_used_;
    if (q_target_.size() >= arm_dof_) {
        // j0=shoulder yaw (joint1), j1=shoulder pitch (joint2), j3=elbow, j5=wrist
        std::cout << " q0=" << q_target_[0] << " q1=" << q_target_[1] << " q3=" << q_target_[3]
                  << " q5=" << q_target_[5];
    }
    std::cout << " dq_ang=" << dbg_delta_ang_ << " vr_ax_world=[" << dbg_vr_axis_world_.transpose()
              << "] rob_ax=[" << dbg_rob_axis_.transpose() << "]" << std::endl;
}
