#include "controller/human_mimic_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
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
    const Eigen::Vector3d up_u = cfg.upward_dir.normalized();
    const Eigen::Vector3d prefer_raw =
        cfg.w_outward * out_u + cfg.w_upward * up_u + cfg.w_downward * down_u;

    const Eigen::Vector3d sw0 = wrist - shoulder;
    const double dn0 = sw0.norm();
    const double elev = (dn0 > 1e-6) ? std::clamp(sw0.z() / dn0, 0.0, 1.0) : 0.0;
    const double medial =
        (dn0 > 1e-6) ? std::clamp(-sw0.dot(out_u) / dn0, 0.0, 1.0) : 0.0;
    // When elevating, do NOT score toward gravity droop — that is what keeps the
    // arms pinned low with facing grippers / raised hands. Medial (inward) also
    // fades droop so a raised tuck is not pulled back down by gravity preference.
    const double droop_w =
        (1.0 - elev * std::max(0.0, cfg.elevation_droop_fade)) * (1.0 - medial);

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
            // Optional raised-elbow preference (off by default: w_upward=0).
            if (cfg.w_upward > 1e-6) {
                Eigen::Vector3d up_p = up_u - axis * axis.dot(up_u);
                if (up_p.norm() > 1e-6) {
                    score += 1.2 * cfg.w_upward * et.bend_dir.dot(up_p.normalized());
                }
            }
            // Optional anti-inward flare (off by default: anti_inward_boost=0).
            if (cfg.anti_inward_boost > 1e-6) {
                Eigen::Vector3d out_p = out_u - axis * axis.dot(out_u);
                const double medial =
                    (dn0 > 1e-6) ? std::clamp(-axis.dot(out_u), 0.0, 1.0) : 0.0;
                if (medial > 1e-3 && out_p.norm() > 1e-6) {
                    score += cfg.anti_inward_boost * medial *
                             et.bend_dir.dot(out_p.normalized());
                }
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
    {
        SmoothingConfig sc;
        sc.joint_output_lowpass_hz = std::max(1.0, config_.joint_output_lowpass_hz);
        sc.max_joint_output_velocity_radps =
            std::max(0.1, config_.max_joint_output_velocity_radps);
        sc.max_joint_output_accel_radps2 =
            std::max(0.1, config_.max_joint_output_accel_radps2);
        joint_out_filter_.set_config(std::move(sc));
    }

    initialized_ = true;

    // Open per-run act/settle log (default: file only; console opt-in).
    if (config_.debug_log) {
        debug_log_path_opened_ = config_.debug_log_path;
        if (debug_log_path_opened_.empty()) {
            const char* dir = std::getenv("OPENARM_HM_LOG_DIR");
            const std::string base =
                (dir != nullptr && dir[0] != '\0') ? std::string(dir) : "recordings/hm_logs";
            std::error_code ec;
            std::filesystem::create_directories(base, ec);
            const std::string side =
                (log_prefix_.find("left") != std::string::npos)
                    ? "left"
                    : ((log_prefix_.find("right") != std::string::npos) ? "right" : "arm");
            debug_log_path_opened_ = base + "/" + side + "_hm.log";
        } else {
            const auto parent = std::filesystem::path(debug_log_path_opened_).parent_path();
            if (!parent.empty()) {
                std::error_code ec;
                std::filesystem::create_directories(parent, ec);
            }
        }
        debug_log_file_.open(debug_log_path_opened_, std::ios::out | std::ios::app);
        if (!debug_log_file_.is_open()) {
            std::cerr << log_prefix_ << "[human-mimic] failed to open log file: "
                      << debug_log_path_opened_ << std::endl;
        }
    }

    const char* ik_backend =
        config_.ik_backend == HumanMimicIkBackend::DIFF_QP ? "diff_qp" : "lm";
    std::ostringstream ready;
    ready << log_prefix_ << "[human-mimic] ready (arm_dof=" << arm_dof_ << ", ori_map="
          << (config_.orientation_mapping_mode == OrientationMappingMode::ABSOLUTE_CALIBRATED
                  ? "absolute_calibrated"
                  : "incremental_clutch")
          << ", ik=" << ik_backend
          << ", elbow="
          << (config_.elbow_task_mode == ElbowTaskMode::PHI ? "phi" : "xyz")
          << ", tip_pri=" << (config_.tip_priority_enabled ? "on" : "off")
          << ", elbow_ns=" << (config_.elbow_nullspace_enabled ? "on" : "off")
          << ", lag_assist=" << (config_.wrist_lag_assist_enabled ? "on" : "off")
          << ", q_filt=" << (config_.joint_output_filter_enabled ? "on" : "off")
          << ")";
    ready << " tip IK frame=" << (config_.ik_track_tcp ? "TCP" : "wrist");
    if (debug_log_file_.is_open()) {
        ready << " log=" << debug_log_path_opened_;
    }
    std::cout << ready.str() << std::endl;
    if (debug_log_file_.is_open()) {
        debug_log_file_ << ready.str() << std::endl;
        debug_log_file_.flush();
    }
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
    // Anti-droop (default): blend toward last COMMANDED pose within a per-joint
    // cap so gravity sink between grips does not ratchet the anchor downward.
    // Pure measured seeding made every re-grip lock onto a lower pose (felt like
    // "press grip → keep falling").
    std::vector<double> q_anchor = q_meas;
    if (config_.reanchor_to_commanded) {
        std::vector<double> q_cmd;
        if (q_target_.size() >= arm_dof_ && all_finite(q_target_)) {
            q_cmd = q_target_;
        } else if (hold_active_ && hold_q_.size() >= arm_dof_ && all_finite(hold_q_)) {
            q_cmd = hold_q_;
        } else if (jrg_ != nullptr && jrg_->initialized()) {
            q_cmd = jrg_->reference();
        }
        if (q_cmd.size() >= arm_dof_ && all_finite(q_cmd)) {
            const double cap = std::max(0.0, config_.max_reanchor_correction_rad);
            for (size_t i = 0; i < arm_dof_ && i < q_anchor.size() && i < q_cmd.size(); ++i) {
                q_anchor[i] = std::clamp(q_cmd[i], q_meas[i] - cap, q_meas[i] + cap);
            }
        }
    }

    // Position is always re-anchored on grip (incremental clutch).
    hand_ref_pos_ = pose.position;
    hand_ref_rot_ = pose.orientation.normalized();
    clutch_idle_anchor_pos_ = pose.position;

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
    bool skel_pose_ok = false;
    if (kin_->ShoulderPos(q_anchor, &S) && kin_->ElbowPosJac(q_anchor, &E, &JpE) &&
        kin_->WristState(q_anchor, &p_wrist, &R_hand, &Jp, &Jw)) {
        skel_pose_ok = true;
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
                if (ik_ != nullptr) {
                    ik_->SetWristToTcp(offset);
                }
                wrist_tcp_calibrated_ = true;
                std::cout << log_prefix_ << "[human-mimic] wrist_to_tcp cal=["
                          << offset.transpose() << "]" << std::endl;
            }
        }
    }
    elbow_cmd_phi_valid_ = false;
    elbow_hint_used_valid_ = false;
    // Seed elbow φ from the clutch posture (continuous slew starts from here).
    // Legacy hard-lock path only arms when clutch_phi_hold_enabled (default OFF).
    if (skel_pose_ok) {
        double phi0 = 0.0;
        if (skeleton_->PhiFromPose(S, E, p_wrist, &phi0, nullptr) && std::isfinite(phi0)) {
            elbow_cmd_phi_ = phi0;
            elbow_cmd_phi_valid_ = true;
            clutch_phi_hold_ = config_.clutch_phi_hold_enabled;
        } else {
            clutch_phi_hold_ = false;
        }
    } else {
        clutch_phi_hold_ = false;
    }
    clutch_hand_depart_m_ = 0.0;
    dbg_d_prev_valid_ = false;
    tip_yield_ = 0.0;

    // Soft-reach: keep progress consistent with current stretch. Zeroing it on every
    // grip slammed the bent margin and pulled the tip inward at extreme poses.
    if (config_.clutch_seed_reach_progress && config_.reach_soft_enabled && skel_pose_ok &&
        skeleton_ != nullptr) {
        const double reach_full = skeleton_->L1() + skeleton_->L2();
        const double margin_full = std::max(1e-4, config_.reach_soft_margin_m);
        const double margin_bent =
            std::max(margin_full, config_.reach_soft_bent_margin_m);
        const double d_now = (p_wrist - S).norm();
        if (reach_full > 1e-3 && (margin_bent - margin_full) > 1e-6) {
            const double margin_des =
                std::clamp(reach_full - d_now, margin_full, margin_bent);
            reach_progress_filt_ =
                std::clamp((margin_bent - margin_des) / (margin_bent - margin_full), 0.0,
                           1.0);
            reach_progress_filt_valid_ = true;
        } else {
            reach_progress_filt_ = 1.0;
            reach_progress_filt_valid_ = true;
        }
    } else {
        reach_progress_filt_ = 0.0;
        reach_progress_filt_valid_ = false;
    }

    std::vector<double> q_arm_anchor(arm_dof_, 0.0);
    for (size_t i = 0; i < arm_dof_ && i < q_anchor.size(); ++i) {
        q_arm_anchor[i] = q_anchor[i];
    }
    q_target_ = q_arm_anchor;
    reset_joint_stream(q_arm_anchor);
    clutch_settle_remaining_ = std::max(0, config_.clutch_settle_ticks);
    clutch_track_settle_remaining_ = 0;
    clutch_idle_ = true;
    clutch_active_ = true;
    hold_active_ = false;
    solve_fail_count_ = 0;

    std::cout << log_prefix_ << "[human-mimic] clutch anchored (from "
              << (config_.reanchor_to_commanded ? "cmd±cap" : "measured")
              << " L1=" << skeleton_->L1() << " L2=" << skeleton_->L2()
              << " reach_p=" << reach_progress_filt_
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

    tip_quiet_ = false;
    hold_active_ = false;  // leave frozen hold so no-grip nudge can stream out
    jog_active_ = true;
    jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);

    const int iters = std::max(1, config_.jog_chase_iters);
    const double dt = std::max(0.005, config_.jog_chase_dt);
    const double tol = std::max(1e-4, config_.jog_wrist_tol_m);
    for (int i = 0; i < iters; ++i) {
        tip_quiet_ = false;
        if (!solve_base_skeleton(q_target_, p_tcp_des, R_tcp, dt)) {
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
        panel_origin_ = p0;
        panel_R_des_ = R0;
        panel_axis_mask_ = Eigen::Vector3d::Zero();
        panel_tcp_des_valid_ = true;
        panel_speed_filt_ = 0.0;
        hold_active_ = false;
    }

    hold_active_ = false;
    tip_quiet_ = false;

    // Sticky axis mask from commanded velocity: up→Z only, left→Y only, etc.
    // Prevents IK compromise from walking the setpoint off-axis (斜向上).
    for (int i = 0; i < 3; ++i) {
        if (std::abs(panel_jog_vel_[i]) > 1e-9) {
            panel_axis_mask_[i] = 1.0;
        }
    }

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
    // Re-project onto allowed body axes through the hold origin.
    {
        Eigen::Vector3d d = panel_tcp_des_ - panel_origin_;
        d = d.cwiseProduct(panel_axis_mask_);
        panel_tcp_des_ = panel_origin_ + d;
    }
    if (clutch_active_) {
        robot_ref_pos_ += dp.cwiseProduct(panel_axis_mask_);
    } else {
        robot_ref_pos_ = panel_tcp_des_;
    }

    // Cartesian chase: tip-first (continuous_tip_drive_) along body axes.
    continuous_tip_drive_ = true;
    jog_active_ = false;
    jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);
    const int iters = std::max(1, config_.jog_chase_iters);
    const double tol = std::max(1e-4, config_.jog_wrist_tol_m);
    for (int i = 0; i < iters; ++i) {
        tip_quiet_ = false;
        if (!solve_base_skeleton(q_target_, panel_tcp_des_, panel_R_des_, dt)) {
            break;
        }
        if (dbg_wrist_err_ <= tol) {
            break;
        }
    }
    continuous_tip_drive_ = false;

    // Slewed q_target_ is already continuous — Reset removes the second JRG lag layer.
    if (jrg_ != nullptr) {
        reset_joint_stream(q_target_);
    }
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

void HumanMimicController::set_couple_follow(bool on) {
    if (on == couple_follow_active_) {
        return;
    }
    couple_follow_active_ = on;
    couple_phi_lock_ = on;
    if (!on) {
        continuous_tip_drive_ = false;
        return;
    }
    // Seed continuity from the commanded joint stream so the first follow tick
    // does not re-solve onto an antipodal elbow branch.
    if (jrg_ != nullptr && jrg_->initialized()) {
        q_target_ = jrg_->reference();
    }
    if (q_target_.size() < arm_dof_ || kin_ == nullptr || skeleton_ == nullptr) {
        return;
    }
    Eigen::Vector3d S = Eigen::Vector3d::Zero();
    Eigen::Vector3d E = Eigen::Vector3d::Zero();
    Eigen::Vector3d W = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_tmp;
    Eigen::MatrixXd JpE, JpW, JwW;
    if (!kin_->ShoulderPos(q_target_, &S) || !S.allFinite()) {
        return;
    }
    if (!kin_->ElbowPosJac(q_target_, &E, &JpE) || !E.allFinite()) {
        return;
    }
    if (!kin_->WristState(q_target_, &W, &R_tmp, &JpW, &JwW) || !W.allFinite()) {
        return;
    }
    double phi0 = 0.0;
    if (skeleton_->PhiFromPose(S, E, W, &phi0, nullptr) && std::isfinite(phi0)) {
        elbow_cmd_phi_ = phi0;
        elbow_cmd_phi_valid_ = true;
    }
    // Prefer soft-reach near full so couple translation is not compressed by a
    // stale bent-margin progress from a prior VR clutch.
    reach_progress_filt_ = 1.0;
    reach_progress_filt_valid_ = true;
    Eigen::Vector3d sw = W - S;
    const double d_seed = sw.norm();
    if (d_seed > 1e-6) {
        const Eigen::Vector3d n = sw / d_seed;
        Eigen::Vector3d bend = E - S;
        bend = bend - n * n.dot(bend);
        if (bend.norm() > 1e-6) {
            skeleton_->SetContinuityBend(bend.normalized());
        }
    }
    std::cout << log_prefix_ << "[human-mimic] couple-follow ON  phi_lock=" << elbow_cmd_phi_
              << std::endl;
}

void HumanMimicController::reanchor_vr_map(const std::vector<double>& q_meas,
                                           const QuestPoseSample& pose) {
    if (!pose.valid || !pose.position.allFinite()) {
        // Still unlock-safe: seed robot_ref from commanded joints if possible.
        Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
        Eigen::Matrix3d R_tcp = Eigen::Matrix3d::Identity();
        if (commanded_tcp_pose(&p_tcp, &R_tcp) && p_tcp.allFinite() && R_tcp.allFinite()) {
            robot_ref_pos_ = p_tcp;
            robot_ref_rot_ = Eigen::Quaterniond(R_tcp).normalized();
        }
        return;
    }
    // Reuse clutch re-anchor (commanded-biased q + absorb hand_ref to current sample).
    activate_clutch(q_meas, pose);
    // Short absorb window so the first post-couple VR ticks stay near zero delta.
    clutch_idle_ = true;
    clutch_track_settle_remaining_ = std::max(clutch_track_settle_remaining_, 4);
    if (!pose.grip_active) {
        // Refs absorbed; do not leave clutch tracking without a grip.
        clutch_active_ = false;
    }
    std::cout << log_prefix_ << "[human-mimic] VR map re-anchored after couple/circle exit"
              << " clutch=" << (clutch_active_ ? 1 : 0) << std::endl;
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
        // Keep left seeded from the streamed reference (continuity > snap).
        q_target_ = jrg_->reference();
    }
    hold_active_ = false;
    tip_quiet_ = false;
    robot_ref_pos_ = p_tcp;
    if (couple_follow_active_) {
        // Continuous tip drive: chase TCP with joint slew — never snap IK.
        continuous_tip_drive_ = true;
        jog_active_ = false;
        jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);
        reach_progress_filt_ = 1.0;
        reach_progress_filt_valid_ = true;
        int iters = 8;
        if (const char* e = std::getenv("OPENARM_COUPLE_CHASE_ITERS")) {
            try {
                iters = std::max(1, std::stoi(e));
            } catch (...) {
            }
        }
        const double tol = std::max(1e-4, config_.jog_wrist_tol_m);
        for (int i = 0; i < iters; ++i) {
            tip_quiet_ = false;
            if (!solve_base_skeleton(q_target_, p_tcp, R_hand, dt)) {
                break;
            }
            if (dbg_wrist_err_ <= tol) {
                break;
            }
        }
        continuous_tip_drive_ = false;
        return;
    }

    jog_active_ = true;
    jog_bypass_remaining_ = std::max(1, config_.jog_bypass_hold_ticks);
    solve_base_skeleton(q_target_, p_tcp, R_hand, dt);
    jog_active_ = false;
}

void HumanMimicController::deactivate_clutch() {
    clutch_active_ = false;
    last_trigger_reset_ = false;
    last_recenter_ = false;
    ori_filt_valid_ = false;
    elbow_cmd_phi_valid_ = false;
    clutch_phi_hold_ = false;
    clutch_hand_depart_m_ = 0.0;
    clutch_track_settle_remaining_ = 0;
    elbow_raise_hold_ = false;
}

void HumanMimicController::reset_joint_stream(const std::vector<double>& q) {
    if (jrg_ != nullptr) {
        jrg_->Reset(q);
    }
    joint_out_filter_.reset(q);
    q_out_filt_ = q;
    dq_out_filt_.assign(q.size(), 0.0);
}

const std::vector<double>& HumanMimicController::advance_joint_stream(
    const std::vector<double>& q_target, double dt, double k_scale, double qdot_scale) {
    if (jrg_ == nullptr) {
        q_out_filt_ = q_target;
        return q_out_filt_;
    }
    const std::vector<double>& q_jrg = jrg_->Step(q_target, dt, k_scale, qdot_scale);
    if (!config_.joint_output_filter_enabled) {
        return q_jrg;
    }
    joint_out_filter_.step(dt, q_jrg, &q_out_filt_, &dq_out_filt_);
    return q_out_filt_;
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
        gripper_stall_q_ref_ = q_meas;
        gripper_grasp_lag_ = 0.0;
    }

    // Clamp a hold angle onto the open..closed stroke.
    auto clamp_stroke = [&](double q) {
        if (config_.gripper_closed >= config_.gripper_open) {
            return std::clamp(q, config_.gripper_open, config_.gripper_closed);
        }
        return std::clamp(q, config_.gripper_closed, config_.gripper_open);
    };
    const double hold_lag_nom = std::max(0.0, config_.gripper_grasp_hold_lag) +
                                std::max(0.0, config_.gripper_hold_preload);
    const double hold_lag_max =
        std::max(hold_lag_nom, std::max(0.0, config_.gripper_grasp_hold_lag_max));

    // Relative MIT spring: q_cmd = q_meas + close_dir * lag  →  τ ≈ Kp · lag.
    auto apply_grasp_cmd = [&](double lag) {
        gripper_grasp_lag_ = std::clamp(lag, 0.0, hold_lag_max);
        gripper_hold_pos_ = clamp_stroke(q_meas + close_dir * gripper_grasp_lag_);
        gripper_target_ = gripper_hold_pos_;
    };
    auto enter_grasp_hold = [&](const char* reason, double lag) {
        gripper_motion_ = GripperMotion::GraspHolding;
        apply_grasp_cmd(lag);
        gripper_stall_timer_ = 0.0;
        gripper_stall_q_ref_ = q_meas;
        std::cout << log_prefix_ << "[human-mimic] gripper grasp-hold (" << reason
                  << ") q_meas=" << q_meas << " lag=" << gripper_grasp_lag_
                  << " q_cmd=" << gripper_target_ << " (B/Y opens to release)" << std::endl;
    };
    // Freeze while closing (no grasp-hold): keep close-side command but cap spring lag.
    auto freeze_close_hold = [&](double prefer_cmd) {
        double hold = (close_dir >= 0.0) ? std::max(q_meas, prefer_cmd)
                                         : std::min(q_meas, prefer_cmd);
        hold += close_dir * std::max(0.0, config_.gripper_hold_preload);
        const double lag = close_dir * (hold - q_meas);
        if (lag > hold_lag_max) {
            hold = q_meas + close_dir * hold_lag_max;
        }
        hold = clamp_stroke(hold);
        gripper_hold_pos_ = hold;
        gripper_target_ = hold;
        gripper_grasp_lag_ = 0.0;
    };
    auto freeze_open_hold = [&]() {
        gripper_hold_pos_ = clamp_stroke(q_meas);
        gripper_target_ = gripper_hold_pos_;
        gripper_grasp_lag_ = 0.0;
    };

    // Auto-stop at the physical open/closed limits.
    if (gripper_motion_ == GripperMotion::Opening && q_meas <= config_.gripper_open + kTol) {
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_pos_ = config_.gripper_open;
        gripper_target_ = config_.gripper_open;
        gripper_stall_timer_ = 0.0;
        gripper_grasp_lag_ = 0.0;
    } else if (gripper_motion_ == GripperMotion::Closing &&
               q_meas >= config_.gripper_closed - kTol) {
        gripper_motion_ = GripperMotion::Stopped;
        gripper_hold_pos_ = config_.gripper_closed;
        gripper_target_ = config_.gripper_closed;
        gripper_stall_timer_ = 0.0;
        gripper_grasp_lag_ = 0.0;
    }

    // Button sample may be stale for one tick; prefer last_* after update below.
    bool holding_close = false;
    bool holding_open = false;

    if (sample.valid) {
        last_button_close_ = sample.button_close;
        last_button_open_ = sample.button_open;

        // Hold-to-move: A/X held → close, B/Y held → open.
        // GraspHolding persists until open (releasing A does NOT release the clamp).
        const bool want_close = sample.button_close && !sample.button_open;
        const bool want_open = sample.button_open && !sample.button_close;
        holding_close = want_close;
        holding_open = want_open;
        const GripperMotion prev = gripper_motion_;

        if (want_open) {
            gripper_grasp_lag_ = 0.0;
            if (std::abs(q_meas - config_.gripper_open) > kTol) {
                if (prev != GripperMotion::Opening) {
                    gripper_motion_ = GripperMotion::Opening;
                    gripper_target_ = q_meas;
                    gripper_stall_timer_ = 0.0;
                    gripper_stall_q_ref_ = q_meas;
                    std::cout << log_prefix_ << "[human-mimic] gripper opening (hold B/Y)"
                              << (prev == GripperMotion::GraspHolding ? " [release grasp-hold]"
                                                                      : "")
                              << std::endl;
                }
            } else {
                gripper_motion_ = GripperMotion::Stopped;
                gripper_hold_pos_ = config_.gripper_open;
                gripper_target_ = config_.gripper_open;
                gripper_stall_timer_ = 0.0;
            }
        } else if (want_close) {
            if (prev == GripperMotion::GraspHolding) {
                // Still squeezing: nudge lag up a little (capped), do not resume full close.
                apply_grasp_cmd(std::min(hold_lag_max, gripper_grasp_lag_ + close_speed * dt));
            } else if (std::abs(q_meas - config_.gripper_closed) > kTol) {
                if (prev != GripperMotion::Closing) {
                    gripper_motion_ = GripperMotion::Closing;
                    gripper_target_ = q_meas;
                    gripper_stall_timer_ = 0.0;
                    gripper_stall_q_ref_ = q_meas;
                    gripper_grasp_lag_ = 0.0;
                    std::cout << log_prefix_ << "[human-mimic] gripper closing (hold A/X)"
                              << std::endl;
                }
            } else {
                gripper_motion_ = GripperMotion::Stopped;
                gripper_hold_pos_ = config_.gripper_closed;
                gripper_target_ = config_.gripper_closed;
                gripper_stall_timer_ = 0.0;
                gripper_grasp_lag_ = 0.0;
            }
        } else {
            // Released A/X: stop Closing/Opening — but GraspHolding keeps clamping until B/Y.
            if (prev == GripperMotion::GraspHolding) {
                apply_grasp_cmd(gripper_grasp_lag_ > 0.0 ? gripper_grasp_lag_ : hold_lag_nom);
            } else if (prev == GripperMotion::Closing) {
                gripper_motion_ = GripperMotion::Stopped;
                freeze_close_hold(gripper_target_);
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_ << "[human-mimic] gripper hold (release A/X) q="
                          << gripper_hold_pos_ << std::endl;
            } else if (prev == GripperMotion::Opening) {
                gripper_motion_ = GripperMotion::Stopped;
                freeze_open_hold();
                gripper_stall_timer_ = 0.0;
                std::cout << log_prefix_ << "[human-mimic] gripper hold (release B/Y) q="
                          << gripper_hold_pos_ << std::endl;
            }
        }
    } else {
        holding_close = last_button_close_ && !last_button_open_;
        holding_open = last_button_open_ && !last_button_close_;
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
        case GripperMotion::GraspHolding:
            apply_grasp_cmd(gripper_grasp_lag_ > 0.0 ? gripper_grasp_lag_ : hold_lag_nom);
            gripper_stall_timer_ = 0.0;
            break;
    }

    // Hold A/X + measured stroke not advancing toward closed → grasp-hold.
    // Progress gate (primary): must move ≥ stall_progress along close within stall_time.
    // Velocity is only an assist; slow creep against an object still counts as jammed.
    if (config_.gripper_soft_stall && config_.gripper_grasp_hold &&
        gripper_motion_ == GripperMotion::Closing && holding_close && !holding_open) {
        const double progress = close_dir * (q_meas - gripper_stall_q_ref_);
        const double progress_need = std::max(0.0, config_.gripper_stall_progress);
        if (progress >= progress_need) {
            gripper_stall_q_ref_ = q_meas;
            gripper_stall_timer_ = 0.0;
        } else {
            gripper_stall_timer_ += dt;
            if (gripper_stall_timer_ >= config_.gripper_stall_time) {
                const double lag_close = close_dir * (gripper_target_ - q_meas);
                const double seeded = std::clamp(
                    std::max(lag_close, hold_lag_nom), hold_lag_nom * 0.5, hold_lag_max);
                enter_grasp_hold("hold-A jammed (no close progress)", seeded);
            }
        }
    } else if (gripper_motion_ != GripperMotion::Closing) {
        gripper_stall_timer_ = 0.0;
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
        // Anti-droop between grips: freeze the last COMMANDED stream (JRG), not the
        // possibly gravity-sagged measurement. Falling back to q_hold only at start.
        if (jrg_ != nullptr && jrg_->initialized()) {
            hold_q_ = jrg_->reference();
        } else if (q_target_.size() >= arm_dof_ && all_finite(q_target_)) {
            hold_q_ = q_target_;
        } else {
            hold_q_ = q_hold;
        }
        if (hold_q_.size() > arm_dof_) {
            hold_q_.resize(arm_dof_);
        }
        while (hold_q_.size() < arm_dof_) {
            hold_q_.push_back(0.0);
        }
        reset_joint_stream(hold_q_);
        q_target_ = hold_q_;
        hold_active_ = true;
    }
    fill_reference(current_joints, hold_q_, reference);
}

bool HumanMimicController::solve_base_skeleton(const std::vector<double>& q_meas,
                                              const Eigen::Vector3d& p_tcp_des,
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
    const Eigen::Vector3d p_wrist_des =
        p_tcp_des - R_hand_des * config_.wrist_to_tcp;
    const double reach_full = skeleton_->L1() + skeleton_->L2();
    Eigen::Vector3d p_wrist_use = p_wrist_des;
    Eigen::Vector3d r_sw = p_wrist_des - S_now;
    const double d_req = r_sw.norm();
    dbg_d_req_ = d_req;
    double reach_progress = 0.0;
    const bool bypass_panel_soft =
        config_.panel_bypass_soft_reach && panel_stream_active();
    if (config_.reach_soft_enabled && !bypass_panel_soft && d_req > 1e-6 &&
        reach_full > 1e-3) {
        const double margin_full = std::max(1e-4, config_.reach_soft_margin_m);
        double margin = margin_full;
        if (config_.reach_soft_progress_enabled) {
            const double down_lim = std::max(0.05, config_.max_teleop_offset_down_m);
            const double fwd_span =
                std::max(0.05, config_.reach_soft_forward_span_m);
            // up_off / fwd_off set in step() before this solve (TCP offset from clutch).
            double p_down = 0.0;
            if (dbg_up_off_ < 0.0) {
                p_down = std::clamp(-dbg_up_off_ / down_lim, 0.0, 1.0);
            }
            // Down: smoothstep (stay bent early). Forward: sqrt ease so mid-reach
            // already opens the ceiling (workspace fwd cap is too long for this).
            const double p_down_s = p_down * p_down * (3.0 - 2.0 * p_down);
            double p_fwd = 0.0;
            if (config_.reach_soft_forward_progress_enabled && dbg_fwd_off_ > 0.0) {
                p_fwd = std::clamp(dbg_fwd_off_ / fwd_span, 0.0, 1.0);
                p_fwd = std::sqrt(p_fwd);
            }
            const double p_raw = std::max(p_down_s, p_fwd);
            // Slew progress so VR jitter / quick retract cannot slam d_star.
            // Allow slightly faster rise than fall so forward straighten responds.
            if (reach_progress_filt_valid_) {
                const double slew = std::max(0.0, config_.reach_progress_slew_per_s);
                const double max_up = 1.6 * slew * std::max(1e-4, solve_dt);
                const double max_dn = slew * std::max(1e-4, solve_dt);
                const double err = p_raw - reach_progress_filt_;
                reach_progress =
                    reach_progress_filt_ +
                    std::clamp(err, -max_dn, max_up);
            } else {
                reach_progress = p_raw;
            }
            reach_progress_filt_ = reach_progress;
            reach_progress_filt_valid_ = true;
            const double margin_bent =
                std::max(margin_full, config_.reach_soft_bent_margin_m);
            margin = margin_bent + reach_progress * (margin_full - margin_bent);
        } else {
            // Progress gate off: fixed near-straight ceiling (no mid-travel bend hold).
            reach_progress = 1.0;
            reach_progress_filt_ = 1.0;
            reach_progress_filt_valid_ = true;
        }
        const double d_star = std::max(1e-3, reach_full - margin);

        // Hard ceiling at d_star (moving when progress is on). Raising lowers
        // progress → compresses again → bend returns. No exponential asymptote.
        const double d_out = std::min(d_req, d_star);
        p_wrist_use = S_now + (r_sw / d_req) * d_out;
    }
    dbg_reach_progress_ = reach_progress;

    Eigen::Vector3d wrist_for_ik = p_wrist_use;
    Eigen::Vector3d elbow_des = Eigen::Vector3d::Zero();
    double elbow_w_scale = 0.0;
    double ori_w_scale = 1.0;
    double cont_w_scale = 1.0;
    double nom_w_scale = 1.0;
    double elbow_phi_for_ik = 0.0;
    const VirtualArmSkeleton* skel_for_ik = nullptr;
    double stretch = 0.0;
    const bool couple_mode = couple_follow_active_;
    const bool continuous_drive = continuous_tip_drive_ || couple_mode;
    // Tip-priority IK weights (skip elbow-straighten fights) without implying joint snap.
    const bool jog_mode = jog_active_ || jog_bypass_remaining_ > 0 || continuous_drive;
    // Distal hard-sat flag (q5 against limit): used by ori weights + q4–q6 slew.
    bool wrist_hard_sat = false;
    if (jog_mode) {
        // Panel jog / couple owns the tip — do not treat the hand as "quiet" and freeze.
        tip_quiet_ = false;
    }
    if (couple_mode) {
        // Avoid stale bent soft-reach compressing the locked relative offset.
        reach_progress_filt_ = 1.0;
        reach_progress_filt_valid_ = true;
    }

    if (!config_.use_elbow_skeleton) {
        // Pika-style / 6D-only: no virtual elbow, force elbow task weight to 0.
        dbg_reach_ = true;
        dbg_phi_base_ = 0.0;
        dbg_d_ = (p_wrist_use - S_now).norm();
        dbg_dmax_ = skeleton_->L1() + skeleton_->L2() - config_.reach_margin_m;
        dbg_stretch_ = 0.0;
        tip_yield_ = 0.0;
        dbg_elbow_flex_ = 0.0;
        dbg_phi_flip_ = 0;
    } else {
    // Seed circle continuity from the REALIZED elbow, not last commanded bend.
    // Commanded SetContinuityBend was self-locking an inward branch on the arc.
    Eigen::Vector3d E_fk_seed = Eigen::Vector3d::Zero();
    Eigen::MatrixXd JpE_seed;
    if (kin_->ElbowPosJac(q_meas, &E_fk_seed, &JpE_seed) && E_fk_seed.allFinite()) {
        Eigen::Vector3d sw_seed = p_wrist_use - S_now;
        const double d_seed = sw_seed.norm();
        if (d_seed > 1e-6) {
            const Eigen::Vector3d n_seed = sw_seed / d_seed;
            Eigen::Vector3d bend_fk = E_fk_seed - S_now;
            bend_fk = bend_fk - n_seed * n_seed.dot(bend_fk);
            if (bend_fk.norm() > 1e-6) {
                skeleton_->SetContinuityBend(bend_fk.normalized());
            }
        }
    }
    const SkeletonResult skel = skeleton_->Solve(S_now, p_wrist_use,
                                                  /*advance_continuity=*/false);
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
    if (d_star_stretch > 1e-3 && d_cmd > 1e-6) {
        const double start_frac = std::clamp(config_.extension_yield_start, 0.0, 0.99);
        const double start_d = start_frac * d_star_stretch;
        const double span = std::max(1e-6, d_star_stretch - start_d);
        stretch = std::clamp((d_cmd - start_d) / span, 0.0, 1.0);
    }
    dbg_stretch_ = stretch;

    // Tip-priority yield amount (used later for IK weights + shoulder slew).
    double tip_raw = 0.0;
    if (!jog_mode && config_.tip_priority_enabled) {
        // Soft-reach already moved the wrist target inward. Do not also kill the
        // elbow just because the human asked for more reach — that chronically
        // saturates tip_y while e_wrist is tiny and frees a sudden proximal drop.
        const double soft_scale =
            std::clamp(config_.tip_priority_soft_reach_scale, 0.0, 1.0);
        if (soft_scale > 1e-9 && d_req > 1e-6) {
            tip_raw = std::max(
                tip_raw,
                soft_scale * std::clamp((d_req - d_cmd) /
                                            std::max(1e-3, config_.reach_soft_zone_m),
                                        0.0, 1.0));
        }
        const double werr0 = std::max(0.0, config_.tip_priority_wrist_err_m);
        if (dbg_wrist_err_ > werr0) {
            tip_raw = std::max(
                tip_raw, std::clamp((dbg_wrist_err_ - werr0) / 0.05, 0.0, 1.0));
        }
    }
    {
        const double max_dy =
            std::max(0.0, config_.tip_yield_slew_per_s) * std::max(1e-4, solve_dt);
        // Fall a bit faster than rise so recovering tracking restores elbow promptly.
        const double lo = -2.0 * max_dy;
        const double hi = max_dy;
        tip_yield_ = std::clamp(tip_yield_ + std::clamp(tip_raw - tip_yield_, lo, hi), 0.0, 1.0);
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

    // Couple follow: freeze elbow branch at engage. Reject antipodal flips and only
    // allow a slow continuous slew so the slave never pops to a mirrored silhouette.
    if (couple_phi_lock_ && elbow_cmd_phi_valid_) {
        if (std::abs(wrap_to_pi(phi_goal - elbow_cmd_phi_)) > 0.5 * M_PI) {
            phi_goal = wrap_to_pi(phi_goal + M_PI);
        }
        const double max_phi_step =
            std::max(0.15, config_.elbow_cmd_slew_radps) * std::max(1e-4, solve_dt);
        const double step = wrap_to_pi(phi_goal - elbow_cmd_phi_);
        phi_goal = wrap_to_pi(elbow_cmd_phi_ + std::clamp(step, -max_phi_step, max_phi_step));
        dbg_phi_flip_ = 0;
    }

    // Continuous path policy: after clutch seed, slew elbow_cmd_phi_ toward skeleton
    // φ_des every tick (elbow_cmd_slew_*). Optional LEGACY hard lock (clutch_phi_hold)
    // freezes φ until unlock distance then blends — that step causes 肘突变; default OFF.
    if (config_.clutch_phi_hold_enabled && clutch_phi_hold_ && elbow_cmd_phi_valid_) {
        const double unlock = std::max(0.0, config_.clutch_phi_unlock_m);
        const double span = std::max(1e-3, config_.clutch_phi_blend_span_m);
        const double depart = clutch_hand_depart_m_;
        if (depart < unlock) {
            phi_goal = elbow_cmd_phi_;
        } else {
            const double w = std::clamp((depart - unlock) / span, 0.0, 1.0);
            const double step = wrap_to_pi(phi_goal - elbow_cmd_phi_);
            phi_goal = wrap_to_pi(elbow_cmd_phi_ + w * step);
            if (w >= 1.0 - 1e-6) {
                clutch_phi_hold_ = false;
            }
        }
    }

    // Medial amount (used for φ blend + later elbow yield).
    double medial_amount = 0.0;
    {
        const Eigen::Vector3d sw_m = skel.wrist_clamped - S_now;
        const double dn_m = sw_m.norm();
        if (dn_m > 1e-6) {
            medial_amount =
                std::clamp(-sw_m.dot(config_.outward_dir.normalized()) / dn_m, 0.0, 1.0);
        }
    }

    // Near-body raised-elbow hysteresis: once tucked/raised, hold until d clearly
    // leaves the band so slight outward moves do not thrash / dump the elbow.
    if (config_.elbow_raise_hold_enabled && !jog_mode) {
        const double d_enter = std::max(0.05, config_.elbow_raise_hold_d_m);
        const double d_exit =
            std::max(d_enter + 0.02, config_.elbow_raise_hold_release_d_m);
        const bool q3_raised =
            q_target_.size() > 3 &&
            q_target_[3] >= std::max(0.5, config_.elbow_raise_hold_q3_rad);
        const bool medial_ok =
            medial_amount >= std::max(0.0, config_.elbow_raise_hold_medial_min);
        if (!elbow_raise_hold_) {
            if (medial_ok && (dbg_d_ <= d_enter || q3_raised)) {
                elbow_raise_hold_ = true;
            }
        } else if (dbg_d_ >= d_exit) {
            elbow_raise_hold_ = false;
        }
    } else if (jog_mode) {
        // Panel/couple continuous chase: do not sticky-hold across modes.
        // (couple still benefits from asymmetric q3 slew + yield-freeze below.)
    }

    // Near straight: freeze commanded phi. The swivel circle collapses and any
    // small phi / HybrIK jitter turns into a visible "elbow spin" around S→W.
    const bool phi_frozen =
        stretch >= std::clamp(config_.extension_phi_freeze_stretch, 0.0, 1.0) ||
        skel.h_radius < std::max(1e-4, config_.elbow_singularity_h_m);

    // Distal wrist margin (j4–j6): used to escape φ freeze when attitude fights a limit.
    double wrist_distal_margin = std::numeric_limits<double>::infinity();
    if (q_meas.size() >= arm_dof_ && q_min_.size() >= arm_dof_ && q_max_.size() >= arm_dof_) {
        const int j0 = std::clamp(config_.wrist_limit_ori_joint_lo, 0,
                                  static_cast<int>(arm_dof_) - 1);
        const int j1 = std::clamp(config_.wrist_limit_ori_joint_hi, j0,
                                  static_cast<int>(arm_dof_) - 1);
        for (int j = j0; j <= j1; ++j) {
            const double qj = q_meas[static_cast<size_t>(j)];
            const double m =
                std::min(qj - q_min_[static_cast<size_t>(j)], q_max_[static_cast<size_t>(j)] - qj);
            wrist_distal_margin = std::min(wrist_distal_margin, m);
        }
    }
    dbg_wrist_margin_ = std::isfinite(wrist_distal_margin) ? wrist_distal_margin : 0.0;
    const double wrist_m0 = std::max(1e-4, config_.wrist_limit_ori_margin_rad);
    const bool wrist_near_lim =
        std::isfinite(wrist_distal_margin) && wrist_distal_margin < wrist_m0;
    const double wrist_sat_m0 = std::max(1e-4, config_.wrist_sat_anti_hop_margin_rad);
    wrist_hard_sat =
        config_.wrist_sat_anti_hop_enabled && !jog_mode &&
        std::isfinite(wrist_distal_margin) && wrist_distal_margin < wrist_sat_m0;
    // Wrist jammed + attitude error: allow φ slew so proximal can free j5.
    // Do NOT chase droop — droop slew scale below still applies.
    const bool wrist_ori_escape =
        config_.wrist_escape_phi_enabled && !jog_mode && !couple_mode && wrist_near_lim &&
        dbg_ori_err_ >= std::max(1e-4, config_.wrist_escape_ori_err_rad);
    dbg_wrist_escape_ = wrist_ori_escape;

    // Tip/medial yield about to soften elbow: freeze φ so we do not retarget a
    // drooped silhouette that then fights the tip for proximal DOF.
    // Wrist-escape overrides: frozen φ was trapping attitude against j5 (log 100935).
    const bool freeze_phi_for_tip =
        config_.elbow_freeze_phi_on_yield && !jog_mode && !couple_mode && !wrist_ori_escape &&
        (medial_amount >= std::max(0.0, config_.elbow_freeze_phi_medial) ||
         (config_.ori_stress_elbow_yield_enabled &&
          dbg_ori_err_ > std::max(1e-4, config_.ori_stress_elbow_err_rad)));

    // Medial tip: skeleton droop pick often tucks the elbow; blend φ toward FK so
    // the arc preference cannot permanently pin an inward branch. While raise-hold
    // is armed, keep this blend tiny so a raised elbow stays raised.
    if (config_.medial_elbow_yield_enabled && !jog_mode && medial_amount > 1e-3 &&
        !phi_frozen && !freeze_phi_for_tip) {
        Eigen::Vector3d E_fk_m = Eigen::Vector3d::Zero();
        Eigen::MatrixXd Jp_m;
        double phi_fk_m = 0.0;
        double h_m = 0.0;
        if (kin_->ElbowPosJac(q_meas, &E_fk_m, &Jp_m) &&
            skeleton_->PhiFromPose(S_now, E_fk_m, skel.wrist_clamped, &phi_fk_m, &h_m) &&
            h_m > 1e-4) {
            double blend = std::clamp(config_.medial_phi_fk_blend, 0.0, 1.0);
            if (elbow_raise_hold_ && !wrist_ori_escape) {
                blend *= std::clamp(config_.elbow_raise_hold_medial_fk_scale, 0.0, 1.0);
            }
            const double a = medial_amount * blend;
            const double step = wrap_to_pi(phi_fk_m - phi_goal);
            phi_goal = wrap_to_pi(phi_goal + a * step);
        }
    }

    if (!elbow_cmd_phi_valid_ || skel.h_radius < 1e-3) {
        elbow_cmd_phi_ = phi_goal;
        elbow_cmd_phi_valid_ = true;
    } else if (phi_frozen || freeze_phi_for_tip) {
        // Keep last commanded phi — do not chase singular / tip-yield observations.
        // Unreachable / near-straight / tip-priority: stay on continuous path end.
    } else if (config_.clutch_phi_hold_enabled && clutch_phi_hold_ &&
               clutch_hand_depart_m_ < std::max(0.0, config_.clutch_phi_unlock_m)) {
        // Legacy fine-motion hard lock (disabled by default).
        elbow_cmd_phi_ = phi_goal;
    } else if (config_.elbow_cmd_slew_enabled) {
        // Continuous participation: rate-limit toward skeleton/hint φ_des.
        double max_step = std::max(0.0, config_.elbow_cmd_slew_radps) * std::max(1e-4, solve_dt);
        if (hint_applied) {
            max_step = std::min(max_step, std::max(0.0, config_.elbow_hint_slew_radps) *
                                              std::max(1e-4, solve_dt));
        }
        max_step = std::min(max_step, std::max(1e-4, config_.elbow_cmd_max_step_rad));
        // Raised-hold / near-body: slow φ steps that lower the elbow (more droop).
        // Wrist-escape still slows droop so freeing j5 does not dump the elbow.
        if (elbow_raise_hold_ || dbg_d_ < std::max(0.05, config_.elbow_raise_hold_d_m)) {
            const ElbowTarget et_cur = skeleton_->compute_elbow_target_from_phi(
                S_now, skel.wrist_clamped, skeleton_->L1(), skeleton_->L2(), elbow_cmd_phi_);
            const ElbowTarget et_goal = skeleton_->compute_elbow_target_from_phi(
                S_now, skel.wrist_clamped, skeleton_->L1(), skeleton_->L2(), phi_goal);
            if (!et_cur.degenerate && !et_goal.degenerate && et_cur.bend_dir.norm() > 1e-6 &&
                et_goal.bend_dir.norm() > 1e-6) {
                const Eigen::Vector3d up_u = config_.upward_dir.normalized();
                const double up_cur = et_cur.bend_dir.dot(up_u);
                const double up_goal = et_goal.bend_dir.dot(up_u);
                if (up_goal + 1e-4 < up_cur) {
                    max_step *= std::clamp(config_.elbow_cmd_slew_droop_scale, 0.05, 1.0);
                }
            }
        }
        double step = wrap_to_pi(phi_goal - elbow_cmd_phi_);
        step = std::clamp(step, -max_step, max_step);
        elbow_cmd_phi_ += step;
    } else {
        elbow_cmd_phi_ = phi_goal;
    }

    elbow_des = skel.elbow_des;
    {
        const ElbowTarget et = skeleton_->compute_elbow_target_from_phi(
            S_now, skel.wrist_clamped, skeleton_->L1(), skeleton_->L2(), elbow_cmd_phi_);
        if (!et.degenerate && et.E.allFinite()) {
            elbow_des = et.E;
            // Do NOT SetContinuityBend(et.bend_dir): that self-locks the commanded
            // inward branch. Continuity is re-seeded from FK at the top of Solve.
        }
    }
    if (hint_applied && !phi_frozen) {
        elbow_hint_phi_used_ = elbow_cmd_phi_;
    }

    // Blend unreachable raised/outward elbow toward current FK elbow (on the
    // swivel circle) so PHI/XYZ IK does not chronically fight a far target while
    // the wrist is already constrained by distal joint limits.
    // Skip while raise-hold or tip-yield freeze — blending to a sagged FK is exactly
    // the "elbow suddenly drops" path. Wrist-escape may slew φ but still skip FK blend.
    if (config_.elbow_target_blend_enabled && !jog_mode && !phi_frozen && !freeze_phi_for_tip &&
        !elbow_raise_hold_ && !wrist_ori_escape && q_meas.size() >= arm_dof_) {
        Eigen::Vector3d E_fk = Eigen::Vector3d::Zero();
        Eigen::MatrixXd JpE;
        if (kin_->ElbowPosJac(q_meas, &E_fk, &JpE) && E_fk.allFinite()) {
            const double err_m = (elbow_des - E_fk).norm();
            const double start_m = std::max(1e-4, config_.elbow_target_blend_start_m);
            const double full_m =
                std::max(start_m + 1e-4, config_.elbow_target_blend_full_m);
            double a_pos = std::clamp((err_m - start_m) / (full_m - start_m), 0.0, 1.0);

            double phi_fk = elbow_cmd_phi_;
            double h_fk = 0.0;
            const bool phi_ok = skeleton_->PhiFromPose(S_now, E_fk, skel.wrist_clamped, &phi_fk,
                                                      &h_fk);
            double a_phi = 0.0;
            if (phi_ok) {
                const double dphi = std::abs(wrap_to_pi(elbow_cmd_phi_ - phi_fk));
                const double p0 = std::max(1e-4, config_.elbow_target_blend_phi_start_rad);
                const double p1 =
                    std::max(p0 + 1e-4, config_.elbow_target_blend_phi_full_rad);
                a_phi = std::clamp((dphi - p0) / (p1 - p0), 0.0, 1.0);
            }
            const double a = std::max(a_pos, a_phi);
            if (a > 1e-6) {
                if (phi_ok) {
                    const double step = wrap_to_pi(phi_fk - elbow_cmd_phi_);
                    elbow_cmd_phi_ = wrap_to_pi(elbow_cmd_phi_ + a * step);
                    const ElbowTarget etb = skeleton_->compute_elbow_target_from_phi(
                        S_now, skel.wrist_clamped, skeleton_->L1(), skeleton_->L2(),
                        elbow_cmd_phi_);
                    if (!etb.degenerate && etb.E.allFinite()) {
                        elbow_des = etb.E;
                    } else {
                        elbow_des = (1.0 - a) * elbow_des + a * E_fk;
                    }
                } else {
                    elbow_des = (1.0 - a) * elbow_des + a * E_fk;
                }
            }
        }
    }

    // Near extension (XYZ elbow task only): pin elbow onto the S–W axis so tip
    // orientation owns distal posture. PHI mode uses phi freeze + weight kill
    // instead — straightening elbow_des would not enter the residual.
    // Skip during panel jog: straighten + elbow fight a pure tip translation and
    // can pop the arm upward on a left/right nudge.
    if (config_.elbow_task_mode == ElbowTaskMode::XYZ && config_.extension_straighten_elbow &&
        stretch > 1e-3 && !jog_mode) {
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
    elbow_w_scale = 1.0;
    ori_w_scale = 1.0;
    if (config_.extension_yield_enabled && stretch > 0.0 && !jog_mode) {
        elbow_w_scale = 1.0 + stretch * (config_.extension_elbow_scale_min - 1.0);
        ori_w_scale = 1.0 + stretch * (config_.extension_ori_scale_min - 1.0);
        cont_w_scale = 1.0 + stretch * (config_.extension_cont_scale_min - 1.0);
        nom_w_scale = 1.0 + stretch * (config_.extension_nom_scale_min - 1.0);
    }

    // Distal wrist near limits.
    if (!jog_mode && q_meas.size() >= arm_dof_ && q_min_.size() >= arm_dof_ &&
        q_max_.size() >= arm_dof_) {
        const int j0 = std::clamp(config_.wrist_limit_ori_joint_lo, 0,
                                  static_cast<int>(arm_dof_) - 1);
        const int j1 = std::clamp(config_.wrist_limit_ori_joint_hi, j0,
                                  static_cast<int>(arm_dof_) - 1);
        double min_margin = std::numeric_limits<double>::infinity();
        for (int j = j0; j <= j1; ++j) {
            const double qj = q_meas[static_cast<size_t>(j)];
            const double m =
                std::min(qj - q_min_[static_cast<size_t>(j)], q_max_[static_cast<size_t>(j)] - qj);
            min_margin = std::min(min_margin, m);
        }
        const double m0 = std::max(1e-4, config_.wrist_limit_ori_margin_rad);
        if (std::isfinite(min_margin) && min_margin < m0) {
            const double t = std::clamp(min_margin / m0, 0.0, 1.0);  // 0=at limit, 1=ok
            const double sat = 1.0 - t;
            const double esc_ori = std::max(1e-4, config_.wrist_escape_ori_err_rad);
            // Large attitude error while saturated: keep ori tracking, free proximal.
            // Softening ori here made e_ori grow then snap (wrist "mutation").
            const bool ori_priority =
                config_.distal_ori_priority_enabled &&
                (dbg_ori_err_ > std::max(1e-4, config_.distal_ori_priority_err_rad) ||
                 (sat > 0.5 && dbg_ori_err_ > esc_ori) || wrist_ori_escape);
            if (ori_priority) {
                const double boost = std::max(1.0, config_.distal_ori_priority_ori_boost);
                if (boost > 1.0 + 1e-9) {
                    ori_w_scale *= 1.0 + sat * (boost - 1.0);
                }
                elbow_w_scale *=
                    (1.0 - sat * (1.0 - std::clamp(config_.distal_ori_priority_elbow_scale, 0.0, 1.0)));
                if (wrist_hard_sat &&
                    dbg_ori_err_ > std::max(1e-4, config_.distal_ori_priority_err_rad)) {
                    // Unreachable tip attitude at a hard wrist stop: softening ori +
                    // boosting continuity stops q4/q6 thrashing between branches.
                    const double ori_floor =
                        std::clamp(config_.wrist_sat_ori_scale, 0.05, 1.0);
                    ori_w_scale *= (ori_floor + (1.0 - ori_floor) * t);
                    cont_w_scale *= std::max(1.0, config_.wrist_sat_cont_boost);
                } else {
                    double cont_floor = std::clamp(config_.distal_ori_priority_cont_min, 0.0, 1.0);
                    if (wrist_ori_escape || dbg_ori_err_ > esc_ori) {
                        cont_floor = std::min(cont_floor,
                                              std::clamp(config_.wrist_escape_cont_min, 0.0, 1.0));
                    }
                    cont_w_scale *= (1.0 - sat * (1.0 - cont_floor));
                }
            } else {
                // Soften ori only for small residuals (Quest noise at the stop).
                if (config_.wrist_limit_ori_soften_enabled && dbg_ori_err_ < esc_ori) {
                    const double floor = std::clamp(config_.wrist_limit_ori_scale_min, 0.0, 1.0);
                    ori_w_scale *= (floor + (1.0 - floor) * t);
                }
                if (config_.wrist_limit_elbow_yield_enabled) {
                    const double floor = std::clamp(config_.wrist_limit_elbow_scale_min, 0.0, 1.0);
                    elbow_w_scale *= (floor + (1.0 - floor) * t);
                }
            }
        }
    }

    // Medial tip: further soften elbow preference (tip 6D owns the arm).
    if (config_.medial_elbow_yield_enabled && !jog_mode && medial_amount > 1e-3) {
        const double floor = std::clamp(config_.medial_elbow_scale_min, 0.0, 1.0);
        elbow_w_scale *= (1.0 - medial_amount * (1.0 - floor));
    }

    // Ori stress: previous frame could not meet hand attitude — drop elbow so the
    // nullspace is free for a proximal reconfiguration next solve.
    if (config_.ori_stress_elbow_yield_enabled && !jog_mode) {
        const double e0 = std::max(1e-4, config_.ori_stress_elbow_err_rad);
        if (dbg_ori_err_ > e0) {
            const double t = std::clamp((dbg_ori_err_ - e0) / 0.35, 0.0, 1.0);
            const double floor = std::clamp(config_.ori_stress_elbow_scale_min, 0.0, 1.0);
            elbow_w_scale *= (1.0 - t * (1.0 - floor));
        }
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
    // Hybrid reach-intent: as soft-reach progress opens (forward/down), smoothly
    // lower elbow weight so the arm can straighten without abandoning posture
    // entirely (floor keeps a residual elbow task vs pika-style zero).
    if (config_.reach_intent_elbow_yield_enabled && !jog_mode) {
        const double intent = std::clamp(reach_progress, 0.0, 1.0);
        const double floor =
            std::clamp(config_.reach_intent_elbow_scale_min, 0.0, 1.0);
        elbow_w_scale *= (1.0 - intent * (1.0 - floor));
    }
    // Tip lagging: soften continuity so primary 6D (esp. with elbow-in-nullspace)
    // can catch large teleop moves. Does not strip elbow weight.
    if (config_.wrist_lag_assist_enabled && !jog_mode) {
        const double e0 = std::max(1e-4, config_.wrist_lag_assist_err_m);
        if (dbg_wrist_err_ > e0) {
            const double t = std::clamp((dbg_wrist_err_ - e0) / 0.06, 0.0, 1.0);
            const double cmin = std::clamp(config_.wrist_lag_assist_cont_min, 0.0, 1.0);
            const double nmin = std::clamp(config_.wrist_lag_assist_nom_min, 0.0, 1.0);
            cont_w_scale *= (1.0 - t * (1.0 - cmin));
            nom_w_scale *= (1.0 - t * (1.0 - nmin));
        }
    }
    if (jog_mode) {
        if (continuous_tip_drive_) {
            // Panel / couple tip drive: position owns the arm. Soft ori + near-zero
            // elbow so pure ±Z/±Y is not bent into a diagonal compromise.
            elbow_w_scale *= 0.05;
            const double panel_ori =
                config_.ik_track_tcp && panel_stream_active()
                    ? std::clamp(config_.panel_tcp_ori_scale, 0.0, 2.0)
                    : 0.15;
            ori_w_scale *= panel_ori;
            cont_w_scale *= 0.25;
            nom_w_scale = 0.0;
        } else if (panel_stream_active()) {
            // Continuous hold-to-jog: keep posture soft so the first frame does not
            // "pop" the tip (old jog_elbow_scale=0.1 caused the upward jerk).
            elbow_w_scale *= 0.75;
            nom_w_scale *= 0.35;
        } else {
            elbow_w_scale *= std::clamp(config_.jog_elbow_scale, 0.0, 1.0);
            nom_w_scale = 0.0;
        }
    }

    wrist_for_ik = skel.wrist_clamped;
    elbow_phi_for_ik = elbow_cmd_phi_;
    skel_for_ik = skeleton_.get();
    }  // use_elbow_skeleton

    // Whole-arm IK: wrist + hand ori + elbow (PHI swivel default, or XYZ legacy).
    // When use_elbow_skeleton=false: 6D only (elbow_w_scale=0, no skeleton).
    const Eigen::Vector3d tip_for_ik =
        config_.ik_track_tcp ? wrist_for_ik + R_hand_des * config_.wrist_to_tcp
                             : wrist_for_ik;
    const HumanMimicIkResult ik =
        ik_->Solve(q_target_, tip_for_ik, R_hand_des, elbow_des, q_nominal_,
                   elbow_w_scale, ori_w_scale, cont_w_scale, nom_w_scale, elbow_phi_for_ik, S_now,
                   skel_for_ik);
    dbg_ik_ok_ = ik.ok;
    dbg_ik_iters_ = ik.iters;
    dbg_wrist_err_ = ik.wrist_pos_err;
    dbg_ori_err_ = ik.hand_ori_err;
    dbg_elbow_err_ = (config_.elbow_task_mode == ElbowTaskMode::PHI) ? ik.elbow_phi_err
                                                                     : ik.elbow_pos_err;
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
        if (jog_active_ && !continuous_drive) {
            // Discrete nudge chase: accept IK promptly so a button press finishes.
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
            // Path continuity > hard reach: tip_yield may ask for faster proximal
            // motion, but fine/quiet moves must not suddenly unlock shoulder slew
            // (felt as "差一点够不着就整臂拧一下"). Unreachable → stay at continuous
            // path end via q_target_slew / elbow_joint_slew; never snap.
            // Couple / continuous panel: use elevated jog_step but still slew.
            double sh_boost =
                1.0 + tip_yield_ * std::max(0.0, config_.tip_priority_shoulder_slew_boost - 1.0);
            if (!jog_mode) {
                const double quiet_start = std::max(1e-4, config_.position_deadband_m);
                const double fine_end = std::max(quiet_start + 1e-4, 0.04);
                double fine_scale = 1.0;
                if (tip_quiet_) {
                    fine_scale = 0.0;
                } else if (clutch_hand_depart_m_ < fine_end) {
                    fine_scale = std::clamp(
                        (clutch_hand_depart_m_ - quiet_start) / (fine_end - quiet_start), 0.0,
                        1.0);
                }
                sh_boost = 1.0 + (sh_boost - 1.0) * fine_scale;
            }
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
                // Couple: never let elbow joint jump faster than elevated slew.
                if (i == 3 && couple_mode) {
                    joint_max = std::min(joint_max, std::max(max_step, jog_step));
                }
                // Wrist hard-sat: rate-limit q4–q6 so a branch hop cannot land in one tick
                // even when solve_dt is large (sparse Quest packets).
                if (wrist_hard_sat && i >= 4 && i <= 6) {
                    const double distal_rate =
                        std::max(0.0, config_.wrist_sat_distal_slew_radps) *
                        std::max(1e-4, solve_dt);
                    const double distal_cap = std::max(1e-4, config_.wrist_sat_distal_max_step);
                    joint_max = std::min(joint_max, std::min(distal_rate, distal_cap));
                }
                double raw_dq = ik.q[i] - q_target_[i];
                // Asymmetric elbow flexion: opening (Δq3 < 0) is slower — stops the
                // "slight outward → elbow dumps" feel while tip stays glued.
                if (i == 3 && raw_dq < 0.0) {
                    joint_max *= std::clamp(config_.elbow_joint_slew_down_scale, 0.05, 1.0);
                }
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
    // gripper hold-to-move so A+B / X+Y does not keep driving the jaws.
    const bool recenter_combo =
        config_.orientation_recenter_button == OrientationRecenterButton::BOTH_FACE_BUTTONS &&
        sample.valid && sample.button_close && sample.button_open;
    const bool recenter_edge = recenter_combo && !last_recenter_;
    if (!recenter_combo) {
        update_gripper_from_buttons(sample, current_joints, dt);
    } else {
        // Freeze any in-flight open/close while the recenter chord is held.
        // GraspHolding keeps its relative lag (do not drop clamp force).
        if (gripper_motion_ == GripperMotion::GraspHolding &&
            current_joints.size() > arm_dof_) {
            const double q_g = current_joints[arm_dof_].position;
            const double close_dir =
                (config_.gripper_closed >= config_.gripper_open) ? 1.0 : -1.0;
            const double lag = gripper_grasp_lag_ > 0.0
                                   ? gripper_grasp_lag_
                                   : std::max(0.0, config_.gripper_grasp_hold_lag);
            double hold = q_g + close_dir * lag;
            if (config_.gripper_closed >= config_.gripper_open) {
                hold = std::clamp(hold, config_.gripper_open, config_.gripper_closed);
            } else {
                hold = std::clamp(hold, config_.gripper_closed, config_.gripper_open);
            }
            gripper_hold_pos_ = hold;
            gripper_target_ = hold;
        } else if (gripper_motion_ == GripperMotion::Closing ||
                   gripper_motion_ == GripperMotion::Opening) {
            if (current_joints.size() > arm_dof_) {
                const double q_g = current_joints[arm_dof_].position;
                if (gripper_motion_ == GripperMotion::Closing) {
                    const double close_dir =
                        (config_.gripper_closed >= config_.gripper_open) ? 1.0 : -1.0;
                    double hold = (close_dir >= 0.0) ? std::max(q_g, gripper_target_)
                                                     : std::min(q_g, gripper_target_);
                    hold += close_dir * std::max(0.0, config_.gripper_hold_preload);
                    const double max_lag =
                        std::max(std::max(0.0, config_.gripper_grasp_hold_lag),
                                 std::max(0.0, config_.gripper_grasp_hold_lag_max));
                    if (close_dir * (hold - q_g) > max_lag && max_lag > 0.0) {
                        hold = q_g + close_dir * max_lag;
                    }
                    if (config_.gripper_closed >= config_.gripper_open) {
                        hold = std::clamp(hold, config_.gripper_open, config_.gripper_closed);
                    } else {
                        hold = std::clamp(hold, config_.gripper_closed, config_.gripper_open);
                    }
                    gripper_hold_pos_ = hold;
                    gripper_target_ = hold;
                } else {
                    gripper_hold_pos_ = q_g;
                    gripper_target_ = q_g;
                }
            }
            gripper_motion_ = GripperMotion::Stopped;
            gripper_stall_timer_ = 0.0;
            gripper_grasp_lag_ = 0.0;
        }
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
        // Continuous JRG (no Reset): higher scales so slave keeps up without snaps.
        double k_scale = 12.0;
        double qdot_scale = 8.0;
        if (const char* e = std::getenv("OPENARM_COUPLE_K_SCALE")) {
            try {
                k_scale = std::max(1.0, std::stod(e));
            } catch (...) {
            }
        }
        if (const char* e = std::getenv("OPENARM_COUPLE_QDOT_SCALE")) {
            try {
                qdot_scale = std::max(1.0, std::stod(e));
            } catch (...) {
            }
        }
        const std::vector<double>& q_ref =
            advance_joint_stream(q_target_, dt, k_scale, qdot_scale);
        fill_reference(current_joints, q_ref, joint_reference);
        return true;
    }

    if (!clutch_active_) {
        if (panel_jogging) {
            if (q_target_.size() < arm_dof_) {
                q_target_ = q_meas;
                reset_joint_stream(q_target_);
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
            const std::vector<double>& q_ref = advance_joint_stream(q_target_, dt);
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
        clutch_hand_depart_m_ = (sample.position - clutch_idle_anchor_pos_).norm();
        if (clutch_settle_remaining_ > 0) {
            // Hold commanded pose right after grip so TCP/wrist frame residuals
            // cannot flash a first-frame jump before the map is zeroed.
            // Absorb VR motion into the clutch zero so settle→track has no backlog.
            hand_ref_pos_ = sample.position;
            hand_ref_rot_ = sample.orientation.normalized();
            ori_filt_ = hand_ref_rot_;
            ori_filt_valid_ = true;
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

            tip_quiet_ = delta_robot.norm() < std::max(1e-4, config_.position_deadband_m);
            // Idle exit uses departure from the grip pose, NOT |delta| from hand_ref.
            // During idle we keep absorbing hand_ref so leftover motion cannot dump
            // into the first IK frame (that was the sudden jerk on grip).
            const double start_m = std::max(config_.position_deadband_m,
                                            config_.clutch_start_motion_m);
            const double depart =
                (sample.position - clutch_idle_anchor_pos_).norm();
            const bool still_idle = clutch_idle_ && depart < start_m;
            if (still_idle) {
                hand_ref_pos_ = sample.position;
                hand_ref_rot_ = sample.orientation.normalized();
                ori_filt_ = hand_ref_rot_;
                ori_filt_valid_ = true;
                maybe_log("idle");
            } else {
                if (clutch_idle_) {
                    clutch_idle_ = false;
                    // Final absorb: start tracking from the current hand pose with
                    // zero backlog (robot_ref stays at the clutch arm pose).
                    hand_ref_pos_ = sample.position;
                    hand_ref_rot_ = sample.orientation.normalized();
                    ori_filt_ = hand_ref_rot_;
                    ori_filt_valid_ = true;
                    clutch_track_settle_remaining_ =
                        std::max(0, config_.clutch_track_settle_ticks);
                    std::cout << log_prefix_
                              << "[human-mimic] clutch motion start (depart="
                              << depart << " ang=" << dbg_delta_ang_ << ")"
                              << std::endl;
                    maybe_log("idle");  // this tick still holds; track-settle next
                } else if (clutch_track_settle_remaining_ > 0) {
                    // 1-tick (default) absorb after idle→track: kill Quest noise at
                    // the start-motion threshold only — no elbow φ freeze.
                    hand_ref_pos_ = sample.position;
                    hand_ref_rot_ = sample.orientation.normalized();
                    ori_filt_ = hand_ref_rot_;
                    ori_filt_valid_ = true;
                    --clutch_track_settle_remaining_;
                    maybe_log("tsettle");
                } else {
                    solve_base_skeleton(q_meas, p_tcp_des, R_hand_des, dt);
                    maybe_log("act");
                }
            }
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

    const std::vector<double>& q_ref = advance_joint_stream(q_target_, dt);
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

    std::ostringstream line;
    line << log_prefix_ << "[human-mimic][" << tag << "] #" << log_tick_
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
         << (config_.elbow_task_mode == ElbowTaskMode::PHI ? " e_phi=" : " e_elb=")
         << dbg_elbow_err_
         << " lim=" << min_margin << "@j" << min_joint << " at_lim=" << at_limit
         << " hold_prev=" << (dbg_hold_prev_ ? 1 : 0)
         << " phi_base=" << dbg_phi_base_ << " phi_cmd=" << elbow_cmd_phi_
         << " flex=" << dbg_elbow_flex_ << " flip=" << dbg_phi_flip_
         << " tip_y=" << tip_yield_ << " hint_ok=" << (elbow_hint_valid_ ? 1 : 0)
         << " phi_used=" << elbow_hint_phi_used_
         << " raise_h=" << (elbow_raise_hold_ ? 1 : 0)
         << " wr_esc=" << (dbg_wrist_escape_ ? 1 : 0)
         << " wr_m=" << dbg_wrist_margin_;
    if (q_target_.size() >= arm_dof_) {
        // j0=shoulder yaw, j1=shoulder pitch, j3=elbow, j4–j6=wrist chain
        line << " q0=" << q_target_[0] << " q1=" << q_target_[1] << " q3=" << q_target_[3]
             << " q4=" << q_target_[4] << " q5=" << q_target_[5] << " q6=" << q_target_[6];
    }
    line << " dq_ang=" << dbg_delta_ang_ << " vr_ax_world=[" << dbg_vr_axis_world_.transpose()
         << "] rob_ax=[" << dbg_rob_axis_.transpose() << "]";

    const std::string text = line.str();
    if (debug_log_file_.is_open()) {
        debug_log_file_ << text << std::endl;
    }
    if (config_.debug_log_console) {
        std::cout << text << std::endl;
    }
}
