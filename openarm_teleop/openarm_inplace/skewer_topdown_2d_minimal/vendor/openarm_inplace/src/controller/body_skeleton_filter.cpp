#include "controller/body_skeleton_filter.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace {
Eigen::Vector3d safe_normalize(const Eigen::Vector3d& v, const Eigen::Vector3d& fallback) {
    const double n = v.norm();
    if (n < 1e-8) {
        return fallback;
    }
    return v / n;
}

// Spherical lerp-ish: blend then renormalize; clamp step by max angular velocity.
Eigen::Vector3d blend_unit(const Eigen::Vector3d& prev, const Eigen::Vector3d& meas, double alpha,
                           double max_ang_vel, double dt) {
    Eigen::Vector3d m = meas;
    if (prev.dot(m) < 0.0) {
        m = -m;  // same hemisphere
    }
    Eigen::Vector3d mixed = alpha * m + (1.0 - alpha) * prev;
    mixed = safe_normalize(mixed, prev);
    if (max_ang_vel > 0.0 && dt > 1e-6) {
        const double c = std::clamp(prev.dot(mixed), -1.0, 1.0);
        const double ang = std::acos(c);
        const double max_ang = max_ang_vel * dt;
        if (ang > max_ang && ang > 1e-8) {
            const Eigen::Vector3d axis = prev.cross(mixed);
            if (axis.norm() > 1e-8) {
                Eigen::AngleAxisd aa(max_ang, axis.normalized());
                mixed = (aa * prev).normalized();
            }
        }
    }
    return mixed;
}
}  // namespace

BodySkeletonFilter::BodySkeletonFilter(SkeletonRetargetConfig config)
    : config_(std::move(config)) {}

void BodySkeletonFilter::set_config(const SkeletonRetargetConfig& config) { config_ = config; }

void BodySkeletonFilter::reset() {
    has_R_ = false;
    has_lu_ = has_lf_ = has_ru_ = has_rf_ = false;
    has_le_ = has_re_ = false;
    latest_ = BodySkeletonMapped{};
}

bool BodySkeletonFilter::build_torso(const BodyPoseSample& sample, Eigen::Matrix3d* R_camera_body) {
    if (R_camera_body == nullptr) {
        return false;
    }
    if (sample.left_shoulder.conf < 0.05 || sample.right_shoulder.conf < 0.05 ||
        sample.left_hip.conf < 0.05 || sample.right_hip.conf < 0.05) {
        return false;
    }
    const Eigen::Vector3d LS = sample.left_shoulder.p;
    const Eigen::Vector3d RS = sample.right_shoulder.p;
    const Eigen::Vector3d LH = sample.left_hip.p;
    const Eigen::Vector3d RH = sample.right_hip.p;
    const Eigen::Vector3d shoulder_center = 0.5 * (LS + RS);
    const Eigen::Vector3d hip_center = 0.5 * (LH + RH);

    Eigen::Vector3d x_body = RS - LS;  // person right
    if (x_body.norm() < 1e-4) {
        return false;
    }
    x_body.normalize();

    // Prefer headward up from hips -> shoulders; flip if inverted.
    Eigen::Vector3d y_raw = shoulder_center - hip_center;
    if (y_raw.norm() < 1e-4) {
        return false;
    }
    y_raw.normalize();

    Eigen::Vector3d z_body = x_body.cross(y_raw);
    if (z_body.norm() < 1e-4) {
        return false;
    }
    z_body.normalize();
    Eigen::Vector3d y_body = z_body.cross(x_body).normalized();
    if (y_body.dot(y_raw) < 0.0) {
        y_body = -y_body;
        z_body = -z_body;  // keep right-handed with fixed x
    }

    Eigen::Matrix3d R;
    R.col(0) = x_body;
    R.col(1) = y_body;
    R.col(2) = z_body;
    if (!is_valid_rotation(R, 2e-2)) {
        return false;
    }
    *R_camera_body = R;
    return true;
}

bool BodySkeletonFilter::arm_dirs_camera(const BodyJoint3d& S, const BodyJoint3d& E,
                                         const BodyJoint3d& W, Eigen::Vector3d* u_upper,
                                         Eigen::Vector3d* u_fore, double* elbow_rad,
                                         double* conf) {
    if (u_upper == nullptr || u_fore == nullptr || elbow_rad == nullptr || conf == nullptr) {
        return false;
    }
    *conf = std::min({S.conf, E.conf, W.conf});
    if (*conf < 0.05) {
        return false;
    }
    Eigen::Vector3d se = E.p - S.p;
    Eigen::Vector3d ew = W.p - E.p;
    if (se.norm() < 1e-4 || ew.norm() < 1e-4) {
        return false;
    }
    *u_upper = se.normalized();
    *u_fore = ew.normalized();
    const double c = std::clamp(u_upper->dot(*u_fore), -1.0, 1.0);
    *elbow_rad = std::acos(c);
    return true;
}

Eigen::Vector3d BodySkeletonFilter::filter_unit(Eigen::Vector3d* state, bool* has,
                                                const Eigen::Vector3d& meas, double tau,
                                                double max_ang_vel, double dt) const {
    const Eigen::Vector3d m = safe_normalize(meas, Eigen::Vector3d::UnitX());
    if (!*has) {
        *state = m;
        *has = true;
        return *state;
    }
    const double alpha = 1.0 - std::exp(-std::max(dt, 1e-4) / std::max(tau, 1e-4));
    *state = blend_unit(*state, m, alpha, max_ang_vel, dt);
    return *state;
}

double BodySkeletonFilter::filter_scalar(double* state, bool* has, double meas, double tau,
                                         double max_vel, double dt) const {
    if (!*has) {
        *state = meas;
        *has = true;
        return *state;
    }
    const double alpha = 1.0 - std::exp(-std::max(dt, 1e-4) / std::max(tau, 1e-4));
    double next = alpha * meas + (1.0 - alpha) * (*state);
    if (max_vel > 0.0 && dt > 1e-6) {
        const double max_d = max_vel * dt;
        next = *state + std::clamp(next - *state, -max_d, max_d);
    }
    *state = next;
    return *state;
}

std::optional<BodySkeletonMapped> BodySkeletonFilter::update(const BodyPoseSample& sample,
                                                             double age_ms, double dt) {
    BodySkeletonMapped out;
    if (!sample.valid || age_ms > config_.pose_timeout_ms) {
        latest_ = out;
        return std::nullopt;
    }

    Eigen::Matrix3d R_cam_body;
    if (!build_torso(sample, &R_cam_body)) {
        latest_ = out;
        return std::nullopt;
    }

    // Mild SO(3) filter via column blend (good enough for phase-2 viz).
    if (!has_R_) {
        R_filt_ = R_cam_body;
        has_R_ = true;
    } else {
        const double alpha =
            1.0 - std::exp(-std::max(dt, 1e-4) / std::max(config_.body_pose_tau, 1e-4));
        for (int c = 0; c < 3; ++c) {
            R_filt_.col(c) =
                blend_unit(R_filt_.col(c), R_cam_body.col(c), alpha, 3.0, dt);
        }
        // Re-orthonormalize (Gram-Schmidt)
        Eigen::Vector3d x = R_filt_.col(0).normalized();
        Eigen::Vector3d z = x.cross(R_filt_.col(1));
        if (z.norm() < 1e-6) {
            z = x.cross(Eigen::Vector3d::UnitY());
        }
        z.normalize();
        Eigen::Vector3d y = z.cross(x).normalized();
        R_filt_.col(0) = x;
        R_filt_.col(1) = y;
        R_filt_.col(2) = z;
    }

    out.torso_valid = true;
    out.R_camera_body = R_filt_;
    const Eigen::Matrix3d R_bT = R_filt_.transpose();  // camera -> body

    auto map_arm = [&](const BodyJoint3d& S, const BodyJoint3d& E, const BodyJoint3d& W,
                       const Eigen::Matrix3d& R_b2r, Eigen::Vector3d* u_state, bool* has_u,
                       Eigen::Vector3d* f_state, bool* has_f, double* e_state, bool* has_e,
                       MappedArmDirs* arm) {
        Eigen::Vector3d u_c, f_c;
        double elbow = M_PI_2;
        double conf = 0.0;
        if (!arm_dirs_camera(S, E, W, &u_c, &f_c, &elbow, &conf)) {
            arm->valid = false;
            arm->confidence = conf;
            return;
        }
        const Eigen::Vector3d u_b_raw = (R_bT * u_c).normalized();
        const Eigen::Vector3d f_b_raw = (R_bT * f_c).normalized();

        // Soften body-lateral (+/- X) so dual-arm abduction is not overstated.
        Eigen::Vector3d u_b = u_b_raw;
        u_b.x() *= std::clamp(config_.lateral_direction_gain, 0.0, 1.0);
        // Soft blend toward body-down (-Y) to reduce single-view forward lean.
        if (config_.upper_down_blend > 1e-6) {
            const double w = std::clamp(config_.upper_down_blend, 0.0, 0.8);
            u_b = (1.0 - w) * u_b + w * Eigen::Vector3d(0.0, -1.0, 0.0);
        }
        if (u_b.norm() < 1e-6) {
            u_b = Eigen::Vector3d(0.0, -1.0, 0.0);
        } else {
            u_b.normalize();
        }

        Eigen::Vector3d f_b = f_b_raw;
        f_b.x() *= std::clamp(config_.lateral_direction_gain, 0.0, 1.0);
        if (f_b.norm() < 1e-6) {
            f_b = f_b_raw;
        } else {
            f_b.normalize();
        }

        Eigen::Vector3d u_r = (R_b2r * u_b).normalized();
        Eigen::Vector3d f_r = (R_b2r * f_b).normalized();

        u_r = filter_unit(u_state, has_u, u_r, config_.upper_direction_tau,
                          config_.max_upper_direction_ang_vel, dt);
        f_r = filter_unit(f_state, has_f, f_r, config_.forearm_direction_tau,
                          config_.max_forearm_direction_ang_vel, dt);
        elbow = filter_scalar(e_state, has_e, elbow, config_.elbow_angle_tau,
                              config_.max_elbow_angle_vel, dt);

        arm->u_upper = u_r;
        arm->u_fore = f_r;
        arm->elbow_angle_rad = elbow;
        arm->confidence = conf;
        arm->valid = conf >= config_.confidence_low;
    };

    map_arm(sample.left_shoulder, sample.left_elbow, sample.left_wrist,
            config_.R_body_to_robot_left, &lu_, &has_lu_, &lf_, &has_lf_, &le_, &has_le_, &out.left);
    map_arm(sample.right_shoulder, sample.right_elbow, sample.right_wrist,
            config_.R_body_to_robot_right, &ru_, &has_ru_, &rf_, &has_rf_, &re_, &has_re_,
            &out.right);

    latest_ = out;
    return out;
}
