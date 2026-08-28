#include <controller/reference_smoother.hpp>

#include <algorithm>
#include <cmath>

CartesianPoseSmoother::CartesianPoseSmoother(SmoothingConfig config) : config_(std::move(config)) {}

void CartesianPoseSmoother::reset(const Eigen::Vector3d& position,
                                  const Eigen::Quaterniond& orientation) {
    target_pos_ = position;
    target_rot_ = orientation.normalized();
    smooth_pos_ = target_pos_;
    smooth_rot_ = target_rot_;
    initialized_ = true;
}

void CartesianPoseSmoother::set_target(const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation) {
    target_pos_ = position;
    target_rot_ = orientation.normalized();
    if (!initialized_) {
        reset(position, orientation);
    }
}

void CartesianPoseSmoother::step(double dt, Eigen::Vector3d* position,
                                 Eigen::Quaterniond* orientation) {
    if (position == nullptr || orientation == nullptr || !initialized_ || dt <= 0.0) {
        return;
    }

    Eigen::Vector3d delta_pos = target_pos_ - smooth_pos_;
    const double dist = delta_pos.norm();
    const double pos_step = config_.max_cartesian_speed_mps * dt;
    if (dist > 1e-9) {
        if (dist <= pos_step) {
            smooth_pos_ = target_pos_;
        } else {
            smooth_pos_ += delta_pos.normalized() * pos_step;
        }
    }

    Eigen::Quaterniond q_err = smooth_rot_.inverse() * target_rot_;
    q_err.normalize();
    Eigen::AngleAxisd aa(q_err);
    double angle = aa.angle();
    if (!std::isfinite(angle)) {
        angle = 0.0;
    }
    const double max_rot_step = config_.max_cartesian_rot_radps * dt;
    if (angle > 1e-9) {
        if (angle <= max_rot_step) {
            smooth_rot_ = target_rot_;
        } else {
            smooth_rot_ =
                (smooth_rot_ * Eigen::Quaterniond(Eigen::AngleAxisd(max_rot_step, aa.axis())))
                    .normalized();
        }
    }

    *position = smooth_pos_;
    *orientation = smooth_rot_;
}

JointReferenceSmoother::JointReferenceSmoother(SmoothingConfig config) : config_(std::move(config)) {}

void JointReferenceSmoother::reset(const std::vector<double>& positions,
                                   const std::vector<double>& velocities) {
    target_pos_ = positions;
    target_vel_ = velocities;
    smooth_pos_ = positions;
    smooth_vel_ = velocities;
    initialized_ = !positions.empty();
}

void JointReferenceSmoother::set_target(const std::vector<double>& positions,
                                        const std::vector<double>& velocities) {
    target_pos_ = positions;
    target_vel_ = velocities;
    if (!initialized_ && !positions.empty()) {
        reset(positions, velocities);
    }
}

void JointReferenceSmoother::step(double dt, std::vector<double>* positions,
                                  std::vector<double>* velocities) {
    if (positions == nullptr || velocities == nullptr || !initialized_ || dt <= 0.0) {
        return;
    }

    const size_t dof = target_pos_.size();
    positions->resize(dof);
    velocities->resize(dof);
    const double max_step = config_.max_joint_velocity_radps * dt;

    for (size_t i = 0; i < dof; ++i) {
        const double prev = smooth_pos_[i];
        double delta = target_pos_[i] - smooth_pos_[i];
        if (std::abs(delta) <= max_step) {
            smooth_pos_[i] = target_pos_[i];
        } else {
            smooth_pos_[i] += std::copysign(max_step, delta);
        }
        smooth_vel_[i] = (smooth_pos_[i] - prev) / dt;
        (*positions)[i] = smooth_pos_[i];
        (*velocities)[i] = smooth_vel_[i];
    }
}

ScalarSmoother::ScalarSmoother(double max_speed) : max_speed_(max_speed) {}

void ScalarSmoother::reset(double value) {
    target_ = value;
    smooth_ = value;
    initialized_ = true;
}

void ScalarSmoother::set_target(double value) { target_ = value; }

double ScalarSmoother::step(double dt) {
    if (!initialized_) {
        reset(target_);
    }
    if (dt <= 0.0) {
        return smooth_;
    }

    const double max_step = max_speed_ * dt;
    const double delta = target_ - smooth_;
    if (std::abs(delta) <= max_step) {
        smooth_ = target_;
    } else {
        smooth_ += std::copysign(max_step, delta);
    }
    return smooth_;
}

JointReferenceOutputFilter::JointReferenceOutputFilter(SmoothingConfig config)
    : config_(std::move(config)) {}

void JointReferenceOutputFilter::reset(const std::vector<double>& positions) {
    q_cmd_ = positions;
    q_out_ = positions;
    dq_out_.assign(positions.size(), 0.0);
    initialized_ = !positions.empty();
}

void JointReferenceOutputFilter::step(double dt, const std::vector<double>& q_in,
                                      std::vector<double>* q_out, std::vector<double>* dq_out) {
    if (q_out == nullptr || dq_out == nullptr || dt <= 0.0 || q_in.empty()) {
        return;
    }
    if (!initialized_) {
        reset(q_in);
    }

    const size_t dof = q_out_.size();
    if (q_in.size() != dof) {
        return;
    }

    const double alpha = 1.0 - std::exp(-2.0 * M_PI * config_.joint_output_lowpass_hz * dt);
    const double vmax = config_.max_joint_output_velocity_radps;
    const double amax = config_.max_joint_output_accel_radps2;
    const double dt_safe = std::max(dt, 1e-4);

    q_out->resize(dof);
    dq_out->resize(dof);
    for (size_t i = 0; i < dof; ++i) {
        q_cmd_[i] += alpha * (q_in[i] - q_cmd_[i]);

        const double pos_err = q_cmd_[i] - q_out_[i];
        double dq_des = std::clamp(pos_err / dt_safe, -vmax, vmax);
        const double ddq = std::clamp((dq_des - dq_out_[i]) / dt_safe, -amax, amax);
        dq_out_[i] += ddq * dt;
        dq_out_[i] = std::clamp(dq_out_[i], -vmax, vmax);
        q_out_[i] += dq_out_[i] * dt;

        (*q_out)[i] = q_out_[i];
        (*dq_out)[i] = dq_out_[i];
    }
}
