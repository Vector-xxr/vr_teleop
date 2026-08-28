#include <controller/pose_kalman_filter.hpp>

#include <cmath>

PoseKalmanFilter::PoseKalmanFilter(double process_noise, double measurement_noise)
    : process_noise_(process_noise), measurement_noise_(measurement_noise) {}

void PoseKalmanFilter::reset() {
    initialized_ = false;
    position_.setZero();
    velocity_.setZero();
    position_cov_ = Eigen::Matrix3d::Identity();
    orientation_ = Eigen::Quaterniond::Identity();
}

bool PoseKalmanFilter::reject_outlier(const QuestPoseSample& measurement) const {
    if (!initialized_) {
        return false;
    }
    const double pos_jump = (measurement.position - position_).norm();
    if (pos_jump > 0.15) {
        return true;
    }
    const Eigen::Quaterniond delta = orientation_.inverse() * measurement.orientation;
    const double angle = 2.0 * std::acos(std::clamp(std::abs(delta.w()), 0.0, 1.0));
    return angle > 0.52;
}

bool PoseKalmanFilter::update(const QuestPoseSample& measurement, double dt,
                              QuestPoseSample* filtered) {
    if (filtered == nullptr || !measurement.valid) {
        return false;
    }

    if (!initialized_) {
        position_ = measurement.position;
        velocity_.setZero();
        orientation_ = measurement.orientation.normalized();
        position_cov_ = Eigen::Matrix3d::Identity() * measurement_noise_;
        initialized_ = true;
        *filtered = measurement;
        filtered->position = position_;
        filtered->orientation = orientation_;
        filtered->valid = true;
        return true;
    }

    if (reject_outlier(measurement)) {
        filtered->timestamp_ms = measurement.timestamp_ms;
        filtered->grip_active = measurement.grip_active;
        filtered->trigger = measurement.trigger;
        filtered->position = position_ + velocity_ * dt;
        filtered->orientation = orientation_;
        filtered->valid = true;
        return true;
    }

    const double dt_safe = std::max(dt, 1e-4);
    Eigen::Matrix3d predicted_cov =
        position_cov_ + Eigen::Matrix3d::Identity() * process_noise_ * dt_safe;
    const Eigen::Matrix3d innovation_cov =
        predicted_cov + Eigen::Matrix3d::Identity() * measurement_noise_;
    const Eigen::Matrix3d gain = predicted_cov * innovation_cov.inverse();

    const Eigen::Vector3d predicted_position = position_ + velocity_ * dt_safe;
    const Eigen::Vector3d innovation = measurement.position - predicted_position;
    position_ = predicted_position + gain * innovation;
    velocity_ = (position_ - predicted_position) / dt_safe;
    position_cov_ = (Eigen::Matrix3d::Identity() - gain) * predicted_cov;

    const Eigen::Quaterniond q_meas = measurement.orientation.normalized();
    orientation_ = orientation_.slerp(orientation_alpha_, q_meas).normalized();

    filtered->timestamp_ms = measurement.timestamp_ms;
    filtered->grip_active = measurement.grip_active;
    filtered->trigger = measurement.trigger;
    filtered->position = position_;
    filtered->orientation = orientation_;
    filtered->valid = true;
    return true;
}

bool PoseKalmanFilter::predict(double dt, QuestPoseSample* filtered) {
    if (filtered == nullptr || !initialized_ || dt <= 0.0) {
        return false;
    }

    const double dt_safe = std::max(dt, 1e-4);
    position_ += velocity_ * dt_safe;
    position_cov_ += Eigen::Matrix3d::Identity() * process_noise_ * dt_safe;

    filtered->position = position_;
    filtered->orientation = orientation_;
    filtered->valid = true;
    return true;
}
