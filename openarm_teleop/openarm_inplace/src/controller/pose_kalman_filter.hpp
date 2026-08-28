#pragma once

#include <controller/cartesian_types.hpp>

#include <Eigen/Dense>

class PoseKalmanFilter {
public:
    explicit PoseKalmanFilter(double process_noise = 1.0, double measurement_noise = 0.01);

    bool update(const QuestPoseSample& measurement, double dt, QuestPoseSample* filtered);
    bool predict(double dt, QuestPoseSample* filtered);
    void reset();

private:
    bool initialized_ = false;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d position_cov_ = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    double process_noise_ = 1.0;
    double measurement_noise_ = 0.01;
    double orientation_alpha_ = 0.25;

    bool reject_outlier(const QuestPoseSample& measurement) const;
};
