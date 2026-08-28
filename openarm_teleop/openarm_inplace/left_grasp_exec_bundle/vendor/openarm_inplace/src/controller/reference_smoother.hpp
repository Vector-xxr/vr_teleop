#pragma once

#include <Eigen/Dense>

#include <vector>

struct SmoothingConfig {
    // Constant-velocity smoothing: move toward target at fixed max rate each step.
    double max_cartesian_speed_mps = 1.5;
    double max_cartesian_rot_radps = 3.5;
    double max_joint_velocity_radps = 3.0;
    double max_gripper_speed = 0.02;  // gripper position units per second
    double joint_output_lowpass_hz = 15.0;
    double max_joint_output_velocity_radps = 2.0;
    double max_joint_output_accel_radps2 = 8.0;
};

class CartesianPoseSmoother {
public:
    explicit CartesianPoseSmoother(SmoothingConfig config = {});

    void reset(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
    void set_target(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
    void step(double dt, Eigen::Vector3d* position, Eigen::Quaterniond* orientation);

private:
    SmoothingConfig config_;
    Eigen::Vector3d target_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond target_rot_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d smooth_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond smooth_rot_ = Eigen::Quaterniond::Identity();
    bool initialized_ = false;
};

class JointReferenceSmoother {
public:
    explicit JointReferenceSmoother(SmoothingConfig config = {});

    void reset(const std::vector<double>& positions, const std::vector<double>& velocities);
    void set_target(const std::vector<double>& positions, const std::vector<double>& velocities);
    void step(double dt, std::vector<double>* positions, std::vector<double>* velocities);

private:
    SmoothingConfig config_;
    std::vector<double> target_pos_;
    std::vector<double> target_vel_;
    std::vector<double> smooth_pos_;
    std::vector<double> smooth_vel_;
    bool initialized_ = false;
};

class ScalarSmoother {
public:
    explicit ScalarSmoother(double max_speed = 0.02);

    void reset(double value);
    void set_target(double value);
    double step(double dt);

private:
    double max_speed_ = 0.02;
    double target_ = 0.0;
    double smooth_ = 0.0;
    bool initialized_ = false;
};

class JointReferenceOutputFilter {
public:
    explicit JointReferenceOutputFilter(SmoothingConfig config = {});

    void reset(const std::vector<double>& positions);
    bool initialized() const { return initialized_; }
    void step(double dt, const std::vector<double>& q_in, std::vector<double>* q_out,
              std::vector<double>* dq_out);

private:
    SmoothingConfig config_;
    std::vector<double> q_cmd_;
    std::vector<double> q_out_;
    std::vector<double> dq_out_;
    bool initialized_ = false;
};
