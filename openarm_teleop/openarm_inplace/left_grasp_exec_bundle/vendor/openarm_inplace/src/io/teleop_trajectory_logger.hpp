#pragma once

#include <controller/cartesian_types.hpp>
#include <controller/dynamics.hpp>
#include <robot_state.hpp>

#include <chrono>
#include <fstream>
#include <string>
#include <unordered_map>

class TeleopTrajectoryLogger {
public:
    static TeleopTrajectoryLogger& shared();

    void configure_from_env();
    bool enabled() const { return enabled_; }

    void log(const std::string& arm_label, Dynamics* dynamics, CartesianControlState state,
             bool grip, const Eigen::Vector3d& tcp_target,
             const std::vector<JointState>& measured, const std::vector<JointState>& reference);

    void close();

private:
    TeleopTrajectoryLogger() = default;

    std::ofstream& stream_for(const std::string& arm_label, size_t dof);
    static const char* state_name(CartesianControlState state);

    bool enabled_ = false;
    std::string log_dir_;
    std::chrono::steady_clock::time_point t0_;
    bool t0_initialized_ = false;
    std::unordered_map<std::string, std::ofstream> files_;
    std::unordered_map<std::string, bool> header_written_;
};
