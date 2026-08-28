// Copyright 2025 Enactic, Inc.
//
// Independent TCP velocity approach stream for dual_replay_control.
// Not part of CSV play / prepend_transition; mutually exclusive with play.

#pragma once

#include <controller/dynamics.hpp>
#include <robot_state.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace openarm_approach {

struct ApproachStatus {
    bool handled = false;
    std::string status;  // e.g. accepted / busy / approach_pose / approach_stopped
    std::string detail;
};

// Per-arm accel-limited TCP integrate → keep orientation → PosIK → joint refs.
class TcpApproachStream {
public:
    TcpApproachStream() = default;

    bool any_active() const { return left_.active || right_.active; }
    bool side_active(const std::string& side) const;
    bool blocks_play(const std::string& side) const;

    // Handle approach_on / tcp_vel / approach_stop / approach_get_pose.
    // Returns handled=true when the message belongs to this protocol.
    ApproachStatus handle_command(const std::string& message, bool left_replay_active,
                                  bool right_replay_active, bool dual_replay_active,
                                  const std::vector<JointState>& hold_left,
                                  const std::vector<JointState>& hold_right,
                                  Dynamics* left_dyn, Dynamics* right_dyn);

    // 1 kHz tick: accel-limit, integrate TCP, IK, write into hold_* (8 joints each).
    void tick(double dt, Dynamics* left_dyn, Dynamics* right_dyn,
              std::vector<JointState>* hold_left, std::vector<JointState>* hold_right);

private:
    struct Side {
        bool active = false;
        Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();  // m/s base frame
        Eigen::Vector3d filt_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d tcp_des = Eigen::Vector3d::Zero();
        Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity();
        bool tcp_valid = false;
        double gripper = 0.0;
        std::vector<double> q_arm;  // 7
        double cmd_timeout_s = 0.15;
        double accel_mps2 = 0.25;
        std::chrono::steady_clock::time_point last_vel_cmd{};
        bool have_vel_cmd = false;
    };

    Side left_;
    Side right_;

    Side* side_mut(const std::string& side);
    const Side* side_const(const std::string& side) const;

    bool seed_side(Side* side, const std::vector<JointState>& hold, Dynamics* dyn);
    bool tick_side(Side* side, double dt, Dynamics* dyn, std::vector<JointState>* hold);
    ApproachStatus make_pose_status(const std::string& side_name, const Side& side) const;
};

}  // namespace openarm_approach
