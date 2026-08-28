// Phase-2+ config / mapped features for BODY_SKELETON_RETARGET.
// Phase 2: map + visualize only (no q_target drive).
#pragma once

#include <controller/body_skeleton_types.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <string>

struct SkeletonRetargetConfig {
    // Human body (x=right, y=up, z=forward-of-chest) -> OpenArm base.
    // Assumes operator faces the robot (workspace along robot +X):
    //   body right  -> robot -Y (robot right)
    //   body up     -> robot +Z
    //   body forward-> robot -X (toward the robot / away from table front)
    // Previous default (right->+Y, forward->+X) was a left-right + front-back mirror.
    Eigen::Matrix3d R_body_to_robot_left =
        (Eigen::Matrix3d() << 0, 0, -1, -1, 0, 0, 0, 1, 0).finished();
    Eigen::Matrix3d R_body_to_robot_right =
        (Eigen::Matrix3d() << 0, 0, -1, -1, 0, 0, 0, 1, 0).finished();

    // Compress body-lateral (+/- X) before mapping so resting arms are not
    // drawn as huge abduction. 1 = full body lateral, 0 = ignore abduction.
    double lateral_direction_gain = 0.40;

    // Soft pull of upper-arm toward body-down (-Y) for viz-stable "arms down".
    // 0 = off; 0.25 softly reduces extreme forward tilt from single-view noise.
    double upper_down_blend = 0.20;

    // First-order filters (seconds). Smaller = snappier.
    double body_pose_tau = 0.12;
    double upper_direction_tau = 0.10;
    double forearm_direction_tau = 0.10;
    double elbow_angle_tau = 0.10;

    double max_upper_direction_ang_vel = 4.0;     // rad/s
    double max_forearm_direction_ang_vel = 5.0;   // rad/s
    double max_elbow_angle_vel = 6.0;             // rad/s

    double confidence_high = 0.55;
    double confidence_low = 0.25;
    double pose_timeout_ms = 250.0;

    // Nominal bone lengths for MuJoCo sticks (meters).
    double viz_L1 = 0.25;
    double viz_L2 = 0.22;
    double viz_stick_width = 0.018;

    bool debug_log = true;
    double debug_log_hz = 5.0;
};

struct MappedArmDirs {
    Eigen::Vector3d u_upper = Eigen::Vector3d::UnitX();   // robot base
    Eigen::Vector3d u_fore = Eigen::Vector3d::UnitX();
    double elbow_angle_rad = M_PI_2;  // interior angle acos(dot(u, f))? Spec uses acos(dot(u_upper, u_fore))
    double confidence = 0.0;
    bool valid = false;
};

struct BodySkeletonMapped {
    bool torso_valid = false;
    Eigen::Matrix3d R_camera_body = Eigen::Matrix3d::Identity();  // columns = body axes in camera
    MappedArmDirs left;
    MappedArmDirs right;
};

inline bool is_valid_rotation(const Eigen::Matrix3d& R, double atol = 1e-3) {
    const Eigen::Matrix3d I = R.transpose() * R;
    if ((I - Eigen::Matrix3d::Identity()).lpNorm<Eigen::Infinity>() > atol) {
        return false;
    }
    return std::abs(R.determinant() - 1.0) < atol;
}
