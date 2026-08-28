// Independent teleop mode: proximal joints follow HybrIK SEW dirs;
// distal (q4–q6) follows VR orientation only (gravity-locked), no wrist position IK.
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

struct SkeletonProximalConfig {
    std::string urdf_path;
    std::string root_link = "openarm_body_link0";
    std::string shoulder_link;
    std::string elbow_link;
    std::string wrist_link;
    Eigen::Vector3d wrist_to_tcp = Eigen::Vector3d(0.0, 0.0, 0.186);

    // Same OpenArm Quest map as human_mimic (VR up -> robot +Z).
    Eigen::Matrix3d vr_axes =
        (Eigen::Matrix3d() << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();
    // Quest / OpenXR: +Y is up (gravity reference for wrist orientation lock).
    Eigen::Vector3d vr_world_up = Eigen::Vector3d(0.0, 1.0, 0.0);
    double rot_scale = 1.0;

    // Proximal: ONLY elev / azim / bend scalars (no visual elbow-swivel transplant).
    Eigen::Vector3d skel_down = Eigen::Vector3d(0.0, 1.0, 0.0);  // HybrIK +Y down
    Eigen::Vector3d robot_up = Eigen::Vector3d(0.0, 0.0, 1.0);
    double prox_elev_gain = 1.0;
    double prox_azim_gain = 0.85;  // slightly <1: damp yaw thrash
    double prox_bend_gain = 0.85;
    double prox_elev_min_rad = -0.20;  // slight below horizon ok
    double prox_elev_max_rad = 1.35;   // ~77° up
    double prox_bend_min_rad = 0.45;   // ~26°
    double prox_bend_max_rad = 2.20;   // ~126°
    double w_prox_elbow = 12.0;
    double w_prox_fore = 1.2;  // light: only gravity-droop forearm plane

    // Task weights: distal is orientation-only.
    double w_hand_ori = 8.0;
    double w_cont_prox = 1.2;
    double w_cont_dist = 0.8;

    int prox_ik_iters = 10;
    int dist_ik_iters = 10;
    double ik_lambda = 1e-2;
    double ik_max_step = 0.25;
    double ik_tol = 1e-5;

    double q_slew_prox_radps = 2.0;
    double q_slew_dist_radps = 3.0;

    double pose_max_age_ms = 250.0;
    double pose_min_conf = 0.35;

    // JRG (reuse HumanMimicConfig numbers via mirrored fields below when constructing JRG).
    double k_joint = 10.0;
    double qdot_max = 3.5;
    double qddot_max = 40.0;

    std::vector<double> q_nominal;

    double gripper_open = 0.0;
    double gripper_closed = 0.04;
    double gripper_speed = 0.05;

    double quest_timeout_sec = 0.25;
    bool debug_log = true;
    double debug_log_hz = 5.0;
};
