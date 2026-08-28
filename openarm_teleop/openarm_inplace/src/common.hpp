#pragma once

#include <safety/safety_filter.hpp>
#include <openarm_constants.hpp>

#include <atomic>
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

// Shared signal handling
inline std::atomic<bool> keep_running{true};

inline void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nSignal " << signal << " detected. Exiting..." << std::endl;
        keep_running = false;
    }
}

// Safety config for real hardware
inline SafetyConfig make_default_safety_config(size_t total_joints) {
    SafetyConfig config;
    config.min_position.resize(total_joints);
    config.max_position.resize(total_joints);
    config.max_velocity.resize(total_joints);
    config.max_acceleration.resize(total_joints);
    config.max_effort.resize(total_joints);
    for (size_t i = 0; i < total_joints; ++i) {
        const size_t idx = i < NJOINTS ? i : NJOINTS - 1;
        config.min_position[i] = position_limit_min_F[idx];
        config.max_position[i] = position_limit_max_F[idx];
        config.max_velocity[i] = std::min(velocity_limit_F[idx], 2.0);
        config.max_acceleration[i] = 8.0;
        config.max_effort[i] = effort_limit_F[idx];
    }
    return config;
}

// Safety config for MuJoCo simulation (relaxed limits)
inline SafetyConfig make_sim_safety_config(size_t total_joints) {
    SafetyConfig config;
    config.min_position.resize(total_joints);
    config.max_position.resize(total_joints);
    config.max_velocity.resize(total_joints);
    config.max_acceleration.resize(total_joints);
    config.max_effort.resize(total_joints);
    for (size_t i = 0; i < total_joints; ++i) {
        const size_t idx = i < NJOINTS ? i : NJOINTS - 1;
        config.min_position[i] = position_limit_min_F[idx];
        config.max_position[i] = position_limit_max_F[idx];
        config.max_velocity[i] = std::max(velocity_limit_F[idx], 12.0);
        config.max_acceleration[i] = 0.0;
        config.max_effort[i] = effort_limit_F[idx];
    }
    config.max_first_step_delta = 2.0;
    return config;
}

inline double env_double_or_common(const char* name, double fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return fallback;
    }
    try {
        return std::stod(v);
    } catch (...) {
        return fallback;
    }
}

inline bool env_bool_or_common(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return fallback;
    }
    const std::string s(v);
    if (s == "1" || s == "true" || s == "TRUE" || s == "yes" || s == "YES" || s == "on" ||
        s == "ON") {
        return true;
    }
    if (s == "0" || s == "false" || s == "FALSE" || s == "no" || s == "NO" || s == "off" ||
        s == "OFF") {
        return false;
    }
    return fallback;
}

// Follower MIT gains for Quest VR teleop + dual_replay (keep both in sync).
// Default: raw config/follower.yaml (same level as unilateral / mujoco_panel).
// Optional Quest soft-scale A/B: OPENARM_QUEST_MIT_SCALE=1
//   proximal Kp×0.30 Kd×1.8; distal j4–j6 Kp×0.85 Kd×2.5; gripper Kp=6 Kd=0.45.
// Real gripper default Kp=10 (yaml used to be 30 and overheated under stall).
inline void apply_quest_vr_follower_mit_gains(std::vector<double>* kp, std::vector<double>* kd,
                                             bool sim = false,
                                             const char* log_tag = "follower-mit") {
    if (kp == nullptr || kd == nullptr) {
        return;
    }
    const bool scale = !sim && env_bool_or_common("OPENARM_QUEST_MIT_SCALE", false);
    if (scale) {
        // Proximal: conservative. Distal j4–j6: keep stiff vs orientation droop.
        for (size_t i = 0; i < kp->size(); ++i) {
            if (i >= 4 && i <= 6) {
                (*kp)[i] *= 0.85;
                if (i < kd->size()) {
                    (*kd)[i] *= 2.5;
                }
            } else {
                (*kp)[i] *= 0.30;
                if (i < kd->size()) {
                    (*kd)[i] *= 1.8;
                }
            }
        }
    }
    if (!kp->empty()) {
        const size_t g = kp->size() - 1;
        // Real: keep grip soft enough for continuous grasp-hold without rotor OT.
        const double default_grip_kp = sim ? (*kp)[g] : (scale ? 6.0 : 10.0);
        const double default_grip_kd =
            (g < kd->size()) ? (sim ? (*kd)[g] : 0.45) : 0.45;
        (*kp)[g] = env_double_or_common("OPENARM_GRIPPER_KP", default_grip_kp);
        if (g < kd->size()) {
            (*kd)[g] = env_double_or_common("OPENARM_GRIPPER_KD", default_grip_kd);
        }
        std::cout << "[" << (log_tag != nullptr ? log_tag : "follower-mit")
                  << "] MIT gains " << (scale ? "Quest soft-scale" : "follower.yaml (raw)")
                  << "; gripper Kp=" << (*kp)[g]
                  << " Kd=" << (g < kd->size() ? (*kd)[g] : 0.0) << std::endl;
    }
}
