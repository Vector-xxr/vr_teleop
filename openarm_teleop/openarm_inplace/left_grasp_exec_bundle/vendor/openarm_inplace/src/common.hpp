#pragma once

#include <safety/safety_filter.hpp>
#include <openarm_constants.hpp>

#include <atomic>
#include <csignal>
#include <iostream>
#include <algorithm>

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
