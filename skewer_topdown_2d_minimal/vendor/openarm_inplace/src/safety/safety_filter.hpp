#pragma once

#include <robot_state.hpp>

#include <string>
#include <vector>

enum class SafetyMode {
    Normal,
    Stop,
};

struct SafetyConfig {
    std::vector<double> min_position;
    std::vector<double> max_position;
    std::vector<double> max_velocity;
    std::vector<double> max_acceleration;
    std::vector<double> max_effort;
    double max_first_step_delta = 0.35;
};

struct SafetyStatus {
    bool ok = true;
    bool clamped = false;
    bool emergency_stop = false;
    std::string reason;
};

class SafetyFilter {
public:
    explicit SafetyFilter(SafetyConfig config);

    SafetyStatus filter(const std::vector<JointState>& raw_cmd,
                        const std::vector<JointState>& current_state, double dt,
                        std::vector<JointState>* safe_cmd);

    void reset(const std::vector<JointState>& initial_cmd);
    void set_mode(SafetyMode mode);

private:
    SafetyConfig config_;
    SafetyMode mode_ = SafetyMode::Normal;
    bool initialized_ = false;
    std::vector<JointState> last_cmd_;
};
