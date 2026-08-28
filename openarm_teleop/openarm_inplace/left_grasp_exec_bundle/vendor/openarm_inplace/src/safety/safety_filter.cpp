#include <safety/safety_filter.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

SafetyFilter::SafetyFilter(SafetyConfig config) : config_(std::move(config)) {}

void SafetyFilter::reset(const std::vector<JointState>& initial_cmd) {
    last_cmd_ = initial_cmd;
    initialized_ = true;
}

void SafetyFilter::set_mode(SafetyMode mode) { mode_ = mode; }

SafetyStatus SafetyFilter::filter(const std::vector<JointState>& raw_cmd,
                                  const std::vector<JointState>& current_state, double dt,
                                  std::vector<JointState>* safe_cmd) {
    (void)dt;
    SafetyStatus status;
    if (safe_cmd == nullptr) {
        status.ok = false;
        status.emergency_stop = true;
        status.reason = "safe_cmd is null";
        return status;
    }

    safe_cmd->clear();

    if (mode_ == SafetyMode::Stop) {
        *safe_cmd = current_state;
        for (auto& joint : *safe_cmd) {
            joint.velocity = 0.0;
            joint.effort = 0.0;
        }
        status.ok = false;
        status.emergency_stop = true;
        status.reason = "safety stop mode";
        reset(*safe_cmd);
        return status;
    }

    if (raw_cmd.empty() || raw_cmd.size() != current_state.size()) {
        status.ok = false;
        status.emergency_stop = true;
        status.reason = "command/state size mismatch";
        return status;
    }

    if (!initialized_ || last_cmd_.size() != raw_cmd.size()) {
        reset(current_state);
    }

    std::ostringstream reason;
    *safe_cmd = raw_cmd;

    for (size_t i = 0; i < safe_cmd->size(); ++i) {
        JointState& out = (*safe_cmd)[i];
        if (!std::isfinite(out.position) || !std::isfinite(out.velocity) ||
            !std::isfinite(out.effort)) {
            out = i < last_cmd_.size() ? last_cmd_[i] : current_state[i];
            out.velocity = 0.0;
            out.effort = 0.0;
            status.ok = false;
            status.clamped = true;
            reason << "non-finite joint " << i << "; ";
        }

        if (i < config_.max_effort.size() && config_.max_effort[i] > 0.0 &&
            std::isfinite(out.effort)) {
            const double limit = config_.max_effort[i];
            const double original_effort = out.effort;
            out.effort = std::clamp(out.effort, -limit, limit);
            if (out.effort != original_effort) {
                status.clamped = true;
                reason << "feedforward effort clamp joint " << i << " "
                       << original_effort << " -> " << out.effort << "; ";
            }
        }
    }

    last_cmd_ = *safe_cmd;
    initialized_ = true;
    status.reason = reason.str();
    return status;
}
