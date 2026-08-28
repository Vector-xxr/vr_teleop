#include "controller/joint_reference_generator.hpp"

#include <algorithm>
#include <cmath>

void JointReferenceGenerator::SetJointLimits(const std::vector<double>& q_min,
                                             const std::vector<double>& q_max) {
    q_min_ = q_min;
    q_max_ = q_max;
    limits_valid_ = !q_min_.empty() && q_min_.size() == q_max_.size();
}

void JointReferenceGenerator::Reset(const std::vector<double>& q) {
    q_ref_ = q;
    qdot_ref_.assign(q.size(), 0.0);
    initialized_ = true;
}

const std::vector<double>& JointReferenceGenerator::Step(const std::vector<double>& q_target,
                                                         double dt, double k_scale,
                                                         double qdot_scale) {
    if (!initialized_ || q_ref_.size() != q_target.size()) {
        Reset(q_target);
        return q_ref_;
    }
    if (dt <= 0.0) {
        return q_ref_;
    }

    const double k = config_.k_joint * std::max(0.1, k_scale);
    const double v_max = config_.qdot_max * std::max(0.1, qdot_scale);
    const double a_max = config_.qddot_max * std::max(0.1, qdot_scale);
    const double dv_max = a_max * dt;
    for (size_t i = 0; i < q_ref_.size(); ++i) {
        // Proportional velocity command toward target.
        double v_cmd = k * (q_target[i] - q_ref_[i]);
        v_cmd = std::clamp(v_cmd, -v_max, v_max);

        // Acceleration limit relative to previous commanded velocity.
        const double dv = std::clamp(v_cmd - qdot_ref_[i], -dv_max, dv_max);
        qdot_ref_[i] += dv;

        q_ref_[i] += qdot_ref_[i] * dt;

        if (limits_valid_ && i < q_min_.size()) {
            const double lo = q_min_[i];
            const double hi = q_max_[i];
            if (q_ref_[i] < lo) {
                q_ref_[i] = lo;
                qdot_ref_[i] = 0.0;
            } else if (q_ref_[i] > hi) {
                q_ref_[i] = hi;
                qdot_ref_[i] = 0.0;
            }
        }
    }
    return q_ref_;
}
