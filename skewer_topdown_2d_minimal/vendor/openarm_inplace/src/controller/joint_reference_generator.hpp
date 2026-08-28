// Velocity/acceleration-limited joint reference generator.
//
// Drives the emitted joint reference q_ref smoothly toward the IK target
// q_target with a first-order proportional law plus per-joint speed and
// acceleration saturation. Runs every control tick.
#pragma once

#include <vector>

#include "controller/human_mimic_types.hpp"

class JointReferenceGenerator {
public:
    explicit JointReferenceGenerator(const HumanMimicConfig& config) : config_(config) {}

    void SetJointLimits(const std::vector<double>& q_min, const std::vector<double>& q_max);

    // Snap the reference to a known configuration (e.g. on clutch / re-anchor).
    void Reset(const std::vector<double>& q);

    // Advance one tick toward q_target; returns the new q_ref.
    // k_scale / qdot_scale > 1 follows the IK target more tightly (panel hold-to-jog).
    const std::vector<double>& Step(const std::vector<double>& q_target, double dt,
                                    double k_scale = 1.0, double qdot_scale = 1.0);

    const std::vector<double>& reference() const { return q_ref_; }
    bool initialized() const { return initialized_; }

private:
    HumanMimicConfig config_;
    std::vector<double> q_ref_;
    std::vector<double> qdot_ref_;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    bool limits_valid_ = false;
    bool initialized_ = false;
};
