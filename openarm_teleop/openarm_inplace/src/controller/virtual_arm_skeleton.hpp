// Virtual human arm skeleton: places the elbow deterministically given the
// shoulder anchor and the desired wrist center.
//
// The 7-DOF redundancy is resolved anatomically: for a fixed shoulder (S) and
// wrist (W), the elbow (E) lies on a circle whose plane is perpendicular to S->W.
// We select the point on that circle using a continuous bend direction blended
// from the previous direction (continuity), an "outward" bias, and a "downward"
// (gravity droop) bias. This makes the arm posture path-independent enough to be
// returnable while staying human-like and singularity-averse.
#pragma once

#include <Eigen/Dense>

#include "controller/human_mimic_types.hpp"

class VirtualArmSkeleton {
public:
    explicit VirtualArmSkeleton(const HumanMimicConfig& config) : config_(config) {}

    // Seed the continuity direction from the current (measured) elbow geometry so
    // the first solve does not jump. Call on clutch activation.
    void Calibrate(const Eigen::Vector3d& shoulder, const Eigen::Vector3d& elbow,
                   const Eigen::Vector3d& wrist);

    void SetSegmentLengths(double L1, double L2) {
        L1_ = L1;
        L2_ = L2;
    }
    double L1() const { return L1_; }
    double L2() const { return L2_; }

    // Compute the desired elbow position for the given shoulder + desired wrist.
    // When `advance_continuity` is false, still reports phi_base / elbow_des for
    // diagnostics but does not update the internal bend memory.
    SkeletonResult Solve(const Eigen::Vector3d& p_shoulder, const Eigen::Vector3d& p_wrist_des,
                         bool advance_continuity = true);

    // Deterministic elbow placement for an explicit swivel angle `phi` (rad) in the
    // same canonical basis used by Solve(). Self-contained (recomputes geometry) so
    // repeated calls with the same inputs are exactly repeatable.
    ElbowTarget compute_elbow_target_from_phi(const Eigen::Vector3d& shoulder,
                                              const Eigen::Vector3d& wrist, double L1, double L2,
                                              double phi) const;

    // Measure SEW / swivel angle of a realized (S,E,W) triple in the canonical basis.
    // Used by the 1D elbow-phi IK residual. Returns false if degenerate.
    bool PhiFromPose(const Eigen::Vector3d& shoulder, const Eigen::Vector3d& elbow,
                     const Eigen::Vector3d& wrist, double* phi, double* h = nullptr) const;

    // Override the continuity bend direction (e.g. after the orientation-aware
    // search commits to a swivel that differs from the base skeleton), so the next
    // Solve() stays continuous with what was actually executed.
    void SetContinuityBend(const Eigen::Vector3d& bend);

    // Build the swivel circle for (shoulder, wrist, L1, L2). Returns false if the
    // geometry is degenerate (coincident points). e1/e2 form a canonical, repeatable
    // orthonormal basis of the circle plane (e1 ~ gravity-down projection).
    bool SwivelCircle(const Eigen::Vector3d& shoulder, const Eigen::Vector3d& wrist, double L1,
                      double L2, Eigen::Vector3d* center, Eigen::Vector3d* axis,
                      Eigen::Vector3d* e1, Eigen::Vector3d* e2, double* h) const;

private:

    HumanMimicConfig config_;
    double L1_ = 0.28;
    double L2_ = 0.22;
    Eigen::Vector3d prev_bend_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    bool prev_valid_ = false;
};
