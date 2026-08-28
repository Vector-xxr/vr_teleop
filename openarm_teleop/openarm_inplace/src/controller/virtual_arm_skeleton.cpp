#include "controller/virtual_arm_skeleton.hpp"

#include <algorithm>
#include <cmath>

namespace {
Eigen::Vector3d project_perp(const Eigen::Vector3d& v, const Eigen::Vector3d& axis) {
    // Component of v perpendicular to unit `axis`.
    return v - axis * (axis.dot(v));
}
}  // namespace

bool VirtualArmSkeleton::SwivelCircle(const Eigen::Vector3d& shoulder, const Eigen::Vector3d& wrist,
                                      double L1, double L2, Eigen::Vector3d* center,
                                      Eigen::Vector3d* axis, Eigen::Vector3d* e1,
                                      Eigen::Vector3d* e2, double* h) const {
    const Eigen::Vector3d sw = wrist - shoulder;
    const double d = sw.norm();
    if (d < 1e-6) {
        return false;
    }
    const Eigen::Vector3d u = sw / d;
    const double a = (L1 * L1 - L2 * L2 + d * d) / (2.0 * d);
    const double h2 = L1 * L1 - a * a;
    const double hh = std::sqrt(std::max(0.0, h2));

    // Canonical, repeatable in-plane basis: e1 = projection of the fixed world
    // "down" reference onto the circle plane (so phi=0 ~ elbow droop). Fall back to
    // the outward reference, then to an arbitrary perpendicular, if degenerate.
    Eigen::Vector3d ref = config_.downward_dir;
    Eigen::Vector3d p = project_perp(ref, u);
    if (p.norm() < 1e-6) {
        p = project_perp(config_.outward_dir, u);
    }
    if (p.norm() < 1e-6) {
        ref = std::abs(u.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
        p = project_perp(ref, u);
    }
    if (p.norm() < 1e-9) {
        return false;
    }
    const Eigen::Vector3d ee1 = p.normalized();
    Eigen::Vector3d ee2 = u.cross(ee1).normalized();
    // Handedness: +phi should move the elbow toward outward_dir (left +Y / right −Y).
    // Without this, e2 = u×e1 is world-right-handed and +phi is always toward +Y —
    // outward for the left arm but inward for the right, which biases right elbows medial.
    if (ee2.dot(config_.outward_dir) < 0.0) {
        ee2 = -ee2;
    }

    *center = shoulder + u * a;
    *axis = u;
    *e1 = ee1;
    *e2 = ee2;
    *h = hh;
    return true;
}

ElbowTarget VirtualArmSkeleton::compute_elbow_target_from_phi(const Eigen::Vector3d& shoulder,
                                                              const Eigen::Vector3d& wrist,
                                                              double L1, double L2,
                                                              double phi) const {
    ElbowTarget et;
    et.phi = phi;
    Eigen::Vector3d center, axis, e1, e2;
    double h = 0.0;
    if (!SwivelCircle(shoulder, wrist, L1, L2, &center, &axis, &e1, &e2, &h)) {
        et.degenerate = true;
        et.reachable = false;
        return et;
    }
    et.h = h;
    et.E = center + h * (std::cos(phi) * e1 + std::sin(phi) * e2);
    Eigen::Vector3d bend = et.E - center;
    et.bend_dir = bend.norm() > 1e-9 ? bend.normalized() : e1;
    const double d = (wrist - shoulder).norm();
    et.reachable = (d > 1e-6) && (d <= L1 + L2) && (d >= std::abs(L1 - L2));
    et.degenerate = (h < 1e-3);
    return et;
}

void VirtualArmSkeleton::SetContinuityBend(const Eigen::Vector3d& bend) {
    if (bend.norm() > 1e-6) {
        prev_bend_ = bend.normalized();
        prev_valid_ = true;
    }
}

bool VirtualArmSkeleton::PhiFromPose(const Eigen::Vector3d& shoulder, const Eigen::Vector3d& elbow,
                                     const Eigen::Vector3d& wrist, double* phi, double* h) const {
    if (phi == nullptr) {
        return false;
    }
    Eigen::Vector3d center, axis, e1, e2;
    double hh = 0.0;
    if (!SwivelCircle(shoulder, wrist, L1_, L2_, &center, &axis, &e1, &e2, &hh)) {
        return false;
    }
    if (h != nullptr) {
        *h = hh;
    }
    if (hh < 1e-6) {
        *phi = 0.0;
        return false;
    }
    const Eigen::Vector3d r = elbow - center;
    *phi = std::atan2(r.dot(e2), r.dot(e1));
    return std::isfinite(*phi);
}

void VirtualArmSkeleton::Calibrate(const Eigen::Vector3d& shoulder, const Eigen::Vector3d& elbow,
                                   const Eigen::Vector3d& wrist) {
    const Eigen::Vector3d sw = wrist - shoulder;
    const double d = sw.norm();
    if (d < 1e-6) {
        prev_valid_ = false;
        return;
    }
    const Eigen::Vector3d n = sw / d;
    Eigen::Vector3d bend = project_perp(elbow - shoulder, n);
    if (bend.norm() < 1e-6) {
        prev_valid_ = false;
        return;
    }
    prev_bend_ = bend.normalized();
    prev_valid_ = true;
}

SkeletonResult VirtualArmSkeleton::Solve(const Eigen::Vector3d& p_shoulder,
                                         const Eigen::Vector3d& p_wrist_des,
                                         bool advance_continuity) {
    SkeletonResult res;
    res.shoulder = p_shoulder;

    Eigen::Vector3d sw = p_wrist_des - p_shoulder;
    double d = sw.norm();

    // Clamp reach to keep off the full-extension / full-fold singularities.
    const double d_min = std::abs(L1_ - L2_) + config_.reach_margin_m;
    const double d_max = L1_ + L2_ - config_.reach_margin_m;
    Eigen::Vector3d wrist_clamped = p_wrist_des;
    res.reachable = true;
    if (d < 1e-6) {
        // Degenerate: wrist coincident with shoulder. Push it out along +x.
        sw = Eigen::Vector3d(1.0, 0.0, 0.0);
        d = 1e-6;
    }
    Eigen::Vector3d n = sw / d;
    if (d > d_max) {
        d = d_max;
        wrist_clamped = p_shoulder + n * d;
        res.reachable = false;
    } else if (d < d_min) {
        d = d_min;
        wrist_clamped = p_shoulder + n * d;
        res.reachable = false;
    }
    res.wrist_clamped = wrist_clamped;

    // Circle of feasible elbow positions (law of cosines).
    const double a = (L1_ * L1_ - L2_ * L2_ + d * d) / (2.0 * d);
    const double h2 = L1_ * L1_ - a * a;
    const double h = std::sqrt(std::max(0.0, h2));
    const Eigen::Vector3d center = p_shoulder + n * a;

    // Desired bend direction: blend continuity + outward + upward + downward, then
    // project perpendicular to the shoulder->wrist axis.
    //
    // Default weights favor gravity droop (natural hanging elbow). Optional upward /
    // anti-inward boosts stay available via config but default to off.
    const Eigen::Vector3d out_u = config_.outward_dir.normalized();
    const Eigen::Vector3d down_u = config_.downward_dir.normalized();
    const Eigen::Vector3d up_u = config_.upward_dir.normalized();
    const double lateral_out = std::clamp(n.dot(out_u), 0.0, 1.0);  // 1 = fully outward
    const double lateral_in = std::clamp(-n.dot(out_u), 0.0, 1.0);  // 1 = fully inward/medial
    const double elev = std::clamp(n.z(), 0.0, 1.0);                // 1 = wrist straight up
    const double elev_fade =
        std::clamp(1.0 - elev * std::max(0.0, config_.elevation_droop_fade), 0.0, 1.0);
    const double medial_droop =
        std::clamp(1.0 - lateral_in * std::max(0.0, config_.medial_droop_fade), 0.0, 1.0);
    const double w_out =
        config_.w_outward *
        ((1.0 - lateral_out) + std::max(0.0, config_.anti_inward_boost) * lateral_in);
    const double cont_fade_lat = std::clamp(config_.lateral_continuity_fade, 0.0, 1.0);
    const double cont_fade_med = std::clamp(config_.medial_continuity_fade, 0.0, 1.0);
    const double w_prev =
        config_.w_prev * (1.0 - cont_fade_lat * lateral_out) * (1.0 - cont_fade_med * lateral_in);
    const double w_down =
        (config_.w_downward + config_.lateral_droop_boost * lateral_out) * elev_fade *
        medial_droop;
    const double w_up = config_.w_upward * (1.0 - 0.5 * elev);

    Eigen::Vector3d raw = Eigen::Vector3d::Zero();
    if (prev_valid_) {
        raw += w_prev * prev_bend_;
    }
    raw += w_out * out_u;
    if (w_up > 1e-9) {
        raw += w_up * up_u;
    }
    raw += w_down * down_u;

    Eigen::Vector3d bend = project_perp(raw, n);
    if (bend.norm() < 1e-6) {
        // Fall back to previous (also projected) or an arbitrary perpendicular.
        Eigen::Vector3d fallback = prev_valid_ ? prev_bend_ : Eigen::Vector3d(0.0, 0.0, -1.0);
        bend = project_perp(fallback, n);
        if (bend.norm() < 1e-6) {
            Eigen::Vector3d ref = std::abs(n.z()) < 0.9 ? Eigen::Vector3d::UnitZ()
                                                        : Eigen::Vector3d::UnitX();
            bend = project_perp(ref, n);
        }
    }
    bend.normalize();

    // Near full extension the circle radius collapses and bend direction from
    // projection noise can flip (±) with no geometric meaning. Prefer the last
    // trusted continuity direction so an exit from straight does not invent a
    // mirror-image (vision-style) elbow branch.
    const double h_sing = std::max(0.0, config_.elbow_singularity_h_m);
    if (h < h_sing && prev_valid_) {
        Eigen::Vector3d keep = project_perp(prev_bend_, n);
        if (keep.norm() > 1e-6) {
            bend = keep.normalized();
        }
    }

    res.bend_dir = bend;
    res.elbow_des = center + h * bend;
    res.h_radius = h;

    // Report the base swivel angle in the canonical basis so callers can enumerate
    // orientation-aware candidates around it (same basis as compute_elbow_target_from_phi).
    Eigen::Vector3d c2, u2, e1, e2;
    double h_c = 0.0;
    if (SwivelCircle(p_shoulder, wrist_clamped, L1_, L2_, &c2, &u2, &e1, &e2, &h_c)) {
        const Eigen::Vector3d r = res.elbow_des - c2;
        res.phi_base = std::atan2(r.dot(e2), r.dot(e1));
        res.h_radius = h_c;
    }

    // Only advance continuity when the circle is well-conditioned and allowed.
    if (advance_continuity && h >= h_sing) {
        prev_bend_ = bend;
        prev_valid_ = true;
    }
    return res;
}
