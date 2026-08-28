#pragma once

#include <controller/body_skeleton_types.hpp>
#include <controller/skeleton_retarget_types.hpp>

#include <optional>

// Phase-2: torso frame + bone directions + EMA filters. No robot joint solve.
class BodySkeletonFilter {
public:
    explicit BodySkeletonFilter(SkeletonRetargetConfig config = {});

    void set_config(const SkeletonRetargetConfig& config);
    const SkeletonRetargetConfig& config() const { return config_; }

    void reset();

    // dt in seconds. age_ms from receiver. Returns nullopt if pose unusable.
    std::optional<BodySkeletonMapped> update(const BodyPoseSample& sample, double age_ms,
                                             double dt);

    const BodySkeletonMapped& latest() const { return latest_; }

private:
    static bool build_torso(const BodyPoseSample& sample, Eigen::Matrix3d* R_camera_body);
    static bool arm_dirs_camera(const BodyJoint3d& S, const BodyJoint3d& E, const BodyJoint3d& W,
                                Eigen::Vector3d* u_upper, Eigen::Vector3d* u_fore,
                                double* elbow_rad, double* conf);
    Eigen::Vector3d filter_unit(Eigen::Vector3d* state, bool* has, const Eigen::Vector3d& meas,
                                double tau, double max_ang_vel, double dt) const;
    double filter_scalar(double* state, bool* has, double meas, double tau, double max_vel,
                         double dt) const;

    SkeletonRetargetConfig config_;
    BodySkeletonMapped latest_;

    bool has_R_ = false;
    Eigen::Matrix3d R_filt_ = Eigen::Matrix3d::Identity();

    bool has_lu_ = false, has_lf_ = false, has_ru_ = false, has_rf_ = false;
    Eigen::Vector3d lu_, lf_, ru_, rf_;
    bool has_le_ = false, has_re_ = false;
    double le_ = M_PI_2, re_ = M_PI_2;
};
