#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller/dynamics.hpp"
#include "controller/human_mimic_kinematics.hpp"
#include "controller/human_mimic_types.hpp"
#include "controller/joint_reference_generator.hpp"
#include "controller/skeleton_proximal_types.hpp"
#include "controller/cartesian_types.hpp"
#include "io/skeleton_pose_receiver.hpp"
#include "robot_state.hpp"

// Proximal (q0–q3) follows HybrIK SEW directions; distal (q4–q6) follows VR
// orientation only (gravity-locked quaternion map). No wrist-position IK.
// Completely separate from HumanMimicController.
class SkeletonProximalController {
public:
    SkeletonProximalController(Dynamics* dynamics, SkeletonProximalConfig config,
                               std::string log_prefix = {});

    bool Init();

    // Feed the latest SEW pose for THIS arm (already left/right selected).
    void set_skeleton_pose(const SkeletonPoseArm& arm, bool valid, double age_ms);

    bool step(const std::vector<JointState>& current_joints, const QuestPoseSample& sample,
              double dt, std::vector<JointState>* joint_reference);

    bool clutch_active() const { return clutch_active_; }
    bool quest_timeout_active() const { return quest_timeout_active_; }
    bool pose_valid() const { return pose_valid_; }
    double pose_conf() const { return pose_arm_.conf; }
    double gripper_target() const { return gripper_target_; }
    double gripper_measured() const { return gripper_meas_; }
    int gripper_motion_code() const { return static_cast<int>(gripper_motion_); }
    bool last_button_close() const { return last_button_close_; }
    bool last_button_open() const { return last_button_open_; }

private:
    enum class GripperMotion { HOLD = 0, CLOSING = 1, OPENING = 2 };

    void activate_clutch(const std::vector<double>& q_meas, const QuestPoseSample& pose);
    void deactivate_clutch();
    void update_gripper_from_buttons(const QuestPoseSample& sample,
                                     const std::vector<JointState>& current_joints, double dt);
    void ramp_arm_reset(double dt);
    void fill_reference(const std::vector<JointState>& current_joints,
                        const std::vector<double>& q_arm, std::vector<JointState>* reference) const;

    Eigen::Matrix3d vr_to_robot_axes() const;
    Eigen::Matrix3d gravity_basis(const Eigen::Vector3d& up) const;
    static double wrap_pi(double a);
    void dir_to_elev_azim(const Eigen::Vector3d& u, const Eigen::Matrix3d& B, double* elev,
                          double* azim) const;
    Eigen::Vector3d elev_azim_to_dir(double elev, double azim, const Eigen::Matrix3d& B) const;
    double elbow_interior_angle(const Eigen::Vector3d& u_upper,
                                const Eigen::Vector3d& u_fore) const;
    // Forearm with gravity droop plane (NO visual swivel).
    Eigen::Vector3d forearm_from_bend_droop(const Eigen::Vector3d& u_upper, double bend_rad) const;
    bool map_proximal_dirs(Eigen::Vector3d* u_robot, Eigen::Vector3d* f_robot) const;
    bool calibrate_human_to_robot(const std::vector<double>& q_meas);
    bool solve_proximal(const Eigen::Vector3d& S, double dt);
    // Orientation-only distal IK on q4–q6 (q0–q3 frozen).
    bool solve_distal(const Eigen::Matrix3d& R_hand_des, double dt);
    Eigen::Matrix3d compute_hand_target(const QuestPoseSample& sample, double dt);
    Eigen::Quaterniond filter_orientation(const Eigen::Quaterniond& raw, double dt);
    bool tcp_pose(const std::vector<double>& q, Eigen::Vector3d* p, Eigen::Matrix3d* R) const;
    void maybe_log(const char* tag);
    void clamp_q(std::vector<double>* q) const;
    void slew_segment(std::vector<double>* q, const std::vector<double>& q_goal, size_t i0,
                      size_t i1, double max_radps, double dt) const;

    Dynamics* dynamics_ = nullptr;
    SkeletonProximalConfig config_;
    std::string log_prefix_;
    std::unique_ptr<HumanMimicKinematics> kin_;
    std::unique_ptr<JointReferenceGenerator> jrg_;

    size_t arm_dof_ = 0;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    std::vector<double> q_target_;
    std::vector<double> q_nominal_;

    bool initialized_ = false;
    bool clutch_active_ = false;
    bool quest_timeout_active_ = false;
    bool last_grip_ = false;
    bool last_trigger_ = false;
    bool last_button_close_ = false;
    bool last_button_open_ = false;

    Eigen::Quaterniond hand_ref_rot_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond robot_ref_rot_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_filt_ = Eigen::Quaterniond::Identity();
    bool ori_filt_valid_ = false;

    // Proximal elev/azim/bend clutch anchors (gravity spherical).
    bool prox_calib_valid_ = false;
    double elev_h0_ = 0.0;
    double azim_h0_ = 0.0;
    double elev_r0_ = 0.0;
    double azim_r0_ = 0.0;
    double bend_h0_ = 1.57;
    double bend_r0_ = 1.57;
    double L1_ = 0.22;
    double L2_ = 0.22;

    SkeletonPoseArm pose_arm_;
    bool pose_valid_ = false;
    double pose_age_ms_ = 1e9;

    GripperMotion gripper_motion_ = GripperMotion::HOLD;
    double gripper_target_ = 0.0;
    double gripper_meas_ = 0.0;

    double dbg_e_elbow_ = 0.0;
    double dbg_e_ori_ = 0.0;
    double dbg_bend_des_ = 0.0;
    double dbg_elev_des_ = 0.0;
    double dbg_azim_des_ = 0.0;
    int dbg_prox_iters_ = 0;
    int dbg_dist_iters_ = 0;

    std::chrono::steady_clock::time_point last_log_time_{};
    uint64_t log_tick_ = 0;
};
