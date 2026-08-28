// HUMAN_MIMIC teleoperation controller (single arm).
//
// Independent of CartesianController: maps VR hand pose -> desired TCP pose,
// resolves wrist/elbow geometry with a virtual human
// skeleton, solves an absolute whole-arm IK for q_target, and streams a
// velocity/acceleration-limited joint reference. Same step() signature as
// CartesianController so the reference thread can drive either.
//
// Stable data flow (never bypassed):
//   VR target -> VirtualArmSkeleton -> HumanMimicIkSolver -> single q_target
//              -> JointReferenceGenerator
// The optional HybrIK elbow-swivel hint only changes which elbow XYZ target
// the (single) whole-arm IK is asked to reach. It never post-edits q.
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "controller/cartesian_types.hpp"
#include "controller/dynamics.hpp"
#include "controller/human_mimic_ik_solver.hpp"
#include "controller/human_mimic_kinematics.hpp"
#include "controller/human_mimic_types.hpp"
#include "controller/joint_reference_generator.hpp"
#include "controller/reference_smoother.hpp"
#include "controller/virtual_arm_skeleton.hpp"
#include "robot_state.hpp"

class HumanMimicController {
public:
    HumanMimicController(Dynamics* dynamics, HumanMimicConfig config, std::string log_prefix = {});

    bool Init();

    // Returns true on a normal step. Fills `joint_reference` (arm + gripper hold).
    bool step(const std::vector<JointState>& current_joints, const QuestPoseSample& sample,
              double dt, std::vector<JointState>* joint_reference);

    // Optional HybrIK elbow swivel (bridge frame). Sign/offset applied in solve.
    void set_elbow_hint(double phi_human, bool valid) {
        elbow_hint_phi_raw_ = phi_human;
        elbow_hint_valid_ = valid;
    }

    bool clutch_active() const { return clutch_active_; }
    bool quest_timeout_active() const { return quest_timeout_active_; }
    bool elbow_hint_valid() const { return elbow_hint_valid_; }
    double elbow_hint_phi_used() const { return elbow_hint_phi_used_; }
    double elbow_cmd_phi() const { return elbow_cmd_phi_; }
    double skeleton_phi_base() const { return dbg_phi_base_; }
    // Diagnostics for the gripper button state machine.
    double gripper_target() const { return gripper_target_; }
    double gripper_measured() const { return gripper_meas_; }
    int gripper_motion_code() const { return static_cast<int>(gripper_motion_); }
    bool gripper_grasp_holding() const {
        return gripper_motion_ == GripperMotion::GraspHolding;
    }
    bool last_button_close() const { return last_button_close_; }
    bool last_button_open() const { return last_button_open_; }

    // Panel jog: shift TCP target in robot base frame (m). Works with or without grip.
    void nudge_tcp_robot(const Eigen::Vector3d& delta_robot);
    // Continuous panel jog velocity in robot/world teleop frame (m/s). Zero = stopped.
    // Polled every control tick; works without Quest grip.
    void set_panel_jog_velocity(const Eigen::Vector3d& vel_robot_mps) {
        panel_jog_vel_ = vel_robot_mps.allFinite() ? vel_robot_mps : Eigen::Vector3d::Zero();
        if (panel_jog_vel_.norm() < 1e-9) {
            panel_tcp_des_valid_ = false;
            panel_speed_filt_ = 0.0;
        }
    }
    // Robot-base orthonormal basis for panel jog (body +Z up, +X forward, +Y left).
    Eigen::Vector3d panel_jog_up_axis() const;
    Eigen::Vector3d panel_jog_left_axis() const;
    Eigen::Vector3d panel_jog_forward_axis() const;

    // Commanded TCP from the joint reference (what the arm is actually holding).
    bool commanded_tcp_pose(Eigen::Vector3d* p, Eigen::Matrix3d* R) const;
    // FK TCP for an arbitrary arm-q vector (measured or commanded).
    bool fk_tcp_pose(const std::vector<double>& q, Eigen::Vector3d* p, Eigen::Matrix3d* R) const {
        return tcp_pose(q, p, R);
    }
    // Freeze hand orientation (master-slave couple / position-only right teleop).
    void set_orientation_lock(bool on, const Eigen::Matrix3d& R_lock = Eigen::Matrix3d::Identity());
    bool orientation_lock_active() const { return ori_lock_active_; }
    // Drive TCP to an absolute robot-base pose (couple follow on the left arm).
    void drive_tcp_robot(const Eigen::Vector3d& p_tcp, const Eigen::Matrix3d& R_hand, double dt);
    // Left-arm couple follow: skip VR teleop; driven only by drive_tcp_robot.
    // On engage: seeds elbow φ from current FK and freezes the φ branch (no antipode flip).
    void set_couple_follow(bool on);
    bool couple_follow_active() const { return couple_follow_active_; }
    // Soft-exit couple/circle: absorb current Quest sample into VR map anchors and
    // seed robot_ref from the commanded TCP so the next teleop tick has ~zero delta.
    void reanchor_vr_map(const std::vector<double>& q_meas, const QuestPoseSample& pose);
    int last_phi_flip() const { return dbg_phi_flip_; }
    double last_wrist_err() const { return dbg_wrist_err_; }
    // Current arm joint command (size arm_dof_); empty if not ready.
    const std::vector<double>& q_target() const { return q_target_; }
    size_t arm_dof() const { return arm_dof_; }

private:
    void activate_clutch(const std::vector<double>& q_meas, const QuestPoseSample& pose);
    void deactivate_clutch();
    void recenter_orientation(const std::vector<double>& q_meas, const QuestPoseSample& pose);
    void hold_reference(const std::vector<JointState>& current_joints,
                        const std::vector<double>& q_hold, std::vector<JointState>* reference);
    void fill_reference(const std::vector<JointState>& current_joints,
                        const std::vector<double>& q_arm, std::vector<JointState>* reference) const;
    void update_gripper_from_buttons(const QuestPoseSample& sample,
                                     const std::vector<JointState>& current_joints, double dt);
    void ramp_arm_reset(double dt);

    // Orientation pipeline.
    Eigen::Quaterniond filter_orientation(const Eigen::Quaterniond& raw, double dt);
    Eigen::Matrix3d compute_hand_target(const QuestPoseSample& sample, double dt);

    // Virtual-skeleton whole-arm solve: updates q_target_ in place and returns whether
    // a fresh solution was accepted (false => held previous target).
    bool solve_base_skeleton(const std::vector<double>& q_meas, const Eigen::Vector3d& p_tcp_des,
                             const Eigen::Matrix3d& R_hand_des, double solve_dt);
    // Integrate panel_jog_vel_ for one tick via velocity-IK (uniform tip reference stream).
    void apply_panel_jog_tick(double dt);
    bool panel_stream_active() const {
        return panel_tcp_des_valid_ && panel_jog_vel_.norm() > 1e-9;
    }

    Eigen::Matrix3d vr_to_robot_axes() const;
    Eigen::Vector3d map_vr_delta_pos(const Eigen::Vector3d& delta_vr) const;
    Eigen::Quaterniond map_vr_delta_rot(const Eigen::Quaterniond& delta_vr) const;
    bool tcp_pose(const std::vector<double>& q, Eigen::Vector3d* p, Eigen::Matrix3d* R) const;
    void maybe_log(const char* tag);
    void reset_joint_stream(const std::vector<double>& q);
    const std::vector<double>& advance_joint_stream(const std::vector<double>& q_target, double dt,
                                                    double k_scale = 1.0, double qdot_scale = 1.0);

    Dynamics* dynamics_ = nullptr;
    HumanMimicConfig config_;
    std::string log_prefix_;

    std::unique_ptr<HumanMimicKinematics> kin_;
    std::unique_ptr<VirtualArmSkeleton> skeleton_;
    std::unique_ptr<HumanMimicIkSolver> ik_;  // whole-arm IK solve
    std::unique_ptr<JointReferenceGenerator> jrg_;
    JointReferenceOutputFilter joint_out_filter_;
    std::vector<double> q_out_filt_;
    std::vector<double> dq_out_filt_;

    size_t arm_dof_ = 0;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    std::vector<double> q_nominal_;

    // Clutch anchors.
    bool clutch_active_ = false;
    bool last_grip_ = false;
    bool require_reclutch_ = false;
    Eigen::Vector3d hand_ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond hand_ref_rot_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d robot_ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond robot_ref_rot_ = Eigen::Quaterniond::Identity();

    // Absolute-orientation calibration (set once at the first clutch or on recenter).
    Eigen::Quaterniond hand_ref_rot_abs_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond robot_ref_rot_abs_ = Eigen::Quaterniond::Identity();
    bool ori_calibrated_ = false;

    // Filtered orientation target (SLERP toward raw with tau/deadband).
    Eigen::Quaterniond ori_filt_ = Eigen::Quaterniond::Identity();
    bool ori_filt_valid_ = false;

    std::vector<double> q_target_;
    bool quest_timeout_active_ = false;
    int solve_fail_count_ = 0;

    // External elbow-swivel hint (HybrIK); blended into virtual phi in solve.
    double elbow_hint_phi_raw_ = 0.0;
    bool elbow_hint_valid_ = false;
    double elbow_hint_phi_used_ = 0.0;
    bool elbow_hint_used_valid_ = false;

    // Applied elbow swivel command (virtual ± simple hint).
    double elbow_cmd_phi_ = 0.0;
    bool elbow_cmd_phi_valid_ = false;

    // First-clutch locks so soft-reach scale / tool frame stay consistent all session.
    bool skel_lengths_locked_ = false;
    bool wrist_tcp_calibrated_ = false;
    int clutch_settle_remaining_ = 0;
    // After idle→track: absorb a few ticks (zero delta) before first IK.
    int clutch_track_settle_remaining_ = 0;
    // True after grip until the hand moves past clutch_start_motion_m (skip IK).
    bool clutch_idle_ = false;
    // Grip-time hand pose used only to decide idle exit (not the tracking zero).
    // hand_ref is absorbed during idle so delta stays ~0 until tracking starts.
    Eigen::Vector3d clutch_idle_anchor_pos_ = Eigen::Vector3d::Zero();
    // Hold elbow φ at clutch FK until hand departs (anti first-micro-move snap).
    bool clutch_phi_hold_ = false;
    double clutch_hand_depart_m_ = 0.0;
    double tip_yield_ = 0.0;  // last tip-priority yield in [0,1]
    double reach_progress_filt_ = 0.0;
    bool reach_progress_filt_valid_ = false;
    bool jog_active_ = false;          // true during in-nudge chase IK loops
    int jog_bypass_remaining_ = 0;     // post-nudge: keep bypassing fine-hold
    Eigen::Vector3d panel_jog_vel_ = Eigen::Vector3d::Zero();  // commanded button velocity (m/s)
    // Persistent Cartesian setpoint + ramped speed for hold-to-jog (uniform tip stream).
    bool panel_tcp_des_valid_ = false;
    Eigen::Vector3d panel_tcp_des_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d panel_R_des_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d panel_origin_ = Eigen::Vector3d::Zero();  // hold-start TCP
    Eigen::Vector3d panel_axis_mask_ = Eigen::Vector3d::Ones();  // which body axes may move
    double panel_speed_filt_ = 0.0;  // |v| ramp (m/s) — accel-limited smooth layer
    // Acceleration limit for tip speed ramp (m/s^2). 0.5 → ~40ms to reach 20mm/s.
    static constexpr double kPanelSpeedAccel = 0.5;

    // Orientation lock (right master in couple mode: position-only VR).
    bool ori_lock_active_ = false;
    Eigen::Matrix3d ori_lock_R_ = Eigen::Matrix3d::Identity();
    bool couple_follow_active_ = false;  // left arm: driven by drive_tcp_robot, skip VR
    // Couple: freeze elbow-φ branch at engage; reject {φ, φ+π} flips while following.
    bool couple_phi_lock_ = false;
    // Tip drive that must keep q_target_slew (couple / continuous panel Cartesian chase).
    bool continuous_tip_drive_ = false;
    // Near-body raised-elbow hysteresis (see HumanMimicConfig::elbow_raise_hold_*).
    bool elbow_raise_hold_ = false;
    // Debug: last-frame wrist-escape / raise-hold flags for HM logs.
    bool dbg_wrist_escape_ = false;
    double dbg_wrist_margin_ = 0.0;

    // Gripper: hold A/X = close, hold B/Y = open; jam-while-holding-close → GraspHolding
    // until open.
    enum class GripperMotion { Stopped, Opening, Closing, GraspHolding };
    GripperMotion gripper_motion_ = GripperMotion::Stopped;
    double gripper_hold_pos_ = 0.0;
    double gripper_target_ = 0.0;
    double gripper_meas_ = 0.0;
    bool gripper_hold_init_ = false;
    double gripper_stall_timer_ = 0.0;
    // Measured pose at the start of the current "progress window" while closing.
    double gripper_stall_q_ref_ = 0.0;
    // Active relative close lag while GraspHolding (rad or m).
    double gripper_grasp_lag_ = 0.0;
    bool last_button_close_ = false;
    bool last_button_open_ = false;

    // Trigger / recenter edge tracking.
    bool last_trigger_reset_ = false;
    bool last_recenter_ = false;

    // Frozen hold pose used whenever not actively teleoperating (idle / released /
    // timeout / awaiting re-grip). Captured once on entry so the arm actively holds
    // its pose (resists gravity) instead of tracking measured.
    bool hold_active_ = false;
    std::vector<double> hold_q_;

    bool initialized_ = false;
    std::chrono::steady_clock::time_point last_log_time_{};
    uint64_t log_tick_ = 0;
    std::ofstream debug_log_file_;
    std::string debug_log_path_opened_;

    // Debug snapshot (updated each solved frame; emitted by maybe_log).
    Eigen::Vector3d dbg_vr_axis_ = Eigen::Vector3d::Zero();        // body-frame VR delta axis (unit)
    Eigen::Vector3d dbg_vr_axis_world_ = Eigen::Vector3d::Zero();  // world-frame VR delta axis (unit)
    Eigen::Vector3d dbg_rob_axis_ = Eigen::Vector3d::Zero();       // mapped robot delta axis (unit)
    double dbg_delta_ang_ = 0.0;                                   // mapped delta angle (rad)
    bool dbg_hold_prev_ = false;
    double dbg_wrist_err_ = 0.0;
    double dbg_ori_err_ = 0.0;
    double dbg_elbow_err_ = 0.0;  // ‖E−E_des‖ elbow XYZ task residual
    bool dbg_ik_ok_ = false;
    int dbg_ik_iters_ = 0;
    bool dbg_reach_ = true;
    double dbg_phi_base_ = 0.0;  // last skeleton base swivel angle (rad)
    double dbg_elbow_flex_ = 0.0;  // interior elbow angle from commanded reach (rad)
    int dbg_phi_flip_ = 0;         // 1 if disambiguation picked the antipode
    double dbg_d_prev_ = 0.0;      // previous commanded reach (m) for extending trend
    bool dbg_d_prev_valid_ = false;
    bool tip_quiet_ = false;       // VR tip essentially still this frame
    double dbg_d_ = 0.0;         // normalized shoulder->wrist distance sent to skeleton (m)
    double dbg_d_req_ = 0.0;     // raw requested shoulder->wrist distance before normalization (m)
    double dbg_dmax_ = 0.0;      // reach clamp d_max = L1+L2-margin (m)
    double dbg_stretch_ = 0.0;   // raw human stretch fraction in [0,1] driving elbow yield
    double dbg_reach_progress_ = 0.0;  // soft-reach progress in [0,1] (max of down/forward)
    double dbg_fwd_off_ = 0.0;   // forward component of the TCP offset from anchor (m)
    double dbg_up_off_ = 0.0;    // vertical component of the TCP offset from anchor (m)
};
