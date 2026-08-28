// Copyright 2026 Enactic, Inc.
//
// Quest 3 Cartesian teleoperation for OpenArm follower.
// VR clutch pose mapping -> smoothed TCP target -> position IK -> joint reference
// -> Control::unilateral_step() -> SafetyFilter -> MIT -> backend.

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <common.hpp>
#include <controller/cartesian_controller.hpp>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <controller/human_mimic_controller.hpp>
#include <controller/skeleton_proximal_controller.hpp>
#include <controller/body_skeleton_filter.hpp>
#include <controller/skeleton_retarget_types.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <hardware/mujoco_teleop_hardware.hpp>
#include <hardware/teleop_hardware.hpp>
#include <iostream>
#include <io/quest_pose_receiver.hpp>
#include <io/skeleton_hint_receiver.hpp>
#include <io/skeleton_pose_receiver.hpp>
#include <io/body_pose_receiver.hpp>
#include <io/teleop_io.hpp>
#include <io/teleop_trajectory_logger.hpp>
#include <memory>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <set>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <yamlloader.hpp>

namespace {

constexpr double kQuestReceiveHz = 90.0;
constexpr double kCartesianDlsHz = FREQUENCY;  // 1000 Hz DLS servo; Quest target ~90 Hz

std::string select_leaf_link(const std::string& urdf_path, const std::string& arm_side) {
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    // Prefer hand_tcp (tool frame). Do not prefer a bare "_hand" name that may not exist.
    const std::vector<std::string> candidates = {
        "openarm_" + side + "_hand_tcp",
        "openarm_" + side + "_link7",
        "openarm_" + side + "_link8",
    };
    for (const auto& candidate : candidates) {
        std::ifstream file(urdf_path);
        if (!file) continue;
        std::stringstream buffer;
        buffer << file.rdbuf();
        if (buffer.str().find("link name=\"" + candidate + "\"") != std::string::npos) {
            return candidate;
        }
    }
    return candidates.front();
}

struct Options {
    std::string arm_side = "right_arm";
    std::string follower_can = "can0";
    std::string follower_can_left = "can1";
    std::string follower_can_right = "can0";
    std::string urdf_path =
        "../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf";
    std::string mujoco_xml = "../openarm_mujoco/v1/scene_table.xml";
    uint16_t quest_port = 47911;
    uint16_t quest_port_left = 47912;
    uint16_t quest_port_right = 47911;
    bool auto_home = false;
    bool sim = false;
    bool bimanual = false;
    bool sim_table_ready = true;  // teleport sim arms to TABLE_READY_POSITION at start
    bool use_nullspace = true;    // TABLE_READY elbow bias (sim + real)
    bool vr_orientation = true;   // default: VR wrist follow (tuning target)
    bool human_skeleton = false;  // human virtual-skeleton swivel redundancy resolution
    // Teleop mode: "cartesian_precision" | "human_mimic" | "skeleton_proximal".
    std::string teleop_mode = "cartesian_precision";
    // HUMAN_MIMIC orientation input mapping: "incremental" (default, re-anchor every
    // grip) or "absolute" (fix hand<->TCP calibration once; same hand pose -> same robot pose).
    std::string orientation_mapping = "incremental";
    // Optional smooth startup motion before teleop (real hardware). Default target
    // is TABLE_READY (matches sim); override with a pose/trajectory file.
    bool startup_move = false;
    std::string startup_file;     // optional pose/trajectory file
    double startup_move_sec = 4.0;  // duration when file has no explicit times
    // Optional elbow-swivel hint source (independent HybrIK bridge). "off" (default)
    // keeps the built-in virtual skeleton. "hybrik" listens for phi packets on UDP.
    std::string elbow_hint = "off";
    uint16_t elbow_hint_port = 47950;
    // skeleton_proximal: HybrIK SEW direction stream (independent of phi on 47950).
    bool skeleton_pose = false;  // auto-enabled when teleop_mode=skeleton_proximal
    uint16_t skeleton_pose_port = 47951;
    // Phase-1 BODY_SKELETON_RETARGET data path: receive + print only (no robot drive).
    bool body_pose = false;
    uint16_t body_pose_port = 47952;
    // Desktop Tk panels (录制 / TCP 微调) talking to these UDP ports.
    bool quest_ui = true;
    uint16_t quest_ui_record_port = 47930;
    uint16_t quest_ui_jog_port = 47931;
    std::string quest_record_dir;  // empty => recordings/quest_sessions/<stamp>
    double quest_jog_step_m = 0.01;      // legacy one-shot nudge (m)
    double quest_jog_speed_mps = 0.02;   // hold-to-jog tip speed (m/s), default 20 mm/s
};

std::vector<double> table_ready_arm_pose() {
    return {TABLE_READY_POSITION[0], TABLE_READY_POSITION[1], TABLE_READY_POSITION[2],
            TABLE_READY_POSITION[3], TABLE_READY_POSITION[4], TABLE_READY_POSITION[5],
            TABLE_READY_POSITION[6]};
}

double env_double_or(const char* name, double fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return fallback;
    }
    try {
        return std::stod(v);
    } catch (...) {
        return fallback;
    }
}

bool env_bool_or(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return fallback;
    }
    return !(v[0] == '0' || v[0] == 'f' || v[0] == 'F' || v[0] == 'n' || v[0] == 'N');
}

// Build the HUMAN_MIMIC config for a single arm ("left_arm"/"right_arm").
HumanMimicConfig make_human_mimic_config(const std::string& urdf_path,
                                         const std::string& arm_side,
                                         const std::string& orientation_mapping, bool sim,
                                         bool enable_elbow_hint = false) {
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    HumanMimicConfig cfg;
    cfg.orientation_mapping_mode = (orientation_mapping == "absolute")
                                       ? OrientationMappingMode::ABSOLUTE_CALIBRATED
                                       : OrientationMappingMode::INCREMENTAL_CLUTCH;
    cfg.urdf_path = urdf_path;
    cfg.root_link = "openarm_body_link0";
    cfg.shoulder_link = "openarm_" + side + "_link1";
    cfg.elbow_link = "openarm_" + side + "_link4";
    cfg.wrist_link = "openarm_" + side + "_link7";
    cfg.wrist_to_tcp = Eigen::Vector3d(0.0, 0.0, 0.186);
    // Right arm bends elbow toward robot -Y; left arm toward +Y.
    cfg.outward_dir =
        (side == "left") ? Eigen::Vector3d(0.0, 1.0, 0.0) : Eigen::Vector3d(0.0, -1.0, 0.0);
    cfg.q_nominal = table_ready_arm_pose();

    // Gripper open/closed/speed differ by BACKEND UNITS:
    //   - Sim  : MuJoCo finger_joint in METERS   (~0.00 open .. 0.04 closed).
    //   - Real : DM4310 motor position in RADIANS (1:1 passthrough, no transmission).
    // A sim-meter value like 0.04 is only ~2 deg on the real motor, so the real
    // gripper barely moves. Set real open/closed to the MEASURED motor angles.
    // Override any of these without recompiling via the env vars below.
    if (sim) {
        cfg.gripper_open = 0.0;
        cfg.gripper_closed = 0.04;
        cfg.gripper_speed = 0.05;
        cfg.gripper_close_speed = 0.02;  // slower close
    } else {
        // Real (DM4310, radians). Confirmed direction: POSITIVE = closing.
        // Power-on rest q0 ~= -0.18; user wants ~0.8 rad of travel PER DIRECTION, so
        // open = q0-0.8 and closed = q0+0.8 (total stroke ~1.6 rad). Override per-arm
        // via the env vars if either end pushes against a mechanical stop.
        cfg.gripper_open = -0.98;
        cfg.gripper_closed = 0.62;
        cfg.gripper_speed = 2.0;         // open rad/s
        cfg.gripper_close_speed = 0.45;   // close slower (~3.5 s full stroke)
    }
    cfg.gripper_open = env_double_or("OPENARM_GRIPPER_OPEN", cfg.gripper_open);
    cfg.gripper_closed = env_double_or("OPENARM_GRIPPER_CLOSED", cfg.gripper_closed);
    cfg.gripper_speed = env_double_or("OPENARM_GRIPPER_SPEED", cfg.gripper_speed);
    cfg.gripper_close_speed =
        env_double_or("OPENARM_GRIPPER_CLOSE_SPEED", cfg.gripper_close_speed);
    cfg.gripper_soft_stall = env_bool_or("OPENARM_GRIPPER_SOFT_STALL", true);
    // Sim stroke is meters (~0–0.04); real is DM4310 radians (~1.6 rad). Defaults
    // scale with that so contact freeze triggers before MIT winds up hard.
    cfg.gripper_stall_error =
        env_double_or("OPENARM_GRIPPER_STALL_ERROR", sim ? 0.003 : 0.04);
    cfg.gripper_stall_velocity =
        env_double_or("OPENARM_GRIPPER_STALL_VEL", sim ? 0.002 : 0.02);
    cfg.gripper_stall_time = env_double_or("OPENARM_GRIPPER_STALL_TIME", 0.12);
    cfg.gripper_hold_preload =
        env_double_or("OPENARM_GRIPPER_HOLD_PRELOAD", cfg.gripper_hold_preload);

    // Reach envelope. The workspace cap is ASYMMETRIC: only BACKWARD stays tight;
    // forward and lateral are opened up.
    //   - backward only  : OPENARM_HM_MAX_OFFSET      (original 0.45 feel, unchanged)
    //   - forward+lateral: OPENARM_HM_MAX_OFFSET_FWD  (opened up so the arm can extend/reach out)
    // reach_margin kept small so the arm can straighten forward (HybrIK guides the
    // elbow away from the singularity). All env-tunable.
    // Soft-reach: progress-based ceiling (bent mid-travel → near-straight at workspace
    // bottom). Disable: OPENARM_HM_REACH_SOFT=0.
    cfg.max_teleop_offset_m = env_double_or("OPENARM_HM_MAX_OFFSET", 0.60);
    cfg.max_teleop_offset_forward_m = env_double_or("OPENARM_HM_MAX_OFFSET_FWD", 0.85);
    cfg.max_teleop_offset_up_m = env_double_or("OPENARM_HM_MAX_OFFSET_UP", 0.75);
    cfg.max_teleop_offset_down_m = env_double_or("OPENARM_HM_MAX_OFFSET_DOWN", 0.55);
    cfg.reach_margin_m = env_double_or("OPENARM_HM_REACH_MARGIN", 0.002);
    cfg.reach_soft_enabled = env_bool_or("OPENARM_HM_REACH_SOFT", true);
    // Down-progress gate (scheme A). Disable alone: OPENARM_HM_REACH_SOFT_PROGRESS=0
    // (keeps soft ceiling with fixed REACH_SOFT_MARGIN). Disable all soft-reach:
    // OPENARM_HM_REACH_SOFT=0.
    cfg.reach_soft_progress_enabled =
        env_bool_or("OPENARM_HM_REACH_SOFT_PROGRESS", true);
    // At workspace bottom (progress=1): ~2mm short of L1+L2 ≈ near-straight.
    cfg.reach_soft_margin_m = env_double_or("OPENARM_HM_REACH_SOFT_MARGIN", 0.002);
    // At clutch height (progress=0): keep a clear bend for fine motion.
    cfg.reach_soft_bent_margin_m =
        env_double_or("OPENARM_HM_REACH_SOFT_BENT_MARGIN", 0.045);
    cfg.reach_soft_zone_m = env_double_or("OPENARM_HM_REACH_SOFT_ZONE", 0.12);
    // Forward = robot base +X by default; flip with OPENARM_HM_FORWARD_SIGN=-1 if the
    // "front" opened the wrong way for your base mounting.
    cfg.forward_axis =
        Eigen::Vector3d(env_double_or("OPENARM_HM_FORWARD_SIGN", 1.0), 0.0, 0.0);

    // Whole-arm IK task weights (env-tunable for on-device balancing of
    // reach vs posture vs orientation).
    cfg.w_wrist_pos = env_double_or("OPENARM_HM_W_WRIST", cfg.w_wrist_pos);
    cfg.w_hand_ori = env_double_or("OPENARM_HM_W_ORI", cfg.w_hand_ori);
    // OPENARM_HM_W_ELBOW scales the elbow XYZ position residual.
    cfg.w_elbow = env_double_or("OPENARM_HM_W_ELBOW", cfg.w_elbow);
    cfg.ik_proximal_max_step =
        env_double_or("OPENARM_HM_IK_PROX_STEP", cfg.ik_proximal_max_step);
    cfg.ik_distal_max_step = env_double_or("OPENARM_HM_IK_DIST_STEP", cfg.ik_distal_max_step);
    {
        const char* bk = std::getenv("OPENARM_HM_IK_BACKEND");
        if (bk != nullptr) {
            const std::string s(bk);
            if (s == "lm" || s == "LM") {
                cfg.ik_backend = HumanMimicIkBackend::LM;
            } else if (s == "diff_qp" || s == "DIFF_QP" || s == "qp" || s == "QP") {
                cfg.ik_backend = HumanMimicIkBackend::DIFF_QP;
            }
        }
    }
    cfg.ik_kp_pos = env_double_or("OPENARM_HM_IK_KP_POS", cfg.ik_kp_pos);
    cfg.ik_kp_ori = env_double_or("OPENARM_HM_IK_KP_ORI", cfg.ik_kp_ori);
    cfg.ik_joint_limit_margin_frac =
        env_double_or("OPENARM_HM_IK_LIMIT_MARGIN", cfg.ik_joint_limit_margin_frac);

    // Reach-forward assist (elbow/orientation yield near full extension).
    cfg.extension_yield_start =
        env_double_or("OPENARM_HM_EXT_START", cfg.extension_yield_start);
    cfg.extension_elbow_scale_min =
        env_double_or("OPENARM_HM_EXT_ELBOW_MIN", cfg.extension_elbow_scale_min);
    cfg.extension_ori_scale_min =
        env_double_or("OPENARM_HM_EXT_ORI_MIN", cfg.extension_ori_scale_min);
    cfg.extension_cont_scale_min =
        env_double_or("OPENARM_HM_EXT_CONT_MIN", cfg.extension_cont_scale_min);
    cfg.extension_nom_scale_min =
        env_double_or("OPENARM_HM_EXT_NOM_MIN", cfg.extension_nom_scale_min);
    cfg.extension_phi_freeze_stretch =
        env_double_or("OPENARM_HM_EXT_PHI_FREEZE", cfg.extension_phi_freeze_stretch);
    cfg.extension_roll_slew_scale =
        env_double_or("OPENARM_HM_EXT_ROLL_SLEW", cfg.extension_roll_slew_scale);
    cfg.q_target_slew_enabled = env_bool_or("OPENARM_HM_Q_SLEW", cfg.q_target_slew_enabled);
    cfg.q_target_slew_max_radps =
        env_double_or("OPENARM_HM_Q_SLEW_MAX", cfg.q_target_slew_max_radps);
    cfg.elbow_cmd_slew_enabled =
        env_bool_or("OPENARM_HM_ELBOW_PHI_SLEW", cfg.elbow_cmd_slew_enabled);
    cfg.elbow_cmd_slew_radps =
        env_double_or("OPENARM_HM_ELBOW_PHI_SLEW_MAX", cfg.elbow_cmd_slew_radps);
    cfg.elbow_cmd_max_step_rad =
        env_double_or("OPENARM_HM_ELBOW_PHI_STEP", cfg.elbow_cmd_max_step_rad);
    cfg.elbow_joint_slew_radps =
        env_double_or("OPENARM_HM_ELBOW_Q_SLEW", cfg.elbow_joint_slew_radps);

    // Elbow bias: with HybrIK guiding phi, suppress the fixed outward flare so the
    // arm can yaw to pure left/right with a drooping elbow. Without HybrIK, keep a
    // milder outward than the old 0.3 default.
    cfg.w_outward = env_double_or("OPENARM_HM_W_OUTWARD",
                                  enable_elbow_hint ? 0.05 : 0.15);
    cfg.w_downward = env_double_or("OPENARM_HM_W_DOWNWARD", 0.25);
    cfg.lateral_droop_boost =
        env_double_or("OPENARM_HM_LATERAL_DROOP", cfg.lateral_droop_boost);
    cfg.elevation_droop_fade =
        env_double_or("OPENARM_HM_ELEV_DROOP_FADE", cfg.elevation_droop_fade);

    // Optional PER-SIDE, per-joint IK limit overrides. joint2 (index 1) is mirrored:
    // right tight side = min (~−10°), left tight side = max (~+10°). That 10° URDF
    // travel is what blocks swinging the arm behind the torso / fully opening the
    // shoulder. In SIM we widen it by default so kinematics can be validated; on
    // hardware it stays at the URDF limit unless explicitly overridden.
    //   OPENARM_HM_QMIN_1_R / OPENARM_HM_QMAX_1_L
    constexpr int kArmDof = 7;
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    cfg.q_min_override.assign(kArmDof, nan_v);
    cfg.q_max_override.assign(kArmDof, nan_v);
    const std::string suf = (side == "left") ? "_L" : "_R";
    bool any_override = false;
    if (sim) {
        // Widen shoulder joint2 (index 1) tight side to ±60° in sim.
        if (side == "right") {
            cfg.q_min_override[1] = env_double_or("OPENARM_HM_QMIN_1_R", -1.047);  // −60°
            any_override = true;
        } else {
            cfg.q_max_override[1] = env_double_or("OPENARM_HM_QMAX_1_L", 1.047);   // +60°
            any_override = true;
        }
        // Widen shoulder yaw joint1 (index 0) past the URDF ~±80° so the arm can
        // swing to pure left/right (±90°+) via upper-arm yaw instead of cheating
        // with an flared elbow. Hardware keeps the stock URDF until calibrated.
        if (side == "right") {
            cfg.q_min_override[0] = env_double_or("OPENARM_HM_QMIN_0_R", -2.5);
            any_override = true;
        } else {
            cfg.q_max_override[0] = env_double_or("OPENARM_HM_QMAX_0_L", 2.5);
            any_override = true;
        }
    }
    for (int i = 0; i < kArmDof; ++i) {
        const std::string kmin = "OPENARM_HM_QMIN_" + std::to_string(i) + suf;
        const std::string kmax = "OPENARM_HM_QMAX_" + std::to_string(i) + suf;
        const double vmin = env_double_or(kmin.c_str(), nan_v);
        const double vmax = env_double_or(kmax.c_str(), nan_v);
        if (std::isfinite(vmin)) {
            cfg.q_min_override[i] = vmin;
            any_override = true;
        }
        if (std::isfinite(vmax)) {
            cfg.q_max_override[i] = vmax;
            any_override = true;
        }
    }
    if (!any_override) {
        cfg.q_min_override.clear();
        cfg.q_max_override.clear();
    }

    // External elbow-swivel hint (independent HybrIK chain). Off unless --elbow-hint
    // wired it in. All blend knobs are env-tunable so sign/offset can be calibrated
    // on-device without recompiling. Right/left may need opposite sign (handedness).
    cfg.elbow_hint_enabled = enable_elbow_hint;
    cfg.elbow_hint_weight = env_double_or("OPENARM_ELBOW_HINT_WEIGHT", cfg.elbow_hint_weight);
    cfg.elbow_hint_max_delta_rad =
        env_double_or("OPENARM_ELBOW_HINT_MAX_DELTA", cfg.elbow_hint_max_delta_rad);
    const double default_sign = (side == "left") ? -1.0 : 1.0;
    cfg.elbow_hint_sign = env_double_or(
        (side == "left") ? "OPENARM_ELBOW_HINT_SIGN_L" : "OPENARM_ELBOW_HINT_SIGN_R", default_sign);
    cfg.elbow_hint_offset_rad = env_double_or(
        (side == "left") ? "OPENARM_ELBOW_HINT_OFFSET_L" : "OPENARM_ELBOW_HINT_OFFSET_R",
        cfg.elbow_hint_offset_rad);
    cfg.elbow_hint_slew_radps =
        env_double_or("OPENARM_ELBOW_HINT_SLEW", cfg.elbow_hint_slew_radps);
    cfg.elbow_phi_disambiguate =
        env_bool_or("OPENARM_HM_ELBOW_PHI_DISAMBIG", cfg.elbow_phi_disambiguate);
    cfg.elbow_phi_reach_lock_stretch =
        env_double_or("OPENARM_HM_ELBOW_REACH_LOCK", cfg.elbow_phi_reach_lock_stretch);
    cfg.elbow_phi_obtuse_flex_rad =
        env_double_or("OPENARM_HM_ELBOW_OBTUSE_FLEX", cfg.elbow_phi_obtuse_flex_rad);
    cfg.elbow_singularity_h_m =
        env_double_or("OPENARM_HM_ELBOW_SING_H", cfg.elbow_singularity_h_m);
    cfg.max_reanchor_correction_rad =
        env_double_or("OPENARM_HM_REANCHOR_CAP", cfg.max_reanchor_correction_rad);
    cfg.clutch_settle_ticks =
        static_cast<int>(env_double_or("OPENARM_HM_CLUTCH_SETTLE", cfg.clutch_settle_ticks));
    cfg.lock_segment_lengths =
        env_bool_or("OPENARM_HM_LOCK_L12", cfg.lock_segment_lengths);
    cfg.auto_calibrate_wrist_tcp =
        env_bool_or("OPENARM_HM_AUTO_WRIST_TCP", cfg.auto_calibrate_wrist_tcp);
    cfg.tip_priority_enabled =
        env_bool_or("OPENARM_HM_TIP_PRIORITY", cfg.tip_priority_enabled);
    cfg.tip_priority_wrist_err_m =
        env_double_or("OPENARM_HM_TIP_WRIST_ERR", cfg.tip_priority_wrist_err_m);
    cfg.tip_priority_elbow_scale_min =
        env_double_or("OPENARM_HM_TIP_ELBOW_MIN", cfg.tip_priority_elbow_scale_min);
    cfg.tip_priority_cont_scale_min =
        env_double_or("OPENARM_HM_TIP_CONT_MIN", cfg.tip_priority_cont_scale_min);
    cfg.tip_priority_shoulder_slew_boost =
        env_double_or("OPENARM_HM_TIP_SH_SLEW", cfg.tip_priority_shoulder_slew_boost);
    cfg.position_map_world_frame =
        env_bool_or("OPENARM_HM_POS_WORLD", cfg.position_map_world_frame);
    cfg.fine_motion_hold_enabled =
        env_bool_or("OPENARM_HM_FINE_HOLD", cfg.fine_motion_hold_enabled);
    cfg.position_deadband_m =
        env_double_or("OPENARM_HM_POS_DEADBAND", cfg.position_deadband_m);
    cfg.q_update_deadband_rad =
        env_double_or("OPENARM_HM_Q_DEADBAND", cfg.q_update_deadband_rad);
    cfg.wrist_hold_err_m =
        env_double_or("OPENARM_HM_WRIST_HOLD_ERR", cfg.wrist_hold_err_m);
    cfg.jog_bypass_hold_ticks = static_cast<int>(
        env_double_or("OPENARM_HM_JOG_BYPASS_TICKS", cfg.jog_bypass_hold_ticks));
    cfg.jog_chase_iters = static_cast<int>(
        env_double_or("OPENARM_HM_JOG_CHASE_ITERS", cfg.jog_chase_iters));
    cfg.jog_chase_dt = env_double_or("OPENARM_HM_JOG_CHASE_DT", cfg.jog_chase_dt);
    cfg.jog_wrist_tol_m = env_double_or("OPENARM_HM_JOG_TOL", cfg.jog_wrist_tol_m);
    cfg.jog_elbow_scale = env_double_or("OPENARM_HM_JOG_ELBOW", cfg.jog_elbow_scale);
    cfg.jog_q_slew_radps = env_double_or("OPENARM_HM_JOG_Q_SLEW", cfg.jog_q_slew_radps);
    return cfg;
}

SkeletonProximalConfig make_skeleton_proximal_config(const std::string& urdf_path,
                                                     const std::string& arm_side, bool sim) {
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    SkeletonProximalConfig cfg;
    cfg.urdf_path = urdf_path;
    cfg.root_link = "openarm_body_link0";
    cfg.shoulder_link = "openarm_" + side + "_link1";
    cfg.elbow_link = "openarm_" + side + "_link4";
    cfg.wrist_link = "openarm_" + side + "_link7";
    cfg.wrist_to_tcp = Eigen::Vector3d(0.0, 0.0, 0.186);
    cfg.q_nominal = table_ready_arm_pose();
    if (sim) {
        cfg.gripper_open = 0.0;
        cfg.gripper_closed = 0.04;
        cfg.gripper_speed = 0.05;
    } else {
        cfg.gripper_open = -0.98;
        cfg.gripper_closed = 0.62;
        cfg.gripper_speed = 2.0;
    }
    cfg.gripper_open = env_double_or("OPENARM_GRIPPER_OPEN", cfg.gripper_open);
    cfg.gripper_closed = env_double_or("OPENARM_GRIPPER_CLOSED", cfg.gripper_closed);
    cfg.gripper_speed = env_double_or("OPENARM_GRIPPER_SPEED", cfg.gripper_speed);
    cfg.rot_scale = env_double_or("OPENARM_SP_ROT_SCALE", cfg.rot_scale);
    cfg.w_hand_ori = env_double_or("OPENARM_SP_W_ORI", cfg.w_hand_ori);
    cfg.prox_elev_gain = env_double_or("OPENARM_SP_ELEV_GAIN", cfg.prox_elev_gain);
    cfg.prox_azim_gain = env_double_or("OPENARM_SP_AZIM_GAIN", cfg.prox_azim_gain);
    cfg.prox_bend_gain = env_double_or("OPENARM_SP_BEND_GAIN", cfg.prox_bend_gain);
    cfg.w_prox_fore = env_double_or("OPENARM_SP_W_FORE", cfg.w_prox_fore);
    return cfg;
}

CartesianControllerConfig make_cartesian_config(bool mirror_translation_y, bool hardware,
                                                bool enable_table_nullspace,
                                                bool vr_orientation, bool human_skeleton) {
    CartesianControllerConfig cfg;
    cfg.position_only_teleop = false;
    // Default: VR wrist follow (what we are tuning). Use --hold-orientation to lock TCP rot.
    cfg.orientation_mode = vr_orientation ? OrientationMode::VR_ORIENTATION
                                          : OrientationMode::HOLD_CLUTCH_ORIENTATION;
    cfg.direct_ik_control = true;
    cfg.mirror_translation_y = mirror_translation_y;
    cfg.pos_scale = 1.0;
    // Desensitise wrist: 1:1 orientation felt "漂"/over-sensitive. 0.6 = calmer, still follows.
    cfg.rot_scale = 0.6;
    // Cap reach so the elbow never straightens into its singularity (log showed j3≈0, sigma_min≈0
    // and the arm jamming at large amplitude). A human can't lock the elbow straight either.
    cfg.workspace.max_teleop_offset_m = 0.55;
    cfg.workspace.max_radius = 0.68;
    cfg.workspace.min_z = 0.10;
    cfg.workspace.max_z = 0.90;
    cfg.safety.quest_stale_sec = 0.050;
    cfg.safety.quest_timeout_sec = 0.150;
    cfg.safety.max_joint_step_rad = 0.04;  // legacy position-IK path only
    cfg.safety.max_ik_position_error_m = 0.10;
    cfg.safety.ik_fail_hold_count = 40;
    cfg.safety.ik_fail_estop_count = 1000;
    cfg.safety.grip_debounce_sec = 0.0;
    cfg.safety.min_clutch_hold_sec = 0.0;
    cfg.safety.lambda0 = 0.08;
    cfg.safety.sigma_min_threshold = 0.05;
    cfg.smoothing.max_cartesian_speed_mps = 1.0e6;
    cfg.smoothing.max_cartesian_rot_radps = 1.0e6;
    cfg.smoothing.max_joint_velocity_radps = 1.0e6;
    cfg.smoothing.joint_output_lowpass_hz = 1.0e6;
    cfg.smoothing.max_joint_output_velocity_radps = 1.0e6;
    cfg.smoothing.max_joint_output_accel_radps2 = 1.0e6;
    cfg.dls.enabled = true;
    cfg.dls.kp_pos = 9.0;
    cfg.dls.kp_ori = 4.0;
    // Raise twist speed caps so the TCP keeps up with fast hand motion (log showed |v|,|w| pinned
    // at the caps with large error -> TCP lagging the hand = "错位/需要动更多").
    cfg.dls.max_linear_speed_mps = 1.5;
    cfg.dls.max_angular_speed_radps = 3.0;
    cfg.dls.lambda = 0.05;
    cfg.dls.max_joint_velocity_radps = 2.2;
    cfg.vr_orientation_filter_alpha = 0.20;  // more smoothing (was 0.32) to reduce jitter
    // Kill the Δrot/dt angular-velocity feedforward: dividing per-frame controller noise by
    // dt(~1ms) makes the TCP orientation drift on its own (esp. the noisier right controller).
    // kp_ori feedback to the (filtered) target still tracks the hand cleanly, without drift.
    cfg.vr_orientation_ff_gain = 0.0;
    // Human wrist ROM clamp: cap TCP orientation deviation from clutch pose (per body axis).
    cfg.wrist_rom_enabled = true;
    cfg.wrist_rom_limit_x_rad = 1.57;  // ~90° roll (pronation/supination)
    cfg.wrist_rom_limit_y_rad = 1.20;  // ~70° pitch (flex/extension, "上下")
    cfg.wrist_rom_limit_z_rad = 0.70;  // ~40° yaw (radial/ulnar deviation)
    // TABLE_READY elbow nullspace: enabled in VR mode for experiment (softer gain vs hold-orientation).
    cfg.dls.use_nullspace = enable_table_nullspace;
    if (cfg.dls.use_nullspace && vr_orientation) {
        cfg.dls.nullspace_gain = 0.28;
    } else if (cfg.dls.use_nullspace) {
        cfg.dls.nullspace_gain = 0.35;
    } else {
        cfg.dls.nullspace_gain = 0.2;
    }
    cfg.dls.nullspace_nominal_mode = NullspaceNominalMode::TABLE_READY_POSE;
    cfg.dls.table_ready_pose = table_ready_arm_pose();
    // Nullspace pulls j3 (大臂末端 / elbow bend in TABLE_READY).
    cfg.dls.nullspace_joint_weights = {0.12, 0.12, 0.12, 1.0, 0.0, 0.0, 0.0};
    // OpenArm joint map (0-based motor index = array index; j7 gripper = trigger, not in IK):
    //   j0–j3 大臂 | j4 小臂整体旋转 | j5 夹爪上下(中幅) | j6 夹爪左右
    // j0–j2 大臂高权重防 j3/j4 陷入奇异；j4–j6 小臂姿态。
    // Wrist preference so TCP rotation (esp. yaw "左右摆头") uses j5/j6, not the whole arm.
    // Moderated from 2.8: too high slammed j6 into its π/2 limit and jammed the arm.
    cfg.dls.task_joint_weights = {0.9, 0.9, 1.0, 0.30, 1.3, 2.2, 2.2};
    cfg.dls.soft_limit_margin_frac = 0.12;
    cfg.dls.max_tracking_error_rad = 0.30;
    cfg.dls.emergency_step_scale = 2.0;
    cfg.dls.debug_log = true;
    cfg.dls.debug_log_hz = 5.0;
    // Human virtual-skeleton swivel redundancy (deterministic => returnable, human-like).
    cfg.skeleton.enabled = human_skeleton;
    cfg.skeleton.upper_arm_len_m = 0.30;
    cfg.skeleton.forearm_len_m = 0.28;
    // Gentle, stable bias (was 3.5 / 2.5 — too aggressive, caused self-spinning near singularities).
    cfg.skeleton.swivel_gain = 1.0;
    cfg.skeleton.max_swivel_rate = 0.5;
    cfg.skeleton.swivel_sign = 1.0;
    cfg.skeleton.swivel_bias_rad = 0.0;
    // Soften the rigid "elbow follows orientation" coupling so the arm isn't locked to one posture.
    cfg.skeleton.orientation_coupling = 0.5;
    cfg.skeleton.down_robot = Eigen::Vector3d(0.0, 0.0, -1.0);
    cfg.skeleton.down_vr = Eigen::Vector3d(0.0, -1.0, 0.0);
    if (human_skeleton) {
        // The skeleton only owns the 1-D null-space (elbow swivel). The task mobility weights do a
        // *different* job: they shape how the 6-DOF task itself is distributed across joints so a
        // wrist rotation uses the wrist (j5/j6), not the whole arm. Keep them. Only the fixed
        // TABLE_READY null-space pull is replaced by the skeleton.
        cfg.dls.use_nullspace = false;
    }
    // Real hardware uses the same Cartesian / DLS / VR / nullspace stack as sim.
    // MIT-PD Kp/Kd are still scaled separately in configure_follower_control().
    if (hardware) {
        cfg.safety.fatal_lock_timeout_sec = 1.5;
    }
    return cfg;
}

void print_real_hardware_banner() {
    std::cout << "[quest-cartesian] REAL HARDWARE mode\n"
              << "  - Same Cartesian stack as sim: VR_ORIENTATION + weighted DLS + TABLE_READY nullspace.\n"
              << "  - DLS velocity IK @ " << kCartesianDlsHz
              << " Hz; Quest target update @ ~" << kQuestReceiveHz << " Hz.\n"
              << "  - Internal q_ref integration; MIT-PD gets q_des + qdot_des.\n"
              << "  - Hold Grip to teleop, release to hold pose. Re-grip after Quest timeout.\n"
              << "  - Start with moderate motions; Ctrl+C / record-panel 退出 / fatal lock:\n"
              << "    Kd damping settle then disable motors."
              << std::endl;
}

void print_teleop_help() {
    std::cout << "[quest-cartesian] build: stable_gentle_skeleton_v26 (2026-07-13)\n"
              << "[quest-cartesian] Teleop logic:\n"
              << "  - Idle: arm holds current pose; no startup homing.\n"
              << "  - Grip rising edge: clutch activates; current hand pose maps to current TCP "
                 "(zero jump).\n"
              << "  - Grip held: VR hand delta -> TCP pose (xyz + orientation) -> one 6-DOF DLS "
                 "-> joint reference.\n"
              << "  - Grip release: clutch deactivates, back to idle.\n"
              << "  - VR_ORIENTATION (default): hand position AND rotation both drive TCP.\n"
              << "  - TABLE_READY nullspace ON in VR mode (elbow j3 bias ~90°, gain=0.28).\n"
              << "  - --hold-orientation: position only; TCP rotation locked at clutch.\n"
              << "  - Left/right use the same VR->robot translation map (no Y mirror).\n"
              << "  - IK lock (hold/estop) while Grip held: exit with [FATAL] after "
              << "1.0s instead of freezing.\n"
              << "  Gripper (right A/B, left X/Y): A/X close, B/Y open; press again to stop "
                 "mid-motion; auto-stop at fully open/closed.\n"
              << "  Hold Trigger while teleoping: lock TCP orientation; VR drives position only.\n"
              << "  --human-skeleton: elbow swivel pinned to a human arm model (deterministic, "
                 "returnable, avoids singular/odd poses)."
              << std::endl;
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "Options:\n"
              << "  --sim                             MuJoCo simulation (no CAN hardware)\n"
              << "  --bimanual                        control both arms (Quest left+right)\n"
              << "  --arm-side right_arm|left_arm     single-arm mode only, default: right_arm\n"
              << "  --follower-can IFACE              single-arm CAN, default: can0\n"
              << "  --follower-can-left IFACE         bimanual left CAN, default: can1\n"
              << "  --follower-can-right IFACE        bimanual right CAN, default: can0\n"
              << "  --urdf PATH                       follower URDF path\n"
              << "  --xml PATH                        MuJoCo scene XML (--sim), default: scene_table.xml\n"
              << "  --quest-port PORT                 single-arm UDP port, default: 47911\n"
              << "  --quest-port-left PORT            bimanual left UDP, default: 47912\n"
              << "  --quest-port-right PORT           bimanual right UDP, default: 47911\n"
              << "  --auto-home                       run startup home motion (real hardware)\n"
              << "  --no-auto-home                    skip startup home (default)\n"
              << "  --sim-table-ready                 teleport sim to table-ready pose (default)\n"
              << "  --no-sim-table-ready              leave sim at MuJoCo zero pose\n"
              << "  --nullspace                       enable TABLE_READY nullspace (default on)\n"
              << "  --no-nullspace                    disable nullspace posture bias\n"
              << "  --vr-orientation                  map VR hand rotation to TCP (default)\n"
              << "  --hold-orientation                hold TCP orientation at clutch\n"
              << "  --human-skeleton                  human virtual-skeleton swivel redundancy (returnable)\n"
              << "  --no-human-skeleton               use fixed TABLE_READY nullspace (default)\n"
              << "  --teleop-mode MODE                cartesian_precision (default) | human_mimic | skeleton_proximal\n"
              << "                                    human_mimic: virtual-skeleton whole-arm absolute IK\n"
              << "                                    skeleton_proximal: q0-q3 follow HybrIK SEW; q4-q6 VR ori only\n"
              << "  --orientation-mapping MODE        human_mimic: incremental (default) | absolute\n"
              << "  --absolute-orientation            alias for --orientation-mapping absolute\n"
              << "  --startup-move                    smoothly move arm(s) to TABLE_READY before teleop\n"
              << "  --no-startup-move                 skip startup move (default)\n"
              << "  --startup-file PATH               startup pose/trajectory file (implies --startup-move)\n"
              << "                                    line: [t] side q0..q6 [grip]; side=left|right|both\n"
              << "  --startup-move-sec SEC            startup duration when file has no times (default 4)\n"
              << "  --elbow-hint SRC                  off (default) | hybrik  (independent skeleton swivel hint)\n"
              << "  --elbow-hint-port PORT            UDP port for elbow-swivel hints (default 47950)\n"
              << "  --skeleton-pose-port PORT         UDP port for SEW pose (skeleton_proximal, default 47951)\n"
              << "  --body-pose                       body keypoints UDP + torso map + MuJoCo sticks (no drive)\n"
              << "  --body-pose-port PORT             UDP port for body keypoints (default 47952)\n"
              << "  --quest-ui / --no-quest-ui         enable/disable record+jog UDP panel ports (default on)\n"
              << "  --quest-ui-record-port PORT       action-record panel UDP (default 47930)\n"
              << "  --quest-ui-jog-port PORT          TCP jog panel UDP (default 47931)\n"
              << "  --quest-record-dir DIR            CSV record directory for panel Start\n"
              << "  --quest-jog-step M                one-shot nudge meters (default 0.01)\n"
              << "  --quest-jog-speed M_S             hold-to-jog tip speed m/s (default 0.02)\n";
}

bool parse_args(int argc, char** argv, Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sim") {
            options->sim = true;
        } else if (arg == "--bimanual") {
            options->bimanual = true;
        } else if (arg == "--arm-side" && i + 1 < argc) {
            options->arm_side = argv[++i];
        } else if (arg == "--follower-can" && i + 1 < argc) {
            options->follower_can = argv[++i];
        } else if (arg == "--follower-can-left" && i + 1 < argc) {
            options->follower_can_left = argv[++i];
        } else if (arg == "--follower-can-right" && i + 1 < argc) {
            options->follower_can_right = argv[++i];
        } else if (arg == "--urdf" && i + 1 < argc) {
            options->urdf_path = argv[++i];
        } else if (arg == "--xml" && i + 1 < argc) {
            options->mujoco_xml = argv[++i];
        } else if (arg == "--quest-port" && i + 1 < argc) {
            options->quest_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-port-left" && i + 1 < argc) {
            options->quest_port_left = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-port-right" && i + 1 < argc) {
            options->quest_port_right = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--no-auto-home") {
            options->auto_home = false;
        } else if (arg == "--auto-home") {
            options->auto_home = true;
        } else if (arg == "--sim-table-ready") {
            options->sim_table_ready = true;
        } else if (arg == "--no-sim-table-ready") {
            options->sim_table_ready = false;
        } else if (arg == "--nullspace") {
            options->use_nullspace = true;
        } else if (arg == "--no-nullspace") {
            options->use_nullspace = false;
        } else if (arg == "--vr-orientation") {
            options->vr_orientation = true;
        } else if (arg == "--hold-orientation") {
            options->vr_orientation = false;
        } else if (arg == "--human-skeleton") {
            options->human_skeleton = true;
        } else if (arg == "--no-human-skeleton") {
            options->human_skeleton = false;
        } else if (arg == "--teleop-mode" && i + 1 < argc) {
            options->teleop_mode = argv[++i];
            if (options->teleop_mode != "cartesian_precision" &&
                options->teleop_mode != "human_mimic" &&
                options->teleop_mode != "skeleton_proximal") {
                std::cerr << "[quest-cartesian][ERROR] invalid --teleop-mode '"
                          << options->teleop_mode
                          << "' (expected cartesian_precision | human_mimic | skeleton_proximal)"
                          << std::endl;
                return false;
            }
            if (options->teleop_mode == "skeleton_proximal") {
                options->skeleton_pose = true;
            }
        } else if (arg == "--orientation-mapping" && i + 1 < argc) {
            options->orientation_mapping = argv[++i];
            if (options->orientation_mapping != "incremental" &&
                options->orientation_mapping != "absolute") {
                std::cerr << "[quest-cartesian][ERROR] invalid --orientation-mapping '"
                          << options->orientation_mapping << "' (expected incremental | absolute)"
                          << std::endl;
                return false;
            }
        } else if (arg == "--absolute-orientation") {
            options->orientation_mapping = "absolute";
        } else if (arg == "--startup-move") {
            options->startup_move = true;
        } else if (arg == "--no-startup-move") {
            options->startup_move = false;
        } else if (arg == "--startup-file" && i + 1 < argc) {
            options->startup_file = argv[++i];
            options->startup_move = true;
        } else if (arg == "--startup-move-sec" && i + 1 < argc) {
            options->startup_move_sec = std::stod(argv[++i]);
        } else if (arg == "--elbow-hint" && i + 1 < argc) {
            options->elbow_hint = argv[++i];
            if (options->elbow_hint != "off" && options->elbow_hint != "hybrik") {
                std::cerr << "[quest-cartesian][ERROR] invalid --elbow-hint '"
                          << options->elbow_hint << "' (expected off | hybrik)" << std::endl;
                return false;
            }
        } else if (arg == "--elbow-hint-port" && i + 1 < argc) {
            options->elbow_hint_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--skeleton-pose-port" && i + 1 < argc) {
            options->skeleton_pose_port = static_cast<uint16_t>(std::stoi(argv[++i]));
            options->skeleton_pose = true;
        } else if (arg == "--body-pose") {
            options->body_pose = true;
        } else if (arg == "--body-pose-port" && i + 1 < argc) {
            options->body_pose_port = static_cast<uint16_t>(std::stoi(argv[++i]));
            options->body_pose = true;
        } else if (arg == "--quest-ui") {
            options->quest_ui = true;
        } else if (arg == "--no-quest-ui") {
            options->quest_ui = false;
        } else if (arg == "--quest-ui-record-port" && i + 1 < argc) {
            options->quest_ui_record_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-ui-jog-port" && i + 1 < argc) {
            options->quest_ui_jog_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--quest-record-dir" && i + 1 < argc) {
            options->quest_record_dir = argv[++i];
        } else if (arg == "--quest-jog-step" && i + 1 < argc) {
            options->quest_jog_step_m = std::stod(argv[++i]);
        } else if (arg == "--quest-jog-speed" && i + 1 < argc) {
            options->quest_jog_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "[quest-cartesian][ERROR] unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return false;
        }
    }
    if (!options->bimanual && options->arm_side != "left_arm" && options->arm_side != "right_arm") {
        std::cerr << "[quest-cartesian][ERROR] invalid arm side: " << options->arm_side
                  << std::endl;
        return false;
    }
    return true;
}

std::vector<JointState> collect_joint_states(const std::shared_ptr<RobotSystemState>& state) {
    std::vector<JointState> joints;
    const auto arm = state->arm_state().get_all_responses();
    const auto hand = state->hand_state().get_all_responses();
    joints.insert(joints.end(), arm.begin(), arm.end());
    joints.insert(joints.end(), hand.begin(), hand.end());
    return joints;
}


struct FollowerParams {
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> Fc;
    std::vector<double> k;
    std::vector<double> Fv;
    std::vector<double> Fo;
};

FollowerParams load_follower_params() {
    YamlLoader follower_loader("config/follower.yaml");
    FollowerParams params;
    params.kp = follower_loader.get_vector("FollowerArmParam", "Kp");
    params.kd = follower_loader.get_vector("FollowerArmParam", "Kd");
    params.Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
    params.k = follower_loader.get_vector("FollowerArmParam", "k");
    params.Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
    params.Fo = follower_loader.get_vector("FollowerArmParam", "Fo");
    return params;
}

// Soft exit: Kp=0, tau_ff=0, hold measured q with Kd so arms settle under gravity.
void send_kd_only_release(openarm::can::socket::OpenArm* left_openarm,
                          openarm::can::socket::OpenArm* right_openarm,
                          const std::vector<double>& kd, double duration_sec = -1.0) {
    double duration = duration_sec;
    if (duration < 0.0) {
        const char* env = std::getenv("OPENARM_QUEST_DAMP_SEC");
        if (env == nullptr || *env == '\0') {
            env = std::getenv("OPENARM_REPLAY_DAMP_SEC");
        }
        if (env != nullptr && *env != '\0') {
            try {
                duration = std::stod(env);
            } catch (...) {
                duration = 6.0;
            }
        } else {
            duration = 6.0;
        }
    }
    duration = std::clamp(duration, 0.2, 60.0);

    double kd_scale = 1.0;
    if (const char* env = std::getenv("OPENARM_QUEST_DAMP_KD_SCALE"); env && *env) {
        try {
            kd_scale = std::stod(env);
        } catch (...) {
            kd_scale = 1.0;
        }
    } else if (const char* env = std::getenv("OPENARM_REPLAY_DAMP_KD_SCALE"); env && *env) {
        try {
            kd_scale = std::stod(env);
        } catch (...) {
            kd_scale = 1.0;
        }
    }
    kd_scale = std::clamp(kd_scale, 0.05, 5.0);

    const int period_ms = 5;
    const int cycles = std::max(1, static_cast<int>(std::lround(duration * 1000.0 / period_ms)));

    const auto send_one_arm = [&kd, kd_scale](openarm::can::socket::OpenArm* openarm) {
        if (openarm == nullptr) {
            return;
        }
        openarm->recv_all(200);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(openarm->get_arm().get_motors().size());
        size_t idx = 0;
        for (const auto& motor : openarm->get_arm().get_motors()) {
            const double damping = (idx < kd.size() ? kd[idx] : 0.35) * kd_scale;
            arm_cmds.push_back({0.0, damping, motor.get_position(), 0.0, 0.0});
            ++idx;
        }

        std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
        gripper_cmds.reserve(openarm->get_gripper().get_motors().size());
        for (const auto& motor : openarm->get_gripper().get_motors()) {
            const double damping = (idx < kd.size() ? kd[idx] : 0.35) * kd_scale;
            gripper_cmds.push_back({0.0, damping, motor.get_position(), 0.0, 0.0});
            ++idx;
        }

        openarm->get_arm().mit_control_all(arm_cmds);
        openarm->get_gripper().mit_control_all(gripper_cmds);
    };

    if (left_openarm == nullptr && right_openarm == nullptr) {
        return;
    }
    std::cout << "[quest-cartesian] Kd-only damping release for " << duration
              << " s (Kp=0, tau_ff=0, kd_scale=" << kd_scale
              << "); motors stay enabled so arms settle under gravity" << std::endl;
    for (int i = 0; i < cycles; ++i) {
        send_one_arm(left_openarm);
        send_one_arm(right_openarm);
        if (left_openarm != nullptr) {
            left_openarm->recv_all(200);
        }
        if (right_openarm != nullptr) {
            right_openarm->recv_all(200);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
}

// Lightweight UDP: left/right gripper measured torque + MOS/rotor temps for Quest HUD.
class GripperTelemSender {
public:
    bool open(uint16_t port) {
        close();
        if (port == 0) {
            return false;
        }
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::cerr << "[quest-grip-telem] socket failed" << std::endl;
            return false;
        }
        addr_ = {};
        addr_.sin_family = AF_INET;
        addr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr_.sin_port = htons(port);
        std::cout << "[quest-grip-telem] -> 127.0.0.1:" << port << std::endl;
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~GripperTelemSender() { close(); }

    void maybe_send(openarm::can::socket::OpenArm* left, openarm::can::socket::OpenArm* right,
                    double min_period_s = 0.05) {
        if (fd_ < 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (last_send_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_send_).count() < min_period_s) {
            return;
        }
        last_send_ = now;

        double l_tau = 0, l_q = 0, r_tau = 0, r_q = 0;
        int l_mos = 0, l_rot = 0, r_mos = 0, r_rot = 0;
        read_gripper(left, &l_tau, &l_q, &l_mos, &l_rot);
        read_gripper(right, &r_tau, &r_q, &r_mos, &r_rot);

        const double t =
            std::chrono::duration<double>(now.time_since_epoch()).count();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4) << t << "," << l_tau << "," << l_mos << ","
            << l_rot << "," << l_q << "," << r_tau << "," << r_mos << "," << r_rot << "," << r_q;
        const std::string msg = oss.str();
        ::sendto(fd_, msg.data(), msg.size(), 0, reinterpret_cast<sockaddr*>(&addr_),
                 sizeof(addr_));
    }

private:
    static void read_gripper(openarm::can::socket::OpenArm* openarm, double* tau, double* q,
                             int* tmos, int* trotor) {
        if (openarm == nullptr) {
            return;
        }
        const auto& motors = openarm->get_gripper().get_motors();
        if (motors.empty()) {
            return;
        }
        const auto& m = motors.front();
        *tau = m.get_torque();
        *q = m.get_position();
        *tmos = m.get_state_tmos();
        *trotor = m.get_state_trotor();
    }

    int fd_ = -1;
    sockaddr_in addr_{};
    std::chrono::steady_clock::time_point last_send_{};
};

void configure_follower_control(Control* control, const FollowerParams& params, size_t total_joints,
                                bool sim) {
    std::vector<double> kp = params.kp;
    std::vector<double> kd = params.kd;
    if (!sim) {
        // Proximal joints: keep conservative MIT gains.
        // Distal arm j4–j6 (indices 4..6): keep stiff so j5/j6 orientation does not droop.
        for (size_t i = 0; i < kp.size(); ++i) {
            if (i >= 4 && i <= 6) {
                kp[i] *= 0.85;
                if (i < kd.size()) {
                    kd[i] *= 2.5;
                }
            } else {
                kp[i] *= 0.30;
                if (i < kd.size()) {
                    kd[i] *= 1.8;
                }
            }
        }
    }
    // Gripper = last DOF: stroke / position MIT with soft Kp (high Kp ⇒ crushing torque when
    // the fingers stall against an object while the stroke command keeps advancing).
    if (!kp.empty()) {
        const size_t g = kp.size() - 1;
        const double default_grip_kp = sim ? kp[g] : 6.0;
        const double default_grip_kd = (g < kd.size()) ? (sim ? kd[g] : 0.45) : 0.45;
        kp[g] = env_double_or("OPENARM_GRIPPER_KP", default_grip_kp);
        if (g < kd.size()) {
            kd[g] = env_double_or("OPENARM_GRIPPER_KD", default_grip_kd);
        }
        std::cout << "[quest-cartesian] gripper stroke MIT: Kp=" << kp[g]
                  << " Kd=" << (g < kd.size() ? kd[g] : 0.0)
                  << " (override with OPENARM_GRIPPER_KP / OPENARM_GRIPPER_KD)" << std::endl;
    }
    control->SetParameter(kp, kd, params.Fc, params.k, params.Fv, params.Fo);
    control->SetSafetyFilter(std::make_unique<SafetyFilter>(
        sim ? make_sim_safety_config(total_joints) : make_default_safety_config(total_joints)));
}

struct ArmTeleopBundle {
    std::string label;
    std::string arm_side;
    std::string mujoco_side;
    Dynamics* dynamics = nullptr;
    Control* control = nullptr;
    std::shared_ptr<RobotSystemState> state;
    std::unique_ptr<CartesianController> cartesian;
    std::unique_ptr<HumanMimicController> human_mimic;
    std::unique_ptr<SkeletonProximalController> skeleton_proximal;
    std::unique_ptr<QuestPoseReceiver> quest_receiver;
    std::shared_ptr<MujocoTeleopHardware> sim_hardware;
    openarm::can::socket::OpenArm* openarm = nullptr;
    uint64_t log_tick = 0;
    // Optional elbow-swivel hint (shared, non-owning; nullptr => built-in skeleton).
    SkeletonHintReceiver* skel_hint = nullptr;
    // Optional SEW pose stream for skeleton_proximal (shared, non-owning).
    SkeletonPoseReceiver* skel_pose = nullptr;
    bool is_left = false;
};

// Desktop-panel bridge: UDP record (start/stop/dump) + TCP jog (hold/release or nudge).
struct QuestUiBridge {
    UdpTextReceiver record_rx;
    UdpTextReceiver jog_rx;
    CsvRecorder recorder_left;
    CsvRecorder recorder_right;
    std::string sessions_root = "recordings/sessions";  // parent of named batches
    std::string record_dir;          // batch: sessions_root/<name>  (empty until first start/dump)
    std::string active_session_dir;  // current segment: .../01 , .../02
    std::string active_left_csv;
    std::string active_right_csv;
    std::string dump_path;
    double jog_step_m = 0.01;
    double jog_speed_mps = 0.02;
    int segment = 0;
    ArmTeleopBundle* left = nullptr;
    ArmTeleopBundle* right = nullptr;
    std::set<std::string> jog_dirs_left;
    std::set<std::string> jog_dirs_right;

    // Master (right) / slave (left) relative pose lock.
    bool couple_active = false;
    Eigen::Vector3d couple_p_rel = Eigen::Vector3d::Zero();   // left in right TCP frame
    Eigen::Matrix3d couple_R_rel = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d couple_R_right = Eigen::Matrix3d::Identity();
    // Relative residual closed-loop (scheme 1): correct left target using
    // Δp = left_tcp − (p_r + R_freeze·p_rel). Integral removes FB/UD bias.
    Eigen::Vector3d couple_integral = Eigen::Vector3d::Zero();
    uint64_t couple_fb_log_tick = 0;
    bool couple_fb_enabled = true;
    double couple_fb_kp = 0.55;
    double couple_fb_ki = 1.2;
    double couple_fb_imax = 0.05;     // m, per-axis integral clamp
    double couple_fb_meas_w = 0.35;   // blend measured residual into command residual
    double couple_fb_leak = 0.15;     // 1/s leak on integral
    double couple_fb_corr_max = 0.08; // m, total correction clamp

    void load_couple_fb_gains() {
        couple_fb_enabled = env_bool_or("OPENARM_COUPLE_FB", true);
        couple_fb_kp = env_double_or("OPENARM_COUPLE_KP", 0.55);
        couple_fb_ki = env_double_or("OPENARM_COUPLE_KI", 1.2);
        couple_fb_imax = env_double_or("OPENARM_COUPLE_IMAX", 0.05);
        couple_fb_meas_w = std::clamp(env_double_or("OPENARM_COUPLE_MEAS_W", 0.35), 0.0, 1.0);
        couple_fb_leak = std::max(0.0, env_double_or("OPENARM_COUPLE_ILEAK", 0.15));
        couple_fb_corr_max = std::max(0.005, env_double_or("OPENARM_COUPLE_CORR_MAX", 0.08));
    }

    bool engage_couple() {
        if (left == nullptr || right == nullptr || left->human_mimic == nullptr ||
            right->human_mimic == nullptr) {
            return false;
        }
        Eigen::Vector3d p_l, p_r;
        Eigen::Matrix3d R_l, R_r;
        if (!left->human_mimic->commanded_tcp_pose(&p_l, &R_l) ||
            !right->human_mimic->commanded_tcp_pose(&p_r, &R_r)) {
            std::cerr << "[quest-ui] couple engage failed: no commanded TCP" << std::endl;
            return false;
        }
        load_couple_fb_gains();
        couple_R_right = R_r;
        couple_p_rel = R_r.transpose() * (p_l - p_r);
        couple_R_rel = R_r.transpose() * R_l;
        couple_integral.setZero();
        couple_fb_log_tick = 0;
        right->human_mimic->set_orientation_lock(true, R_r);
        left->human_mimic->set_orientation_lock(true, R_l);
        left->human_mimic->set_couple_follow(true);
        couple_active = true;
        jog_dirs_left.clear();
        std::cout << "[quest-ui] couple ON — right=position-only master, left=relative slave  "
                     "||p_rel||="
                  << couple_p_rel.norm() << " m"
                  << "  fb=" << (couple_fb_enabled ? 1 : 0) << " kp=" << couple_fb_kp
                  << " ki=" << couple_fb_ki << std::endl;
        return true;
    }

    void disengage_couple() {
        if (!couple_active) {
            return;
        }
        couple_active = false;
        couple_integral.setZero();
        if (left != nullptr && left->human_mimic != nullptr) {
            left->human_mimic->set_couple_follow(false);
            left->human_mimic->set_orientation_lock(false);
        }
        if (right != nullptr && right->human_mimic != nullptr) {
            right->human_mimic->set_orientation_lock(false);
        }
        std::cout << "[quest-ui] couple OFF" << std::endl;
    }

    void apply_couple_left_follow(double dt) {
        if (!couple_active || left == nullptr || right == nullptr || left->human_mimic == nullptr ||
            right->human_mimic == nullptr) {
            return;
        }
        Eigen::Vector3d p_r;
        Eigen::Matrix3d R_r_cmd;
        if (!right->human_mimic->commanded_tcp_pose(&p_r, &R_r_cmd)) {
            return;
        }
        // Open-loop geometric slave pose (right orientation frozen at engage).
        const Eigen::Vector3d p_l_ol = p_r + couple_R_right * couple_p_rel;
        const Eigen::Matrix3d R_l = couple_R_right * couple_R_rel;

        Eigen::Vector3d p_l_des = p_l_ol;
        if (couple_fb_enabled && dt > 0.0) {
            Eigen::Vector3d p_l_cmd;
            Eigen::Matrix3d R_tmp;
            if (!left->human_mimic->commanded_tcp_pose(&p_l_cmd, &R_tmp)) {
                left->human_mimic->drive_tcp_robot(p_l_ol, R_l, dt);
                return;
            }
            Eigen::Vector3d residual = p_l_cmd - p_l_ol;
            if (couple_fb_meas_w > 1e-6 && left->state != nullptr) {
                const auto arm = left->state->arm_state().get_all_responses();
                std::vector<double> q_meas;
                q_meas.reserve(arm.size());
                for (const auto& j : arm) {
                    q_meas.push_back(j.position);
                }
                Eigen::Vector3d p_l_meas;
                if (left->human_mimic->fk_tcp_pose(q_meas, &p_l_meas, &R_tmp) &&
                    p_l_meas.allFinite()) {
                    const Eigen::Vector3d r_meas = p_l_meas - p_l_ol;
                    residual = (1.0 - couple_fb_meas_w) * residual + couple_fb_meas_w * r_meas;
                }
            }
            if (residual.allFinite()) {
                couple_integral *= std::max(0.0, 1.0 - couple_fb_leak * dt);
                couple_integral += couple_fb_ki * residual * dt;
                for (int i = 0; i < 3; ++i) {
                    couple_integral[i] =
                        std::clamp(couple_integral[i], -couple_fb_imax, couple_fb_imax);
                }
                Eigen::Vector3d corr = couple_fb_kp * residual + couple_integral;
                const double cn = corr.norm();
                if (cn > couple_fb_corr_max && cn > 1e-9) {
                    corr *= couple_fb_corr_max / cn;
                }
                p_l_des = p_l_ol - corr;
                if (++couple_fb_log_tick >= 50) {  // ~1 Hz at 50 Hz teleop
                    couple_fb_log_tick = 0;
                    std::cout << "[quest-ui] couple-fb  |res|=" << residual.norm() * 1000.0
                              << " mm  |I|=" << couple_integral.norm() * 1000.0
                              << " mm  |corr|=" << corr.norm() * 1000.0 << " mm" << std::endl;
                }
            }
        }
        left->human_mimic->drive_tcp_robot(p_l_des, R_l, dt);
    }

    bool open(const Options& opt) {
        if (!opt.quest_ui) {
            return true;
        }
        jog_step_m = std::max(1e-4, opt.quest_jog_step_m);
        jog_speed_mps = std::max(1e-4, opt.quest_jog_speed_mps);
        if (!record_rx.open(opt.quest_ui_record_port, "quest-ui-record")) {
            return false;
        }
        if (!jog_rx.open(opt.quest_ui_jog_port, "quest-ui-jog")) {
            return false;
        }
        if (!opt.quest_record_dir.empty()) {
            sessions_root = opt.quest_record_dir;
        } else {
            sessions_root = "recordings/sessions";
        }
        record_dir.clear();  // set on first start,<name> / dump,<name>
        dump_path.clear();
        std::cout << "Quest UI       : record UDP " << opt.quest_ui_record_port
                  << "  jog UDP " << opt.quest_ui_jog_port
                  << "  sessions_root=" << sessions_root
                  << "  jog_speed=" << jog_speed_mps << " m/s" << std::endl;
        std::cout << "  [quest-ui] session folder created only after Start/Dump; "
                     "name comes from panel (start,<name>)"
                  << std::endl;
        return true;
    }

    static std::string sanitize_session_name(std::string name) {
        for (char& c : name) {
            const unsigned char uc = static_cast<unsigned char>(c);
            if (std::isalnum(uc) || c == '_' || c == '-' || c == '.') {
                continue;
            }
            c = '_';
        }
        while (!name.empty() && (name.front() == '_' || name.front() == '.' || name.front() == '-')) {
            name.erase(name.begin());
        }
        while (!name.empty() && (name.back() == '_' || name.back() == '.' || name.back() == '-')) {
            name.pop_back();
        }
        if (name.empty()) {
            const auto stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm tm{};
            localtime_r(&stamp, &tm);
            char buf[64];
            std::strftime(buf, sizeof(buf), "quest_%Y%m%d_%H%M%S", &tm);
            name = buf;
        }
        return name;
    }

    bool ensure_batch_dir(const std::string& raw_name, bool ensure_dirs) {
        const std::string name = sanitize_session_name(raw_name);
        record_dir = sessions_root + "/" + name;
        dump_path = record_dir + "/pose_snapshots.txt";
        if (ensure_dirs) {
            std::error_code ec;
            std::filesystem::create_directories(record_dir, ec);
            if (ec) {
                std::cerr << "[quest-ui] failed to create " << record_dir << ": " << ec.message()
                          << std::endl;
                return false;
            }
        }
        return true;
    }

    void close_recorders() {
        recorder_left.close();
        recorder_right.close();
    }

    void finalize_recording_if_needed() {
        const bool was_recording = recorder_left.active() || recorder_right.active();
        close_recorders();
        if (was_recording) {
            merge_active_session_for_replay();
        }
    }

    ~QuestUiBridge() { finalize_recording_if_needed(); }

    void merge_active_session_for_replay() {
        if (active_left_csv.empty() || active_right_csv.empty() || active_session_dir.empty()) {
            return;
        }
        namespace fs = std::filesystem;
        if (!fs::is_regular_file(active_left_csv) || !fs::is_regular_file(active_right_csv)) {
            std::cerr << "[quest-ui] skip merge: missing left/right CSV" << std::endl;
            return;
        }
        // Header-only files also fail merge ("not enough valid frames").
        auto row_count = [](const std::string& path) -> std::size_t {
            std::ifstream in(path);
            std::size_t n = 0;
            std::string line;
            while (std::getline(in, line)) {
                ++n;
            }
            return n;
        };
        const auto left_rows = row_count(active_left_csv);
        const auto right_rows = row_count(active_right_csv);
        if (left_rows < 3 || right_rows < 3) {
            std::cerr << "[quest-ui] skip merge: need >=2 data rows per arm (got left_lines="
                      << left_rows << " right_lines=" << right_rows
                      << "). Split CSVs kept; re-record after rebuild if one side is empty."
                      << std::endl;
            return;
        }
        const std::string merged = active_session_dir + "/real_teleop_dual_merged.csv";
        // Prefer inplace script (cwd is openarm_inplace when launched from quest/*.sh).
        const std::string merge_py = "script/merge_dual_real_recordings.py";
        const std::string cmd = "python3 \"" + merge_py + "\" --left \"" + active_left_csv +
                                "\" --right \"" + active_right_csv + "\" --output \"" + merged +
                                "\" --crop min";
        std::cout << "[quest-ui] merging dual replay CSV..." << std::endl;
        const int rc = std::system(cmd.c_str());
        if (rc != 0) {
            std::cerr << "[quest-ui] merge failed (rc=" << rc << "). Split CSVs are still at:\n  "
                      << active_left_csv << "\n  " << active_right_csv << std::endl;
            return;
        }
        std::cout << "[quest-ui] replay-ready session: " << active_session_dir << std::endl;
        std::cout << "  merged : " << merged << std::endl;
        std::cout << "  replay : script/run_cpp_real_replay_both.sh " << active_session_dir
                  << std::endl;
        std::cout << "  or panel: script/run_cpp_real_replay_panel.sh  (pick this folder / "
                     "sessions parent)"
                  << std::endl;
    }

    static Eigen::Vector3d jog_delta(const std::string& dir, double step) {
        // Fallback axes (robot base) for legacy nudge when no controller is attached.
        if (dir == "up") {
            return {0.0, 0.0, step};
        }
        if (dir == "down") {
            return {0.0, 0.0, -step};
        }
        if (dir == "left") {
            return {0.0, step, 0.0};
        }
        if (dir == "right") {
            return {0.0, -step, 0.0};
        }
        return Eigen::Vector3d::Zero();
    }

    // World/teleop frame: up = config.up_axis, left = up×forward (matches VR lift).
    static Eigen::Vector3d velocity_from_dirs(HumanMimicController* hm,
                                             const std::set<std::string>& dirs, double speed) {
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        if (hm != nullptr) {
            const Eigen::Vector3d up = hm->panel_jog_up_axis();
            const Eigen::Vector3d left = hm->panel_jog_left_axis();
            for (const auto& d : dirs) {
                if (d == "up") {
                    v += up;
                } else if (d == "down") {
                    v -= up;
                } else if (d == "left") {
                    v += left;
                } else if (d == "right") {
                    v -= left;
                }
            }
        } else {
            for (const auto& d : dirs) {
                v += jog_delta(d, 1.0);
            }
        }
        const double n = v.norm();
        if (n < 1e-12) {
            return Eigen::Vector3d::Zero();
        }
        return (v / n) * speed;
    }

    void apply_nudge(ArmTeleopBundle* arm, const std::string& dir) {
        if (arm == nullptr || arm->human_mimic == nullptr) {
            return;
        }
        HumanMimicController* hm = arm->human_mimic.get();
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        if (dir == "up") {
            d = hm->panel_jog_up_axis() * jog_step_m;
        } else if (dir == "down") {
            d = -hm->panel_jog_up_axis() * jog_step_m;
        } else if (dir == "left") {
            d = hm->panel_jog_left_axis() * jog_step_m;
        } else if (dir == "right") {
            d = -hm->panel_jog_left_axis() * jog_step_m;
        }
        if (d.allFinite() && d.norm() > 1e-12) {
            hm->nudge_tcp_robot(d);
        }
    }

    void push_jog_velocities() {
        if (couple_active) {
            // Left is slave — ignore left panel holds while coupled.
            jog_dirs_left.clear();
        }
        const Eigen::Vector3d vl =
            velocity_from_dirs(left != nullptr ? left->human_mimic.get() : nullptr, jog_dirs_left,
                               jog_speed_mps);
        const Eigen::Vector3d vr =
            velocity_from_dirs(right != nullptr ? right->human_mimic.get() : nullptr,
                               jog_dirs_right, jog_speed_mps);
        if (left != nullptr && left->human_mimic != nullptr) {
            left->human_mimic->set_panel_jog_velocity(couple_active ? Eigen::Vector3d::Zero()
                                                                    : vl);
        }
        if (right != nullptr && right->human_mimic != nullptr) {
            right->human_mimic->set_panel_jog_velocity(vr);
        }
    }

    void dump_pose() {
        std::ostringstream line;
        line << std::fixed << std::setprecision(6) << "[positions] source=measured";
        auto append_arm = [&](const char* name, ArmTeleopBundle* arm) {
            line << " " << name << "=[";
            if (arm != nullptr && arm->state != nullptr) {
                const auto joints = collect_joint_states(arm->state);
                for (size_t i = 0; i < joints.size(); ++i) {
                    if (i) {
                        line << " ";
                    }
                    line << joints[i].position;
                }
            } else if (arm != nullptr && arm->human_mimic != nullptr) {
                const auto& q = arm->human_mimic->q_target();
                for (size_t i = 0; i < q.size(); ++i) {
                    if (i) {
                        line << " ";
                    }
                    line << q[i];
                }
            }
            line << "]";
        };
        append_arm("left", left);
        append_arm("right", right);
        const std::string text = line.str();
        std::cout << text << std::endl;
        std::ofstream out(dump_path, std::ios::app);
        if (out) {
            out << text << "\n";
        }
    }

    void handle_record_msg(const std::string& msg) {
        // Protocols: start | start,<name> | stop | dump | dump,<name> | shutdown
        std::string cmd;
        std::string arg;
        {
            const auto comma = msg.find(',');
            if (comma == std::string::npos) {
                cmd = msg;
            } else {
                cmd = msg.substr(0, comma);
                arg = msg.substr(comma + 1);
            }
            auto trim = [](std::string& s) {
                while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
                    s.erase(s.begin());
                }
                while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
                    s.pop_back();
                }
            };
            trim(cmd);
            trim(arg);
            for (char& c : cmd) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
        }

        if (cmd == "shutdown" || cmd == "exit" || cmd == "quit") {
            if (recorder_left.active() || recorder_right.active()) {
                close_recorders();
                merge_active_session_for_replay();
            }
            disengage_couple();
            std::cout << "[quest-ui] shutdown requested — exiting teleop after damping release"
                      << std::endl;
            keep_running = false;
            return;
        }

        if (cmd == "start") {
            const bool was_recording = recorder_left.active() || recorder_right.active();
            close_recorders();
            if (was_recording) {
                merge_active_session_for_replay();
            }
            if (!ensure_batch_dir(arg, /*ensure_dirs=*/true)) {
                return;
            }
            ++segment;
            char seg[16];
            std::snprintf(seg, sizeof(seg), "%02d", segment);
            active_session_dir = record_dir + "/" + seg;
            active_left_csv =
                active_session_dir + "/real_teleop_split/left/real_teleop_left.csv";
            active_right_csv =
                active_session_dir + "/real_teleop_split/right/real_teleop_right.csv";
            std::error_code ec;
            std::filesystem::create_directories(
                std::filesystem::path(active_left_csv).parent_path(), ec);
            std::filesystem::create_directories(
                std::filesystem::path(active_right_csv).parent_path(), ec);
            const size_t jl = (left != nullptr && left->human_mimic != nullptr)
                                  ? left->human_mimic->arm_dof() + 1
                                  : 8;
            const size_t jr = (right != nullptr && right->human_mimic != nullptr)
                                  ? right->human_mimic->arm_dof() + 1
                                  : 8;
            bool opened = false;
            if (left != nullptr) {
                opened = recorder_left.open(active_left_csv, jl) || opened;
            }
            if (right != nullptr) {
                opened = recorder_right.open(active_right_csv, jr) || opened;
            }
            if (!opened) {
                std::cerr << "[quest-ui] failed to open recorders under " << active_session_dir
                          << std::endl;
                active_session_dir.clear();
                active_left_csv.clear();
                active_right_csv.clear();
                return;
            }
            std::cout << "[quest-ui] recording segment " << seg << " -> " << active_session_dir
                      << std::endl;
            std::cout << "  left =" << active_left_csv << std::endl;
            std::cout << "  right=" << active_right_csv << std::endl;
        } else if (cmd == "stop") {
            close_recorders();
            std::cout << "[quest-ui] recording stopped" << std::endl;
            merge_active_session_for_replay();
        } else if (cmd == "dump") {
            if (!ensure_batch_dir(arg.empty() && !record_dir.empty()
                                     ? std::filesystem::path(record_dir).filename().string()
                                     : arg,
                                 /*ensure_dirs=*/true)) {
                return;
            }
            dump_pose();
        }
    }

    static bool is_jog_dir(const std::string& d) {
        return d == "up" || d == "down" || d == "left" || d == "right";
    }

    void set_dir(std::set<std::string>* dirs, const std::string& dir, bool on) {
        if (dirs == nullptr || !is_jog_dir(dir)) {
            return;
        }
        if (on) {
            dirs->insert(dir);
        } else {
            dirs->erase(dir);
        }
    }

    void handle_jog_msg(const std::string& msg) {
        // hold,left,up | release,left,up | nudge,left,up | couple,toggle|on|off
        std::string s = msg;
        for (char& c : s) {
            if (c == ' ') {
                c = ',';
            }
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        std::vector<std::string> parts;
        std::stringstream ss(s);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) {
                parts.push_back(tok);
            }
        }
        if (parts.empty()) {
            return;
        }
        if (parts[0] == "couple") {
            const std::string mode = parts.size() >= 2 ? parts[1] : "toggle";
            if (mode == "on" || mode == "1" || mode == "true") {
                engage_couple();
            } else if (mode == "off" || mode == "0" || mode == "false") {
                disengage_couple();
            } else {
                if (couple_active) {
                    disengage_couple();
                } else {
                    engage_couple();
                }
            }
            return;
        }
        if (parts.size() < 3) {
            return;
        }
        const std::string& cmd = parts[0];
        const std::string& side = parts[1];
        const std::string& dir = parts[2];

        if (cmd == "hold" || cmd == "release") {
            const bool on = (cmd == "hold");
            if (side == "left" || side == "l") {
                set_dir(&jog_dirs_left, dir, on);
            } else if (side == "right" || side == "r") {
                set_dir(&jog_dirs_right, dir, on);
            } else if (side == "both") {
                set_dir(&jog_dirs_left, dir, on);
                set_dir(&jog_dirs_right, dir, on);
            }
            return;
        }

        if (cmd != "nudge" || !is_jog_dir(dir)) {
            return;
        }
        if (side == "left" || side == "l") {
            apply_nudge(left, dir);
        } else if (side == "right" || side == "r") {
            apply_nudge(right, dir);
        } else if (side == "both") {
            apply_nudge(left, dir);
            apply_nudge(right, dir);
        }
    }

    void poll() {
        for (const auto& m : record_rx.receive_all()) {
            handle_record_msg(m);
        }
        for (const auto& m : jog_rx.receive_all()) {
            handle_jog_msg(m);
        }
        push_jog_velocities();
    }

    void append_recording(ArmTeleopBundle* arm, const std::vector<JointState>& reference) {
        if (arm == nullptr || reference.empty()) {
            return;
        }
        if (arm->is_left) {
            if (recorder_left.active()) {
                recorder_left.append(reference);
            }
        } else if (recorder_right.active()) {
            recorder_right.append(reference);
        }
    }
};

void prime_hardware_arm(ArmTeleopBundle* arm, size_t arm_dof, size_t hand_dof) {
    for (int i = 0; i < 5; ++i) {
        arm->control->unilateral_step();
    }
    const auto current = collect_joint_states(arm->state);
    arm->state->set_all_references(current);
    arm->control->ResetSafetyFilter(current);
    (void)arm_dof;
    (void)hand_dof;
}

void prime_sim_arm(ArmTeleopBundle* arm, size_t arm_dof, size_t hand_dof, bool table_ready) {
    if (table_ready && arm->sim_hardware) {
        const std::vector<double> q_arm = table_ready_arm_pose();
        const double q_grip = TABLE_READY_POSITION[7];
        arm->sim_hardware->set_home(q_arm, q_grip);

        std::vector<JointState> ready(arm_dof + hand_dof);
        for (size_t i = 0; i < arm_dof && i < q_arm.size(); ++i) {
            ready[i].position = q_arm[i];
            ready[i].velocity = 0.0;
            ready[i].effort = 0.0;
        }
        if (hand_dof > 0 && arm_dof < ready.size()) {
            ready[arm_dof].position = q_grip;
            ready[arm_dof].velocity = 0.0;
            ready[arm_dof].effort = 0.0;
        }
        arm->state->set_all_references(ready);
        std::cout << "  [" << arm->label << "] sim table-ready pose (elbow≈90°)" << std::endl;
    }
    for (int i = 0; i < 5; ++i) {
        arm->control->unilateral_step();
    }
    const auto current = collect_joint_states(arm->state);
    arm->state->set_all_references(current);
    arm->control->ResetSafetyFilter(current);
}

// ---------------------------------------------------------------------------
// Configurable startup motion (smoothly move arms to a pose / play a trajectory)
// ---------------------------------------------------------------------------
//
// File format (whitespace or comma separated, '#' comments, blank lines ignored):
//   [t]  side  q0 q1 q2 q3 q4 q5 q6  [grip]
//   - side : left | right | both
//   - t    : OPTIONAL leading time in seconds. If present on any row, rows are
//            played at those times; if absent, rows are spaced evenly over
//            --startup-move-sec. A single row per side => just move to that pose.
//   - grip : OPTIONAL 8th value (gripper). If omitted, the gripper is held.
// Example (target pose, both arms):   both 0 0 0 1.5708 0 0 0 0
struct StartupWaypoint {
    bool has_t = false;
    double t = 0.0;
    std::string side = "both";
    std::vector<double> q;  // 7 arm joints
    bool has_grip = false;
    double grip = 0.0;
};

bool parse_startup_line(const std::string& raw, StartupWaypoint* wp, std::string* err) {
    std::string line = raw;
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
        line = line.substr(0, hash);
    }
    for (char& c : line) {
        if (c == ',') {
            c = ' ';
        }
    }
    std::istringstream iss(line);
    std::vector<std::string> tok;
    std::string t;
    while (iss >> t) {
        tok.push_back(t);
    }
    if (tok.empty()) {
        return false;  // blank/comment: skip (not an error)
    }

    auto is_side = [](const std::string& s) {
        return s == "left" || s == "right" || s == "both";
    };
    auto to_num = [](const std::string& s, double* out) {
        try {
            size_t pos = 0;
            *out = std::stod(s, &pos);
            return pos == s.size();
        } catch (...) {
            return false;
        }
    };

    size_t idx = 0;
    // Optional leading time.
    if (!is_side(tok[0])) {
        double v = 0.0;
        if (to_num(tok[0], &v) && tok.size() > 1 && is_side(tok[1])) {
            wp->has_t = true;
            wp->t = v;
            idx = 1;
        }
    }
    // Side (default both if the row is pure numbers).
    if (idx < tok.size() && is_side(tok[idx])) {
        wp->side = tok[idx];
        ++idx;
    } else {
        wp->side = "both";
    }
    // Remaining tokens are joint values (+ optional grip).
    std::vector<double> nums;
    for (; idx < tok.size(); ++idx) {
        double v = 0.0;
        if (!to_num(tok[idx], &v)) {
            *err = "invalid number '" + tok[idx] + "'";
            return false;
        }
        nums.push_back(v);
    }
    if (nums.size() != 7 && nums.size() != 8) {
        *err = "expected 7 (arm) or 8 (arm+grip) joint values, got " +
               std::to_string(nums.size());
        return false;
    }
    wp->q.assign(nums.begin(), nums.begin() + 7);
    if (nums.size() == 8) {
        wp->has_grip = true;
        wp->grip = nums[7];
    }
    return true;
}

bool load_startup_waypoints(const std::string& path, std::vector<StartupWaypoint>* out,
                            std::string* err) {
    std::ifstream file(path);
    if (!file.good()) {
        *err = "cannot open startup file: " + path;
        return false;
    }
    std::string line;
    int lineno = 0;
    while (std::getline(file, line)) {
        ++lineno;
        StartupWaypoint wp;
        std::string line_err;
        if (parse_startup_line(line, &wp, &line_err)) {
            out->push_back(wp);
        } else if (!line_err.empty()) {
            *err = "line " + std::to_string(lineno) + ": " + line_err;
            return false;
        }
    }
    if (out->empty()) {
        *err = "no waypoints parsed from " + path;
        return false;
    }
    return true;
}

// Timed per-arm waypoint (arm joints + gripper), sorted by time.
struct TimedPose {
    double t = 0.0;
    std::vector<double> q;  // arm joints
    double grip = 0.0;
};

// Build a timed sequence for one arm: starts from the current measured pose at
// t=0, then the file waypoints applicable to this arm (side match or "both").
std::vector<TimedPose> build_arm_sequence(const std::string& arm_label,
                                          const std::vector<StartupWaypoint>& wps,
                                          const std::vector<double>& q_start,
                                          double grip_start, double move_sec, Dynamics* dyn) {
    std::vector<StartupWaypoint> mine;
    for (const auto& w : wps) {
        if (w.side == "both" || w.side == arm_label) {
            mine.push_back(w);
        }
    }
    std::vector<TimedPose> seq;
    TimedPose start;
    start.t = 0.0;
    start.q = q_start;
    start.grip = grip_start;
    seq.push_back(start);
    if (mine.empty()) {
        return seq;
    }

    const bool timed = mine.front().has_t;
    double prev_grip = grip_start;
    for (size_t i = 0; i < mine.size(); ++i) {
        TimedPose tp;
        tp.q = mine[i].q;
        if (dyn != nullptr) {
            std::vector<double> full = tp.q;
            full.resize(std::max<size_t>(full.size(), dyn->GetDof()), 0.0);
            dyn->ClampJointPositions(full.data());
            for (size_t k = 0; k < tp.q.size() && k < full.size(); ++k) {
                tp.q[k] = full[k];
            }
        }
        tp.grip = mine[i].has_grip ? mine[i].grip : prev_grip;
        prev_grip = tp.grip;
        if (timed) {
            tp.t = mine[i].t;
        } else {
            tp.t = move_sec * static_cast<double>(i + 1) / static_cast<double>(mine.size());
        }
        seq.push_back(tp);
    }
    return seq;
}

double smoothstep01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// Interpolate an arm pose from a timed sequence at query time `t` (smoothstep).
TimedPose sample_sequence(const std::vector<TimedPose>& seq, double t) {
    if (seq.empty()) {
        return TimedPose{};
    }
    if (t <= seq.front().t) {
        return seq.front();
    }
    if (t >= seq.back().t) {
        return seq.back();
    }
    for (size_t i = 1; i < seq.size(); ++i) {
        if (t <= seq[i].t) {
            const TimedPose& a = seq[i - 1];
            const TimedPose& b = seq[i];
            const double span = std::max(1e-6, b.t - a.t);
            const double s = smoothstep01((t - a.t) / span);
            TimedPose out;
            out.t = t;
            out.q.resize(a.q.size());
            for (size_t k = 0; k < a.q.size(); ++k) {
                out.q[k] = a.q[k] + (b.q[k] - a.q[k]) * s;
            }
            out.grip = a.grip + (b.grip - a.grip) * s;
            return out;
        }
    }
    return seq.back();
}

// Smoothly drive the given arms through their startup sequences. Assumes the
// follower control thread is already running (it applies the references).
void run_startup_move(const std::vector<ArmTeleopBundle*>& arms,
                      const std::vector<StartupWaypoint>& wps, double move_sec) {
    if (arms.empty()) {
        return;
    }
    std::vector<std::vector<TimedPose>> seqs;
    std::vector<size_t> arm_dofs;
    std::vector<size_t> total_dofs;
    double t_end = 0.0;
    for (ArmTeleopBundle* arm : arms) {
        const auto cur = collect_joint_states(arm->state);
        const size_t arm_dof = arm->dynamics->GetDof();
        std::vector<double> q_start(arm_dof, 0.0);
        for (size_t i = 0; i < arm_dof && i < cur.size(); ++i) {
            q_start[i] = cur[i].position;
        }
        const double grip_start = (cur.size() > arm_dof) ? cur[arm_dof].position : 0.0;
        auto seq = build_arm_sequence(arm->label, wps, q_start, grip_start, move_sec,
                                      arm->dynamics);
        t_end = std::max(t_end, seq.back().t);
        seqs.push_back(std::move(seq));
        arm_dofs.push_back(arm_dof);
        total_dofs.push_back(cur.size());
    }
    if (t_end <= 1e-3) {
        return;
    }

    std::cout << "[quest-cartesian] startup move: driving " << arms.size()
              << " arm(s) over " << t_end << "s..." << std::endl;

    const double rate_hz = 200.0;
    const double dt = 1.0 / rate_hz;
    const auto t0 = std::chrono::steady_clock::now();
    double t = 0.0;
    while (keep_running && t <= t_end + dt) {
        for (size_t a = 0; a < arms.size(); ++a) {
            const TimedPose tp = sample_sequence(seqs[a], t);
            std::vector<JointState> ref(total_dofs[a]);
            for (size_t i = 0; i < total_dofs[a]; ++i) {
                if (i < arm_dofs[a] && i < tp.q.size()) {
                    ref[i].position = tp.q[i];
                } else if (i == arm_dofs[a]) {
                    ref[i].position = tp.grip;
                }
                ref[i].velocity = 0.0;
                ref[i].effort = 0.0;
            }
            arms[a]->state->set_all_references(ref);
        }
        std::this_thread::sleep_until(t0 + std::chrono::duration<double>(t + dt));
        t += dt;
    }

    // Settle at the final target and hold.
    for (size_t a = 0; a < arms.size(); ++a) {
        const TimedPose tp = sample_sequence(seqs[a], t_end);
        std::vector<JointState> ref(total_dofs[a]);
        for (size_t i = 0; i < total_dofs[a]; ++i) {
            if (i < arm_dofs[a] && i < tp.q.size()) {
                ref[i].position = tp.q[i];
            } else if (i == arm_dofs[a]) {
                ref[i].position = tp.grip;
            }
        }
        arms[a]->state->set_all_references(ref);
    }
    std::cout << "[quest-cartesian] startup move complete." << std::endl;
}

// Resolve the startup waypoints: from file if provided, else a single
// TABLE_READY target (both arms), matching the sim posture.
bool make_startup_waypoints(const Options& options, std::vector<StartupWaypoint>* wps,
                            std::string* err) {
    if (!options.startup_file.empty()) {
        return load_startup_waypoints(options.startup_file, wps, err);
    }
    StartupWaypoint w;
    w.side = "both";
    w.q = table_ready_arm_pose();
    w.has_grip = true;
    w.grip = TABLE_READY_POSITION[7];
    wps->push_back(w);
    return true;
}

}  // namespace

void maybe_log_teleop_trajectory(const std::string& label, Dynamics* dynamics,
                                 CartesianController* cartesian, bool grip_active,
                                 const std::vector<JointState>& current,
                                 const std::vector<JointState>& reference) {
    auto& logger = TeleopTrajectoryLogger::shared();
    if (!logger.enabled() || dynamics == nullptr || cartesian == nullptr) {
        return;
    }
    logger.log(label, dynamics, cartesian->state(), grip_active,
               cartesian->tcp_target_position(), current, reference);
}

class FollowerControlThread : public PeriodicTimerThread {
public:
    FollowerControlThread(std::shared_ptr<RobotSystemState> follower_state, Control* control,
                          std::shared_ptr<MujocoTeleopHardware> sim_hardware = nullptr,
                          double hz = FREQUENCY)
        : PeriodicTimerThread(hz),
          follower_state_(std::move(follower_state)),
          control_(control),
          sim_hardware_(std::move(sim_hardware)) {}

protected:
    void before_start() override { std::cout << "[quest-cartesian] follower thread start"
                                             << std::endl; }

    void after_stop() override { std::cout << "[quest-cartesian] follower thread stop" << std::endl; }

    void on_timer() override {
        control_->unilateral_step();
        if (sim_hardware_ && sim_hardware_->window_should_close()) {
            keep_running = false;
        }
    }

private:
    std::shared_ptr<RobotSystemState> follower_state_;
    Control* control_;
    std::shared_ptr<MujocoTeleopHardware> sim_hardware_;
};

// ~90 Hz: drain Quest UDP and stamp receive time / sequence. Does not run DLS.
class QuestReceiveThread : public PeriodicTimerThread {
public:
    QuestReceiveThread(QuestPoseReceiver* receiver, std::string label, double hz = kQuestReceiveHz)
        : PeriodicTimerThread(hz), receiver_(receiver), label_(std::move(label)) {}

protected:
    void before_start() override {
        std::cout << "[quest-cartesian] Quest receive @" << kQuestReceiveHz << " Hz (" << label_
                  << ")" << std::endl;
    }

    void on_timer() override {
        if (!keep_running || receiver_ == nullptr) {
            return;
        }
        (void)receiver_->receive_latest();
    }

private:
    QuestPoseReceiver* receiver_ = nullptr;
    std::string label_;
};

// Drains the (optional) HybrIK elbow-swivel hint stream. Independent of teleop:
// if no bridge is running, this simply finds nothing and consumers fall back to
// the built-in virtual skeleton.
class SkeletonHintReceiveThread : public PeriodicTimerThread {
public:
    SkeletonHintReceiveThread(SkeletonHintReceiver* receiver, double hz = kQuestReceiveHz)
        : PeriodicTimerThread(hz), receiver_(receiver) {}

protected:
    void before_start() override {
        std::cout << "[skel-hint] receive @" << kQuestReceiveHz << " Hz" << std::endl;
    }
    void on_timer() override {
        if (!keep_running || receiver_ == nullptr) {
            return;
        }
        (void)receiver_->receive_latest();
    }

private:
    SkeletonHintReceiver* receiver_ = nullptr;
};

class SkeletonPoseReceiveThread : public PeriodicTimerThread {
public:
    SkeletonPoseReceiveThread(SkeletonPoseReceiver* receiver, double hz = kQuestReceiveHz)
        : PeriodicTimerThread(hz), receiver_(receiver) {}

protected:
    void before_start() override {
        std::cout << "[skel-pose] receive @" << kQuestReceiveHz << " Hz" << std::endl;
    }
    void on_timer() override {
        if (!keep_running || receiver_ == nullptr) {
            return;
        }
        (void)receiver_->receive_latest();
    }

private:
    SkeletonPoseReceiver* receiver_ = nullptr;
};

class BodyPoseReceiveThread : public PeriodicTimerThread {
public:
    BodyPoseReceiveThread(BodyPoseReceiver* receiver, double hz = kQuestReceiveHz)
        : PeriodicTimerThread(hz), receiver_(receiver) {}

protected:
    void before_start() override {
        std::cout << "[body-pose] receive @" << kQuestReceiveHz
                  << " Hz (phase-2 map+viz when --body-pose; no robot drive)" << std::endl;
    }
    void on_timer() override {
        if (!keep_running || receiver_ == nullptr) {
            return;
        }
        (void)receiver_->receive_latest();
    }

private:
    BodyPoseReceiver* receiver_ = nullptr;
};

void maybe_print_body_pose(BodyPoseReceiver* receiver, BodySkeletonFilter* filter,
                           double* last_print_s) {
    if (receiver == nullptr || last_print_s == nullptr) {
        return;
    }
    const double now =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now - *last_print_s < 0.5) {
        return;
    }
    *last_print_s = now;
    const auto sample = receiver->latest();
    const double age_ms = receiver->latest_age_ms();
    if (!sample.has_value()) {
        std::cout << "[body-pose] waiting for UDP packets (age_ms=n/a)" << std::endl;
        return;
    }
    const BodyPoseSample& s = *sample;
    const double l_se = (s.left_elbow.p - s.left_shoulder.p).norm();
    const double r_se = (s.right_elbow.p - s.right_shoulder.p).norm();
    std::cout << "[body-pose] age_ms=" << age_ms << " valid=" << (s.valid ? 1 : 0)
              << " conf_L=" << s.arm_confidence_left() << " conf_R=" << s.arm_confidence_right()
              << " |L_SE|=" << l_se << " |R_SE|=" << r_se << " seq=" << s.sequence_id;

    if (filter != nullptr) {
        // Keep filter warm even if MuJoCo render path is not updating.
        (void)filter->update(s, age_ms, 0.5);
        const BodySkeletonMapped& m = filter->latest();
        if (m.torso_valid) {
            auto elev_azim = [](const Eigen::Vector3d& u) {
                Eigen::Vector3d v = u.normalized();
                const double elev = std::asin(std::clamp(v.z(), -1.0, 1.0));
                const double azim = std::atan2(v.y(), v.x());
                return std::pair<double, double>(elev * 180.0 / M_PI, azim * 180.0 / M_PI);
            };
            if (m.left.valid) {
                const auto ea = elev_azim(m.left.u_upper);
                std::cout << " | L elev=" << ea.first << "deg azim=" << ea.second
                          << "deg elbow=" << (m.left.elbow_angle_rad * 180.0 / M_PI) << "deg";
            }
            if (m.right.valid) {
                const auto ea = elev_azim(m.right.u_upper);
                std::cout << " | R elev=" << ea.first << "deg azim=" << ea.second
                          << "deg elbow=" << (m.right.elbow_angle_rad * 180.0 / M_PI) << "deg";
            }
        } else {
            std::cout << " | torso=0";
        }
    }
    std::cout << std::endl;
}

Eigen::Vector3d mujoco_body_xpos(const mjModel* model, const mjData* data, const char* name) {
    if (model == nullptr || data == nullptr || name == nullptr) {
        return Eigen::Vector3d::Zero();
    }
    const int id = mj_name2id(model, mjOBJ_BODY, name);
    if (id < 0) {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(data->xpos[3 * id], data->xpos[3 * id + 1], data->xpos[3 * id + 2]);
}

void append_mapped_arm_overlays(const MappedArmDirs& arm, const Eigen::Vector3d& shoulder,
                                const SkeletonRetargetConfig& cfg, bool is_left,
                                std::vector<MujocoDebugSegment>* segments,
                                std::vector<MujocoDebugSphere>* spheres) {
    if (segments == nullptr || spheres == nullptr || !arm.valid) {
        return;
    }
    const Eigen::Vector3d elbow = shoulder + cfg.viz_L1 * arm.u_upper.normalized();
    const Eigen::Vector3d wrist = elbow + cfg.viz_L2 * arm.u_fore.normalized();
    MujocoDebugSegment upper;
    upper.p0 = shoulder;
    upper.p1 = elbow;
    upper.width = static_cast<float>(cfg.viz_stick_width);
    if (is_left) {
        upper.rgba[0] = 0.1f;
        upper.rgba[1] = 0.85f;
        upper.rgba[2] = 1.0f;
        upper.rgba[3] = 1.0f;
    } else {
        upper.rgba[0] = 1.0f;
        upper.rgba[1] = 0.45f;
        upper.rgba[2] = 0.1f;
        upper.rgba[3] = 1.0f;
    }
    MujocoDebugSegment fore = upper;
    fore.p0 = elbow;
    fore.p1 = wrist;
    if (is_left) {
        fore.rgba[0] = 0.85f;
        fore.rgba[1] = 0.2f;
        fore.rgba[2] = 1.0f;
    } else {
        fore.rgba[0] = 1.0f;
        fore.rgba[1] = 0.9f;
        fore.rgba[2] = 0.15f;
    }
    segments->push_back(upper);
    segments->push_back(fore);

    MujocoDebugSphere s0;
    MujocoDebugSphere s1;
    MujocoDebugSphere s2;
    s0.p = shoulder;
    s1.p = elbow;
    s2.p = wrist;
    s0.radius = s1.radius = s2.radius = 0.025f;
    for (int i = 0; i < 4; ++i) {
        s0.rgba[i] = upper.rgba[i];
        s1.rgba[i] = upper.rgba[i];
        s2.rgba[i] = fore.rgba[i];
    }
    spheres->push_back(s0);
    spheres->push_back(s1);
    spheres->push_back(s2);
}

void update_body_pose_mujoco_viz(MujocoSimWorld* world, BodyPoseReceiver* receiver,
                                 BodySkeletonFilter* filter, SkeletonRetargetConfig* cfg,
                                 double* last_filter_t) {
    if (world == nullptr || receiver == nullptr || filter == nullptr || cfg == nullptr ||
        last_filter_t == nullptr) {
        return;
    }
    const double now =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double dt = now - *last_filter_t;
    if (*last_filter_t <= 0.0 || dt <= 0.0 || dt > 0.5) {
        dt = 1.0 / 60.0;
    }
    *last_filter_t = now;

    const auto sample = receiver->latest();
    if (!sample.has_value()) {
        world->set_debug_overlays({}, {});
        return;
    }
    const double age_ms = receiver->latest_age_ms();
    const auto mapped = filter->update(*sample, age_ms, dt);

    std::vector<MujocoDebugSegment> segs;
    std::vector<MujocoDebugSphere> spheres;
    if (mapped.has_value() && mapped->torso_valid) {
        const mjModel* model = world->model();
        mjData* data = world->data();
        Eigen::Vector3d S_l = Eigen::Vector3d::Zero();
        Eigen::Vector3d S_r = Eigen::Vector3d::Zero();
        {
            std::lock_guard<std::mutex> lock(world->mutex());
            mj_forward(const_cast<mjModel*>(model), data);
            S_l = mujoco_body_xpos(model, data, "openarm_left_link1");
            S_r = mujoco_body_xpos(model, data, "openarm_right_link1");
        }
        append_mapped_arm_overlays(mapped->left, S_l, *cfg, true, &segs, &spheres);
        append_mapped_arm_overlays(mapped->right, S_r, *cfg, false, &segs, &spheres);
    }
    world->set_debug_overlays(std::move(segs), std::move(spheres));
}

// Max staleness of a hint before it's treated as absent (dropout fallback).
constexpr double kSkeletonHintMaxAgeMs = 250.0;
// Confidence below this => ignore the hint entirely.
constexpr double kSkeletonHintMinConf = 0.5;

// Fresh/valid HybrIK phi for one side (bridge frame; sign/offset in controller).
inline bool usable_elbow_hint(SkeletonHintReceiver* receiver, bool is_left, double* phi,
                              double* conf_out = nullptr, double* age_out = nullptr) {
    if (receiver == nullptr) {
        return false;
    }
    auto sample = receiver->latest();
    if (!sample.has_value() || !sample->valid) {
        return false;
    }
    const double age_ms = receiver->latest_age_ms();
    if (age_out != nullptr) {
        *age_out = age_ms;
    }
    const double conf = is_left ? sample->conf_left : sample->conf_right;
    if (conf_out != nullptr) {
        *conf_out = conf;
    }
    if (age_ms > kSkeletonHintMaxAgeMs || conf < kSkeletonHintMinConf) {
        return false;
    }
    if (phi != nullptr) {
        *phi = is_left ? sample->phi_left : sample->phi_right;
    }
    return true;
}

// High-rate DLS servo: reuses latest T_target from Quest cache; dt = controller period.
class CartesianReferenceThread : public PeriodicTimerThread {
public:
    CartesianReferenceThread(std::shared_ptr<RobotSystemState> follower_state,
                             CartesianController* cartesian_controller,
                             QuestPoseReceiver* quest_receiver, Dynamics* dynamics,
                             std::string label, double hz = kCartesianDlsHz,
                             HumanMimicController* human_mimic = nullptr,
                             SkeletonHintReceiver* skel_hint = nullptr, bool is_left = false,
                             SkeletonProximalController* skeleton_proximal = nullptr,
                             SkeletonPoseReceiver* skel_pose = nullptr)
        : PeriodicTimerThread(hz),
          follower_state_(std::move(follower_state)),
          cartesian_controller_(cartesian_controller),
          human_mimic_(human_mimic),
          skeleton_proximal_(skeleton_proximal),
          quest_receiver_(quest_receiver),
          skel_hint_(skel_hint),
          skel_pose_(skel_pose),
          is_left_(is_left),
          dynamics_(dynamics),
          label_(std::move(label)),
          nominal_dt_(1.0 / hz) {}

protected:
    void before_start() override {
        last_step_time_ = std::chrono::steady_clock::now();
        std::cout << "[quest-cartesian] DLS reference thread start @" << (1.0 / nominal_dt_)
                  << " Hz" << std::endl;
    }

    void after_stop() override { std::cout << "[quest-cartesian] DLS reference thread stop" << std::endl; }

    void on_timer() override {
        if (!keep_running) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        double dt = nominal_dt_;
        if (last_step_time_.time_since_epoch().count() != 0) {
            dt = std::chrono::duration<double>(now - last_step_time_).count();
            dt = std::clamp(dt, 0.5 * nominal_dt_, 3.0 * nominal_dt_);
        }
        last_step_time_ = now;

        const auto current = collect_joint_states(follower_state_);
        // Use cached Quest sample only — do not drain UDP here (QuestReceiveThread owns that).
        auto quest_pose = quest_receiver_->latest();
        QuestPoseSample sample;
        if (quest_pose.has_value()) {
            sample = quest_pose.value();
        } else {
            sample.valid = false;
        }

        // skeleton_proximal mode: q0-q3 follow SEW; q4-q6 VR orientation only.
        if (skeleton_proximal_ != nullptr) {
            SkeletonPoseArm pose_arm;
            bool pose_ok = false;
            double age_ms = 1e9;
            if (skel_pose_ != nullptr) {
                auto latest = skel_pose_->latest();
                age_ms = skel_pose_->latest_age_ms();
                if (latest.has_value() && latest->valid) {
                    pose_arm = is_left_ ? latest->left : latest->right;
                    pose_ok = true;
                }
            }
            skeleton_proximal_->set_skeleton_pose(pose_arm, pose_ok, age_ms);
            std::vector<JointState> reference;
            (void)skeleton_proximal_->step(current, sample, dt, &reference);
            if (!reference.empty()) {
                follower_state_->set_all_references(reference);
            }
            maybe_log_teleop_trajectory(label_, dynamics_, nullptr, sample.grip_active, current,
                                        reference);
            if (++log_tick_ >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
                log_tick_ = 0;
                const auto stats = quest_receiver_->snapshot_stats();
                std::cout << "[" << label_ << "-skel-prox] udp=" << stats.packets_window
                          << "Hz grip=" << (sample.grip_active ? 1 : 0)
                          << " clutch=" << (skeleton_proximal_->clutch_active() ? 1 : 0)
                          << " pose=" << (skeleton_proximal_->pose_valid() ? 1 : 0)
                          << " conf=" << skeleton_proximal_->pose_conf() << std::endl;
            }
            return;
        }

        // HUMAN_MIMIC mode: independent whole-arm absolute-IK path.
        if (human_mimic_ != nullptr) {
            double hint_conf = 0.0, hint_age = -1.0, hint_phi = 0.0;
            const bool hint_ok =
                usable_elbow_hint(skel_hint_, is_left_, &hint_phi, &hint_conf, &hint_age);
            if (hint_ok) {
                human_mimic_->set_elbow_hint(hint_phi, true);
            } else {
                human_mimic_->set_elbow_hint(0.0, false);
            }

            std::vector<JointState> reference;
            const bool ok = human_mimic_->step(current, sample, dt, &reference);
            (void)ok;
            if (!reference.empty()) {
                follower_state_->set_all_references(reference);
            }
            maybe_log_teleop_trajectory(label_, dynamics_, nullptr, sample.grip_active, current,
                                        reference);
            if (++log_tick_ >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
                log_tick_ = 0;
                const auto stats = quest_receiver_->snapshot_stats();
                std::cout << "[" << label_ << "-human-mimic] udp=" << stats.packets_window
                          << "Hz age_ms=" << stats.last_age_ms
                          << " grip=" << (stats.last_grip ? 1 : 0)
                          << " trig=" << stats.last_trigger
                          << " clutch=" << (human_mimic_->clutch_active() ? 1 : 0)
                          << " btnClose=" << (human_mimic_->last_button_close() ? 1 : 0)
                          << " btnOpen=" << (human_mimic_->last_button_open() ? 1 : 0)
                          << " grip_mot=" << human_mimic_->gripper_motion_code()
                          << " grip_tgt=" << human_mimic_->gripper_target()
                          << " grip_q=" << human_mimic_->gripper_measured()
                          << " quest_to=" << (human_mimic_->quest_timeout_active() ? 1 : 0);
                if (skel_hint_ != nullptr) {
                    std::cout << " skel_hint=" << (hint_ok ? 1 : 0)
                              << " phi_hint=" << (hint_ok ? hint_phi : 0.0)
                              << " phi_used=" << human_mimic_->elbow_hint_phi_used()
                              << " phi_base=" << human_mimic_->skeleton_phi_base()
                              << " conf=" << hint_conf << " hint_age=" << hint_age;
                }
                std::cout << std::endl;
            }
            return;
        }

        std::vector<JointState> reference;
        const bool ok = cartesian_controller_->step(current, sample, dt, &reference);
        const auto cart_state = cartesian_controller_->state();
        if (cart_state == CartesianControlState::Estop && last_warn_state_ != CartesianControlState::Estop) {
            std::cerr << "[" << label_ << "-quest-cartesian] repeated IK failure, holding last safe reference"
                      << std::endl;
        }
        last_warn_state_ = cart_state;
        (void)ok;
        if (!reference.empty()) {
            follower_state_->set_all_references(reference);
        }

        maybe_log_teleop_trajectory(label_, dynamics_, cartesian_controller_, sample.grip_active,
                                    current, reference);

        std::string fatal;
        if (cartesian_controller_->consume_fatal_error(&fatal)) {
            std::cerr << "[quest-cartesian][FATAL][" << label_ << "] " << fatal << std::endl;
            keep_running = false;
            return;
        }

        if (++log_tick_ >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
            log_tick_ = 0;
            const auto stats = quest_receiver_->snapshot_stats();
            const char* state_name = "idle";
            switch (cartesian_controller_->state()) {
                case CartesianControlState::Active:
                    state_name = "active";
                    break;
                case CartesianControlState::Degraded:
                    state_name = "degraded";
                    break;
                case CartesianControlState::Hold:
                    state_name = "hold";
                    break;
                case CartesianControlState::Estop:
                    state_name = "estop";
                    break;
                case CartesianControlState::Idle:
                default:
                    state_name = "idle";
                    break;
            }
            std::cout << "[" << label_ << "-quest-cartesian] udp=" << stats.packets_window
                      << "Hz age_ms=" << stats.last_age_ms
                      << " has_pose=" << (stats.has_latest ? 1 : 0)
                      << " grip=" << (stats.last_grip ? 1 : 0)
                      << " trigger=" << stats.last_trigger
                      << " cartesian=" << state_name
                      << " ori=" << (cartesian_controller_->orientation_follow_enabled() ? "VR" : "HOLD")
                      << " quest_to=" << (cartesian_controller_->quest_timeout_active() ? 1 : 0);
            if (stats.last_grip && !reference.empty() && !current.empty()) {
                double max_ref = 0.0;
                double max_q = 0.0;
                double ref_delta = 0.0;
                const size_t arm_n = std::min(reference.size(), current.size());
                for (size_t i = 0; i < arm_n; ++i) {
                    max_ref = std::max(max_ref, std::abs(reference[i].position));
                    max_q = std::max(max_q, std::abs(current[i].position));
                    ref_delta = std::max(ref_delta,
                                         std::abs(reference[i].position - current[i].position));
                }
                std::cout << " tcp_off=" << cartesian_controller_->tcp_target_offset_norm()
                          << "/" << cartesian_controller_->teleop_offset_limit_m()
                          << " off_sat=" << (cartesian_controller_->teleop_offset_saturated() ? 1 : 0)
                          << " rad_sat="
                          << (cartesian_controller_->workspace_radius_saturated() ? 1 : 0)
                          << " rot_off=" << cartesian_controller_->rotation_target_offset_rad()
                          << " max_ref=" << max_ref << " max_q=" << max_q
                          << " ref_d=" << ref_delta;
            }
            std::cout << std::endl;
        }
    }

private:
    std::shared_ptr<RobotSystemState> follower_state_;
    CartesianController* cartesian_controller_;
    HumanMimicController* human_mimic_ = nullptr;
    SkeletonProximalController* skeleton_proximal_ = nullptr;
    QuestPoseReceiver* quest_receiver_;
    SkeletonHintReceiver* skel_hint_ = nullptr;
    SkeletonPoseReceiver* skel_pose_ = nullptr;
    bool is_left_ = false;
    Dynamics* dynamics_ = nullptr;
    std::string label_;
    double nominal_dt_ = 1.0 / kCartesianDlsHz;
    std::chrono::steady_clock::time_point last_step_time_;
    uint64_t log_tick_ = 0;
    CartesianControlState last_warn_state_ = CartesianControlState::Idle;
};

class BimanualFollowerControlThread : public PeriodicTimerThread {
public:
    BimanualFollowerControlThread(ArmTeleopBundle* left, ArmTeleopBundle* right,
                                  std::shared_ptr<MujocoSimWorld> sim_world, double hz = FREQUENCY)
        : PeriodicTimerThread(hz), left_(left), right_(right), sim_world_(std::move(sim_world)) {}

protected:
    void on_timer() override {
        if (left_ != nullptr) {
            left_->control->unilateral_step();
        }
        if (right_ != nullptr) {
            right_->control->unilateral_step();
        }
        if (sim_world_ && sim_world_->window_should_close()) {
            keep_running = false;
        }
    }

private:
    ArmTeleopBundle* left_;
    ArmTeleopBundle* right_;
    std::shared_ptr<MujocoSimWorld> sim_world_;
};

class BimanualCartesianReferenceThread : public PeriodicTimerThread {
public:
    BimanualCartesianReferenceThread(ArmTeleopBundle* left, ArmTeleopBundle* right,
                                     QuestUiBridge* ui = nullptr, double hz = kCartesianDlsHz)
        : PeriodicTimerThread(hz), left_(left), right_(right), ui_(ui),
          nominal_dt_(1.0 / hz) {}

protected:
    void before_start() override {
        last_step_time_ = std::chrono::steady_clock::now();
        std::cout << "[quest-cartesian] bimanual DLS reference thread start @"
                  << (1.0 / nominal_dt_) << " Hz" << std::endl;
    }

    void on_timer() override {
        if (ui_ != nullptr) {
            ui_->poll();
        }
        const auto now = std::chrono::steady_clock::now();
        double dt = nominal_dt_;
        if (last_step_time_.time_since_epoch().count() != 0) {
            dt = std::chrono::duration<double>(now - last_step_time_).count();
            dt = std::clamp(dt, 0.5 * nominal_dt_, 3.0 * nominal_dt_);
        }
        last_step_time_ = now;

        if (ui_ != nullptr && ui_->couple_active) {
            // Right master first, then left slave follows locked relative pose.
            step_arm(right_, dt);
            ui_->apply_couple_left_follow(dt);
            step_arm(left_, dt);
        } else {
            step_arm(left_, dt);
            step_arm(right_, dt);
        }
    }

private:
    void step_arm(ArmTeleopBundle* arm, double dt) {
        if (!keep_running) {
            return;
        }
        if (arm == nullptr || arm->quest_receiver == nullptr || arm->state == nullptr) {
            return;
        }
        if (arm->human_mimic == nullptr && arm->cartesian == nullptr &&
            arm->skeleton_proximal == nullptr) {
            return;
        }
        const auto current = collect_joint_states(arm->state);
        auto quest_pose = arm->quest_receiver->latest();
        QuestPoseSample sample;
        if (quest_pose.has_value()) {
            sample = quest_pose.value();
        } else {
            sample.valid = false;
        }

        // skeleton_proximal: q0-q3 follow HybrIK SEW; q4-q6 VR orientation only.
        if (arm->skeleton_proximal != nullptr) {
            SkeletonPoseArm pose_arm;
            bool pose_ok = false;
            double age_ms = 1e9;
            if (arm->skel_pose != nullptr) {
                auto latest = arm->skel_pose->latest();
                age_ms = arm->skel_pose->latest_age_ms();
                if (latest.has_value() && latest->valid) {
                    pose_arm = arm->is_left ? latest->left : latest->right;
                    pose_ok = true;
                }
            }
            arm->skeleton_proximal->set_skeleton_pose(pose_arm, pose_ok, age_ms);

            std::vector<JointState> reference;
            (void)arm->skeleton_proximal->step(current, sample, dt, &reference);
            if (!reference.empty()) {
                arm->state->set_all_references(reference);
            }
            if (ui_ != nullptr) {
                ui_->append_recording(arm, reference);
            }
            maybe_log_teleop_trajectory(arm->label, arm->dynamics, nullptr, sample.grip_active,
                                        current, reference);
            if (++arm->log_tick >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
                arm->log_tick = 0;
                const auto stats = arm->quest_receiver->snapshot_stats();
                std::cout << "[" << arm->label << "-skel-prox] udp=" << stats.packets_window
                          << "Hz grip=" << (sample.grip_active ? 1 : 0)
                          << " clutch=" << (arm->skeleton_proximal->clutch_active() ? 1 : 0)
                          << " pose=" << (arm->skeleton_proximal->pose_valid() ? 1 : 0)
                          << " conf=" << arm->skeleton_proximal->pose_conf() << std::endl;
            }
            return;
        }

        // HUMAN_MIMIC mode: independent whole-arm absolute-IK path.
        if (arm->human_mimic != nullptr) {
            double hint_conf = 0.0;
            double hint_age = 1e9;
            double hint_phi = 0.0;
            const bool hint_ok =
                usable_elbow_hint(arm->skel_hint, arm->is_left, &hint_phi, &hint_conf, &hint_age);
            if (hint_ok) {
                arm->human_mimic->set_elbow_hint(hint_phi, true);
            } else {
                arm->human_mimic->set_elbow_hint(0.0, false);
            }

            std::vector<JointState> reference;
            (void)arm->human_mimic->step(current, sample, dt, &reference);
            if (!reference.empty()) {
                arm->state->set_all_references(reference);
            }
            if (ui_ != nullptr) {
                ui_->append_recording(arm, reference);
            }
            maybe_log_teleop_trajectory(arm->label, arm->dynamics, nullptr, sample.grip_active,
                                        current, reference);
            if (++arm->log_tick >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
                arm->log_tick = 0;
                const auto stats = arm->quest_receiver->snapshot_stats();
                std::cout << "[" << arm->label << "-hm] udp=" << stats.packets_window
                          << "Hz grip=" << (sample.grip_active ? 1 : 0)
                          << " clutch=" << (arm->human_mimic->clutch_active() ? 1 : 0)
                          << " btnClose=" << (arm->human_mimic->last_button_close() ? 1 : 0)
                          << " btnOpen=" << (arm->human_mimic->last_button_open() ? 1 : 0)
                          << " grip_mot=" << arm->human_mimic->gripper_motion_code()
                          << " grip_tgt=" << arm->human_mimic->gripper_target()
                          << " grip_q=" << arm->human_mimic->gripper_measured()
                          << " quest_to=" << (arm->human_mimic->quest_timeout_active() ? 1 : 0);
                if (arm->skel_hint != nullptr) {
                    std::cout << " hint_ok=" << (hint_ok ? 1 : 0) << " conf=" << hint_conf
                              << " age_ms=" << hint_age
                              << " phi_used=" << arm->human_mimic->elbow_hint_phi_used()
                              << " phi_base=" << arm->human_mimic->skeleton_phi_base();
                }
                std::cout << std::endl;
            }
            return;
        }

        if (arm->cartesian == nullptr) {
            return;
        }

        std::vector<JointState> reference;
        (void)arm->cartesian->step(current, sample, dt, &reference);
        if (!reference.empty()) {
            arm->state->set_all_references(reference);
        }
        if (ui_ != nullptr) {
            ui_->append_recording(arm, reference);
        }

        maybe_log_teleop_trajectory(arm->label, arm->dynamics, arm->cartesian.get(),
                                    sample.grip_active, current, reference);

        std::string fatal;
        if (arm->cartesian->consume_fatal_error(&fatal)) {
            std::cerr << "[quest-cartesian][FATAL][" << arm->label << "] " << fatal << std::endl;
            keep_running = false;
            return;
        }

        if (++arm->log_tick >= static_cast<uint64_t>(std::max(1.0, 1.0 / nominal_dt_))) {
            arm->log_tick = 0;
            const auto stats = arm->quest_receiver->snapshot_stats();
            const char* state_name = "idle";
            switch (arm->cartesian->state()) {
                case CartesianControlState::Active:
                    state_name = "active";
                    break;
                case CartesianControlState::Degraded:
                    state_name = "degraded";
                    break;
                case CartesianControlState::Hold:
                    state_name = "hold";
                    break;
                case CartesianControlState::Estop:
                    state_name = "estop";
                    break;
                default:
                    break;
            }
            std::cout << "[" << arm->label << "-quest-cartesian] udp=" << stats.packets_window
                      << "Hz age_ms=" << stats.last_age_ms
                      << " grip=" << (stats.last_grip ? 1 : 0)
                      << " cartesian=" << state_name
                      << " ori=" << (arm->cartesian->orientation_follow_enabled() ? "VR" : "HOLD");
            if (stats.last_grip && !reference.empty()) {
                std::cout << " tcp_off=" << arm->cartesian->tcp_target_offset_norm()
                          << " rot_off=" << arm->cartesian->rotation_target_offset_rad();
            }
            std::cout << std::endl;
        }
    }

    ArmTeleopBundle* left_;
    ArmTeleopBundle* right_;
    QuestUiBridge* ui_ = nullptr;
    double nominal_dt_ = 1.0 / kCartesianDlsHz;
    std::chrono::steady_clock::time_point last_step_time_;
};

class SimRenderThread : public PeriodicTimerThread {
public:
    SimRenderThread(std::shared_ptr<MujocoSimWorld> world, double hz = 60.0,
                    BodyPoseReceiver* body_pose = nullptr, BodySkeletonFilter* body_filter = nullptr,
                    SkeletonRetargetConfig* body_cfg = nullptr)
        : PeriodicTimerThread(hz),
          world_(std::move(world)),
          body_pose_(body_pose),
          body_filter_(body_filter),
          body_cfg_(body_cfg) {}

protected:
    void on_timer() override {
        if (body_pose_ != nullptr && body_filter_ != nullptr && body_cfg_ != nullptr) {
            update_body_pose_mujoco_viz(world_.get(), body_pose_, body_filter_, body_cfg_,
                                        &last_filter_t_);
        }
        world_->forward_and_render();
        if (world_->window_should_close()) {
            keep_running = false;
        }
    }

private:
    std::shared_ptr<MujocoSimWorld> world_;
    BodyPoseReceiver* body_pose_ = nullptr;
    BodySkeletonFilter* body_filter_ = nullptr;
    SkeletonRetargetConfig* body_cfg_ = nullptr;
    double last_filter_t_ = 0.0;
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        Options options;
        if (!parse_args(argc, argv, &options)) {
            return 1;
        }
        TeleopTrajectoryLogger::shared().configure_from_env();
        if (!std::filesystem::exists(options.urdf_path)) {
            std::cerr << "[quest-cartesian][ERROR] URDF not found: " << options.urdf_path
                      << std::endl;
            return 1;
        }
        if (options.sim && !std::filesystem::exists(options.mujoco_xml)) {
            std::cerr << "[quest-cartesian][ERROR] MuJoCo XML not found: " << options.mujoco_xml
                      << std::endl;
            return 1;
        }

        const std::string root_link = "openarm_body_link0";
        const FollowerParams follower_params = load_follower_params();

        std::cout << "=== OpenArm Quest Cartesian Control ===" << std::endl;
        std::cout << "Backend        : " << (options.sim ? "MuJoCo sim" : "CAN hardware")
                  << std::endl;
        std::cout << "Mode           : " << (options.bimanual ? "bimanual (6DOF pose)" : "single-arm")
                  << std::endl;
        std::cout << "Teleop mode    : "
                  << (options.teleop_mode == "human_mimic"
                          ? "HUMAN_MIMIC (virtual-skeleton whole-arm absolute IK)"
                          : options.teleop_mode == "skeleton_proximal"
                                ? "SKELETON_PROXIMAL (q0-q3 SEW, q4-q6 VR ori)"
                                : "CARTESIAN_PRECISION (DLS + nullspace)")
                  << std::endl;
        std::cout << "Control        : DLS velocity IK @ " << kCartesianDlsHz
                  << " Hz; Quest receive @ " << kQuestReceiveHz
                  << " Hz; follower MIT @ " << FREQUENCY << " Hz" << std::endl;
        std::cout << "URDF           : " << options.urdf_path << std::endl;

        auto setup_arm =
            [&](ArmTeleopBundle* bundle, const std::string& label, const std::string& arm_side,
                const std::string& mujoco_side, const std::string& leaf_link,
                const std::string& can_iface, uint16_t quest_port, bool mirror_y,
                const std::shared_ptr<MujocoSimWorld>& world) -> bool {
                bundle->label = label;
                bundle->arm_side = arm_side;
                bundle->mujoco_side = mujoco_side;
                bundle->dynamics = new Dynamics(options.urdf_path, root_link, leaf_link);
                if (!bundle->dynamics->Init()) {
                    std::cerr << "[quest-cartesian][ERROR] failed to initialize dynamics for "
                              << label << std::endl;
                    return false;
                }

                std::shared_ptr<TeleopHardware> hardware;
                size_t arm_dof = 0;
                size_t hand_dof = 0;
                if (options.sim) {
                    bundle->sim_hardware =
                        std::make_shared<MujocoTeleopHardware>(world, mujoco_side);
                    hardware = bundle->sim_hardware;
                    arm_dof = bundle->sim_hardware->arm_motor_count();
                    hand_dof = bundle->sim_hardware->gripper_motor_count();
                } else {
                    bundle->openarm =
                        openarm_init::OpenArmInitializer::initialize_openarm(can_iface, true);
                    arm_dof = bundle->openarm->get_arm().get_motors().size();
                    hand_dof = bundle->openarm->get_gripper().get_motors().size();
                    hardware = std::make_shared<CanTeleopHardware>(bundle->openarm);
                }

                bundle->state = std::make_shared<RobotSystemState>(arm_dof, hand_dof);
                bundle->control =
                    new Control(hardware, bundle->dynamics, bundle->dynamics, bundle->state,
                                1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side, arm_dof, hand_dof);
                configure_follower_control(bundle->control, follower_params, arm_dof + hand_dof,
                                           options.sim);

                if (options.sim) {
                    prime_sim_arm(bundle, arm_dof, hand_dof, options.sim_table_ready);
                } else if (options.auto_home) {
                    std::cout << "  [" << label << "] homing to default pose..." << std::endl;
                    bundle->control->AdjustPosition();
                    prime_hardware_arm(bundle, arm_dof, hand_dof);
                } else {
                    bundle->control->unilateral_step();
                    prime_hardware_arm(bundle, arm_dof, hand_dof);
                }

                bundle->quest_receiver = std::make_unique<QuestPoseReceiver>();
                if (!bundle->quest_receiver->open(quest_port)) {
                    std::cerr << "[quest-cartesian][ERROR] failed to open Quest UDP port "
                              << quest_port << " for " << label << std::endl;
                    return false;
                }
                const bool enable_nullspace = options.use_nullspace;
                bundle->cartesian = std::make_unique<CartesianController>(
                    bundle->dynamics,
                    make_cartesian_config(mirror_y, !options.sim, enable_nullspace,
                                          options.vr_orientation, options.human_skeleton),
                    "[" + label + "] ");
                {
                    const auto hold_ref = collect_joint_states(bundle->state);
                    bundle->cartesian->seed_hold_reference(hold_ref);
                }
                if (options.teleop_mode == "human_mimic") {
                    const bool enable_elbow_hint = (options.elbow_hint == "hybrik");
                    HumanMimicConfig hm_cfg =
                        make_human_mimic_config(options.urdf_path, arm_side,
                                                options.orientation_mapping, options.sim,
                                                enable_elbow_hint);
                    bundle->human_mimic = std::make_unique<HumanMimicController>(
                        bundle->dynamics, hm_cfg, "[" + label + "] ");
                    if (!bundle->human_mimic->Init()) {
                        std::cerr << "[quest-cartesian][ERROR] failed to init HUMAN_MIMIC for "
                                  << label << std::endl;
                        return false;
                    }
                    std::cout << "  [" << label
                              << "] teleop_mode=HUMAN_MIMIC (virtual-skeleton whole-arm IK)"
                              << std::endl;
                } else if (options.teleop_mode == "skeleton_proximal") {
                    SkeletonProximalConfig sp_cfg =
                        make_skeleton_proximal_config(options.urdf_path, arm_side, options.sim);
                    bundle->skeleton_proximal = std::make_unique<SkeletonProximalController>(
                        bundle->dynamics, sp_cfg, "[" + label + "] ");
                    if (!bundle->skeleton_proximal->Init()) {
                        std::cerr << "[quest-cartesian][ERROR] failed to init skeleton_proximal for "
                                  << label << std::endl;
                        return false;
                    }
                    std::cout << "  [" << label
                              << "] teleop_mode=SKELETON_PROXIMAL (q0-q3 SEW, q4-q6 VR ori)"
                              << std::endl;
                }
                std::cout << "  [" << label << "] leaf=" << leaf_link << " udp=" << quest_port;
                if (!options.sim) {
                    std::cout << " can=" << can_iface;
                }
                std::cout << " nullspace="
                          << (options.human_skeleton ? "HUMAN_SKELETON"
                                                     : (enable_nullspace ? "TABLE_READY" : "off"));
                std::cout << " ori="
                          << (options.vr_orientation ? "VR_ORIENTATION" : "HOLD_CLUTCH");
                std::cout << " wrist_split="
                          << (options.vr_orientation ? "on" : "off");
                std::cout << std::endl;
                return true;
            };

        std::shared_ptr<MujocoSimWorld> sim_world;

        if (options.bimanual) {
            if (options.sim) {
                std::cout << "MuJoCo XML     : " << options.mujoco_xml << std::endl;
            } else {
                std::cout << "Follower CAN   : left=" << options.follower_can_left
                          << " right=" << options.follower_can_right << std::endl;
            }
            std::cout << "Quest UDP      : left=" << options.quest_port_left
                      << " right=" << options.quest_port_right << std::endl;

            ArmTeleopBundle left_arm;
            ArmTeleopBundle right_arm;
            if (options.sim) {
                sim_world = std::make_shared<MujocoSimWorld>(options.mujoco_xml,
                                                             "OpenArm Quest Bimanual Sim");
            }
            const std::string left_leaf = select_leaf_link(options.urdf_path, "left_arm");
            const std::string right_leaf = select_leaf_link(options.urdf_path, "right_arm");
            if (!setup_arm(&left_arm, "left", "left_arm", "left", left_leaf,
                           options.follower_can_left, options.quest_port_left, false, sim_world)) {
                return 1;
            }
            if (!setup_arm(&right_arm, "right", "right_arm", "right", right_leaf,
                           options.follower_can_right, options.quest_port_right, false, sim_world)) {
                return 1;
            }
            // Always tag sides (recording / couple / hint all key off is_left).
            // Previously this was only set inside --elbow-hint / skeleton_pose blocks,
            // so plain human_mimic bimanual left CSV stayed empty (header only).
            left_arm.is_left = true;
            right_arm.is_left = false;

            // Optional independent elbow-swivel hint chain (HybrIK). One shared UDP
            // stream (one person) feeds both arms. Off unless --elbow-hint hybrik.
            std::unique_ptr<SkeletonHintReceiver> skel_hint;
            std::unique_ptr<SkeletonHintReceiveThread> skel_hint_thread;
            if (options.elbow_hint == "hybrik") {
                skel_hint = std::make_unique<SkeletonHintReceiver>();
                if (!skel_hint->open(options.elbow_hint_port)) {
                    std::cerr << "[quest-cartesian][ERROR] failed to open elbow-hint UDP port "
                              << options.elbow_hint_port << std::endl;
                    return 1;
                }
                left_arm.skel_hint = skel_hint.get();
                right_arm.skel_hint = skel_hint.get();
                skel_hint_thread =
                    std::make_unique<SkeletonHintReceiveThread>(skel_hint.get(), kQuestReceiveHz);
                std::cout << "Elbow hint     : HybrIK swivel on UDP " << options.elbow_hint_port
                          << std::endl;
            }

            std::unique_ptr<SkeletonPoseReceiver> skel_pose;
            std::unique_ptr<SkeletonPoseReceiveThread> skel_pose_thread;
            if (options.skeleton_pose || options.teleop_mode == "skeleton_proximal") {
                skel_pose = std::make_unique<SkeletonPoseReceiver>();
                if (!skel_pose->open(options.skeleton_pose_port)) {
                    std::cerr << "[quest-cartesian][ERROR] failed to open skeleton-pose UDP port "
                              << options.skeleton_pose_port << std::endl;
                    return 1;
                }
                left_arm.skel_pose = skel_pose.get();
                right_arm.skel_pose = skel_pose.get();
                skel_pose_thread =
                    std::make_unique<SkeletonPoseReceiveThread>(skel_pose.get(), kQuestReceiveHz);
                std::cout << "Skeleton pose  : HybrIK SEW dirs on UDP " << options.skeleton_pose_port
                          << std::endl;
            }

            std::unique_ptr<BodyPoseReceiver> body_pose;
            std::unique_ptr<BodyPoseReceiveThread> body_pose_thread;
            std::unique_ptr<BodySkeletonFilter> body_filter;
            SkeletonRetargetConfig body_cfg;
            double body_pose_last_print_s = 0.0;
            if (options.body_pose) {
                body_pose = std::make_unique<BodyPoseReceiver>();
                if (!body_pose->open(options.body_pose_port)) {
                    std::cerr << "[quest-cartesian][ERROR] failed to open body-pose UDP port "
                              << options.body_pose_port << std::endl;
                    return 1;
                }
                body_filter = std::make_unique<BodySkeletonFilter>(body_cfg);
                body_pose_thread =
                    std::make_unique<BodyPoseReceiveThread>(body_pose.get(), kQuestReceiveHz);
                std::cout << "Body pose      : keypoints on UDP " << options.body_pose_port
                          << " (phase-2 torso map + MuJoCo sticks, no robot drive)" << std::endl;
                if (!is_valid_rotation(body_cfg.R_body_to_robot_left) ||
                    !is_valid_rotation(body_cfg.R_body_to_robot_right)) {
                    std::cerr << "[quest-cartesian][ERROR] R_body_to_robot is not a proper rotation"
                              << std::endl;
                    return 1;
                }
            }

            BimanualFollowerControlThread follower_thread(&left_arm, &right_arm, sim_world,
                                                         FREQUENCY);
            QuestReceiveThread quest_left(left_arm.quest_receiver.get(), "left", kQuestReceiveHz);
            QuestReceiveThread quest_right(right_arm.quest_receiver.get(), "right", kQuestReceiveHz);

            QuestUiBridge quest_ui;
            quest_ui.left = &left_arm;
            quest_ui.right = &right_arm;
            options.quest_ui = env_bool_or("OPENARM_QUEST_UI", options.quest_ui);
            options.quest_jog_step_m =
                env_double_or("OPENARM_QUEST_JOG_STEP", options.quest_jog_step_m);
            options.quest_jog_speed_mps =
                env_double_or("OPENARM_QUEST_JOG_SPEED", options.quest_jog_speed_mps);
            if (const char* rd = std::getenv("OPENARM_QUEST_RECORD_DIR");
                rd != nullptr && *rd != '\0' && options.quest_record_dir.empty()) {
                options.quest_record_dir = rd;
            }
            if (!quest_ui.open(options)) {
                std::cerr << "[quest-cartesian][ERROR] failed to open quest UI UDP ports"
                          << std::endl;
                return 1;
            }

            BimanualCartesianReferenceThread reference_thread(
                &left_arm, &right_arm, options.quest_ui ? &quest_ui : nullptr, kCartesianDlsHz);
            SimRenderThread render_thread(sim_world, 60.0, body_pose.get(), body_filter.get(),
                                          options.body_pose ? &body_cfg : nullptr);

            follower_thread.start_thread();
            if (options.startup_move) {
                std::vector<StartupWaypoint> wps;
                std::string wperr;
                if (!make_startup_waypoints(options, &wps, &wperr)) {
                    std::cerr << "[quest-cartesian][ERROR] " << wperr << std::endl;
                    return 1;
                }
                run_startup_move({&left_arm, &right_arm}, wps, options.startup_move_sec);
            }
            quest_left.start_thread();
            quest_right.start_thread();
            if (skel_hint_thread) {
                skel_hint_thread->start_thread();
            }
            if (skel_pose_thread) {
                skel_pose_thread->start_thread();
            }
            if (body_pose_thread) {
                body_pose_thread->start_thread();
            }
            reference_thread.start_thread();
            if (options.sim) {
                render_thread.start_thread();
            }

            std::cout << "[quest-cartesian] bimanual running." << std::endl;
            print_teleop_help();
            if (!options.sim) {
                print_real_hardware_banner();
            }

            GripperTelemSender grip_telem;
            const uint16_t grip_telem_port = static_cast<uint16_t>(
                std::clamp(env_double_or("OPENARM_QUEST_GRIP_TELEM_PORT", 47945.0), 0.0, 65535.0));
            if (!options.sim && grip_telem_port > 0) {
                grip_telem.open(grip_telem_port);
            }

            while (keep_running) {
                maybe_print_body_pose(body_pose.get(), body_filter.get(), &body_pose_last_print_s);
                if (!options.sim) {
                    grip_telem.maybe_send(left_arm.openarm, right_arm.openarm, 0.05);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            reference_thread.stop_thread();
            if (body_pose_thread) {
                body_pose_thread->stop_thread();
            }
            if (skel_pose_thread) {
                skel_pose_thread->stop_thread();
            }
            if (skel_hint_thread) {
                skel_hint_thread->stop_thread();
            }
            quest_left.stop_thread();
            quest_right.stop_thread();
            follower_thread.stop_thread();
            if (options.sim) {
                render_thread.stop_thread();
            }
            TeleopTrajectoryLogger::shared().close();
            if (!options.sim) {
                send_kd_only_release(left_arm.openarm, right_arm.openarm, follower_params.kd);
                left_arm.control->Shutdown();
                right_arm.control->Shutdown();
            }
            delete left_arm.control;
            delete right_arm.control;
            delete left_arm.dynamics;
            delete right_arm.dynamics;
            delete left_arm.openarm;
            delete right_arm.openarm;
            return 0;
        }

        const std::string leaf_link = select_leaf_link(options.urdf_path, options.arm_side);
        const std::string mujoco_side = (options.arm_side == "left_arm") ? "left" : "right";
        const std::string label = (options.arm_side == "left_arm") ? "left" : "right";
        std::cout << "Arm side       : " << options.arm_side << std::endl;
        if (!options.sim) {
            std::cout << "Follower CAN   : " << options.follower_can << std::endl;
        } else {
            std::cout << "MuJoCo XML     : " << options.mujoco_xml << std::endl;
            std::cout << "MuJoCo side    : " << mujoco_side << std::endl;
        }
        std::cout << "Quest UDP port : " << options.quest_port << std::endl;
        std::cout << "Leaf link      : " << leaf_link << std::endl;

        ArmTeleopBundle arm;
        if (options.sim) {
            sim_world = std::make_shared<MujocoSimWorld>(options.mujoco_xml,
                                                         "OpenArm Quest Cartesian Sim");
        }
        if (!setup_arm(&arm, label, options.arm_side, mujoco_side, leaf_link, options.follower_can,
                       options.quest_port, false, sim_world)) {
            return 1;
        }

        std::cout << "Teleop mode    : "
                  << (options.teleop_mode == "human_mimic"
                          ? "HUMAN_MIMIC (virtual-skeleton whole-arm absolute IK)"
                          : options.teleop_mode == "skeleton_proximal"
                                ? "SKELETON_PROXIMAL (q0-q3 SEW, q4-q6 VR ori)"
                                : "CARTESIAN_PRECISION (DLS + nullspace)")
                  << std::endl;

        std::unique_ptr<SkeletonHintReceiver> skel_hint;
        std::unique_ptr<SkeletonHintReceiveThread> skel_hint_thread;
        const bool arm_is_left = (options.arm_side == "left_arm");
        if (options.elbow_hint == "hybrik") {
            skel_hint = std::make_unique<SkeletonHintReceiver>();
            if (!skel_hint->open(options.elbow_hint_port)) {
                std::cerr << "[quest-cartesian][ERROR] failed to open elbow-hint UDP port "
                          << options.elbow_hint_port << std::endl;
                return 1;
            }
            arm.skel_hint = skel_hint.get();
            arm.is_left = arm_is_left;
            skel_hint_thread =
                std::make_unique<SkeletonHintReceiveThread>(skel_hint.get(), kQuestReceiveHz);
            std::cout << "Elbow hint     : HybrIK swivel on UDP " << options.elbow_hint_port
                      << std::endl;
        }

        std::unique_ptr<SkeletonPoseReceiver> skel_pose;
        std::unique_ptr<SkeletonPoseReceiveThread> skel_pose_thread;
        if (options.skeleton_pose || options.teleop_mode == "skeleton_proximal") {
            skel_pose = std::make_unique<SkeletonPoseReceiver>();
            if (!skel_pose->open(options.skeleton_pose_port)) {
                std::cerr << "[quest-cartesian][ERROR] failed to open skeleton-pose UDP port "
                          << options.skeleton_pose_port << std::endl;
                return 1;
            }
            arm.skel_pose = skel_pose.get();
            arm.is_left = arm_is_left;
            skel_pose_thread =
                std::make_unique<SkeletonPoseReceiveThread>(skel_pose.get(), kQuestReceiveHz);
            std::cout << "Skeleton pose  : HybrIK SEW dirs on UDP " << options.skeleton_pose_port
                      << std::endl;
        }

        std::unique_ptr<BodyPoseReceiver> body_pose;
        std::unique_ptr<BodyPoseReceiveThread> body_pose_thread;
        std::unique_ptr<BodySkeletonFilter> body_filter;
        SkeletonRetargetConfig body_cfg;
        double body_pose_last_print_s = 0.0;
        if (options.body_pose) {
            body_pose = std::make_unique<BodyPoseReceiver>();
            if (!body_pose->open(options.body_pose_port)) {
                std::cerr << "[quest-cartesian][ERROR] failed to open body-pose UDP port "
                          << options.body_pose_port << std::endl;
                return 1;
            }
            body_filter = std::make_unique<BodySkeletonFilter>(body_cfg);
            body_pose_thread =
                std::make_unique<BodyPoseReceiveThread>(body_pose.get(), kQuestReceiveHz);
            std::cout << "Body pose      : keypoints on UDP " << options.body_pose_port
                      << " (phase-2 torso map + MuJoCo sticks, no robot drive)" << std::endl;
            if (!is_valid_rotation(body_cfg.R_body_to_robot_left) ||
                !is_valid_rotation(body_cfg.R_body_to_robot_right)) {
                std::cerr << "[quest-cartesian][ERROR] R_body_to_robot is not a proper rotation"
                          << std::endl;
                return 1;
            }
        }

        FollowerControlThread follower_thread(arm.state, arm.control, arm.sim_hardware, FREQUENCY);
        QuestReceiveThread quest_thread(arm.quest_receiver.get(), label, kQuestReceiveHz);
        CartesianReferenceThread reference_thread(
            arm.state, arm.cartesian.get(), arm.quest_receiver.get(), arm.dynamics, label,
            kCartesianDlsHz, arm.human_mimic.get(), arm.skel_hint, arm_is_left,
            arm.skeleton_proximal.get(), arm.skel_pose);
        SimRenderThread render_thread(sim_world, 60.0, body_pose.get(), body_filter.get(),
                                      options.body_pose ? &body_cfg : nullptr);

        follower_thread.start_thread();
        if (options.startup_move) {
            std::vector<StartupWaypoint> wps;
            std::string wperr;
            if (!make_startup_waypoints(options, &wps, &wperr)) {
                std::cerr << "[quest-cartesian][ERROR] " << wperr << std::endl;
                return 1;
            }
            run_startup_move({&arm}, wps, options.startup_move_sec);
        }
        quest_thread.start_thread();
        if (skel_hint_thread) {
            skel_hint_thread->start_thread();
        }
        if (skel_pose_thread) {
            skel_pose_thread->start_thread();
        }
        if (body_pose_thread) {
            body_pose_thread->start_thread();
        }
        reference_thread.start_thread();
        if (options.sim) {
            render_thread.start_thread();
        }

        std::cout << "[quest-cartesian] running." << std::endl;
        print_teleop_help();
        if (!options.sim) {
            print_real_hardware_banner();
        }

        while (keep_running) {
            maybe_print_body_pose(body_pose.get(), body_filter.get(), &body_pose_last_print_s);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        reference_thread.stop_thread();
        if (body_pose_thread) {
            body_pose_thread->stop_thread();
        }
        if (skel_pose_thread) {
            skel_pose_thread->stop_thread();
        }
        if (skel_hint_thread) {
            skel_hint_thread->stop_thread();
        }
        quest_thread.stop_thread();
        follower_thread.stop_thread();
        if (options.sim) {
            render_thread.stop_thread();
        }
        TeleopTrajectoryLogger::shared().close();
        if (!options.sim) {
            send_kd_only_release(arm.openarm, nullptr, follower_params.kd);
            arm.control->Shutdown();
        }
        delete arm.control;
        delete arm.dynamics;
        delete arm.openarm;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[quest-cartesian][ERROR] " << e.what() << std::endl;
        return 1;
    }
}
