// HUMAN_MIMIC teleoperation mode — shared configuration and result types.
//
// This mode is fully independent of the DLS + nullspace CartesianController path.
// It generates a full-arm target configuration (shoulder-elbow-wrist-hand) from
// VR input via a virtual human skeleton, then solves a position-level whole-arm
// inverse kinematics for a smooth, returnable, anatomically-natural posture.
//
// The stable pipeline is intentionally minimal and MUST stay:
//   VR target -> VirtualArmSkeleton -> HumanMimicIkSolver -> single q_target
//              -> JointReferenceGenerator
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

// How the VR hand orientation is mapped onto the robot hand orientation target.
enum class OrientationMappingMode {
    INCREMENTAL_CLUTCH = 0,   // orientation anchor re-set on every grip (legacy feel)
    ABSOLUTE_CALIBRATED = 1,  // fixed calibration; same hand pose -> same robot pose
};

// Gesture that re-establishes the ABSOLUTE_CALIBRATED orientation calibration.
enum class OrientationRecenterButton {
    NONE = 0,
    BOTH_FACE_BUTTONS = 1,  // press A+B (right) / X+Y (left) together while gripping
};

enum class HumanMimicIkBackend {
    LM = 0,        // position-level damped Gauss-Newton (legacy)
    DIFF_QP = 1,   // velocity-level weighted least squares + joint-limit box projection
};

// Static, per-run configuration for the HUMAN_MIMIC pipeline (single arm).
struct HumanMimicConfig {
    // --- Robot model / chain ---
    std::string urdf_path;
    std::string root_link = "openarm_body_link0";
    // Frames along the arm used by the virtual skeleton (filled from arm side).
    std::string shoulder_link;  // e.g. openarm_right_link1 (fixed base point ~ shoulder)
    std::string elbow_link;     // e.g. openarm_right_link4 (elbow flexion)
    std::string wrist_link;     // e.g. openarm_right_link7 (wrist center + hand orientation)
    // Fixed tool offset from wrist_link origin to the TCP, expressed in wrist_link frame.
    Eigen::Vector3d wrist_to_tcp = Eigen::Vector3d(0.0, 0.0, 0.186);

    // --- VR -> robot mapping (mirrors CartesianController conventions) ---
    double pos_scale = 1.0;   // VR translation -> robot translation gain
    double rot_scale = 1.0;   // VR rotation magnitude scaling (1.0 = 1:1 twist/roll)
    // VR world "up" axis (gravity). Quest/OpenXR world is +Y up. Orientation mapping
    // locks this axis to robot-up so a horizontal controller yaw stays a horizontal
    // gripper yaw regardless of how the controller is tilted in the hand.
    Eigen::Vector3d vr_world_up = Eigen::Vector3d(0.0, 1.0, 0.0);
    bool mirror_translation_y = false;
    double vr_tcp_roll_sign = 1.0;
    double vr_tcp_pitch_sign = -1.0;
    double vr_tcp_yaw_sign = 1.0;

    // --- Orientation input mapping ---
    OrientationMappingMode orientation_mapping_mode = OrientationMappingMode::INCREMENTAL_CLUTCH;
    OrientationRecenterButton orientation_recenter_button =
        OrientationRecenterButton::BOTH_FACE_BUTTONS;
    // Light SLERP filter on the target orientation: alpha = 1 - exp(-dt/tau).
    double tau_orientation = 0.03;          // s (0.02-0.05 recommended)
    double orientation_deadband_deg = 0.5;  // ignore sub-deadband target motion

    // Position delta frame for VR translation.
    // WORLD: Quest world lift (+Y) maps to robot +Z even when the controller is
    // tilted (facing grippers). BODY used to make world-vertical become diagonal.
    bool position_map_world_frame = true;

    // Fine-motion hold: suppress IK micro-chasing when the hand is essentially
    // still (jitter that is not a VR signal spike, but residual IK polishing).
    bool fine_motion_hold_enabled = true;
    double position_deadband_m = 0.002;      // quiet tip motion threshold (robot frame)
    double q_update_deadband_rad = 0.004;   // skip tinier per-joint IK corrections when quiet
    double wrist_hold_err_m = 0.006;        // if wrist residual already small + quiet → freeze q

    // Panel TCP jog: chase tip without VR fine-hold stick-slip, and temporarily
    // drop elbow/tip-yield so a side nudge cannot suddenly lift the arm.
    int jog_bypass_hold_ticks = 40;   // ~40 ms at 1 kHz after each nudge
    int jog_chase_iters = 8;          // IK loops per button press
    double jog_chase_dt = 0.02;       // dt used inside the chase loop (slew budget)
    double jog_wrist_tol_m = 0.003;   // stop chase early when this close
    double jog_elbow_scale = 0.1;     // multiply elbow IK weight while jogging
    double jog_q_slew_radps = 6.0;    // looser joint slew during chase

    // --- Workspace guard ---
    // Cap TCP offset from the clutch anchor. Kept below the arm's full reach so the
    // wrist target stays inside the workspace (avoids reach=0 / full-extension jams).
    // Axes are SEPARATE: vertical must not share a spherical budget with lateral —
    // otherwise "grippers facing each other then lift" hits the lateral cap and
    // vertical motion is crushed (arm feels locked / sinks).
    //   backward (delta . forward_axis < 0) : max_teleop_offset_m
    //   forward                            : max_teleop_offset_forward_m
    //   lateral (in horizontal plane ⊥ forward) : max_teleop_offset_forward_m
    //   up/down (world +Z / −Z)             : max_teleop_offset_up_m / _down_m
    double max_teleop_offset_m = 0.45;          // backward-only envelope (original feel)
    double max_teleop_offset_forward_m = 0.45;  // forward + lateral allowance
    double max_teleop_offset_up_m = 0.55;       // raise room (bimanual facing / lift)
    double max_teleop_offset_down_m = 0.40;     // separate sink room
    // Robot base-frame "forward" (away from the torso). +X for OpenArm; sign-tunable.
    Eigen::Vector3d forward_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d up_axis = Eigen::Vector3d(0.0, 0.0, 1.0);

    // --- Virtual arm skeleton (elbow placement) ---
    // Upper-arm (L1) and forearm (L2) lengths. If <= 0, measured from FK at clutch.
    double L1 = -1.0;
    double L2 = -1.0;
    // Keep the wrist target off full extension so the elbow stays bent and the wrist
    // joints keep range to track orientation (larger = safer, less reach).
    double reach_margin_m = 0.06;

    // --- Reach normalization (progress-based soft ceiling) ---
    // Human arms are longer than the robot. A fixed asymptote short of L1+L2 never
    // lets the arm straighten and "bounces" when the operator tries. Instead, the
    // allowed wrist distance ceiling rises with DOWNWARD teleop progress from the
    // clutch anchor (0 = still high / bent, 1 = at workspace bottom / near-straight):
    //   progress = smoothstep( clamp(−up_off / max_teleop_offset_down) )
    //   margin   = lerp(bent_margin, soft_margin, progress)
    //   d_star   = L1+L2 − margin
    //   d_cmd    = min(d_req, d_star)   // fully reversible when raising
    bool reach_soft_enabled = true;
    // When true (default): margin follows downward progress. When false: fixed
    // margin = reach_soft_margin_m (soft ceiling still on, but no down-progress gate).
    bool reach_soft_progress_enabled = true;
    // Margin at progress=1 (workspace bottom): near-straight but off exact singularity.
    double reach_soft_margin_m = 0.002;
    // Margin at progress=0 (not yet down): keep a visible bend for fine motion.
    double reach_soft_bent_margin_m = 0.045;
    double reach_soft_zone_m = 0.12;  // still used for tip_yield scaling of (d_req−d_cmd)
    // Blend weights for the elbow bend direction (normalized internally):
    // previous (continuity), outward (away from torso), downward (gravity droop).
    //
    // Only TRUE OUTWARD side-reach fades outward / boosts droop. Medial/inward
    // (grippers facing each other) used to be treated the same via |n·out|, which
    // pinned elbows down and fought raising the arms. Elevation (S→W with +Z)
    // also fades droop so a rising wrist can keep a raised elbow/shoulder.
    double w_prev = 0.6;
    double w_outward = 0.3;
    double w_downward = 0.1;
    double lateral_droop_boost = 0.8;  // added to w_downward only for outward side-reach
    double elevation_droop_fade = 1.0;  // 1 = fully kill droop when S→W is straight up
    // "outward" unit direction in robot base frame (right arm bends to robot -Y).
    Eigen::Vector3d outward_dir = Eigen::Vector3d(0.0, -1.0, 0.0);
    Eigen::Vector3d downward_dir = Eigen::Vector3d(0.0, 0.0, -1.0);

    // --- Whole-arm IK (LM or diff-QP) ---
    // Task layout: wrist XYZ (3) + hand ori (3) + elbow XYZ (3) + continuity + nominal.
    double w_wrist_pos = 12.0;   // wrist-center position residual
    double w_hand_ori = 8.0;     // hand orientation residual (higher = crisper wrist)
    double w_elbow = 2.0;        // elbow XYZ position residual (env OPENARM_HM_W_ELBOW)
    double w_continuity = 1.5;   // stay near previous solution
    double w_nominal = 0.1;      // weak pull toward nominal posture
    int ik_max_iters = 12;
    double ik_lambda = 1e-2;     // LM / diff-QP damping
    // IK backend: LM (position Gauss-Newton) or DIFF_QP (velocity-level + box constraints).
    HumanMimicIkBackend ik_backend = HumanMimicIkBackend::DIFF_QP;
    double ik_kp_pos = 1.0;       // diff-QP: wrist/elbow error gain per iteration
    double ik_kp_ori = 1.0;
    double ik_joint_limit_margin_frac = 0.08;  // soft URDF margin for diff-QP dq scaling
    // Per-iteration trust: proximal (q0–q3) tighter than distal so shoulder/elbow
    // cannot thrash to chase unreachable wrist orientation.
    double ik_proximal_max_step = 0.12;
    double ik_distal_max_step = 0.30;

    // Optional per-joint IK position-limit overrides (rad). Same size as arm dof; a
    // NaN entry keeps the URDF limit for that joint. Use to TEST (in sim) whether a
    // conservative URDF limit is what blocks a motion (e.g. shoulder joint2 only allows
    // ~10 deg on one side, which prevents straightening the arm forward). Do NOT widen
    // beyond the real mechanical range on hardware.
    std::vector<double> q_min_override;  // empty => no overrides
    std::vector<double> q_max_override;

    // --- Reach-forward assist (ONLY near the robot's own full extension) ---
    // Stretch is measured from the soft-normalized commanded wrist distance, NOT the
    // raw human d_req. Human arms are longer than the robot, so d_req/(L1+L2) is ~1 for
    // most teleop — using that used to permanently zero elbow/continuity and disable
    // HybrIK, which free'd the 7DoF redundancy and caused mid-motion IK branch hops
    // ("换逆解") and left/right desync on elbow-driven gestures (e.g. wine-shake).
    // Near straight: FREEZE swivel (phi) and DROP the elbow-phi task so the
    // forearm/wrist follow TCP orientation — do NOT soften ori (that lets the arm
    // "spin around" the S→W axis at the singularity).
    bool extension_yield_enabled = true;
    double extension_yield_start = 0.92;      // begin yield this fraction of commanded d_star
    double extension_elbow_scale_min = 0.05;  // kill elbow-phi task near straight (anti-spin)
    double extension_ori_scale_min = 1.25;    // BOOST ori — tip attitude owns distal DOF
    double extension_cont_scale_min = 0.85;   // keep continuity so free DOF cannot wander
    double extension_nom_scale_min = 0.0;      // drop TABLE_READY nominal (locks elbow ~90°)
    // Legacy flag (XYZ pin). With 1D SEW, near-extension uses phi freeze + weight
    // kill instead; kept for env compatibility only.
    bool extension_straighten_elbow = true;
    // Freeze commanded phi above this stretch (circle radius is tiny / ill-defined).
    double extension_phi_freeze_stretch = 0.55;
    // Tighten upper-arm roll (j2) slew near extension — that joint drives the "spin".
    double extension_roll_slew_scale = 0.25;

    // Rate-limit q_target after IK so a discrete branch switch cannot jump the
    // command in one control tick (hardware-unsafe, feels like "换逆解").
    bool q_target_slew_enabled = true;
    double q_target_slew_max_radps = 2.5;  // per-joint max |dq|/dt
    double ik_tol = 1e-5;        // convergence on |delta_q|
    double ik_max_step = 0.3;    // per-iteration per-joint clamp (rad)
    std::vector<double> q_nominal;  // size = arm dof; empty => use clutch posture

    // Freeze-and-hold after this many consecutive whole-arm solve failures.
    int solve_fail_freeze_count = 30;

    // --- Joint reference generator (1 kHz interpolation to q_target) ---
    // Higher gain / speed caps => the arm tracks the hand faster (less lag).
    double k_joint = 10.0;     // proportional gain (q_target - q_ref)
    double qdot_max = 3.5;     // rad/s per joint
    double qddot_max = 40.0;   // rad/s^2 per joint

    // --- Re-anchor on grip: fight gravity droop ---
    // On re-grip, seed TCP / JRG / q_target from the last COMMANDED pose (capped vs
    // measured) so gravity droop between grips does not accumulate. A soft "start
    // from measured" path feels jump-free but leaves the arm continuously sinking.
    // Note: an IK branch hop on reinit (肘对不齐 → 换逆解) is a separate hard problem;
    // do not "fix" it by abandoning anti-droop.
    bool reanchor_to_commanded = true;
    double max_reanchor_correction_rad = 0.45;
    // Brief pause before IK so the first post-grip solve does not flash; keep small.
    int clutch_settle_ticks = 2;

    // Freeze measured L1/L2 after the first successful clutch so soft-reach scale
    // does not wander across re-grips over a long session.
    bool lock_segment_lengths = true;

    // Auto-calibrate wrist_to_tcp from FK at the first clutch (GetEE vs link7).
    bool auto_calibrate_wrist_tcp = true;

    // --- Tip-priority yield (wrist vs elbow-phi preference) ---
    // May lower elbow-phi preference and nominal ONLY. Must NOT lower q continuity,
    // phi continuity, or branch stability (those caused proximal thrash).
    bool tip_priority_enabled = true;
    double tip_priority_wrist_err_m = 0.015;     // start yielding above this residual
    double tip_priority_elbow_scale_min = 0.15;  // floor for elbow-phi weight scale
    double tip_priority_cont_scale_min = 1.0;    // unused (kept for env compat; no-op)
    double tip_priority_shoulder_slew_boost = 1.5;  // extra slew on j0/j1 while yielding

    // --- Gripper (A/X close, B/Y open; stroke / position ramp) ---
    double gripper_open = 0.0;
    double gripper_closed = 0.04;
    double gripper_speed = 0.02;        // open ramp (position units / s)
    double gripper_close_speed = 0.01;  // close ramp; usually slower than open
    // Soft-stall: while closing, if command advances ahead of measured stroke and
    // velocity dies (contact / jammed), freeze stroke advance but KEEP the close-side
    // command (do not snap back to q_meas — that releases grasp preload).
    bool gripper_soft_stall = true;
    double gripper_stall_error = 0.04;      // |q_cmd − q_meas| along close (rad or m)
    double gripper_stall_velocity = 0.02;   // |qdot| below this counts as stalled
    double gripper_stall_time = 0.12;       // seconds of continuous stall before freeze
    // Extra close bias applied when freezing a grasp (0 = keep command lag only).
    double gripper_hold_preload = 0.0;

    // --- Trigger = uniform "return to start" ---
    // Holding the trigger uniformly drives the arm back to the start posture
    // (q_nominal / TABLE_READY). On release the clutch re-anchors at the current
    // pose so control continues without a jump.
    double trigger_threshold = 0.5;
    double reset_speed = 1.0;      // rad/s, uniform ramp toward the start pose
    int reset_first_joint = 0;     // reset joints [reset_first_joint, dof); 0 = whole arm

    // --- External elbow-swivel hint (optional HybrIK skeleton chain) ---
    // A separate, independent process (scripts/skeleton_swivel_bridge.py) can stream
    // the human's measured elbow swivel angle phi. When a FRESH, CONFIDENT hint is
    // available it biases the virtual skeleton's elbow toward the human posture;
    // otherwise the built-in skeleton is used unchanged. This never post-edits q and
    // never changes the wrist/hand targets — it only nudges the redundant elbow DOF.
    bool elbow_hint_enabled = false;  // set true only when a hint source is wired in
    // Blend: phi_used = (1-w)*phi_skeleton + w*phi_hint_mapped. Keep w modest: HybrIK
    // ϕ is often ambiguous (wave / side-view), and a high weight yanks the arm into
    // pretzel poses. The virtual skeleton remains the primary redundancy lock.
    double elbow_hint_weight = 0.35;
    // Clip |phi_hint − phi_skeleton| before blending. Outliers beyond this (often
    // ±1.5–2 rad in the logs) are camera-axis ambiguities, not real elbow motion.
    double elbow_hint_max_delta_rad = 0.7;
    // Map human phi -> robot phi: robot_phi = sign*human_phi + offset. The canonical
    // basis matches between bridge and robot (e1 ~ gravity-down projection), but the
    // human/robot handedness and torso facing may need a per-arm sign / offset.
    double elbow_hint_sign = 1.0;
    double elbow_hint_offset_rad = 0.0;
    // Slew the applied phi so hint jitter / dropouts can never jump the elbow.
    double elbow_hint_slew_radps = 1.2;

    // --- Elbow-swivel semantic disambiguation (primary anti-ambiguity) ---
    // Camera / HybrIK depth often flips the elbow across the S→W axis, which
    // reports as a sudden phi jump (≈ +π antipode) or an "acute" folded look
    // while the operator is still reaching forward (obtuse / near-straight).
    // Resolve by scoring the observed phi AND its antipode with continuity +
    // anatomical prior + reach context, and commit to the winning branch —
    // do not slowly chase the wrong one.
    bool elbow_phi_disambiguate = true;
    // When shoulder→wrist stretch is above this, or the arm is extending, reject
    // antipodal flips (|Δphi| > π/2 from the last command) as vision ambiguity.
    double elbow_phi_reach_lock_stretch = 0.25;
    // Elbow interior angle (from law of cosines on commanded d) above this is
    // treated as "open / obtuse": prefer continuity over flipped observations.
    double elbow_phi_obtuse_flex_rad = 1.8;  // ~103°
    // Circle radius below which bend direction is numerically unstable; freeze
    // continuity so exit-from-straight does not inherit a flipped branch.
    double elbow_singularity_h_m = 0.025;

    // --- Commanded elbow swivel (phi) rate limit — secondary safety net ---
    bool elbow_cmd_slew_enabled = true;
    double elbow_cmd_slew_radps = 1.5;  // ~86 deg/s hard ceiling on commanded phi
    double elbow_cmd_max_step_rad = 0.12;  // ~7 deg absolute max per solve

    // Tighter post-IK slew for the elbow flexion joint (index 3 / joint4) so a
    // residual IK hop cannot bang that joint even if phi is already limited.
    double elbow_joint_slew_radps = 1.8;

    // --- Safety (Quest freshness) ---
    double quest_stale_sec = 0.05;
    double quest_timeout_sec = 0.15;

    // --- Debug ---
    bool debug_log = true;
    double debug_log_hz = 5.0;
};

// Output of the virtual arm skeleton for one control step.
struct SkeletonResult {
    Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
    Eigen::Vector3d elbow_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d bend_dir = Eigen::Vector3d::Zero();  // unit, perpendicular to S->W
    bool reachable = true;                               // false if target was clamped in reach
    Eigen::Vector3d wrist_clamped = Eigen::Vector3d::Zero();  // wrist target after reach clamp
    double phi_base = 0.0;   // swivel angle of the base elbow (rad, canonical basis)
    double h_radius = 0.0;   // radius of the elbow swivel circle (m); ~0 near straight
};

// A candidate elbow placement produced by swiveling around the shoulder->wrist axis.
struct ElbowTarget {
    Eigen::Vector3d E = Eigen::Vector3d::Zero();
    Eigen::Vector3d bend_dir = Eigen::Vector3d::Zero();
    double h = 0.0;    // swivel circle radius (m)
    double phi = 0.0;  // swivel angle (rad)
    bool reachable = false;
    bool degenerate = false;  // near-straight arm; swivel ill-defined
};

// Output of the whole-arm IK solve.
struct HumanMimicIkResult {
    std::vector<double> q;       // arm joint solution
    bool ok = false;            // converged / finite
    bool accepted = true;       // false on kinematic / numeric failure
    double wrist_pos_err = 0.0;
    double hand_ori_err = 0.0;
    double elbow_pos_err = 0.0;  // elbow XYZ task residual
    double elbow_phi_err = 0.0;  // unused in XYZ elbow mode (kept for ABI / logs)
    int iters = 0;
};
