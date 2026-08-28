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

// How the elbow redundancy is constrained in the whole-arm IK.
enum class ElbowTaskMode {
    XYZ = 0,  // 3D elbow position residual (default — more stable / over-constrained)
    PHI = 1,  // 1D swivel angle on the SEW circle (OPENARM_HM_ELBOW_TASK=phi)
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
    // Position task frame for whole-arm IK. TCP is the physical hand_tcp point;
    // false preserves the legacy link7 wrist-origin task for A/B rollback.
    bool ik_track_tcp = true;

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
    // Keep short — large tau feels laggy / "filtered dump" on re-grip.
    double tau_orientation = 0.006;         // s (was 0.012; less paste on micro twists)
    double orientation_deadband_deg = 0.05; // was 0.2 — tiny hand twists were ignored

    // Position delta frame for VR translation.
    // WORLD: Quest world lift (+Y) maps to robot +Z even when the controller is
    // tilted (facing grippers). BODY used to make world-vertical become diagonal.
    bool position_map_world_frame = true;

    // Fine-motion hold freezes q when tip looks "quiet" — feels sticky on nudges.
    // Default off; slew + IK damping already limit jitter.
    bool fine_motion_hold_enabled = false;
    double position_deadband_m = 0.0006;     // tip_quiet / idle gate (was 0.0015)
    double q_update_deadband_rad = 0.0;     // was 0.002 — pass micro IK corrections
    double wrist_hold_err_m = 0.006;        // only if fine_motion_hold re-enabled

    // Panel TCP jog: chase tip without VR fine-hold stick-slip, and temporarily
    // drop elbow/tip-yield so a side nudge cannot suddenly lift the arm.
    int jog_bypass_hold_ticks = 40;   // ~40 ms at 1 kHz after each nudge
    int jog_chase_iters = 8;          // IK loops per button press
    double jog_chase_dt = 0.02;       // dt used inside the chase loop (slew budget)
    double jog_wrist_tol_m = 0.003;   // stop chase early when this close
    double jog_elbow_scale = 0.1;     // multiply elbow IK weight while jogging
    double jog_q_slew_radps = 10.0;   // looser joint slew during chase / tip drive
    // Continuous panel jog should preserve its requested Cartesian line rather
    // than being radially compressed by the shoulder->wrist soft-reach ceiling.
    bool panel_bypass_soft_reach = true;
    // Orientation scale during TCP panel chase. A firm attitude prevents the
    // 186 mm tool lever arm from turning small wrist drift into lateral TCP drift.
    double panel_tcp_ori_scale = 0.85;

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
    double max_teleop_offset_down_m = 0.80;     // sink room (平举→自然下垂)
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
    // allowed wrist distance ceiling rises with teleop "reach intent" from the
    // clutch anchor (0 = bent / fine motion, 1 = near-straight):
    //   progress = max(down_progress, forward_progress)   // each smoothstep+slew
    //   margin   = lerp(bent_margin, soft_margin, progress)
    //   d_star   = L1+L2 − margin
    //   d_cmd    = min(d_req, d_star)   // fully reversible when retracting
    bool reach_soft_enabled = true;
    // When true (default): margin follows reach progress. When false: fixed
    // margin = reach_soft_margin_m (soft ceiling still on, but no progress gate).
    bool reach_soft_progress_enabled = true;
    // Include forward (+forward_axis) offset in reach progress (in addition to down).
    bool reach_soft_forward_progress_enabled = true;
    // Forward offset (m) that maps to progress=1. Shorter than the workspace forward
    // cap so the arm can straighten mid-reach without needing the full 0.85 m shove.
    double reach_soft_forward_span_m = 0.35;
    // Margin at progress=1 (full reach intent): near-straight but off exact singularity.
    double reach_soft_margin_m = 0.002;
    // Margin at progress=0 (not yet reaching out/down): keep a visible bend for fine motion.
    double reach_soft_bent_margin_m = 0.045;
    double reach_soft_zone_m = 0.12;  // still used for tip_yield scaling of (d_req−d_cmd)
    // Soften elbow IK weight as reach progress rises (hybrid with skeleton still on).
    // progress=0 → full w_elbow; progress=1 → scale down to this floor (posture still held).
    bool reach_intent_elbow_yield_enabled = true;
    double reach_intent_elbow_scale_min = 0.25;
    // Blend weights for the elbow bend direction (normalized internally):
    // previous (continuity), outward (away from torso), upward (optional),
    // downward (gravity droop — default preference for a natural hanging elbow).
    //
    // Only TRUE OUTWARD side-reach fades outward. Elevation fades droop so a
    // rising wrist can still lift the elbow when needed. Default favors droop
    // over flare (外展/抬肘 was too strong and fought later teleop).
    double w_prev = 0.45;       // was 0.6 — strong self-lock when seeded from cmd bend
    double w_outward = 0.25;
    double w_upward = 0.0;      // off: natural droop, not raised-elbow bias
    double w_downward = 0.20;   // primary posture prior (gravity)
    // When S→W points medial (toward torso), boost outward so the circle pick does
    // not settle on a tucked/inward elbow that then jams the wrist joints.
    double anti_inward_boost = 1.0;
    double lateral_droop_boost = 0.0;
    double elevation_droop_fade = 1.0;  // 1 = fully kill droop when S→W is straight up
    // Fade gravity droop on medial reaches (1 = fully kill droop when fully medial).
    double medial_droop_fade = 0.85;
    // How much continuity fades when wrist is fully outward (0=keep prev, 1=drop prev).
    double lateral_continuity_fade = 0.25;
    // Also fade continuity when medial so a tucked branch can be escaped.
    double medial_continuity_fade = 0.55;
    // "outward" unit direction in robot base frame.
    // Bimanual: make_human_mimic_config sets left +Y / right −Y.
    Eigen::Vector3d outward_dir = Eigen::Vector3d(0.0, -1.0, 0.0);
    Eigen::Vector3d downward_dir = Eigen::Vector3d(0.0, 0.0, -1.0);
    Eigen::Vector3d upward_dir = Eigen::Vector3d(0.0, 0.0, 1.0);

    // --- Whole-arm IK (LM or diff-QP) ---
    // Task layout:
    //   PHI (default): wrist + ori + elbow swivel φ (1) — less over-constrained vs XYZ
    //   XYZ (legacy):  wrist XYZ (3) + hand ori (3) + elbow XYZ (3) + cont + nom
    ElbowTaskMode elbow_task_mode = ElbowTaskMode::PHI;
    // When false (pika_style), skip VirtualArmSkeleton::Solve and force elbow IK
    // weight to 0 — pure 6D wrist+ori + continuity/nominal. Default true keeps HM.
    bool use_elbow_skeleton = true;
    double w_wrist_pos = 12.0;   // wrist-center position residual
    double w_hand_ori = 8.0;     // hand orientation residual (higher = crisper wrist)
    // Soft preference only — large w_elbow + hard φ lock jams forearm (j4–j6).
    double w_elbow = 0.8;        // PHI: swivel residual; XYZ: elbow position residual
    double w_continuity = 1.0;   // was 1.5 — less glue on micro tip moves
    double w_nominal = 0.1;      // weak pull toward nominal posture

    // Hierarchical elbow: primary = wrist 6D (+ light cont); elbow φ in 1D nullspace.
    bool elbow_nullspace_enabled = true;
    double elbow_nullspace_gain = 0.45;      // was 1.0 — less secondary fight
    double elbow_nullspace_sigma_min = 0.02; // skip secondary near 6D singularity
    double elbow_nullspace_dq_max = 0.10;    // per-iter clamp on ‖dq_sec‖
    // Light continuity in primary with elbow_ns (tip still dominates).
    double elbow_nullspace_cont_scale = 0.22;  // was 0.15 — more glue for stability

    // When wrist residual is large (tip lagging), gently soften continuity so the
    // primary 6D task can catch up. Does NOT strip elbow (elbow is already nullspace).
    bool wrist_lag_assist_enabled = true;
    double wrist_lag_assist_err_m = 0.035;     // start assist above this e_wrist
    double wrist_lag_assist_cont_min = 0.35;   // floor for cont_w_scale
    double wrist_lag_assist_nom_min = 0.0;

    // When distal wrist joints (esp. j5) are near limits:
    // Soften ori only for small residuals (noise). For large attitude error (tip-down),
    // skip ori soften and gently yield elbow — do NOT boost ori / kill continuity /
    // hijack tip_yield (that made normal teleop feel uncontrolled).
    bool wrist_limit_ori_soften_enabled = true;
    double wrist_limit_ori_margin_rad = 0.08;
    double wrist_limit_ori_scale_min = 0.35;
    int wrist_limit_ori_joint_lo = 4;
    int wrist_limit_ori_joint_hi = 6;
    bool wrist_limit_elbow_yield_enabled = true;
    double wrist_limit_elbow_scale_min = 0.15;
    // Tip-down escape: keep ori weight, soften elbow only. Conservative.
    bool distal_ori_priority_enabled = true;
    double distal_ori_priority_err_rad = 0.22;  // only clear tip-down / large attitude fights
    double distal_ori_priority_ori_boost = 1.0; // no boost (1.0 = leave ori_w_scale as-is)
    double distal_ori_priority_cont_min = 0.55; // mild cont ease, not a dump
    double distal_ori_priority_elbow_scale = 0.08;
    // Wrist-limit + attitude fight: unfreeze elbow φ so proximal can free j5.
    // Keeps asymmetric q3 opening slow; only relaxes raise-hold / yield φ freeze.
    bool wrist_escape_phi_enabled = true;
    double wrist_escape_ori_err_rad = 0.12;   // e_ori above this while wrist near lim
    double wrist_escape_cont_min = 0.40;      // stronger continuity ease while escaping

    // Hard distal saturation + large e_ori: do NOT drop continuity / chase ori via
    // q4↔q6 branch hops (log quest_20260811_163801: q5@-0.785, e_ori~0.5, flip=0).
    // Soften unreachable attitude, glue free DOF, rate-limit distal joints.
    bool wrist_sat_anti_hop_enabled = true;
    double wrist_sat_anti_hop_margin_rad = 0.04;
    double wrist_sat_ori_scale = 0.35;
    double wrist_sat_cont_boost = 1.35;
    double wrist_sat_distal_slew_radps = 3.0;
    double wrist_sat_distal_max_step = 0.12;

    // Medial tip (hands toward midline): yield elbow preference + blend φ toward FK.
    bool medial_elbow_yield_enabled = true;
    double medial_elbow_scale_min = 0.1;        // elbow_w_scale floor when fully medial
    double medial_phi_fk_blend = 0.65;          // how much FK φ overrides skeleton when medial

    // High tip orientation residual: gently yield elbow (not a hard dump).
    bool ori_stress_elbow_yield_enabled = true;
    double ori_stress_elbow_err_rad = 0.20;     // was 0.12 — less hair-trigger
    double ori_stress_elbow_scale_min = 0.20;   // was 0.05

    // If commanded elbow (φ / XYZ) is far from current FK elbow, blend target toward
    // the feasible current posture so IK does not chronically fight an unreachable
    // raised/outward elbow while the wrist is already constrained.
    bool elbow_target_blend_enabled = true;
    double elbow_target_blend_start_m = 0.06;   // |E_des−E_fk| where blend begins
    double elbow_target_blend_full_m = 0.18;    // fully toward FK elbow at this error
    double elbow_target_blend_phi_start_rad = 0.40;
    double elbow_target_blend_phi_full_rad = 1.20;
    int ik_max_iters = 12;
    double ik_lambda = 1e-2;     // LM / diff-QP damping
    // IK backend: LM (position Gauss-Newton) or DIFF_QP (velocity-level + box constraints).
    HumanMimicIkBackend ik_backend = HumanMimicIkBackend::DIFF_QP;
    double ik_kp_pos = 1.0;       // diff-QP: wrist/elbow error gain per iteration
    double ik_kp_ori = 1.0;
    double ik_kp_phi = 1.0;       // diff-QP: elbow-phi error gain per iteration
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
    double extension_elbow_scale_min = 0.05;  // drop elbow task near straight (anti-spin)
    double extension_ori_scale_min = 1.25;    // BOOST ori — tip attitude owns distal DOF
    double extension_cont_scale_min = 0.85;   // keep continuity so free DOF cannot wander
    double extension_nom_scale_min = 0.0;      // drop TABLE_READY nominal (locks elbow ~90°)
    // Near extension (XYZ): pin elbow onto S–W axis so tip orientation owns distal DOF.
    bool extension_straighten_elbow = true;
    // Freeze commanded phi above this stretch (circle radius is tiny / ill-defined).
    double extension_phi_freeze_stretch = 0.55;
    // Tighten upper-arm roll (j2) slew near extension — that joint drives the "spin".
    double extension_roll_slew_scale = 0.25;

    // Rate-limit q_target after IK so a discrete branch switch cannot jump the
    // command in one control tick (hardware-unsafe, feels like "换逆解").
    bool q_target_slew_enabled = true;
    // Raised for snappier VR / couple follow (was 6.0). Still finite — do NOT disable.
    double q_target_slew_max_radps = 10.0;
    double ik_tol = 1e-5;        // convergence on |delta_q|
    double ik_max_step = 0.3;    // per-iteration per-joint clamp (rad)
    std::vector<double> q_nominal;  // size = arm dof; empty => use clutch posture

    // Freeze-and-hold after this many consecutive whole-arm solve failures.
    int solve_fail_freeze_count = 30;

    // --- Joint reference generator (1 kHz interpolation to q_target) ---
    // Higher gain / speed caps => the arm tracks the hand faster (less lag).
    double k_joint = 14.0;     // proportional gain (q_target - q_ref)
    double qdot_max = 6.0;     // rad/s per joint (was 3.5 — main couple/VR lag source)
    double qddot_max = 80.0;   // rad/s^2 per joint (was 40)

    // Post-JRG output lowpass — DEFAULT OFF. Enabling caused tip-still proximal
    // thrash (log quest_20260724_140954). Keep code path for A/B via env only.
    bool joint_output_filter_enabled = false;
    double joint_output_lowpass_hz = 14.0;
    double max_joint_output_velocity_radps = 3.2;
    double max_joint_output_accel_radps2 = 16.0;

    // --- Re-anchor on grip (anti-droop) ---
    // Default ON: seed from last COMMANDED pose (clamped vs measured). Without this,
    // every re-grip locks onto a gravity-sagged measurement and the arm ratchets down
    // (log: settle/idle with falling q1 and no [act] IK). Cap limits the yank.
    bool reanchor_to_commanded = true;
    double max_reanchor_correction_rad = 0.45;
    // Brief pause before IK so the first post-grip solve does not flash; keep small.
    int clutch_settle_ticks = 1;
    // After grip, skip skeleton IK until the hand has departed this far (m) from the
    // grip pose. While idle/settle, hand_ref is continuously re-zeroed so crossing
    // the threshold does NOT dump accumulated delta into IK (that felt like a jerk).
    // Keep small for a responsive clutch — large values eat the first intentional move.
    double clutch_start_motion_m = 0.003;
    // After idle→track, absorb this many extra ticks (zero delta) before first IK.
    // Kills Quest noise right at the start-motion threshold without freezing elbow φ.
    int clutch_track_settle_ticks = 1;
    // Legacy hard lock: freeze φ until unlock distance then blend preference.
    // DEFAULT OFF — that step felt like "肘突然参与". Prefer continuous rate-limited
    // slew from clutch-seeded φ0 toward skeleton φ_des (elbow_cmd_slew_*).
    // Set OPENARM_HM_CLUTCH_PHI_HOLD=1 only for A/B of the old behavior.
    bool clutch_phi_hold_enabled = false;
    double clutch_phi_unlock_m = 0.015;
    double clutch_phi_blend_span_m = 0.04;
    // On re-grip, seed soft-reach progress from the current S→W stretch so the
    // bent-margin gate does not slam shut and pull the wrist inward.
    bool clutch_seed_reach_progress = true;

    // Freeze measured L1/L2 after the first successful clutch so soft-reach scale
    // does not wander across re-grips over a long session.
    bool lock_segment_lengths = true;

    // Auto-calibrate wrist_to_tcp from FK at the first clutch (GetEE vs link7).
    bool auto_calibrate_wrist_tcp = true;

    // --- Tip-priority yield (wrist vs elbow preference) ---
    // When on: may lower elbow / nominal weights if wrist residual is high so the tip
    // can catch up — but that frees shoulder/elbow reconfiguration (felt as sudden
    // drop / elbow tuck). With incremental clutch the operator can re-grip and drag
    // again, so default OFF; re-enable via OPENARM_HM_TIP_PRIORITY=1 if needed.
    // soft-reach scale of tip_yield stays 0 even when enabled (see soft_reach_scale).
    bool tip_priority_enabled = false;
    double tip_priority_wrist_err_m = 0.015;     // start yielding above this residual
    double tip_priority_elbow_scale_min = 0.40;  // never abandon elbow (was 0.15 → drop)
    double tip_priority_cont_scale_min = 1.0;    // unused (kept for env compat; no-op)
    double tip_priority_shoulder_slew_boost = 1.2;  // mild proximal boost while tip_yield
    // How much of soft-reach compression (d_req−d_cmd) may raise tip_yield. 0 = off.
    double tip_priority_soft_reach_scale = 0.0;
    // Slew tip_yield so a residual spike cannot instantly kill elbow weight.
    double tip_yield_slew_per_s = 2.5;
    // Slew downward soft-reach progress so d_star does not jump with VR noise.
    double reach_progress_slew_per_s = 1.5;

    // --- Gripper (hold A/X = close, hold B/Y = open; release = freeze) ---
    double gripper_open = 0.0;
    double gripper_closed = 0.04;
    double gripper_speed = 0.02;        // open ramp (position units / s)
    double gripper_close_speed = 0.01;  // close ramp; usually slower than open
    // Soft-stall / jammed-close: while holding A/X to close, if measured stroke stops
    // advancing toward closed, enter grasp-hold (relative MIT lag) instead of winding
    // q_cmd to the hard closed limit (overheats the gripper).
    bool gripper_soft_stall = true;
    double gripper_stall_error = 0.04;      // legacy lag gate (kept; progress gate is primary)
    double gripper_stall_velocity = 0.05;   // |close-dir qdot| below this aids stall (rad/s)
    double gripper_stall_time = 0.10;       // no meaningful close progress for this long → jam
    // Must advance at least this far (along close) within stall_time, else jammed.
    double gripper_stall_progress = 0.005;
    // Extra close bias applied when freezing a non-grasp hold (0 = keep command lag only).
    double gripper_hold_preload = 0.0;
    // Grasp-hold: after jam, track q_cmd = q_meas + close_dir * lag until open (B/Y).
    // With MIT, τ ≈ Kp · lag (Kp≈10 → lag 0.22 ≈ 2.2 Nm continuous).
    bool gripper_grasp_hold = true;
    double gripper_grasp_hold_lag = 0.22;      // real rad / sim m
    double gripper_grasp_hold_lag_max = 0.30;  // cap if operator squeezes again

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
    // Continuous φ participation after clutch seed (rad/s). Lower = more gradual
    // elbow entry along the path; avoid clutch hard-lock/unlock steps.
    double elbow_cmd_slew_radps = 2.0;
    double elbow_cmd_max_step_rad = 0.20;

    // Post-IK slew for elbow flexion (index 3). Must not be far below q_target_slew
    // or tip tracking waits on j3 while the hand has already moved.
    double elbow_joint_slew_radps = 4.5;  // was 6.0 — less elbow thrash in normal teleop
    // Opening / droop (Δq3 < 0 on OpenArm) is slower than flexing tighter so a
    // slight outward move from a deep tuck does not dump the elbow.
    double elbow_joint_slew_down_scale = 0.35;  // OPENARM_HM_ELBOW_Q_SLEW_DOWN

    // Inward raised-elbow hold: once tucked/raised, keep φ from thrashing or
    // chasing gravity droop until the wrist clearly leaves the near-body band.
    bool elbow_raise_hold_enabled = true;
    double elbow_raise_hold_d_m = 0.30;           // enter when commanded S→W below this
    double elbow_raise_hold_q3_rad = 1.90;        // or elbow flexion already raised
    double elbow_raise_hold_release_d_m = 0.38;   // hysteresis exit (must be > enter)
    double elbow_raise_hold_medial_min = 0.08;    // require some medial before arming hold
    double elbow_raise_hold_medial_fk_scale = 0.15;  // scale medial→FK blend while holding
    double elbow_cmd_slew_droop_scale = 0.35;     // slower φ slew when bend goes more down
    // When elbow task is yielded for tip, freeze commanded φ (do not retarget droop).
    bool elbow_freeze_phi_on_yield = true;
    double elbow_freeze_phi_medial = 0.35;  // medial_amount above this ⇒ freeze φ

    // --- Safety (Quest freshness) ---
    double quest_stale_sec = 0.05;
    double quest_timeout_sec = 0.15;

    // --- Debug ---
    // Periodic act/settle lines: default to a per-run file (not the terminal).
    // Console stays quiet so grips/errors remain readable; set debug_log_console
    // or OPENARM_HM_LOG_CONSOLE=1 to also mirror act lines to stdout.
    bool debug_log = true;
    bool debug_log_console = false;
    double debug_log_hz = 5.0;
    // Empty => HumanMimicController::Init() picks
    //   recordings/hm_logs/quest_<timestamp>/<side>_hm.log
    // Override with OPENARM_HM_LOG_DIR / OPENARM_HM_LOG_PATH.
    std::string debug_log_path;
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
    double elbow_pos_err = 0.0;  // XYZ mode ||E-Edes||; PHI mode unused
    double elbow_phi_err = 0.0;  // PHI mode |wrap(phi-phi_des)| (rad)
    int iters = 0;
};
