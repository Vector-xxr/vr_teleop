// Copyright 2025 Enactic, Inc.
//
// Independent TCP velocity approach stream (not CSV play).

#include <controller/tcp_approach_stream.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace openarm_approach {
namespace {

std::vector<std::string> split_text(const std::string& text, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, delimiter)) {
        parts.push_back(part);
    }
    return parts;
}

bool parse_side(const std::string& text, std::string* side_out) {
    if (text != "left" && text != "right") return false;
    if (side_out != nullptr) *side_out = text;
    return true;
}

bool finite3(const Eigen::Vector3d& v) { return v.allFinite(); }

}  // namespace

bool TcpApproachStream::side_active(const std::string& side) const {
    if (side == "left") return left_.active;
    if (side == "right") return right_.active;
    return false;
}

bool TcpApproachStream::blocks_play(const std::string& side) const {
    if (side.empty()) {
        // Dual play requires both sides free of approach.
        return left_.active || right_.active;
    }
    return side_active(side);
}

TcpApproachStream::Side* TcpApproachStream::side_mut(const std::string& side) {
    if (side == "left") return &left_;
    if (side == "right") return &right_;
    return nullptr;
}

const TcpApproachStream::Side* TcpApproachStream::side_const(const std::string& side) const {
    if (side == "left") return &left_;
    if (side == "right") return &right_;
    return nullptr;
}

bool TcpApproachStream::seed_side(Side* side, const std::vector<JointState>& hold, Dynamics* dyn) {
    if (side == nullptr || dyn == nullptr || hold.size() < 8) return false;
    const size_t arm_dof = dyn->GetDof();
    if (arm_dof < 7 || hold.size() < arm_dof) return false;

    side->q_arm.assign(arm_dof, 0.0);
    for (size_t i = 0; i < arm_dof; ++i) {
        side->q_arm[i] = hold[i].position;
    }
    side->gripper = hold.size() > 7 ? hold[7].position : 0.0;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    dyn->GetEECordinate(side->q_arm.data(), R, p);
    if (!finite3(p) || !R.allFinite()) return false;

    side->tcp_des = p;
    side->R_des = R;
    side->tcp_valid = true;
    side->cmd_vel.setZero();
    side->filt_vel.setZero();
    side->have_vel_cmd = false;
    side->active = true;
    return true;
}

ApproachStatus TcpApproachStream::make_pose_status(const std::string& side_name,
                                                   const Side& side) const {
    ApproachStatus out;
    out.handled = true;
    out.status = "approach_pose";
    std::ostringstream detail;
    detail << side_name << "|q=";
    for (size_t i = 0; i < side.q_arm.size(); ++i) {
        if (i) detail << ",";
        detail << side.q_arm[i];
    }
    detail << "|g=" << side.gripper;
    if (side.tcp_valid) {
        detail << "|tcp=" << side.tcp_des.x() << "," << side.tcp_des.y() << ","
               << side.tcp_des.z();
    }
    out.detail = detail.str();
    return out;
}

ApproachStatus TcpApproachStream::handle_command(const std::string& message,
                                                 bool left_replay_active,
                                                 bool right_replay_active,
                                                 bool dual_replay_active,
                                                 const std::vector<JointState>& hold_left,
                                                 const std::vector<JointState>& hold_right,
                                                 Dynamics* left_dyn, Dynamics* right_dyn) {
    ApproachStatus out;
    const auto parts = split_text(message, '|');
    if (parts.empty()) return out;

    const std::string& op = parts[0];
    if (op != "approach_on" && op != "tcp_vel" && op != "approach_stop" &&
        op != "approach_get_pose") {
        return out;
    }
    out.handled = true;

    if (parts.size() < 2) {
        out.status = "approach_error";
        out.detail = "missing_side|" + message;
        return out;
    }
    std::string side_name;
    if (!parse_side(parts[1], &side_name)) {
        out.status = "approach_error";
        out.detail = "bad_side|" + parts[1];
        return out;
    }

    Side* side = side_mut(side_name);
    Dynamics* dyn = (side_name == "left") ? left_dyn : right_dyn;
    const std::vector<JointState>& hold = (side_name == "left") ? hold_left : hold_right;
    const bool replay_busy =
        dual_replay_active ||
        (side_name == "left" ? left_replay_active : right_replay_active);

    if (op == "approach_on") {
        if (replay_busy) {
            out.status = "busy";
            out.detail = "approach_on_blocked_by_play|" + side_name;
            return out;
        }
        if (side->active) {
            // Re-seed from current hold (idempotent restart).
        }
        if (parts.size() >= 3 && !parts[2].empty()) {
            try {
                const double timeout = std::stod(parts[2]);
                if (std::isfinite(timeout) && timeout > 0.0 && timeout < 5.0) {
                    side->cmd_timeout_s = timeout;
                }
            } catch (...) {
            }
        }
        if (parts.size() >= 4 && !parts[3].empty()) {
            try {
                const double accel = std::stod(parts[3]);
                if (std::isfinite(accel) && accel > 0.0 && accel < 5.0) {
                    side->accel_mps2 = accel;
                }
            } catch (...) {
            }
        }
        if (!seed_side(side, hold, dyn)) {
            out.status = "approach_error";
            out.detail = "seed_failed|" + side_name;
            side->active = false;
            return out;
        }
        std::cout << "[tcp-approach] ON side=" << side_name
                  << " timeout=" << side->cmd_timeout_s << "s accel=" << side->accel_mps2
                  << " m/s^2" << std::endl;
        out.status = "approach_on";
        out.detail = side_name;
        return out;
    }

    if (op == "approach_stop") {
        if (!side->active) {
            out.status = "approach_stopped";
            out.detail = side_name + "|already_idle";
            return out;
        }
        side->cmd_vel.setZero();
        side->filt_vel.setZero();
        side->have_vel_cmd = false;
        side->active = false;
        std::cout << "[tcp-approach] STOP side=" << side_name << std::endl;
        out.status = "approach_stopped";
        out.detail = side_name;
        return out;
    }

    if (op == "approach_get_pose") {
        if (!side->active && !side->tcp_valid) {
            // Allow pose query from last hold even if inactive: seed read-only.
            Side tmp;
            if (!seed_side(&tmp, hold, dyn)) {
                out.status = "approach_error";
                out.detail = "pose_failed|" + side_name;
                return out;
            }
            tmp.active = false;
            return make_pose_status(side_name, tmp);
        }
        return make_pose_status(side_name, *side);
    }

    // tcp_vel|side|vx|vy|vz
    if (op == "tcp_vel") {
        if (!side->active) {
            out.status = "busy";
            out.detail = "tcp_vel_requires_approach_on|" + side_name;
            return out;
        }
        if (replay_busy) {
            out.status = "busy";
            out.detail = "tcp_vel_blocked_by_play|" + side_name;
            return out;
        }
        if (parts.size() < 5) {
            out.status = "approach_error";
            out.detail = "tcp_vel_args|" + message;
            return out;
        }
        try {
            Eigen::Vector3d vel(std::stod(parts[2]), std::stod(parts[3]), std::stod(parts[4]));
            if (!finite3(vel)) {
                out.status = "approach_error";
                out.detail = "tcp_vel_nan|" + side_name;
                return out;
            }
            // Soft cap: 0.15 m/s tip speed.
            const double vmax = 0.15;
            const double n = vel.norm();
            if (n > vmax) vel *= (vmax / n);
            side->cmd_vel = vel;
            side->last_vel_cmd = std::chrono::steady_clock::now();
            side->have_vel_cmd = true;
            out.status = "tcp_vel_ok";
            out.detail = side_name;
            return out;
        } catch (const std::exception& e) {
            out.status = "approach_error";
            out.detail = std::string("tcp_vel_parse|") + e.what();
            return out;
        }
    }

    out.status = "approach_error";
    out.detail = "unhandled|" + message;
    return out;
}

bool TcpApproachStream::tick_side(Side* side, double dt, Dynamics* dyn,
                                  std::vector<JointState>* hold) {
    if (side == nullptr || !side->active || dyn == nullptr || hold == nullptr) return false;
    if (!side->tcp_valid || hold->size() < 8 || side->q_arm.size() < dyn->GetDof()) {
        return false;
    }
    if (dt <= 0.0 || !std::isfinite(dt)) return false;

    // Command timeout → zero commanded velocity (filt decelerates via accel limit).
    if (side->have_vel_cmd) {
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                         side->last_vel_cmd)
                               .count();
        if (age > side->cmd_timeout_s) {
            side->cmd_vel.setZero();
        }
    } else {
        side->cmd_vel.setZero();
    }

    // Accel-limit tip speed magnitude (same idea as panel jog, new code path).
    const double speed_cmd = side->cmd_vel.norm();
    const double speed_filt = side->filt_vel.norm();
    const double dv_max = std::max(1e-6, side->accel_mps2) * dt;
    double speed_next = speed_filt + std::clamp(speed_cmd - speed_filt, -dv_max, dv_max);
    Eigen::Vector3d v_smooth = Eigen::Vector3d::Zero();
    if (speed_cmd > 1e-9) {
        v_smooth = (side->cmd_vel / speed_cmd) * speed_next;
    } else if (speed_next > 1e-9 && side->filt_vel.norm() > 1e-9) {
        v_smooth = (side->filt_vel / side->filt_vel.norm()) * speed_next;
    }
    side->filt_vel = v_smooth;
    side->tcp_des += v_smooth * dt;

    double q_out[16];
    const size_t arm_dof = dyn->GetDof();
    if (arm_dof > 16) return false;
    if (!dyn->SolvePosIk(side->q_arm.data(), side->R_des, side->tcp_des, q_out)) {
        // IK fail: freeze TCP at last good joints (hold velocity).
        side->filt_vel.setZero();
        side->cmd_vel.setZero();
        return false;
    }
    dyn->ClampJointPositions(q_out);
    for (size_t i = 0; i < arm_dof; ++i) {
        side->q_arm[i] = q_out[i];
        (*hold)[i].position = q_out[i];
        (*hold)[i].velocity = 0.0;
        (*hold)[i].effort = 0.0;
    }
    if (hold->size() > 7) {
        (*hold)[7].position = side->gripper;
        (*hold)[7].velocity = 0.0;
        (*hold)[7].effort = 0.0;
    }
    // Refresh FK so tcp_des stays consistent with solved joints (avoid drift).
    Eigen::Matrix3d R = side->R_des;
    Eigen::Vector3d p = side->tcp_des;
    dyn->GetEECordinate(side->q_arm.data(), R, p);
    if (finite3(p)) {
        // Keep commanded orientation lock; accept FK position.
        side->tcp_des = p;
    }
    return true;
}

void TcpApproachStream::tick(double dt, Dynamics* left_dyn, Dynamics* right_dyn,
                             std::vector<JointState>* hold_left,
                             std::vector<JointState>* hold_right) {
    if (left_.active) {
        tick_side(&left_, dt, left_dyn, hold_left);
    }
    if (right_.active) {
        tick_side(&right_, dt, right_dyn, hold_right);
    }
}

}  // namespace openarm_approach
