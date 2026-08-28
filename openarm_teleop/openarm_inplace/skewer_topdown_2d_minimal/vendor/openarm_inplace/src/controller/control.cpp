// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <iostream>
#include <thread>

namespace {
double env_double_or_default(const char* name, double default_value, double min_value,
                             double max_value) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') return default_value;

    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (end == raw || !std::isfinite(value)) {
        std::cerr << "[openarm][WARN] ignoring invalid " << name << "='" << raw
                  << "', using " << default_value << std::endl;
        return default_value;
    }

    if (value < min_value || value > max_value) {
        const double clamped = std::clamp(value, min_value, max_value);
        std::cerr << "[openarm][WARN] clamped " << name << " from " << value << " to "
                  << clamped << std::endl;
        return clamped;
    }
    return value;
}

double leader_friction_scale() {
    static const double scale = env_double_or_default("OPENARM_LEADER_FRICTION_SCALE", 0.3, 0.0,
                                                      2.0);
    return scale;
}

bool env_flag_or_default(const char* name, bool default_value) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') return default_value;
    return std::string(raw) != "0";
}

struct GripperHoldConfig {
    bool enabled = true;
    double contact_effort = 0.30;
    double stall_velocity = 0.02;
    double contact_position_error = 0.01;
    double contact_hold_time = 0.20;
    double release_delta = 0.06;
    double release_hold_time = 0.08;
    double hold_kp_scale = 0.8;
    bool slip_boost_enabled = true;
    double slip_boost_kp_scale = 1.15;
    double slip_boost_position_delta = 0.012;
    double slip_boost_velocity = 0.04;
    double slip_boost_hold_time = 0.25;
    double close_direction = 0.0;
};

const GripperHoldConfig& gripper_hold_config() {
    static const GripperHoldConfig config{
        env_flag_or_default("OPENARM_GRIPPER_CONTACT_HOLD", false),
        env_double_or_default("OPENARM_GRIPPER_CONTACT_EFFORT", 0.30, 0.0, 5.0),
        env_double_or_default("OPENARM_GRIPPER_STALL_VELOCITY", 0.08, 0.0, 2.0),
        env_double_or_default("OPENARM_GRIPPER_CONTACT_POSITION_ERROR", 0.01, 0.0, 2.0),
        env_double_or_default("OPENARM_GRIPPER_CONTACT_HOLD_TIME", 0.20, 0.0, 5.0),
        env_double_or_default("OPENARM_GRIPPER_RELEASE_DELTA", 0.06, 0.0, 2.0),
        env_double_or_default("OPENARM_GRIPPER_RELEASE_HOLD_TIME", 0.08, 0.0, 2.0),
        env_double_or_default("OPENARM_GRIPPER_HOLD_KP_SCALE", 0.8, 0.05, 1.5),
        env_flag_or_default("OPENARM_GRIPPER_SLIP_BOOST", true),
        env_double_or_default("OPENARM_GRIPPER_SLIP_BOOST_KP_SCALE", 1.15, 0.05, 1.5),
        env_double_or_default("OPENARM_GRIPPER_SLIP_BOOST_POSITION_DELTA", 0.012, 0.0, 1.0),
        env_double_or_default("OPENARM_GRIPPER_SLIP_BOOST_VELOCITY", 0.04, 0.0, 2.0),
        env_double_or_default("OPENARM_GRIPPER_SLIP_BOOST_HOLD_TIME", 0.25, 0.0, 2.0),
        env_double_or_default("OPENARM_GRIPPER_CLOSE_DIRECTION", 0.0, -1.0, 1.0),
    };
    return config;
}

double quintic_smoothstep(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) + 6.0 * std::pow(s, 5);
}

double quintic_smoothstep_derivative(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 30.0 * std::pow(s, 2) - 60.0 * std::pow(s, 3) + 30.0 * std::pow(s, 4);
}
}  // namespace

Control::Control(std::shared_ptr<TeleopHardware> hardware, Dynamics* dynamics_l,
                 Dynamics* dynamics_f, std::shared_ptr<RobotSystemState> robot_state, double Ts,
                 int role, size_t arm_motor_num, size_t hand_motor_num)
    : hardware_(std::move(hardware)),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJointConverter(hand_motor_num_);
}

Control::Control(std::shared_ptr<TeleopHardware> hardware, Dynamics* dynamics_l,
                 Dynamics* dynamics_f, std::shared_ptr<RobotSystemState> robot_state, double Ts,
                 int role, std::string arm_type, size_t arm_motor_num, size_t hand_motor_num)
    : hardware_(std::move(hardware)),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJointConverter(hand_motor_num_);

    arm_type_ = arm_type;
}

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 size_t arm_motor_num, size_t hand_motor_num)
    : Control(std::make_shared<CanTeleopHardware>(arm), dynamics_l, dynamics_f, robot_state, Ts,
              role, arm_motor_num, hand_motor_num) {}

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 std::string arm_type, size_t arm_motor_num, size_t hand_motor_num)
    : Control(std::make_shared<CanTeleopHardware>(arm), dynamics_l, dynamics_f, robot_state, Ts,
              role, arm_type, arm_motor_num, hand_motor_num) {}

Control::~Control() {
    std::cout << "Control destructed " << std::endl;
    delete openarmgripperjointconverter_;
    delete openarmjointconverter_;
    delete differentiator_;
}

// bool Control::Setup(void)
// {
//         // double motor_position[NMOTORS] = {0.0};

//         // ComputeJointPosition(motor_position, response_->position.data());

//         std::cout << "!control->Setup()  finished "<< std::endl;

//         return true;
// }

void Control::Shutdown(void) {
    std::cout << "control shutdown !!!" << std::endl;

    hardware_->disable_all();
}

void Control::SetParameter(const std::vector<double>& Kp, const std::vector<double>& Kd,
                           const std::vector<double>& Fc, const std::vector<double>& k,
                           const std::vector<double>& Fv, const std::vector<double>& Fo) {
    Kp_ = Kp;
    Kd_ = Kd;
    Fc_ = Fc;
    k_ = k;
    Fv_ = Fv;
    Fo_ = Fo;
}

void Control::SetSafetyFilter(std::unique_ptr<SafetyFilter> safety_filter) {
    safety_filter_ = std::move(safety_filter);
}

void Control::ResetSafetyFilter(const std::vector<JointState>& initial_cmd) {
    if (safety_filter_) {
        safety_filter_->reset(initial_cmd);
    }
}

std::vector<double> Control::GetLastCommandEfforts() const {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    return last_command_efforts_;
}

bool Control::GripperContactHoldActive() const {
    return gripper_contact_hold_active_.load();
}

bool Control::GripperSlipBoostActive() const {
    return gripper_slip_boost_active_.load();
}

bool Control::bilateral_step() {
    // get motor status
    std::vector<MotorState> arm_motor_states = hardware_->read_arm();
    std::vector<MotorState> gripper_motor_states = hardware_->read_gripper();

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_arm_efforts(arm_dof, 0.0);

    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);
    std::vector<double> joint_gripper_efforts(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    std::vector<JointState> joint_arm_states_ref = robot_state_->arm_state().get_all_references();
    std::vector<JointState> joint_gripper_states_ref =
        robot_state_->hand_state().get_all_references();

    if (safety_filter_) {
        std::vector<JointState> combined_ref;
        combined_ref.reserve(joint_arm_states_ref.size() + joint_gripper_states_ref.size());
        combined_ref.insert(combined_ref.end(), joint_arm_states_ref.begin(),
                            joint_arm_states_ref.end());
        combined_ref.insert(combined_ref.end(), joint_gripper_states_ref.begin(),
                            joint_gripper_states_ref.end());

        std::vector<JointState> combined_state;
        combined_state.reserve(joint_arm_states.size() + joint_gripper_states.size());
        combined_state.insert(combined_state.end(), joint_arm_states.begin(),
                              joint_arm_states.end());
        combined_state.insert(combined_state.end(), joint_gripper_states.begin(),
                              joint_gripper_states.end());

        std::vector<JointState> safe_ref;
        const SafetyStatus status =
            safety_filter_->filter(combined_ref, combined_state, Ts_, &safe_ref);
        if (status.emergency_stop && !status.reason.empty()) {
            std::cerr << "[SafetyFilter] " << status.reason << std::endl;
        }
        if (status.emergency_stop) {
            robot_state_->set_all_references(safe_ref);
            return true;
        }
        joint_arm_states_ref.assign(safe_ref.begin(),
                                    safe_ref.begin() + joint_arm_states_ref.size());
        joint_gripper_states_ref.assign(safe_ref.begin() + joint_arm_states_ref.size(),
                                        safe_ref.end());
        robot_state_->set_all_references(safe_ref);
    }

    std::vector<double> joint_arm_positions_ref(arm_dof);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions_ref[i] = joint_arm_states_ref[i].position;
    }

    if (role_ == ROLE_LEADER) {
        dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());

    } else if (role_ == ROLE_FOLLOWER) {
        dynamics_f_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_f_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());
    }

    // Friction (compute joint friction)
    for (size_t i = 0; i < joint_arm_velocities.size(); ++i)
        ComputeFriction(joint_arm_velocities.data(), friction.data(), i);
    for (size_t i = 0; i < joint_gripper_velocities.size(); ++i)
        ComputeFriction(joint_gripper_velocities.data(), friction.data(),
                        joint_arm_velocities.size() + i);

    // set gravity and friciton comp joint torque value
    for (size_t i = 0; i < arm_dof; i++) {
        joint_arm_states_ref[i].effort = gravity[i] + friction[i];
    }

    for (size_t i = 0; i < gripper_dof; i++) {
        joint_gripper_states_ref[i].effort = friction[i + arm_dof];
    }

    if (safety_filter_) {
        std::vector<JointState> combined_ref;
        combined_ref.reserve(joint_arm_states_ref.size() + joint_gripper_states_ref.size());
        combined_ref.insert(combined_ref.end(), joint_arm_states_ref.begin(),
                            joint_arm_states_ref.end());
        combined_ref.insert(combined_ref.end(), joint_gripper_states_ref.begin(),
                            joint_gripper_states_ref.end());

        std::vector<JointState> combined_state;
        combined_state.reserve(joint_arm_states.size() + joint_gripper_states.size());
        combined_state.insert(combined_state.end(), joint_arm_states.begin(),
                              joint_arm_states.end());
        combined_state.insert(combined_state.end(), joint_gripper_states.begin(),
                              joint_gripper_states.end());

        std::vector<JointState> safe_ref;
        const SafetyStatus status =
            safety_filter_->filter(combined_ref, combined_state, Ts_, &safe_ref);
        if (status.emergency_stop && !status.reason.empty()) {
            std::cerr << "[SafetyFilter] " << status.reason << std::endl;
        }
        if (status.emergency_stop) {
            robot_state_->set_all_references(safe_ref);
            return true;
        }
        joint_arm_states_ref.assign(safe_ref.begin(),
                                    safe_ref.begin() + joint_arm_states_ref.size());
        joint_gripper_states_ref.assign(safe_ref.begin() + joint_arm_states_ref.size(),
                                        safe_ref.end());
        robot_state_->set_all_references(safe_ref);
    }

    std::vector<MotorState> motor_arm_states =
        openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
    std::vector<MotorState> motor_gripper_states =
        openarmgripperjointconverter_->joint_to_motor(joint_gripper_states_ref);

    // kp kd q dq tau
    std::vector<openarm::damiao_motor::MITParam> arm_cmds;
    arm_cmds.reserve(arm_dof);
    for (size_t i = 0; i < arm_dof; ++i) {
        arm_cmds.emplace_back(openarm::damiao_motor::MITParam{
            Kp_[i], Kd_[i], motor_arm_states[i].position, motor_arm_states[i].velocity,
            motor_arm_states[i].effort});
    }

    // gripper command mit param
    std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
    gripper_cmds.reserve(gripper_dof);
    for (size_t i = 0; i < gripper_dof; ++i) {
        gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
            Kp_[i + arm_dof], Kd_[i + arm_dof], motor_gripper_states[i].position,
            motor_gripper_states[i].velocity, motor_gripper_states[i].effort});
    }

    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        last_command_efforts_.clear();
        last_command_efforts_.reserve(motor_arm_states.size() + motor_gripper_states.size());
        for (const auto& motor : motor_arm_states) {
            last_command_efforts_.push_back(motor.effort);
        }
        for (const auto& motor : motor_gripper_states) {
            last_command_efforts_.push_back(motor.effort);
        }
    }

    // send command to arm
    hardware_->send_arm(arm_cmds);
    // send command to gripper
    hardware_->send_gripper(gripper_cmds);

    std::this_thread::sleep_for(std::chrono::microseconds(200));

    hardware_->recv_all(220);

    return true;
}

bool Control::unilateral_step() {
    // get motor status
    std::vector<MotorState> arm_motor_states = hardware_->read_arm();
    std::vector<MotorState> gripper_motor_states = hardware_->read_gripper();

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    if (role_ == ROLE_LEADER) {
        // calc dynamics
        dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());

        for (size_t i = 0; i < joint_arm_velocities.size(); ++i)
            ComputeFriction(joint_arm_velocities.data(), friction.data(), i);

        for (size_t i = 0; i < joint_gripper_velocities.size(); ++i)
            ComputeFriction(joint_gripper_velocities.data(), friction.data(), arm_dof + i);

        // arm joint state
        std::vector<JointState> joint_arm_state_torque(arm_dof);
        const double friction_scale = leader_friction_scale();
        for (size_t i = 0; i < arm_dof; ++i) {
            joint_arm_state_torque[i].position = joint_arm_positions[i];
            joint_arm_state_torque[i].velocity = joint_arm_velocities[i];
            joint_arm_state_torque[i].effort =
                gravity[i] + friction[i] * friction_scale + coriolis[i] * 0.1;
        }

        // gripper joint state
        std::vector<JointState> joint_gripper_state_torque(gripper_dof);
        for (size_t i = 0; i < gripper_dof; ++i) {
            joint_gripper_state_torque[i].position = joint_gripper_positions[i];
            joint_gripper_state_torque[i].velocity = joint_gripper_velocities[i];
            joint_gripper_state_torque[i].effort = friction[arm_dof + i] * friction_scale;
        }

        std::vector<MotorState> motor_arm_states =
            openarmjointconverter_->joint_to_motor(joint_arm_state_torque);
        std::vector<MotorState> motor_gripper_states =
            openarmgripperjointconverter_->joint_to_motor(joint_gripper_state_torque);

        // arm command mit param
        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_dof);
        for (size_t i = 0; i < arm_dof; ++i) {
            arm_cmds.emplace_back(
                openarm::damiao_motor::MITParam{0.0, 0.15, 0.0, 0.0, 0.0});
        }

        // gripper command mit param
        std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
        gripper_cmds.reserve(gripper_dof);
        for (size_t i = 0; i < gripper_dof; ++i) {
            gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
                0.0, 0.0, 0.0, 0.0, 0.0});
        }

        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            last_command_efforts_.clear();
            last_command_efforts_.resize(arm_cmds.size() + gripper_cmds.size(), 0.0);
        }

        // send command to arm
        hardware_->send_arm(arm_cmds);
        // send command to gripper
        hardware_->send_gripper(gripper_cmds);

        hardware_->recv_all(200);

        return true;

    }

    else if (role_ == ROLE_FOLLOWER) {
        std::vector<JointState> joint_arm_states_ref =
            robot_state_->arm_state().get_all_references();
        std::vector<JointState> joint_hand_states_ref =
            robot_state_->hand_state().get_all_references();

        std::vector<JointState> combined_state;
        combined_state.reserve(joint_arm_states.size() + joint_gripper_states.size());
        combined_state.insert(combined_state.end(), joint_arm_states.begin(),
                              joint_arm_states.end());
        combined_state.insert(combined_state.end(), joint_gripper_states.begin(),
                              joint_gripper_states.end());

        std::vector<JointState> combined_ref;
        combined_ref.reserve(joint_arm_states_ref.size() + joint_hand_states_ref.size());
        combined_ref.insert(combined_ref.end(), joint_arm_states_ref.begin(),
                            joint_arm_states_ref.end());
        combined_ref.insert(combined_ref.end(), joint_hand_states_ref.begin(),
                            joint_hand_states_ref.end());
        if (safety_filter_) {
            std::vector<JointState> safe_ref;
            const SafetyStatus status =
                safety_filter_->filter(combined_ref, combined_state, Ts_, &safe_ref);
            if (status.emergency_stop && !status.reason.empty()) {
                std::cerr << "[SafetyFilter] " << status.reason << std::endl;
            }
            if (status.emergency_stop) {
                robot_state_->set_all_references(safe_ref);
                return true;
            }
            joint_arm_states_ref.assign(safe_ref.begin(),
                                        safe_ref.begin() + joint_arm_states_ref.size());
            joint_hand_states_ref.assign(safe_ref.begin() + joint_arm_states_ref.size(),
                                         safe_ref.end());
            robot_state_->set_all_references(safe_ref);
        }

        double gripper_kp_scale = 1.0;
        if (!joint_hand_states_ref.empty() && !joint_gripper_states.empty()) {
            const auto& config = gripper_hold_config();
            if (!config.enabled) {
                gripper_contact_hold_active_ = false;
                gripper_slip_boost_active_ = false;
                gripper_contact_timer_ = 0.0;
                gripper_release_timer_ = 0.0;
                gripper_slip_boost_timer_ = 0.0;
            } else {
                JointState& grip_ref = joint_hand_states_ref.back();
                const JointState& grip_state = joint_gripper_states.back();
                const double raw_target = grip_ref.position;
                const double target_error = raw_target - grip_state.position;
                const double configured_close_sign =
                    (config.close_direction > 0.0) ? 1.0
                    : (config.close_direction < 0.0) ? -1.0
                                                     : 0.0;
                const double closing_error =
                    configured_close_sign == 0.0
                        ? std::abs(target_error)
                        : configured_close_sign * target_error;
                const bool closing_command =
                    closing_error >= config.contact_position_error;
                const bool stalled = std::abs(grip_state.velocity) <= config.stall_velocity;
                const bool loaded = std::abs(grip_state.effort) >= config.contact_effort;

                if (!gripper_contact_hold_active_) {
                    if (closing_command && stalled && loaded) {
                        gripper_contact_timer_ += Ts_;
                        if (gripper_contact_timer_ >= config.contact_hold_time) {
                            gripper_contact_hold_active_ = true;
                            gripper_hold_position_ = grip_state.position;
                            gripper_close_sign_ =
                                configured_close_sign == 0.0
                                    ? ((target_error >= 0.0) ? 1.0 : -1.0)
                                    : configured_close_sign;
                            gripper_release_timer_ = 0.0;
                            std::cout << "[gripper-hold] contact hold enabled at "
                                      << gripper_hold_position_ << " effort="
                                      << grip_state.effort << " velocity="
                                      << grip_state.velocity << " target="
                                      << raw_target << " close_sign="
                                      << gripper_close_sign_ << std::endl;
                        }
                    } else {
                        gripper_contact_timer_ = 0.0;
                    }
                }

                if (gripper_contact_hold_active_) {
                    const double open_delta =
                        -gripper_close_sign_ * (raw_target - gripper_hold_position_);
                    if (open_delta >= config.release_delta) {
                        gripper_release_timer_ += Ts_;
                        if (gripper_release_timer_ >= config.release_hold_time) {
                            gripper_contact_hold_active_ = false;
                            gripper_slip_boost_active_ = false;
                            gripper_contact_timer_ = 0.0;
                            gripper_slip_boost_timer_ = 0.0;
                            std::cout << "[gripper-hold] released by open command, delta="
                                      << open_delta << std::endl;
                        }
                    } else {
                        gripper_release_timer_ = 0.0;
                    }
                }

                if (gripper_contact_hold_active_) {
                    if (config.slip_boost_enabled) {
                        const double opening_position_delta =
                            -gripper_close_sign_ *
                            (grip_state.position - gripper_hold_position_);
                        const double opening_velocity =
                            -gripper_close_sign_ * grip_state.velocity;
                        const bool slip_detected =
                            opening_position_delta >= config.slip_boost_position_delta ||
                            opening_velocity >= config.slip_boost_velocity;
                        if (slip_detected) {
                            const bool was_active = gripper_slip_boost_active_.load();
                            gripper_slip_boost_timer_ = config.slip_boost_hold_time;
                            gripper_slip_boost_active_ = true;
                            if (!was_active) {
                                std::cout << "[gripper-hold] slip boost enabled, open_delta="
                                          << opening_position_delta
                                          << " open_velocity=" << opening_velocity
                                          << " kp_scale="
                                          << config.slip_boost_kp_scale << std::endl;
                            }
                        } else if (gripper_slip_boost_timer_ > 0.0) {
                            gripper_slip_boost_timer_ =
                                std::max(0.0, gripper_slip_boost_timer_ - Ts_);
                            gripper_slip_boost_active_ =
                                gripper_slip_boost_timer_ > 0.0;
                        } else {
                            gripper_slip_boost_active_ = false;
                        }
                    } else {
                        gripper_slip_boost_active_ = false;
                        gripper_slip_boost_timer_ = 0.0;
                    }
                    grip_ref.position = gripper_hold_position_;
                    grip_ref.velocity = 0.0;
                    gripper_kp_scale = gripper_slip_boost_active_.load()
                                           ? config.slip_boost_kp_scale
                                           : config.hold_kp_scale;
                } else {
                    gripper_slip_boost_active_ = false;
                    gripper_slip_boost_timer_ = 0.0;
                }
                gripper_last_raw_target_ = raw_target;
            }
        }

        // Joint → Motor
        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_states_ref);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{
                Kp_[i], Kd_[i], arm_motor_refs[i].position, arm_motor_refs[i].velocity, 0.0});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            const bool is_gripper = i + 1 == hand_motor_refs.size();
            const double kp_scale =
                is_gripper ? gripper_kp_scale : 1.0;
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                Kp_[i + arm_dof] * kp_scale, Kd_[i + arm_dof], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            last_command_efforts_.clear();
            last_command_efforts_.resize(arm_cmds.size() + hand_cmds.size(), 0.0);
        }

        hardware_->send_arm(arm_cmds);
        hardware_->send_gripper(hand_cmds);

        hardware_->recv_all(200);

        return true;
    }

    return true;
}

void Control::ComputeFriction(const double* velocity, double* friction, size_t index) {
    if (TANHFRIC) {
        const double amp_tmp = 1.0;
        const double coef_tmp = 0.1;

        const double v = velocity[index];
        const double Fc = Fc_.at(index);
        const double k = k_.at(index);
        const double Fv = Fv_.at(index);
        const double Fo = Fo_.at(index);

        friction[index] = amp_tmp * Fc * std::tanh(coef_tmp * k * v) + Fv * v + Fo;
    } else {
        friction[index] = velocity[index] * Dn_.at(index);
    }
}

bool Control::AdjustPosition(void) {
    std::vector<JointState> goal(NMOTORS);
    for (size_t i = 0; i < NMOTORS - 1; ++i) {
        goal[i].position = INITIAL_POSITION[i];
    }
    goal[NMOTORS - 1].position = 0.0;
    return AdjustPositionTo(goal);
}

bool Control::AdjustPositionTo(const std::vector<JointState>& goal) {
    std::vector<MotorState> arm_motor_states = hardware_->read_arm();
    std::vector<MotorState> gripper_motor_states = hardware_->read_gripper();

    std::vector<JointState> joint_arm_now =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_hand_now =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    std::vector<JointState> joint_arm_goal(NMOTORS - 1);
    for (size_t i = 0; i < NMOTORS - 1; ++i) {
        joint_arm_goal[i].position =
            (i < goal.size()) ? goal[i].position : INITIAL_POSITION[i];
        joint_arm_goal[i].velocity = 0.0;
        joint_arm_goal[i].effort = 0.0;
    }

    std::vector<JointState> joint_hand_goal(joint_hand_now.size());
    for (size_t i = 0; i < joint_hand_goal.size(); ++i) {
        const size_t goal_idx = (NMOTORS - 1) + i;
        joint_hand_goal[i].position = (goal_idx < goal.size()) ? goal[goal_idx].position : 0.0;
        joint_hand_goal[i].velocity = 0.0;
        joint_hand_goal[i].effort = 0.0;
    }

    std::vector<double> kp_arm_temp = {50, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    std::vector<double> kd_arm_temp = {1.2, 1.2, 1.2, 1.2, 0.3, 0.2, 0.3};

    std::vector<double> kp_hand_temp = {10.0};
    std::vector<double> kd_hand_temp = {0.5};

    double max_delta = 0.0;
    for (size_t i = 0; i < joint_arm_goal.size() && i < joint_arm_now.size(); ++i) {
        max_delta = std::max(max_delta,
                             std::abs(joint_arm_goal[i].position - joint_arm_now[i].position));
    }
    for (size_t i = 0; i < joint_hand_goal.size() && i < joint_hand_now.size(); ++i) {
        max_delta = std::max(max_delta,
                             std::abs(joint_hand_goal[i].position - joint_hand_now[i].position));
    }

    const double speed = env_double_or_default("OPENARM_ADJUST_POSITION_SPEED", 0.25, 0.02, 2.0);
    const double min_duration =
        env_double_or_default("OPENARM_ADJUST_POSITION_MIN_DURATION", 4.0, 0.5, 20.0);
    const double duration = std::max(min_duration, max_delta / speed);
    const int nstep = std::max(1, static_cast<int>(std::ceil(duration / 0.01)));
    std::cout << "[AdjustPosition] smooth startup duration=" << duration
              << "s, max_delta=" << max_delta << " rad" << std::endl;

    for (int step = 0; step < nstep; ++step) {
        const double s = static_cast<double>(step + 1) / nstep;
        const double alpha = quintic_smoothstep(s);
        const double alpha_dot = quintic_smoothstep_derivative(s) / duration;

        std::vector<JointState> joint_arm_interp(NMOTORS - 1);
        for (size_t i = 0; i < NMOTORS - 1; ++i) {
            const double delta = joint_arm_goal[i].position - joint_arm_now[i].position;
            joint_arm_interp[i].position =
                joint_arm_now[i].position + delta * alpha;
            joint_arm_interp[i].velocity = delta * alpha_dot;
        }

        std::vector<JointState> joint_hand_interp(joint_hand_goal.size());
        for (size_t i = 0; i < joint_hand_interp.size(); ++i) {
            const double delta = joint_hand_goal[i].position - joint_hand_now[i].position;
            joint_hand_interp[i].position =
                joint_hand_now[i].position + delta * alpha;
            joint_hand_interp[i].velocity = delta * alpha_dot;
        }

        if (safety_filter_) {
            std::vector<JointState> combined_interp;
            combined_interp.reserve(joint_arm_interp.size() + joint_hand_interp.size());
            combined_interp.insert(combined_interp.end(), joint_arm_interp.begin(),
                                   joint_arm_interp.end());
            combined_interp.insert(combined_interp.end(), joint_hand_interp.begin(),
                                   joint_hand_interp.end());

            std::vector<JointState> combined_now;
            combined_now.reserve(joint_arm_now.size() + joint_hand_now.size());
            combined_now.insert(combined_now.end(), joint_arm_now.begin(), joint_arm_now.end());
            combined_now.insert(combined_now.end(), joint_hand_now.begin(), joint_hand_now.end());

            std::vector<JointState> safe_interp;
            const SafetyStatus status =
                safety_filter_->filter(combined_interp, combined_now, 0.01, &safe_interp);
            if (status.emergency_stop && !status.reason.empty()) {
                std::cerr << "[SafetyFilter][AdjustPosition] " << status.reason << std::endl;
            }
            if (!status.emergency_stop) {
                joint_arm_interp.assign(safe_interp.begin(),
                                        safe_interp.begin() + joint_arm_interp.size());
                joint_hand_interp.assign(safe_interp.begin() + joint_arm_interp.size(),
                                         safe_interp.end());
            }
        }

        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_interp);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_interp);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{kp_arm_temp[i], kd_arm_temp[i],
                                                                  arm_motor_refs[i].position,
                                                                  arm_motor_refs[i].velocity, 0.0});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                kp_hand_temp[i], kd_hand_temp[i], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        hardware_->send_arm(arm_cmds);
        hardware_->send_gripper(hand_cmds);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        hardware_->recv_all();
    }

    std::vector<MotorState> arm_motor_states_final = hardware_->read_arm();
    std::vector<MotorState> gripper_motor_states_final = hardware_->read_gripper();

    std::vector<JointState> joint_arm_final =
        openarmjointconverter_->motor_to_joint(arm_motor_states_final);
    std::vector<JointState> joint_hand_final =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states_final);

    robot_state_->arm_state().set_all_references(joint_arm_final);
    robot_state_->hand_state().set_all_references(joint_hand_final);

    return true;
}

bool Control::DetectVibration(const double* velocity, bool* what_axis) {
    bool vibration_detected = false;

    for (int i = 0; i < NJOINTS; ++i) {
        what_axis[i] = false;

        velocity_buffer_[i].push_back(velocity[i]);
        if (velocity_buffer_[i].size() > VEL_WINDOW_SIZE) velocity_buffer_[i].pop_front();

        if (velocity_buffer_[i].size() < VEL_WINDOW_SIZE) continue;

        double mean = std::accumulate(velocity_buffer_[i].begin(), velocity_buffer_[i].end(), 0.0) /
                      velocity_buffer_[i].size();

        double var = 0.0;
        for (double v : velocity_buffer_[i]) {
            var += (v - mean) * (v - mean);
        }

        double stddev = std::sqrt(var / velocity_buffer_[i].size());

        if (stddev > VIB_THRESHOLD) {
            what_axis[i] = true;
            vibration_detected = true;
            std::cout << "[VIBRATION] Joint " << i << " stddev: " << stddev << std::endl;
        }
    }

    return vibration_detected;
}
