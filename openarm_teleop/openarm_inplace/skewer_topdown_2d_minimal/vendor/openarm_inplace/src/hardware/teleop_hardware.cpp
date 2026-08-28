#include <hardware/teleop_hardware.hpp>

CanTeleopHardware::CanTeleopHardware(openarm::can::socket::OpenArm* openarm)
    : openarm_(openarm) {}

std::vector<MotorState> CanTeleopHardware::read_arm() {
    std::vector<MotorState> states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    return states;
}

std::vector<MotorState> CanTeleopHardware::read_gripper() {
    std::vector<MotorState> states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    return states;
}

void CanTeleopHardware::send_arm(const std::vector<openarm::damiao_motor::MITParam>& cmds) {
    openarm_->get_arm().mit_control_all(cmds);
}

void CanTeleopHardware::send_gripper(const std::vector<openarm::damiao_motor::MITParam>& cmds) {
    openarm_->get_gripper().mit_control_all(cmds);
}

void CanTeleopHardware::recv_all(int first_timeout_us) { openarm_->recv_all(first_timeout_us); }

void CanTeleopHardware::disable_all() { openarm_->disable_all(); }

size_t CanTeleopHardware::arm_motor_count() const {
    return openarm_->get_arm().get_motors().size();
}

size_t CanTeleopHardware::gripper_motor_count() const {
    return openarm_->get_gripper().get_motors().size();
}
