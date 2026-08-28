#pragma once

#include <joint_state_converter.hpp>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <memory>
#include <vector>

class TeleopHardware {
public:
    virtual ~TeleopHardware() = default;

    virtual std::vector<MotorState> read_arm() = 0;
    virtual std::vector<MotorState> read_gripper() = 0;
    virtual void send_arm(const std::vector<openarm::damiao_motor::MITParam>& cmds) = 0;
    virtual void send_gripper(const std::vector<openarm::damiao_motor::MITParam>& cmds) = 0;
    virtual void recv_all(int first_timeout_us = 500) = 0;
    virtual void disable_all() = 0;
    virtual size_t arm_motor_count() const = 0;
    virtual size_t gripper_motor_count() const = 0;
};

class CanTeleopHardware : public TeleopHardware {
public:
    explicit CanTeleopHardware(openarm::can::socket::OpenArm* openarm);

    std::vector<MotorState> read_arm() override;
    std::vector<MotorState> read_gripper() override;
    void send_arm(const std::vector<openarm::damiao_motor::MITParam>& cmds) override;
    void send_gripper(const std::vector<openarm::damiao_motor::MITParam>& cmds) override;
    void recv_all(int first_timeout_us = 500) override;
    void disable_all() override;
    size_t arm_motor_count() const override;
    size_t gripper_motor_count() const override;

private:
    openarm::can::socket::OpenArm* openarm_;
};
