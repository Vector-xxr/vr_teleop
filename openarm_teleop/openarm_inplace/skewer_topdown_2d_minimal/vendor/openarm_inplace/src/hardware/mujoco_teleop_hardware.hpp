#pragma once

#include <hardware/teleop_hardware.hpp>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <Eigen/Dense>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct MujocoDebugSegment {
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d p1 = Eigen::Vector3d::UnitX();
    float rgba[4] = {1.f, 0.2f, 0.2f, 1.f};
    float width = 0.015f;
};

struct MujocoDebugSphere {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    float rgba[4] = {1.f, 1.f, 0.f, 1.f};
    float radius = 0.02f;
};

class MujocoSimWorld {
public:
    MujocoSimWorld(const std::string& xml_path, const std::string& title);
    ~MujocoSimWorld();

    mjModel* model() const;
    mjData* data() const;
    std::mutex& mutex();
    bool window_should_close() const;
    void forward();
    void forward_and_render();

    // Thread-safe replace of overlay geometry for next render (phase-2 body viz).
    void set_debug_overlays(std::vector<MujocoDebugSegment> segments,
                            std::vector<MujocoDebugSphere> spheres);

private:
    void render();
    void append_debug_geoms();

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_ {};
    mjvOption opt_ {};
    mjvScene scene_ {};
    mjrContext context_ {};
    std::chrono::steady_clock::time_point last_render_;
    mutable std::mutex mutex_;

    std::mutex overlay_mutex_;
    std::vector<MujocoDebugSegment> debug_segments_;
    std::vector<MujocoDebugSphere> debug_spheres_;
};

class MujocoTeleopHardware : public TeleopHardware {
public:
    MujocoTeleopHardware(const std::string& xml_path, const std::string& side,
                         const std::string& title);
    MujocoTeleopHardware(std::shared_ptr<MujocoSimWorld> world, const std::string& side);
    ~MujocoTeleopHardware() override;

    std::vector<MotorState> read_arm() override;
    std::vector<MotorState> read_gripper() override;
    void send_arm(const std::vector<openarm::damiao_motor::MITParam>& cmds) override;
    void send_gripper(const std::vector<openarm::damiao_motor::MITParam>& cmds) override;
    void recv_all(int first_timeout_us = 500) override;
    void disable_all() override;
    size_t arm_motor_count() const override;
    size_t gripper_motor_count() const override;
    bool window_should_close() const;
    void set_home(const std::vector<double>& arm_positions, double gripper_position);
    static uint64_t consume_arm_send_count();

private:
    int joint_qpos_addr(const std::string& joint_name) const;
    int joint_dof_addr(const std::string& joint_name) const;
    int actuator_id(const std::string& actuator_name) const;

    std::shared_ptr<MujocoSimWorld> world_;
    std::array<int, 7> arm_qpos_ {};
    std::array<int, 7> arm_dof_ {};
    std::array<int, 7> arm_ctrl_ {};
    int gripper_qpos_ = -1;
    int gripper_mirror_qpos_ = -1;
    int gripper_ctrl_ = -1;
    std::string side_;
};
