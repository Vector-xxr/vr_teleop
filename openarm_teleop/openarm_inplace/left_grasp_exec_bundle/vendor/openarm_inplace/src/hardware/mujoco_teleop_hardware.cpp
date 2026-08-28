#include <hardware/mujoco_teleop_hardware.hpp>

#include <stdexcept>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

namespace {

int glfw_ref_count = 0;
std::mutex glfw_mutex;
std::atomic<uint64_t> arm_send_count{0};

}  // namespace

MujocoSimWorld::MujocoSimWorld(const std::string& xml_path, const std::string& title) {
    char error[1024] = "";
    model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
        throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
    }
    data_ = mj_makeData(model_);

    std::lock_guard<std::mutex> lock(glfw_mutex);
    if (glfw_ref_count++ == 0 && !glfwInit()) {
        throw std::runtime_error("glfwInit failed");
    }
    window_ = glfwCreateWindow(960, 720, title.c_str(), nullptr, nullptr);
    if (!window_) {
        throw std::runtime_error("glfwCreateWindow failed");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    glfwMakeContextCurrent(nullptr);
    cam_.distance = 2.5;
    cam_.azimuth = 160;
    cam_.elevation = -15;
    cam_.lookat[2] = 0.7;
    mj_forward(model_, data_);
    last_render_ = std::chrono::steady_clock::now();
}

MujocoSimWorld::~MujocoSimWorld() {
    std::lock_guard<std::mutex> lock(glfw_mutex);
    if (window_) glfwDestroyWindow(window_);
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    if (--glfw_ref_count == 0) glfwTerminate();
}

mjModel* MujocoSimWorld::model() const { return model_; }

mjData* MujocoSimWorld::data() const { return data_; }

std::mutex& MujocoSimWorld::mutex() { return mutex_; }

bool MujocoSimWorld::window_should_close() const {
    return window_ == nullptr || glfwWindowShouldClose(window_);
}

void MujocoSimWorld::forward() { mj_forward(model_, data_); }

void MujocoSimWorld::set_debug_overlays(std::vector<MujocoDebugSegment> segments,
                                        std::vector<MujocoDebugSphere> spheres) {
    std::lock_guard<std::mutex> lock(overlay_mutex_);
    debug_segments_ = std::move(segments);
    debug_spheres_ = std::move(spheres);
}

void MujocoSimWorld::append_debug_geoms() {
    std::vector<MujocoDebugSegment> segs;
    std::vector<MujocoDebugSphere> spheres;
    {
        std::lock_guard<std::mutex> lock(overlay_mutex_);
        segs = debug_segments_;
        spheres = debug_spheres_;
    }
    for (const auto& s : segs) {
        if (scene_.ngeom >= scene_.maxgeom) {
            break;
        }
        mjvGeom* g = &scene_.geoms[scene_.ngeom++];
        const mjtNum from[3] = {s.p0.x(), s.p0.y(), s.p0.z()};
        const mjtNum to[3] = {s.p1.x(), s.p1.y(), s.p1.z()};
        mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, s.rgba);
        mjv_connector(g, mjGEOM_CAPSULE, s.width, from, to);
    }
    for (const auto& sph : spheres) {
        if (scene_.ngeom >= scene_.maxgeom) {
            break;
        }
        mjvGeom* g = &scene_.geoms[scene_.ngeom++];
        const mjtNum size[3] = {sph.radius, sph.radius, sph.radius};
        const mjtNum pos[3] = {sph.p.x(), sph.p.y(), sph.p.z()};
        const mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        mjv_initGeom(g, mjGEOM_SPHERE, size, pos, mat, sph.rgba);
    }
}

void MujocoSimWorld::forward_and_render() {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_render_).count() >= 1.0 / 60.0) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            mj_forward(model_, data_);
            mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
            append_debug_geoms();
        }
        render();
        last_render_ = now;
    }
}

void MujocoSimWorld::render() {
    if (!window_) return;
    std::lock_guard<std::mutex> lock(glfw_mutex);
    glfwMakeContextCurrent(window_);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    glfwMakeContextCurrent(nullptr);
    glfwPollEvents();
}

MujocoTeleopHardware::MujocoTeleopHardware(const std::string& xml_path, const std::string& side,
                                           const std::string& title)
    : MujocoTeleopHardware(std::make_shared<MujocoSimWorld>(xml_path, title), side) {}

MujocoTeleopHardware::MujocoTeleopHardware(std::shared_ptr<MujocoSimWorld> world,
                                           const std::string& side)
    : world_(std::move(world)), side_(side) {
    for (int i = 0; i < 7; ++i) {
        const std::string joint = "openarm_" + side_ + "_joint" + std::to_string(i + 1);
        arm_qpos_[i] = joint_qpos_addr(joint);
        arm_dof_[i] = joint_dof_addr(joint);
        arm_ctrl_[i] = actuator_id(side_ + "_joint" + std::to_string(i + 1) + "_ctrl");
    }
    gripper_qpos_ = joint_qpos_addr("openarm_" + side_ + "_finger_joint1");
    gripper_mirror_qpos_ = joint_qpos_addr("openarm_" + side_ + "_finger_joint2");
    gripper_ctrl_ = actuator_id(side_ + "_finger1_ctrl");
}

MujocoTeleopHardware::~MujocoTeleopHardware() = default;

int MujocoTeleopHardware::joint_qpos_addr(const std::string& joint_name) const {
    const int id = mj_name2id(world_->model(), mjOBJ_JOINT, joint_name.c_str());
    if (id < 0) throw std::runtime_error("missing joint: " + joint_name);
    return world_->model()->jnt_qposadr[id];
}

int MujocoTeleopHardware::joint_dof_addr(const std::string& joint_name) const {
    const int id = mj_name2id(world_->model(), mjOBJ_JOINT, joint_name.c_str());
    if (id < 0) throw std::runtime_error("missing joint: " + joint_name);
    return world_->model()->jnt_dofadr[id];
}

int MujocoTeleopHardware::actuator_id(const std::string& actuator_name) const {
    const int id = mj_name2id(world_->model(), mjOBJ_ACTUATOR, actuator_name.c_str());
    if (id < 0) throw std::runtime_error("missing actuator: " + actuator_name);
    return id;
}

std::vector<MotorState> MujocoTeleopHardware::read_arm() {
    std::lock_guard<std::mutex> lock(world_->mutex());
    mjData* data = world_->data();
    std::vector<MotorState> states(7);
    for (int i = 0; i < 7; ++i) {
        states[i] = {data->qpos[arm_qpos_[i]], data->qvel[arm_dof_[i]], 0.0};
    }
    return states;
}

std::vector<MotorState> MujocoTeleopHardware::read_gripper() {
    std::lock_guard<std::mutex> lock(world_->mutex());
    return {{world_->data()->qpos[gripper_qpos_], 0.0, 0.0}};
}

void MujocoTeleopHardware::send_arm(const std::vector<openarm::damiao_motor::MITParam>& cmds) {
    arm_send_count.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(world_->mutex());
    mjData* data = world_->data();
    for (size_t i = 0; i < cmds.size() && i < 7; ++i) {
        const auto& cmd = cmds[i];
        if (cmd.kp > 0.0 || cmd.kd > 0.0) {
            data->qpos[arm_qpos_[i]] = cmd.q;
            data->qvel[arm_dof_[i]] = cmd.dq;
            data->ctrl[arm_ctrl_[i]] = 0.0;
            continue;
        }
        data->ctrl[arm_ctrl_[i]] = cmd.tau;
    }
}

void MujocoTeleopHardware::send_gripper(
    const std::vector<openarm::damiao_motor::MITParam>& cmds) {
    if (cmds.empty()) return;
    std::lock_guard<std::mutex> lock(world_->mutex());
    mjData* data = world_->data();
    data->qpos[gripper_qpos_] = cmds[0].q;
    data->qpos[gripper_mirror_qpos_] = cmds[0].q;
    data->ctrl[gripper_ctrl_] = 0.0;
}

void MujocoTeleopHardware::recv_all(int) {
    std::lock_guard<std::mutex> lock(world_->mutex());
    world_->forward();
}

void MujocoTeleopHardware::disable_all() {}

size_t MujocoTeleopHardware::arm_motor_count() const { return 7; }

size_t MujocoTeleopHardware::gripper_motor_count() const { return 1; }

bool MujocoTeleopHardware::window_should_close() const {
    return world_->window_should_close();
}

uint64_t MujocoTeleopHardware::consume_arm_send_count() {
    return arm_send_count.exchange(0, std::memory_order_relaxed);
}

void MujocoTeleopHardware::set_home(const std::vector<double>& arm_positions,
                                    double gripper_position) {
    {
        std::lock_guard<std::mutex> lock(world_->mutex());
        mjData* data = world_->data();
        for (size_t i = 0; i < arm_positions.size() && i < 7; ++i) {
            data->qpos[arm_qpos_[i]] = arm_positions[i];
            data->qvel[arm_dof_[i]] = 0.0;
            data->ctrl[arm_ctrl_[i]] = 0.0;
        }
        data->qpos[gripper_qpos_] = gripper_position;
        data->qpos[gripper_mirror_qpos_] = gripper_position;
        data->ctrl[gripper_ctrl_] = 0.0;
    }
    world_->forward_and_render();
}
