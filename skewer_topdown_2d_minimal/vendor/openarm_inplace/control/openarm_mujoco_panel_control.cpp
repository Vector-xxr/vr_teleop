#include <algorithm>
#include <chrono>
#include <common.hpp>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <hardware/mujoco_teleop_hardware.hpp>
#include <io/teleop_io.hpp>
#include <openarm_constants.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <yamlloader.hpp>

class DebugCsv {
public:
    bool open(const std::string& path) {
        file_.open(path);
        if (!file_) {
            std::cerr << "[debug] failed to open: " << path << std::endl;
            return false;
        }
        start_ = std::chrono::steady_clock::now();
        file_ << "time,side,global_index,local_index,q_target,dq_target,q_ref,dq_ref,"
                 "q_actual,dq_actual,tracking_error,control_dt,input_update_dt\n";
        return true;
    }

    void append(const std::string& side, size_t global_offset,
                const std::vector<JointState>& target, const std::vector<JointState>& ref,
                const std::vector<JointState>& actual, double control_dt,
                double input_update_dt) {
        if (!file_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - start_).count();
        const size_t count = std::min({target.size(), ref.size(), actual.size()});
        for (size_t i = 0; i < count; ++i) {
            file_ << t << "," << side << "," << global_offset + i << "," << i << ","
                  << target[i].position << "," << target[i].velocity << ","
                  << ref[i].position << "," << ref[i].velocity << ","
                  << actual[i].position << "," << actual[i].velocity << ","
                  << ref[i].position - actual[i].position << "," << control_dt << ","
                  << input_update_dt << "\n";
        }
    }

    void close() {
        if (file_) file_.close();
    }

private:
    std::ofstream file_;
    std::chrono::steady_clock::time_point start_;
    std::mutex mutex_;
};

struct DebugTargets {
    std::mutex mutex;
    std::vector<JointState> targets;
    double input_update_dt = 0.0;
    std::chrono::steady_clock::time_point last_update = std::chrono::steady_clock::now();
};

struct FrequencyStats {
    std::atomic<uint64_t> control_ticks{0};
    std::atomic<uint64_t> render_ticks{0};
};

class SimControlThread : public PeriodicTimerThread {
public:
    SimControlThread(Control* control, std::shared_ptr<MujocoTeleopHardware> hardware,
                     std::shared_ptr<RobotSystemState> state,
                     std::shared_ptr<DebugTargets> debug_targets, DebugCsv* debug_csv,
                     std::shared_ptr<FrequencyStats> frequency_stats, std::string side,
                     size_t global_offset, bool debug_target_from_state = false,
                     double hz = 500.0)
        : PeriodicTimerThread(hz),
          control_(control),
          hardware_(std::move(hardware)),
          state_(std::move(state)),
          debug_targets_(std::move(debug_targets)),
          debug_csv_(debug_csv),
          frequency_stats_(std::move(frequency_stats)),
          side_(std::move(side)),
          global_offset_(global_offset),
          debug_target_from_state_(debug_target_from_state) {}

protected:
    void on_timer() override {
        const auto start = std::chrono::steady_clock::now();
        control_->unilateral_step();
        const auto end = std::chrono::steady_clock::now();
        if (frequency_stats_ != nullptr) {
            frequency_stats_->control_ticks.fetch_add(1, std::memory_order_relaxed);
        }
        if (debug_csv_ != nullptr && state_ != nullptr && debug_targets_ != nullptr) {
            std::vector<JointState> target;
            double input_update_dt = 0.0;
            if (debug_target_from_state_) {
                target = state_->get_all_references();
            } else {
                std::lock_guard<std::mutex> lock(debug_targets_->mutex);
                if (debug_targets_->targets.size() >= global_offset_ + 8) {
                    target.assign(debug_targets_->targets.begin() + global_offset_,
                                  debug_targets_->targets.begin() + global_offset_ + 8);
                    input_update_dt = debug_targets_->input_update_dt;
                }
            }
            if (!target.empty()) {
                debug_csv_->append(
                    side_, global_offset_, target, state_->get_all_references(),
                    state_->get_all_responses(),
                    std::chrono::duration<double>(end - start).count(), input_update_dt);
            }
        }
        if (hardware_->window_should_close()) keep_running = false;
    }

private:
    Control* control_;
    std::shared_ptr<MujocoTeleopHardware> hardware_;
    std::shared_ptr<RobotSystemState> state_;
    std::shared_ptr<DebugTargets> debug_targets_;
    DebugCsv* debug_csv_;
    std::shared_ptr<FrequencyStats> frequency_stats_;
    std::string side_;
    size_t global_offset_;
    bool debug_target_from_state_;
};

class SimRenderThread : public PeriodicTimerThread {
public:
    SimRenderThread(std::shared_ptr<MujocoSimWorld> world,
                    std::shared_ptr<FrequencyStats> frequency_stats, double hz = 60.0)
        : PeriodicTimerThread(hz),
          world_(std::move(world)),
          frequency_stats_(std::move(frequency_stats)) {}

protected:
    void on_timer() override {
        world_->forward_and_render();
        if (frequency_stats_ != nullptr) {
            frequency_stats_->render_ticks.fetch_add(1, std::memory_order_relaxed);
        }
        if (world_->window_should_close()) keep_running = false;
    }

private:
    std::shared_ptr<MujocoSimWorld> world_;
    std::shared_ptr<FrequencyStats> frequency_stats_;
};

class SimAdminThread : public PeriodicTimerThread {
public:
    SimAdminThread(std::shared_ptr<RobotSystemState> leader_left_state,
                   std::shared_ptr<RobotSystemState> leader_right_state,
                   std::shared_ptr<RobotSystemState> follower_left_state,
                   std::shared_ptr<RobotSystemState> follower_right_state,
                   UdpPanelReceiver* panel, CsvReplay* replay, CsvRecorder* recorder,
                   std::shared_ptr<DebugTargets> debug_targets,
                   int replay_repeat = 0, double hz = 200.0)
        : PeriodicTimerThread(hz),
          leader_left_state_(std::move(leader_left_state)),
          leader_right_state_(std::move(leader_right_state)),
          follower_left_state_(std::move(follower_left_state)),
          follower_right_state_(std::move(follower_right_state)),
          panel_(panel),
          replay_(replay),
          recorder_(recorder),
          debug_targets_(std::move(debug_targets)),
          replay_repeat_(replay_repeat) {}

protected:
    void on_timer() override {
        // --- Phase 1: Get external input (panel or replay) ---
        std::vector<JointState> external_refs;
        if (replay_ != nullptr) {
            auto sample = replay_->sample();
            if (sample.has_value()) last_external_ref_ = sample.value();
        } else if (panel_ != nullptr) {
            auto sample = panel_->receive_latest(kBimanualJointCount);
            if (sample.has_value()) last_external_ref_ = sample.value();
        }

        if (last_external_ref_.empty()) return;
        external_refs = last_external_ref_;
        if (external_refs.size() != kBimanualJointCount) return;

        std::vector<JointState> left_refs(external_refs.begin(), external_refs.begin() + kSingleArmJointCount);
        std::vector<JointState> right_refs(external_refs.begin() + kSingleArmJointCount, external_refs.end());

        // --- Phase 2: Set leader references ---
        // Panel mode: external input drives leader (like real hardware).
        // Replay mode: leader stays at home, replay drives follower directly.
        if (replay_ == nullptr) {
            leader_left_state_->set_all_references(left_refs);
            leader_right_state_->set_all_references(right_refs);
        }

        // --- Phase 3: Determine follower references ---
        // Panel mode: read leader actual state as follower reference (matches real hardware Live mode).
        // Replay mode: use replay data directly as follower reference.
        std::vector<JointState> follower_refs;
        if (replay_ != nullptr) {
            follower_refs = external_refs;
        } else {
            auto leader_left_resp = leader_left_state_->get_all_responses();
            auto leader_right_resp = leader_right_state_->get_all_responses();
            if (leader_left_resp.empty() || leader_right_resp.empty()) return;
            follower_refs.reserve(leader_left_resp.size() + leader_right_resp.size());
            follower_refs.insert(follower_refs.end(), leader_left_resp.begin(), leader_left_resp.end());
            follower_refs.insert(follower_refs.end(), leader_right_resp.begin(), leader_right_resp.end());
        }

        // --- Phase 4: Set follower references and record ---
        follower_left_state_->set_all_references(
            std::vector<JointState>(follower_refs.begin(), follower_refs.begin() + kSingleArmJointCount));
        follower_right_state_->set_all_references(
            std::vector<JointState>(follower_refs.begin() + kSingleArmJointCount, follower_refs.end()));

        if (debug_targets_ != nullptr) {
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(debug_targets_->mutex);
            debug_targets_->input_update_dt =
                std::chrono::duration<double>(now - debug_targets_->last_update).count();
            debug_targets_->last_update = now;
            debug_targets_->targets = follower_refs;
        }
        if (recorder_ != nullptr && recorder_->active()) recorder_->append(follower_refs);
        handle_replay_end();
    }

private:
    void handle_replay_end() {
        if (replay_done_) return;
        if (replay_ == nullptr || !replay_->finished()) return;
        if (replay_repeat_ == 0) return;

        ++completed_replays_;
        if (replay_repeat_ > 0 && completed_replays_ >= replay_repeat_) {
            std::cout << "[replay] completed " << completed_replays_ << " replay cycle(s)"
                      << std::endl;
            replay_done_ = true;
            keep_running = false;
            return;
        }

        replay_->reset();
        last_external_ref_.clear();
        std::cout << "[replay] restart cycle " << (completed_replays_ + 1) << std::endl;
    }

    static constexpr size_t kSingleArmJointCount = 8;
    static constexpr size_t kBimanualJointCount = 16;
    std::shared_ptr<RobotSystemState> leader_left_state_;
    std::shared_ptr<RobotSystemState> leader_right_state_;
    std::shared_ptr<RobotSystemState> follower_left_state_;
    std::shared_ptr<RobotSystemState> follower_right_state_;
    UdpPanelReceiver* panel_;
    CsvReplay* replay_;
    CsvRecorder* recorder_;
    std::shared_ptr<DebugTargets> debug_targets_;
    std::vector<JointState> last_external_ref_;
    int replay_repeat_ = 0;
    int completed_replays_ = 0;
    bool replay_done_ = false;
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string xml_path = "../openarm_mujoco/v1/scene.xml";
    std::string urdf_path = "../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf";
    std::string side = "right";
    uint16_t panel_port = 47910;
    std::string record_path;
    std::string replay_path;
    std::string debug_csv_path;
    int replay_repeat = 0;
    bool generate_transitions = true;
    bool clean_transitions = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--xml" && i + 1 < argc) {
            xml_path = argv[++i];
        } else if (arg == "--urdf" && i + 1 < argc) {
            urdf_path = argv[++i];
        } else if (arg == "--side" && i + 1 < argc) {
            side = argv[++i];
        } else if (arg == "--panel-port" && i + 1 < argc) {
            panel_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--record" && i + 1 < argc) {
            record_path = argv[++i];
        } else if (arg == "--replay" && i + 1 < argc) {
            replay_path = argv[++i];
        } else if (arg == "--transition" && i + 1 < argc) {
            generate_transitions = std::stoi(argv[++i]) != 0;
        } else if (arg == "--clean-transitions") {
            clean_transitions = true;
        } else if (arg == "--debug-csv" && i + 1 < argc) {
            debug_csv_path = argv[++i];
        } else if (arg == "--loop") {
            replay_repeat = -1;
        } else if (arg == "--repeat" && i + 1 < argc) {
            replay_repeat = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--xml MODEL.xml] [--urdf ROBOT.urdf] [--side right|left]"
                         " [--panel-port PORT] [--record CSV] [--replay CSV_OR_DIR]"
                         " [--transition 0|1] [--clean-transitions]"
                         " [--debug-csv CSV] [--loop] [--repeat N]\n";
            return 0;
        }
    }

    YamlLoader follower_loader("config/follower.yaml");
    std::vector<double> kp = follower_loader.get_vector("FollowerArmParam", "Kp");
    std::vector<double> kd = follower_loader.get_vector("FollowerArmParam", "Kd");
    std::vector<double> fc = follower_loader.get_vector("FollowerArmParam", "Fc");
    std::vector<double> k = follower_loader.get_vector("FollowerArmParam", "k");
    std::vector<double> fv = follower_loader.get_vector("FollowerArmParam", "Fv");
    std::vector<double> fo = follower_loader.get_vector("FollowerArmParam", "Fo");

    Dynamics dynamics_left(urdf_path, "openarm_body_link0", "openarm_left_hand_tcp");
    Dynamics dynamics_right(urdf_path, "openarm_body_link0", "openarm_right_hand_tcp");
    if (!dynamics_left.Init() || !dynamics_right.Init()) return 1;

    auto leader_world = std::make_shared<MujocoSimWorld>(xml_path, "OpenArm C++ Leader Sim");
    auto follower_world = std::make_shared<MujocoSimWorld>(xml_path, "OpenArm C++ Follower Sim");
    auto leader_left_hw = std::make_shared<MujocoTeleopHardware>(leader_world, "left");
    auto leader_right_hw = std::make_shared<MujocoTeleopHardware>(leader_world, "right");
    auto follower_left_hw = std::make_shared<MujocoTeleopHardware>(follower_world, "left");
    auto follower_right_hw = std::make_shared<MujocoTeleopHardware>(follower_world, "right");

    auto leader_left_state = std::make_shared<RobotSystemState>(
        leader_left_hw->arm_motor_count(), leader_left_hw->gripper_motor_count());
    auto leader_right_state = std::make_shared<RobotSystemState>(
        leader_right_hw->arm_motor_count(), leader_right_hw->gripper_motor_count());
    auto follower_left_state = std::make_shared<RobotSystemState>(
        follower_left_hw->arm_motor_count(), follower_left_hw->gripper_motor_count());
    auto follower_right_state = std::make_shared<RobotSystemState>(
        follower_right_hw->arm_motor_count(), follower_right_hw->gripper_motor_count());

    Control leader_left_control(leader_left_hw, &dynamics_left, &dynamics_left,
                                leader_left_state, 1.0 / FREQUENCY, ROLE_FOLLOWER,
                                "left_arm", leader_left_hw->arm_motor_count(),
                                leader_left_hw->gripper_motor_count());
    Control leader_right_control(leader_right_hw, &dynamics_right, &dynamics_right,
                                 leader_right_state, 1.0 / FREQUENCY, ROLE_FOLLOWER,
                                 "right_arm", leader_right_hw->arm_motor_count(),
                                 leader_right_hw->gripper_motor_count());
    Control follower_left_control(follower_left_hw, &dynamics_left, &dynamics_left,
                                  follower_left_state, 1.0 / FREQUENCY, ROLE_FOLLOWER,
                                  "left_arm", follower_left_hw->arm_motor_count(),
                                  follower_left_hw->gripper_motor_count());
    Control follower_right_control(follower_right_hw, &dynamics_right, &dynamics_right,
                                   follower_right_state, 1.0 / FREQUENCY, ROLE_FOLLOWER,
                                   "right_arm", follower_right_hw->arm_motor_count(),
                                   follower_right_hw->gripper_motor_count());
    leader_left_control.SetParameter(kp, kd, fc, k, fv, fo);
    leader_right_control.SetParameter(kp, kd, fc, k, fv, fo);
    follower_left_control.SetParameter(kp, kd, fc, k, fv, fo);
    follower_right_control.SetParameter(kp, kd, fc, k, fv, fo);
    leader_left_control.SetSafetyFilter(std::make_unique<SafetyFilter>(make_sim_safety_config(8)));
    leader_right_control.SetSafetyFilter(std::make_unique<SafetyFilter>(make_sim_safety_config(8)));
    follower_left_control.SetSafetyFilter(std::make_unique<SafetyFilter>(make_sim_safety_config(8)));
    follower_right_control.SetSafetyFilter(std::make_unique<SafetyFilter>(make_sim_safety_config(8)));

    std::vector<JointState> initial(8);
    for (size_t i = 0; i < 7; ++i) initial[i].position = 0.0;
    initial[7].position = 0.0;
    std::vector<double> initial_arm(7);
    for (size_t i = 0; i < initial_arm.size(); ++i) initial_arm[i] = initial[i].position;
    leader_left_hw->set_home(initial_arm, 0.0);
    leader_right_hw->set_home(initial_arm, 0.0);
    follower_left_hw->set_home(initial_arm, 0.0);
    follower_right_hw->set_home(initial_arm, 0.0);
    leader_left_state->set_all_references(initial);
    leader_right_state->set_all_references(initial);
    follower_left_state->set_all_references(initial);
    follower_right_state->set_all_references(initial);
    leader_left_control.ResetSafetyFilter(initial);
    leader_right_control.ResetSafetyFilter(initial);
    follower_left_control.ResetSafetyFilter(initial);
    follower_right_control.ResetSafetyFilter(initial);

    UdpPanelReceiver panel;
    CsvReplay replay;
    CsvRecorder recorder;
    DebugCsv debug_csv;
    auto debug_targets = std::make_shared<DebugTargets>();
    auto frequency_stats = std::make_shared<FrequencyStats>();
    {
        std::lock_guard<std::mutex> lock(debug_targets->mutex);
        debug_targets->targets = initial;
        debug_targets->targets.insert(debug_targets->targets.end(), initial.begin(), initial.end());
        debug_targets->last_update = std::chrono::steady_clock::now();
        debug_targets->input_update_dt = 0.0;
    }
    UdpPanelReceiver* panel_ptr = nullptr;
    CsvReplay* replay_ptr = nullptr;
    if (clean_transitions && !replay_path.empty() && std::filesystem::is_directory(replay_path)) {
        CsvReplay::clean_transitions(replay_path);
    }
    if (!replay_path.empty()) {
        bool loaded = false;
        if (std::filesystem::is_directory(replay_path)) {
            loaded = replay.load_directory(replay_path, 16, generate_transitions);
        } else {
            loaded = replay.load(replay_path, 16);
        }
        if (!loaded) return 1;
        if (replay_repeat < 0) {
            std::cout << "[replay] loop mode enabled" << std::endl;
        } else if (replay_repeat > 0) {
            std::cout << "[replay] repeat count: " << replay_repeat << std::endl;
        }
        replay_ptr = &replay;
    } else {
        if (!panel.open(panel_port)) return 1;
        panel_ptr = &panel;
    }
    if (!record_path.empty()) recorder.open(record_path, 16);
    if (!debug_csv_path.empty()) debug_csv.open(debug_csv_path);

    const bool replay_mode = replay_ptr != nullptr;
    SimControlThread leader_left_thread(&leader_left_control, leader_left_hw, leader_left_state,
                                        debug_targets, &debug_csv, frequency_stats, "leader_left",
                                        0, replay_mode, FREQUENCY);
    SimControlThread leader_right_thread(&leader_right_control, leader_right_hw, leader_right_state,
                                         debug_targets, &debug_csv, frequency_stats, "leader_right",
                                         8, replay_mode, FREQUENCY);
    SimControlThread follower_left_thread(&follower_left_control, follower_left_hw,
                                          follower_left_state, debug_targets, &debug_csv,
                                          frequency_stats, "follower_left", 0, false, FREQUENCY);
    SimControlThread follower_right_thread(&follower_right_control, follower_right_hw,
                                           follower_right_state, debug_targets, &debug_csv,
                                           frequency_stats, "follower_right", 8, false, FREQUENCY);
    SimRenderThread leader_render_thread(leader_world, frequency_stats, 60.0);
    SimRenderThread follower_render_thread(follower_world, frequency_stats, 60.0);
    SimAdminThread admin_thread(leader_left_state, leader_right_state, follower_left_state,
                                follower_right_state, panel_ptr, replay_ptr, &recorder,
                                debug_targets, replay_repeat, 200.0);
    leader_left_thread.start_thread();
    leader_right_thread.start_thread();
    follower_left_thread.start_thread();
    follower_right_thread.start_thread();
    admin_thread.start_thread();
    leader_render_thread.start_thread();
    follower_render_thread.start_thread();

    auto last_stats = std::chrono::steady_clock::now();
    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_stats).count();
        if (elapsed >= 1.0) {
            const uint64_t control_ticks =
                frequency_stats->control_ticks.exchange(0, std::memory_order_relaxed);
            const uint64_t render_ticks =
                frequency_stats->render_ticks.exchange(0, std::memory_order_relaxed);
            const uint64_t backend_sends = MujocoTeleopHardware::consume_arm_send_count();
            std::cout << "[freq] control_total_hz=" << control_ticks / elapsed
                      << " control_per_arm_hz=" << (control_ticks / 4.0) / elapsed
                      << " backend_send_total_hz=" << backend_sends / elapsed
                      << " backend_send_per_arm_hz=" << (backend_sends / 4.0) / elapsed
                      << " render_total_hz=" << render_ticks / elapsed
                      << " render_per_window_hz=" << (render_ticks / 2.0) / elapsed
                      << std::endl;
            last_stats = now;
        }
    }

    leader_left_thread.stop_thread();
    leader_right_thread.stop_thread();
    follower_left_thread.stop_thread();
    follower_right_thread.stop_thread();
    admin_thread.stop_thread();
    leader_render_thread.stop_thread();
    follower_render_thread.stop_thread();
    recorder.close();
    debug_csv.close();
    return 0;
}
