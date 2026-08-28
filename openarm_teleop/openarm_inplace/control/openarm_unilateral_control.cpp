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

#include <chrono>
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <common.hpp>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <ctime>
#include <cmath>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <io/teleop_io.hpp>
#include <iostream>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <yamlloader.hpp>

enum class ReferenceSource {
    Live,
    Panel,
    Replay,
};

struct TeleopOptions {
    std::string arm_side = "right_arm";
    std::string leader_can_interface = "can0";
    std::string follower_can_interface = "can2";
    ReferenceSource reference_source = ReferenceSource::Live;
    uint16_t panel_port = 47910;
    uint16_t record_start_port = 0;
    uint16_t replay_start_port = 0;
    std::string record_path;
    std::string position_dump_path;
    std::string replay_path;
    bool auto_home = true;
    std::string home_pose_path;
    bool generate_transitions = true;
    bool clean_transitions = false;
    double replay_transition_duration = 0.0;
    double replay_initial_transition_duration = 0.0;
    double live_ramp_duration = 2.0;
    double live_velocity_scale = 0.0;
    bool live_wait_for_start = false;
    uint16_t monitor_port = 0;
};

class TeleopMonitorSender {
public:
    bool open(uint16_t port) {
        close();
        if (port == 0) return false;
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::cerr << "[teleop-monitor] socket failed: " << std::strerror(errno)
                      << std::endl;
            return false;
        }
        addr_ = {};
        addr_.sin_family = AF_INET;
        addr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr_.sin_port = htons(port);
        std::cout << "[teleop-monitor] sending leader/follower state to 127.0.0.1:"
                  << port << std::endl;
        return true;
    }

    void send(const std::string& arm_side, double t,
              const std::vector<JointState>& leader_arm,
              const std::vector<JointState>& leader_hand,
              const std::vector<JointState>& follower_ref,
              const std::vector<JointState>& follower_arm,
              const std::vector<JointState>& follower_hand,
              bool live_started,
              bool gripper_contact_hold_active,
              bool gripper_slip_boost_active) {
        if (fd_ < 0) return;
        std::ostringstream payload;
        payload << arm_side << "," << (live_started ? 1 : 0) << ","
                << (gripper_contact_hold_active ? 1 : 0) << ","
                << (gripper_slip_boost_active ? 1 : 0) << "," << t;
        append_joints(payload, leader_arm);
        append_joints(payload, leader_hand);
        append_joints(payload, follower_ref);
        append_joints(payload, follower_arm);
        append_joints(payload, follower_hand);
        const std::string text = payload.str();
        ::sendto(fd_, text.data(), text.size(), 0,
                 reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~TeleopMonitorSender() { close(); }

private:
    static void append_joints(std::ostringstream& payload,
                              const std::vector<JointState>& joints) {
        for (const auto& joint : joints) {
            payload << "," << joint.position << "," << joint.velocity << ","
                    << joint.effort;
        }
    }

    int fd_ = -1;
    sockaddr_in addr_ {};
};

std::string reference_source_name(ReferenceSource source) {
    switch (source) {
        case ReferenceSource::Live:
            return "live";
        case ReferenceSource::Panel:
            return "panel";
        case ReferenceSource::Replay:
            return "replay";
    }
    return "live";
}

bool parse_reference_source(const std::string& value, ReferenceSource* source) {
    if (value == "live") {
        *source = ReferenceSource::Live;
        return true;
    }
    if (value == "panel") {
        *source = ReferenceSource::Panel;
        return true;
    }
    if (value == "replay") {
        *source = ReferenceSource::Replay;
        return true;
    }
    return false;
}

bool file_contains(const std::string& path, const std::string& text) {
    std::ifstream file(path);
    if (!file) return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str().find(text) != std::string::npos;
}

std::string select_leaf_link(const std::string& urdf_path, const std::string& arm_side) {
    const std::string side = (arm_side == "left_arm") ? "left" : "right";
    const std::vector<std::string> candidates = {
        "openarm_" + side + "_hand",
        "openarm_" + side + "_hand_tcp",
        "openarm_" + side + "_link8",
        "openarm_" + side + "_link7",
    };
    for (const auto& candidate : candidates) {
        if (file_contains(urdf_path, "link name=\"" + candidate + "\"")) {
            return candidate;
        }
    }
    return candidates.front();
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " <leader_urdf_path> <follower_urdf_path> [arm_side] [leader_can] "
                 "[follower_can]\n"
              << "Options:\n"
              << "  --reference-source live|panel|replay   default: live\n"
              << "  --panel-port PORT                      default: 47910\n"
              << "  --record PATH                          record final references to CSV\n"
              << "  --record-start-port PORT               wait for UDP 'start' before recording\n"
              << "  --position-dump PATH                   append current motor positions on UDP 'dump'\n"
              << "  --replay PATH                          replay CSV (or directory of CSVs) and set source=replay\n"
              << "  --replay-start-port PORT               wait for UDP 'start' before replay motion\n"
              << "  --transition 0|1                      generate transition files (default: 1)\n"
              << "  --clean-transitions                    delete all transition files in replay directory\n"
              << "  --live-ramp-duration SEC              smooth live leader->follower handoff\n"
              << "  --live-velocity-scale SCALE           scale live leader velocity feedforward\n"
              << "  --live-wait-for-start 0|1             hold follower until UDP start in live mode\n"
              << "  --monitor-port PORT                   UDP leader/follower monitor output\n"
              << "  --home-pose PATH                      CSV first frame used as startup home pose\n"
              << "  --no-auto-home                         skip startup AdjustPosition\n";
}

double quintic_smoothstep(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) + 6.0 * std::pow(s, 5);
}

std::vector<JointState> parse_named_pose_from_position_dump_line(const std::string& line,
                                                                 const std::string& marker,
                                                                 size_t joint_count) {
    const std::string begin_marker = marker + "={";
    const size_t begin = line.find(begin_marker);
    if (begin == std::string::npos) return {};
    const size_t values_begin = begin + begin_marker.size();
    const size_t end = line.find('}', values_begin);
    if (end == std::string::npos) return {};

    std::vector<JointState> joints;
    joints.reserve(joint_count);
    std::stringstream stream(line.substr(values_begin, end - values_begin));
    std::string item;
    while (std::getline(stream, item, ',') && joints.size() < joint_count) {
        const size_t eq = item.find('=');
        if (eq == std::string::npos) continue;
        try {
            JointState joint;
            joint.position = std::stod(item.substr(eq + 1));
            joints.push_back(joint);
        } catch (...) {
            return {};
        }
    }
    if (joints.size() != joint_count) return {};
    return joints;
}

std::vector<JointState> read_home_pose_from_position_dump(const std::string& path,
                                                          const std::string& arm_side,
                                                          size_t joint_count) {
    std::ifstream in(path);
    if (!in) return {};
    std::string line;
    while (std::getline(in, line)) {
        if (line.find("arm_side=" + arm_side) == std::string::npos) continue;
        // The teleop startup home pose is one shared target for leader and follower.
        // Position dumps often contain the desired open pose under leader while
        // follower is still zero, so prefer leader and only fall back to follower.
        std::vector<JointState> joints =
            parse_named_pose_from_position_dump_line(line, "leader", joint_count);
        if (!joints.empty()) return joints;
        joints = parse_named_pose_from_position_dump_line(line, "follower", joint_count);
        if (!joints.empty()) return joints;
    }
    return {};
}

std::vector<JointState> read_home_pose_from_csv(const std::string& path,
                                                const std::string& arm_side,
                                                size_t joint_count) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "[home-pose][WARN] failed to open: " << path << std::endl;
        return {};
    }

    std::vector<JointState> dump_pose =
        read_home_pose_from_position_dump(path, arm_side, joint_count);
    if (!dump_pose.empty()) return dump_pose;

    in.clear();
    in.seekg(0);
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(ss, cell, ',')) {
            try {
                values.push_back(std::stod(cell));
            } catch (...) {
                values.clear();
                break;
            }
        }
        if (values.size() == 1 + 2 * joint_count || values.size() == 1 + 3 * joint_count) {
            const size_t stride = (values.size() == 1 + 3 * joint_count) ? 3 : 2;
            std::vector<JointState> joints(joint_count);
            for (size_t i = 0; i < joint_count; ++i) {
                joints[i].position = values[1 + stride * i];
            }
            return joints;
        }
        if (values.size() == 1 + 2 * 16 || values.size() == 1 + 3 * 16) {
            const size_t stride = (values.size() == 1 + 3 * 16) ? 3 : 2;
            const size_t start = (arm_side == "left_arm") ? 0 : 8;
            std::vector<JointState> joints(joint_count);
            for (size_t i = 0; i < joint_count && start + i < 16; ++i) {
                joints[i].position = values[1 + stride * (start + i)];
            }
            return joints;
        }
    }
    std::cerr << "[home-pose][WARN] no valid first frame in: " << path << std::endl;
    return {};
}

class LeaderArmThread : public PeriodicTimerThread {
public:
    LeaderArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_l,
                    double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_l_(control_l) {}

protected:
    void before_start() override { std::cout << "leader start thread " << std::endl; }

    void after_stop() override { std::cout << "leader stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();

        control_l_->unilateral_step();

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        // std::cout << "[Leader] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_l_;
};

class FollowerArmThread : public PeriodicTimerThread {
public:
    FollowerArmThread(std::shared_ptr<RobotSystemState> robot_state, Control *control_f,
                      double hz = 500.0)
        : PeriodicTimerThread(hz), robot_state_(robot_state), control_f_(control_f) {}

protected:
    void before_start() override { std::cout << "follower start thread " << std::endl; }

    void after_stop() override { std::cout << "follower stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();

        control_f_->unilateral_step();

        auto now = std::chrono::steady_clock::now();

        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        // std::cout << "[Follower] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control *control_f_;
};

class AdminThread : public PeriodicTimerThread {
public:
    AdminThread(std::shared_ptr<RobotSystemState> leader_state,
                std::shared_ptr<RobotSystemState> follower_state, Control *control_l,
                Control *control_f, ReferenceSource source, UdpPanelReceiver *panel_receiver,
                CsvReplay *replay, CsvRecorder *recorder, UdpTextReceiver *record_start_receiver,
                TeleopMonitorSender *monitor_sender,
                std::string record_path, size_t recorder_joint_count,
                std::string arm_side_label, std::string position_dump_path,
                bool wait_for_replay_start, double live_ramp_duration,
                double live_velocity_scale, bool live_wait_for_start, double hz = 500.0)
        : PeriodicTimerThread(hz),
          leader_state_(leader_state),
          follower_state_(follower_state),
          control_l_(control_l),
          control_f_(control_f),
          source_(source),
          panel_receiver_(panel_receiver),
          replay_(replay),
          recorder_(recorder),
          record_start_receiver_(record_start_receiver),
          monitor_sender_(monitor_sender),
          record_path_(std::move(record_path)),
          recorder_joint_count_(recorder_joint_count),
          arm_side_label_(std::move(arm_side_label)),
          position_dump_path_(std::move(position_dump_path)),
          live_ramp_duration_(live_ramp_duration),
          live_velocity_scale_(live_velocity_scale),
          live_started_(!live_wait_for_start),
          replay_started_(!wait_for_replay_start) {}

protected:
    void before_start() override { std::cout << "admin start thread " << std::endl; }

    void after_stop() override { std::cout << "admin stop thread " << std::endl; }

    void on_timer() override {
        static auto prev_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();

        // get response
        auto leader_arm_resp = leader_state_->arm_state().get_all_responses();
        auto follower_arm_resp = follower_state_->arm_state().get_all_responses();

        auto leader_hand_resp = leader_state_->hand_state().get_all_responses();
        auto follower_hand_resp = follower_state_->hand_state().get_all_responses();

        handle_trigger_messages(leader_arm_resp, leader_hand_resp,
                                follower_arm_resp, follower_hand_resp);

        // set reference
        leader_state_->arm_state().set_all_references(follower_arm_resp);
        leader_state_->hand_state().set_all_references(follower_hand_resp);

        std::vector<JointState> follower_refs;
        follower_refs.reserve(follower_arm_resp.size() + follower_hand_resp.size());
        std::vector<JointState> recording_refs;

        if (source_ == ReferenceSource::Live) {
            if (!live_started_) {
                follower_refs.insert(follower_refs.end(), follower_arm_resp.begin(),
                                     follower_arm_resp.end());
                follower_refs.insert(follower_refs.end(), follower_hand_resp.begin(),
                                     follower_hand_resp.end());
                follower_refs = sanitize_live_reference(follower_refs, 0.0);
                recording_refs = follower_refs;
            } else {
                std::vector<JointState> live_refs;
                live_refs.reserve(leader_arm_resp.size() + leader_hand_resp.size());
                live_refs.insert(live_refs.end(), leader_arm_resp.begin(),
                                 leader_arm_resp.end());
                live_refs.insert(live_refs.end(), leader_hand_resp.begin(),
                                 leader_hand_resp.end());
                follower_refs = live_refs;
                follower_refs = apply_live_startup_ramp(follower_refs, follower_arm_resp,
                                                        follower_hand_resp, now);
                recording_refs = preserve_live_gripper_effort_for_recording(
                    follower_refs, live_refs);
            }
        } else if (source_ == ReferenceSource::Panel && panel_receiver_ != nullptr) {
            const size_t total = follower_arm_resp.size() + follower_hand_resp.size();
            auto panel_cmd = panel_receiver_->receive_latest(total);
            if (panel_cmd.has_value()) {
                last_external_ref_ = panel_cmd.value();
            }
            if (last_external_ref_.empty()) {
                last_external_ref_.insert(last_external_ref_.end(), follower_arm_resp.begin(),
                                          follower_arm_resp.end());
                last_external_ref_.insert(last_external_ref_.end(), follower_hand_resp.begin(),
                                          follower_hand_resp.end());
            }
            follower_refs = last_external_ref_;
            recording_refs = follower_refs;
        } else if (source_ == ReferenceSource::Replay && replay_ != nullptr) {
            if (last_external_ref_.empty()) {
                last_external_ref_.insert(last_external_ref_.end(), follower_arm_resp.begin(),
                                          follower_arm_resp.end());
                last_external_ref_.insert(last_external_ref_.end(), follower_hand_resp.begin(),
                                          follower_hand_resp.end());
            }
            if (replay_started_) {
                auto replay_cmd = replay_->sample();
                if (replay_cmd.has_value()) {
                    last_external_ref_ = replay_cmd.value();
                }
            }
            follower_refs = last_external_ref_;
            recording_refs = follower_refs;
        }

        if (!follower_refs.empty()) {
            follower_state_->set_all_references(follower_refs);
            if (recorder_ != nullptr && recorder_->active()) {
                recorder_->append(recording_refs.empty() ? follower_refs : recording_refs);
            }
        }
        send_monitor_sample(leader_arm_resp, leader_hand_resp, follower_refs,
                            follower_arm_resp, follower_hand_resp, now);
        auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(now - prev_time).count();
        prev_time = now;

        // std::cout << "[Admin] Period: " << elapsed_us << " us" << std::endl;
    }

private:
    void send_monitor_sample(const std::vector<JointState>& leader_arm_resp,
                             const std::vector<JointState>& leader_hand_resp,
                             const std::vector<JointState>& follower_refs,
                             const std::vector<JointState>& follower_arm_resp,
                             const std::vector<JointState>& follower_hand_resp,
                             std::chrono::steady_clock::time_point now) {
        if (monitor_sender_ == nullptr || now < next_monitor_time_) {
            return;
        }
        if (monitor_start_time_.time_since_epoch().count() == 0) {
            monitor_start_time_ = now;
        }
        const double t = std::chrono::duration<double>(now - monitor_start_time_).count();
        const bool gripper_contact_hold_active =
            control_f_ != nullptr && control_f_->GripperContactHoldActive();
        const bool gripper_slip_boost_active =
            control_f_ != nullptr && control_f_->GripperSlipBoostActive();
        monitor_sender_->send(arm_side_label_, t, leader_arm_resp, leader_hand_resp, follower_refs,
                              follower_arm_resp, follower_hand_resp, live_started_,
                              gripper_contact_hold_active, gripper_slip_boost_active);
        next_monitor_time_ = now + std::chrono::milliseconds(20);
    }

    void handle_trigger_messages(const std::vector<JointState>& leader_arm_resp,
                                 const std::vector<JointState>& leader_hand_resp,
                                 const std::vector<JointState>& follower_arm_resp,
                                 const std::vector<JointState>& follower_hand_resp) {
        if (record_start_receiver_ == nullptr) {
            return;
        }
        for (const auto& message : record_start_receiver_->receive_all()) {
            if (message.find("dump") != std::string::npos) {
                dump_positions(leader_arm_resp, leader_hand_resp,
                               follower_arm_resp, follower_hand_resp);
            }
            if (message.find("start") != std::string::npos) {
                maybe_start_live();
                maybe_start_recording();
                maybe_start_replay();
            }
            if (message.find("stop") != std::string::npos) {
                maybe_stop_recording();
            }
        }
    }

    void maybe_start_live() {
        if (source_ != ReferenceSource::Live || live_started_) {
            return;
        }
        live_started_ = true;
        live_ramp_initialized_ = false;
        std::cout << "[teleop] live follow started by trigger" << std::endl;
    }

    void maybe_start_replay() {
        if (source_ != ReferenceSource::Replay || replay_ == nullptr || replay_started_) {
            return;
        }
        replay_->reset();
        replay_started_ = true;
        std::cout << "[replay] started by synchronized trigger" << std::endl;
    }

    void maybe_start_recording() {
        if (recorder_ == nullptr || recorder_->active()) {
            return;
        }
        if (record_path_.empty()) {
            std::cerr << "[recorder] start requested but record path is empty" << std::endl;
            return;
        }
        const std::string segment_path = next_record_segment_path();
        if (recorder_->open(segment_path, recorder_joint_count_)) {
            std::cout << "[recorder] started by trigger: " << segment_path << std::endl;
        }
    }

    void maybe_stop_recording() {
        if (recorder_ == nullptr || !recorder_->active()) {
            return;
        }
        recorder_->close();
        std::cout << "[recorder] stopped by trigger" << std::endl;
    }

    std::string next_record_segment_path() {
        ++record_segment_index_;
        const std::filesystem::path base(record_path_);
        std::ostringstream suffix;
        suffix << "_" << std::setw(2) << std::setfill('0') << record_segment_index_;
        const std::filesystem::path segmented =
            base.parent_path() / (base.stem().string() + suffix.str() + base.extension().string());
        return segmented.string();
    }

    void dump_positions(const std::vector<JointState>& leader_arm_resp,
                        const std::vector<JointState>& leader_hand_resp,
                        const std::vector<JointState>& follower_arm_resp,
                        const std::vector<JointState>& follower_hand_resp) const {
        std::cout << std::fixed << std::setprecision(6);
        std::ostringstream line;
        line << "[positions] timestamp=" << current_timestamp()
             << " arm_side=" << arm_side_label_
             << " leader=" << format_positions(leader_arm_resp, leader_hand_resp)
             << " follower=" << format_positions(follower_arm_resp, follower_hand_resp);

        std::cout << line.str() << std::endl;

        if (!position_dump_path_.empty()) {
            std::ofstream out(position_dump_path_, std::ios::app);
            if (!out) {
                std::cerr << "[positions] failed to open dump file: "
                          << position_dump_path_ << std::endl;
                return;
            }
            out << line.str() << "\n";
        }
    }

    static std::string current_timestamp() {
        const auto now = std::chrono::system_clock::now();
        const auto time = std::chrono::system_clock::to_time_t(now);
        std::tm tm {};
        localtime_r(&time, &tm);
        std::ostringstream out;
        out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
        return out.str();
    }

    static std::string format_positions(const std::vector<JointState>& arm,
                                        const std::vector<JointState>& hand) {
        std::ostringstream out;
        out << "{";
        for (size_t i = 0; i < arm.size(); ++i) {
            if (i > 0) out << ", ";
            out << "arm_m" << (i + 1) << "=" << arm[i].position;
        }
        for (size_t i = 0; i < hand.size(); ++i) {
            if (!arm.empty() || i > 0) out << ", ";
            out << "gripper_m" << (i + 1) << "=" << hand[i].position;
        }
        out << "}";
        return out.str();
    }

    std::vector<JointState> apply_live_startup_ramp(
        const std::vector<JointState>& leader_refs,
        const std::vector<JointState>& follower_arm_resp,
        const std::vector<JointState>& follower_hand_resp,
        std::chrono::steady_clock::time_point now) {
        if (live_ramp_duration_ <= 0.0) {
            return sanitize_live_reference(leader_refs, live_velocity_scale_);
        }
        if (!live_ramp_initialized_) {
            live_ramp_start_time_ = now;
            live_ramp_start_ref_.clear();
            live_ramp_start_ref_.reserve(follower_arm_resp.size() + follower_hand_resp.size());
            live_ramp_start_ref_.insert(live_ramp_start_ref_.end(), follower_arm_resp.begin(),
                                        follower_arm_resp.end());
            live_ramp_start_ref_.insert(live_ramp_start_ref_.end(), follower_hand_resp.begin(),
                                        follower_hand_resp.end());
            for (auto& joint : live_ramp_start_ref_) {
                joint.velocity = 0.0;
                joint.effort = 0.0;
            }
            live_ramp_initialized_ = true;
            std::cout << "[teleop] live startup ramp duration=" << live_ramp_duration_
                      << "s" << std::endl;
        }

        const double elapsed =
            std::chrono::duration<double>(now - live_ramp_start_time_).count();
        const double alpha = quintic_smoothstep(elapsed / live_ramp_duration_);
        if (alpha >= 1.0 || live_ramp_start_ref_.size() != leader_refs.size()) {
            return sanitize_live_reference(leader_refs, live_velocity_scale_);
        }

        std::vector<JointState> ramped = leader_refs;
        for (size_t i = 0; i < ramped.size(); ++i) {
            const double delta = leader_refs[i].position - live_ramp_start_ref_[i].position;
            ramped[i].position = live_ramp_start_ref_[i].position + delta * alpha;
            ramped[i].velocity = leader_refs[i].velocity * alpha * live_velocity_scale_;
            ramped[i].effort = 0.0;
        }
        return ramped;
    }

    static std::vector<JointState> sanitize_live_reference(
        std::vector<JointState> refs, double velocity_scale) {
        for (auto& joint : refs) {
            joint.velocity *= velocity_scale;
            joint.effort = 0.0;
        }
        return refs;
    }

    static std::vector<JointState> preserve_live_gripper_effort_for_recording(
        std::vector<JointState> command_refs, const std::vector<JointState>& live_refs) {
        if (!command_refs.empty() && command_refs.size() == live_refs.size()) {
            command_refs.back().effort = live_refs.back().effort;
        }
        return command_refs;
    }

    std::shared_ptr<RobotSystemState> leader_state_;
    std::shared_ptr<RobotSystemState> follower_state_;
    Control *control_l_;
    Control *control_f_;
    ReferenceSource source_;
    UdpPanelReceiver *panel_receiver_;
    CsvReplay *replay_;
    CsvRecorder *recorder_;
    UdpTextReceiver *record_start_receiver_;
    TeleopMonitorSender *monitor_sender_;
    std::string record_path_;
    size_t recorder_joint_count_ = 0;
    int record_segment_index_ = 0;
    std::vector<JointState> last_external_ref_;
    std::string arm_side_label_;
    std::string position_dump_path_;
    double live_ramp_duration_ = 2.0;
    double live_velocity_scale_ = 0.0;
    bool live_started_ = true;
    bool live_ramp_initialized_ = false;
    std::chrono::steady_clock::time_point live_ramp_start_time_;
    std::vector<JointState> live_ramp_start_ref_;
    std::chrono::steady_clock::time_point monitor_start_time_;
    std::chrono::steady_clock::time_point next_monitor_time_;
    bool replay_started_ = true;
};

int main(int argc, char **argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::string leader_urdf_path;
        std::string follower_urdf_path;
        TeleopOptions options;

        if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
            print_usage(argv[0]);
            return 0;
        }

        if (argc < 3) {
            print_usage(argv[0]);
            return 1;
        }

        // Required: URDF paths
        leader_urdf_path = argv[1];
        follower_urdf_path = argv[2];

        std::vector<std::string> positional;
        for (int i = 3; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--reference-source" && i + 1 < argc) {
                if (!parse_reference_source(argv[++i], &options.reference_source)) {
                    std::cerr << "[ERROR] invalid reference source: " << argv[i] << std::endl;
                    return 1;
                }
            } else if (arg == "--panel-port" && i + 1 < argc) {
                options.panel_port = static_cast<uint16_t>(std::stoi(argv[++i]));
            } else if (arg == "--record" && i + 1 < argc) {
                options.record_path = argv[++i];
            } else if (arg == "--record-start-port" && i + 1 < argc) {
                options.record_start_port = static_cast<uint16_t>(std::stoi(argv[++i]));
            } else if (arg == "--replay-start-port" && i + 1 < argc) {
                options.replay_start_port = static_cast<uint16_t>(std::stoi(argv[++i]));
            } else if (arg == "--position-dump" && i + 1 < argc) {
                options.position_dump_path = argv[++i];
            } else if (arg == "--replay" && i + 1 < argc) {
                options.replay_path = argv[++i];
                options.reference_source = ReferenceSource::Replay;
            } else if (arg == "--transition" && i + 1 < argc) {
                options.generate_transitions = std::stoi(argv[++i]) != 0;
            } else if (arg == "--transition-duration" && i + 1 < argc) {
                options.replay_transition_duration = std::stod(argv[++i]);
            } else if (arg == "--initial-transition-duration" && i + 1 < argc) {
                options.replay_initial_transition_duration = std::stod(argv[++i]);
            } else if (arg == "--live-ramp-duration" && i + 1 < argc) {
                options.live_ramp_duration = std::stod(argv[++i]);
            } else if (arg == "--live-velocity-scale" && i + 1 < argc) {
                options.live_velocity_scale = std::stod(argv[++i]);
            } else if (arg == "--live-wait-for-start" && i + 1 < argc) {
                options.live_wait_for_start = std::stoi(argv[++i]) != 0;
            } else if (arg == "--monitor-port" && i + 1 < argc) {
                options.monitor_port = static_cast<uint16_t>(std::stoi(argv[++i]));
            } else if (arg == "--home-pose" && i + 1 < argc) {
                options.home_pose_path = argv[++i];
            } else if (arg == "--clean-transitions") {
                options.clean_transitions = true;
            } else if (arg == "--no-auto-home") {
                options.auto_home = false;
            } else if (arg == "--help" || arg == "-h") {
                print_usage(argv[0]);
                return 0;
            } else if (arg.rfind("--", 0) == 0) {
                std::cerr << "[ERROR] unknown option: " << arg << std::endl;
                print_usage(argv[0]);
                return 1;
            } else {
                positional.push_back(arg);
            }
        }

        if (positional.size() >= 1) options.arm_side = positional[0];
        if (positional.size() >= 2) options.leader_can_interface = positional[1];
        if (positional.size() >= 3) options.follower_can_interface = positional[2];
        if (positional.size() > 3) {
            std::cerr << "[ERROR] too many positional arguments" << std::endl;
            return 1;
        }
        if (options.arm_side != "left_arm" && options.arm_side != "right_arm") {
            std::cerr << "[ERROR] Invalid arm_side: " << options.arm_side
                      << ". Must be 'left_arm' or 'right_arm'." << std::endl;
            return 1;
        }

        // URDF file existence check
        if (!std::filesystem::exists(leader_urdf_path)) {
            std::cerr << "[ERROR] Leader URDF not found: " << leader_urdf_path << std::endl;
            return 1;
        }
        if (!std::filesystem::exists(follower_urdf_path)) {
            std::cerr << "[ERROR] Follower URDF not found: " << follower_urdf_path << std::endl;
            return 1;
        }

        // Setup dynamics
        std::string root_link = "openarm_body_link0";
        std::string leaf_link = select_leaf_link(follower_urdf_path, options.arm_side);

        // Output confirmation
        std::cout << "=== OpenArm Unilateral Control ===" << std::endl;
        std::cout << "Arm side         : " << options.arm_side << std::endl;
        std::cout << "Leader CAN       : " << options.leader_can_interface << std::endl;
        std::cout << "Follower CAN     : " << options.follower_can_interface << std::endl;
        std::cout << "Reference source : " << reference_source_name(options.reference_source)
                  << std::endl;
        std::cout << "Auto home        : " << (options.auto_home ? "on" : "off") << std::endl;
        std::cout << "Home pose        : "
                  << (options.home_pose_path.empty() ? std::string("INITIAL_POSITION")
                                                     : options.home_pose_path)
                  << std::endl;
        std::cout << "Live ramp        : " << options.live_ramp_duration << "s" << std::endl;
        std::cout << "Live velocity scale: " << options.live_velocity_scale << std::endl;
        std::cout << "Live wait start  : " << (options.live_wait_for_start ? "on" : "off")
                  << std::endl;
        std::cout << "Monitor port     : "
                  << (options.monitor_port == 0 ? std::string("off")
                                                 : std::to_string(options.monitor_port))
                  << std::endl;
        std::cout << "Leader URDF path : " << leader_urdf_path << std::endl;
        std::cout << "Follower URDF path: " << follower_urdf_path << std::endl;
        std::cout << "Root link         : " << root_link << std::endl;
        std::cout << "Leaf link         : " << leaf_link << std::endl;

        YamlLoader leader_loader("config/leader.yaml");
        YamlLoader follower_loader("config/follower.yaml");

        // Leader parameters
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        // Follower parameters
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        Dynamics *leader_arm_dynamics = new Dynamics(leader_urdf_path, root_link, leaf_link);
        leader_arm_dynamics->Init();

        Dynamics *follower_arm_dynamics = new Dynamics(follower_urdf_path, root_link, leaf_link);
        follower_arm_dynamics->Init();

        std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *leader_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(options.leader_can_interface, true);

        std::cout << "=== Initializing Follower OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm *follower_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(options.follower_can_interface,
                                                                 true);

        size_t leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
        size_t follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
        size_t leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
        size_t follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();

        std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
        std::cout << "follower arm motor num : " << follower_arm_motor_num << std::endl;
        std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
        std::cout << "follower hand motor num : " << follower_hand_motor_num << std::endl;

        // Declare robot_state (Joint and motor counts are assumed to be equal)
        std::shared_ptr<RobotSystemState> leader_state =
            std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);

        std::shared_ptr<RobotSystemState> follower_state =
            std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);

        Control *control_leader = new Control(
            leader_openarm, leader_arm_dynamics, follower_arm_dynamics, leader_state,
            1.0 / FREQUENCY, ROLE_LEADER, options.arm_side, leader_arm_motor_num,
            leader_hand_motor_num);
        Control *control_follower =
            new Control(follower_openarm, leader_arm_dynamics, follower_arm_dynamics,
                        follower_state, 1.0 / FREQUENCY, ROLE_FOLLOWER, options.arm_side,
                        follower_arm_motor_num, follower_hand_motor_num);

        control_leader->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv,
                                     leader_Fo);

        control_follower->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k,
                                       follower_Fv, follower_Fo);

        control_leader->SetSafetyFilter(std::make_unique<SafetyFilter>(
            make_default_safety_config(leader_arm_motor_num + leader_hand_motor_num)));
        control_follower->SetSafetyFilter(std::make_unique<SafetyFilter>(
            make_default_safety_config(follower_arm_motor_num + follower_hand_motor_num)));

        // Always seed MIT hold from measured encoders first. RobotSystemState defaults
        // to zeros; chasing INITIAL_POSITION={0..} on enable causes a hard startup jerk.
        control_leader->SeedHoldMeasured();
        control_follower->SeedHoldMeasured();
        std::cout << "[startup] hold seeded from measured motor pose (avoid zero-ref jerk)"
                  << std::endl;

        // set home position
        if (options.auto_home) {
            std::vector<JointState> home_pose;
            if (!options.home_pose_path.empty()) {
                home_pose = read_home_pose_from_csv(
                    options.home_pose_path, options.arm_side,
                    follower_arm_motor_num + follower_hand_motor_num);
            }
            if (home_pose.empty()) {
                std::cout << "[WARN] --auto-home without --home-pose: keep measured hold "
                             "(will NOT move to all-zero INITIAL_POSITION)"
                          << std::endl;
            } else {
                std::thread thread_l([&]() { control_leader->AdjustPositionTo(home_pose); });
                std::thread thread_f([&]() { control_follower->AdjustPositionTo(home_pose); });
                thread_l.join();
                thread_f.join();
            }
        } else {
            std::cout << "[WARN] --no-auto-home: keeping current robot positions as references"
                      << std::endl;
        }
        control_leader->ResetSafetyFilter(leader_state->get_all_references());
        control_follower->ResetSafetyFilter(follower_state->get_all_references());

        UdpPanelReceiver panel_receiver;
        if (options.reference_source == ReferenceSource::Panel) {
            if (!panel_receiver.open(options.panel_port)) return 1;
        }

        CsvReplay replay;
        if (options.reference_source == ReferenceSource::Replay) {
            if (options.replay_path.empty()) {
                std::cerr << "[ERROR] replay source requires --replay PATH" << std::endl;
                return 1;
            }
            if (options.clean_transitions && std::filesystem::is_directory(options.replay_path)) {
                CsvReplay::clean_transitions(options.replay_path);
            }
            bool loaded = false;
            if (std::filesystem::is_directory(options.replay_path)) {
                loaded = replay.load_directory(options.replay_path,
                                follower_arm_motor_num + follower_hand_motor_num,
                                options.generate_transitions,
                                options.replay_transition_duration);
            } else {
                loaded = replay.load(options.replay_path,
                            follower_arm_motor_num + follower_hand_motor_num);
            }
            if (!loaded) return 1;
            replay.prepend_transition_from(follower_state->get_all_references(),
                                           follower_arm_motor_num + follower_hand_motor_num,
                                           options.replay_initial_transition_duration);
        }

        CsvRecorder recorder;
        TeleopMonitorSender monitor_sender;
        TeleopMonitorSender *monitor_sender_ptr = nullptr;
        if (options.monitor_port != 0 && monitor_sender.open(options.monitor_port)) {
            monitor_sender_ptr = &monitor_sender;
        }

        UdpTextReceiver record_start_receiver;
        UdpTextReceiver *record_start_receiver_ptr = nullptr;
        if (options.reference_source == ReferenceSource::Replay && options.replay_start_port != 0) {
            if (!record_start_receiver.open(options.replay_start_port, "replay-start")) {
                return 1;
            }
            record_start_receiver_ptr = &record_start_receiver;
            std::cout << "[replay] waiting for start trigger on UDP port "
                      << options.replay_start_port << std::endl;
        } else if (!options.record_path.empty() && options.record_start_port != 0) {
            if (!record_start_receiver.open(options.record_start_port, "record-start")) {
                return 1;
            }
            record_start_receiver_ptr = &record_start_receiver;
            std::cout << "[recorder] waiting for start trigger on UDP port "
                      << options.record_start_port << std::endl;
        } else if (!options.record_path.empty()) {
            if (!recorder.open(options.record_path,
                               follower_arm_motor_num + follower_hand_motor_num)) {
                return 1;
            }
        }

        // Start control process
        LeaderArmThread leader_thread(leader_state, control_leader, FREQUENCY);
        FollowerArmThread follower_thread(follower_state, control_follower, FREQUENCY);
        AdminThread admin_thread(leader_state, follower_state, control_leader, control_follower,
                                 options.reference_source, &panel_receiver, &replay, &recorder,
                                 record_start_receiver_ptr, monitor_sender_ptr, options.record_path,
                                 follower_arm_motor_num + follower_hand_motor_num,
                                 options.arm_side, options.position_dump_path,
                                 options.reference_source == ReferenceSource::Replay &&
                                     options.replay_start_port != 0,
                                 options.live_ramp_duration, options.live_velocity_scale,
                                 options.live_wait_for_start,
                                 FREQUENCY);

        leader_thread.start_thread();
        follower_thread.start_thread();
        admin_thread.start_thread();

        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        leader_thread.stop_thread();
        follower_thread.stop_thread();
        admin_thread.stop_thread();
        recorder.close();

        leader_openarm->disable_all();
        follower_openarm->disable_all();

    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
