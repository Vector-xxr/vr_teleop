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

#include <common.hpp>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <controller/tcp_approach_stream.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <io/teleop_io.hpp>
#include <iostream>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm_port/openarm_init.hpp>
#include <optional>
#include <robot_state.hpp>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>
#include <yamlloader.hpp>

namespace {

struct Options {
    std::string urdf_path;
    std::string replay_path;
    std::string left_follower_can = "follower_left";
    std::string right_follower_can = "follower_right";
    bool auto_home = true;
    bool generate_transitions = true;
    bool clean_transitions = false;
    bool damping_release_on_exit = true;
    double initial_transition_duration = 0.0;
    double transition_duration = 5.0;
    double start_delay = 1.5;
    double speed = 1.0;
    double init_hold_duration = 0.5;
    double startup_gain_ramp_duration = 1.0;
    double startup_gain_ramp_start_scale = 0.1;
    double torque_recovery_speed = 0.25;
    double torque_recovery_min_duration = 3.0;
    std::string startup_pose_path;
    std::string home_pose_path;
    uint16_t torque_telemetry_port = 0;
    uint16_t command_port = 0;
    uint16_t status_port = 0;
    // Endurance / motor health CSV: torque + MOS/rotor temp for all 16 motors.
    std::string motor_log_path;
    double motor_log_hz = 100.0;
};

std::string select_leaf_link(const std::string& urdf_path, const std::string& side) {
    const std::vector<std::string> candidates = {
        "openarm_" + side + "_hand",
        "openarm_" + side + "_hand_tcp",
        "openarm_" + side + "_link8",
        "openarm_" + side + "_link7",
    };

    std::ifstream file(urdf_path);
    if (!file) return candidates.front();
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string content = buffer.str();
    for (const auto& candidate : candidates) {
        if (content.find("link name=\"" + candidate + "\"") != std::string::npos) {
            return candidate;
        }
    }
    return candidates.front();
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " --urdf URDF --replay MERGED_16_JOINT_CSV "
                 "[--left-follower-can IFACE] [--right-follower-can IFACE]\n"
              << "Options:\n"
              << "  --initial-transition-duration SEC   current pose to first frame; 0 = adaptive\n"
              << "  --transition-duration SEC           transition between merged action files\n"
              << "  --transition 0|1                    generate transitions between action files\n"
              << "  --clean-transitions                 delete generated transition files in replay dir\n"
              << "  --start-delay SEC                    wait after init before replay starts\n"
              << "  --speed SCALE                        replay time scale, e.g. 0.5 or 4.0\n"
              << "  --init-hold-duration SEC             hold current feedback pose during startup init\n"
              << "  --startup-gain-ramp-duration SEC     ramp follower Kp/Kd while holding startup pose\n"
              << "  --startup-gain-ramp-start-scale S    initial Kp/Kd scale for startup ramp\n"
              << "  --torque-recovery-speed RAD_PER_SEC  slow global recovery speed after torque limit\n"
              << "  --torque-recovery-min-duration SEC   minimum smooth recovery duration\n"
              << "  --startup-pose PATH                  position dump used before startup home pose\n"
              << "  --home-pose PATH                     position dump file used for startup home pose\n"
              << "  --torque-telemetry-port PORT         UDP port for 16 motor torque telemetry\n"
              << "  --motor-log PATH                     CSV log: 16x torque + MOS/rotor temp (+fault)\n"
              << "  --motor-log-hz HZ                    motor log sample rate (default 100)\n"
              << "  --command-port PORT                  keep process alive and receive replay commands\n"
              << "  --status-port PORT                   UDP port for replay server status messages\n"
              << "  --no-damping-release-on-exit         exit without Kd-only gravity release\n"
              << "  --no-auto-home                       skip startup AdjustPosition\n";
}

bool parse_args(int argc, char** argv, Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--urdf" && i + 1 < argc) {
            options->urdf_path = argv[++i];
        } else if (arg == "--replay" && i + 1 < argc) {
            options->replay_path = argv[++i];
        } else if (arg == "--left-follower-can" && i + 1 < argc) {
            options->left_follower_can = argv[++i];
        } else if (arg == "--right-follower-can" && i + 1 < argc) {
            options->right_follower_can = argv[++i];
        } else if (arg == "--initial-transition-duration" && i + 1 < argc) {
            options->initial_transition_duration = std::stod(argv[++i]);
        } else if (arg == "--transition-duration" && i + 1 < argc) {
            options->transition_duration = std::stod(argv[++i]);
        } else if (arg == "--transition" && i + 1 < argc) {
            options->generate_transitions = std::stoi(argv[++i]) != 0;
        } else if (arg == "--clean-transitions") {
            options->clean_transitions = true;
        } else if (arg == "--start-delay" && i + 1 < argc) {
            options->start_delay = std::stod(argv[++i]);
        } else if (arg == "--speed" && i + 1 < argc) {
            options->speed = std::stod(argv[++i]);
        } else if (arg == "--init-hold-duration" && i + 1 < argc) {
            options->init_hold_duration = std::stod(argv[++i]);
        } else if (arg == "--startup-gain-ramp-duration" && i + 1 < argc) {
            options->startup_gain_ramp_duration = std::stod(argv[++i]);
        } else if (arg == "--startup-gain-ramp-start-scale" && i + 1 < argc) {
            options->startup_gain_ramp_start_scale = std::stod(argv[++i]);
        } else if (arg == "--torque-recovery-speed" && i + 1 < argc) {
            options->torque_recovery_speed = std::stod(argv[++i]);
        } else if (arg == "--torque-recovery-min-duration" && i + 1 < argc) {
            options->torque_recovery_min_duration = std::stod(argv[++i]);
        } else if (arg == "--startup-pose" && i + 1 < argc) {
            options->startup_pose_path = argv[++i];
        } else if (arg == "--home-pose" && i + 1 < argc) {
            options->home_pose_path = argv[++i];
        } else if (arg == "--torque-telemetry-port" && i + 1 < argc) {
            options->torque_telemetry_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--motor-log" && i + 1 < argc) {
            options->motor_log_path = argv[++i];
        } else if (arg == "--motor-log-hz" && i + 1 < argc) {
            options->motor_log_hz = std::stod(argv[++i]);
        } else if (arg == "--command-port" && i + 1 < argc) {
            options->command_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--status-port" && i + 1 < argc) {
            options->status_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--no-damping-release-on-exit") {
            options->damping_release_on_exit = false;
        } else if (arg == "--no-auto-home") {
            options->auto_home = false;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "[ERROR] unknown or incomplete option: " << arg << std::endl;
            print_usage(argv[0]);
            return false;
        }
    }

    if (options->urdf_path.empty() ||
        (options->replay_path.empty() && options->command_port == 0)) {
        print_usage(argv[0]);
        return false;
    }
    return true;
}

std::vector<double> parse_positions_from_line(const std::string& line,
                                              const std::string& pose_name) {
    const std::string marker = pose_name + "={";
    const size_t begin = line.find(marker);
    if (begin == std::string::npos) return {};
    const size_t values_begin = begin + marker.size();
    const size_t end = line.find('}', values_begin);
    if (end == std::string::npos) return {};

    std::vector<double> values;
    std::stringstream stream(line.substr(values_begin, end - values_begin));
    std::string item;
    while (std::getline(stream, item, ',')) {
        const size_t eq = item.find('=');
        if (eq == std::string::npos) continue;
        values.push_back(std::stod(item.substr(eq + 1)));
    }
    return values;
}

bool load_pose_file(const std::string& path, const std::string& preferred_pose_name,
                    std::vector<JointState>* left, std::vector<JointState>* right) {
    if (path.empty() || left == nullptr || right == nullptr) return false;
    std::ifstream file(path);
    if (!file) {
        std::cerr << "[dual-replay][WARN] home pose file not found: " << path << std::endl;
        return false;
    }

    std::unordered_map<std::string, std::vector<double>> side_positions;
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("arm_side=left_arm") != std::string::npos) {
            side_positions["left"] = parse_positions_from_line(line, preferred_pose_name);
            if (side_positions["left"].empty() && preferred_pose_name != "follower") {
                side_positions["left"] = parse_positions_from_line(line, "follower");
            }
        } else if (line.find("arm_side=right_arm") != std::string::npos) {
            side_positions["right"] = parse_positions_from_line(line, preferred_pose_name);
            if (side_positions["right"].empty() && preferred_pose_name != "follower") {
                side_positions["right"] = parse_positions_from_line(line, "follower");
            }
        }
    }

    const auto left_it = side_positions.find("left");
    const auto right_it = side_positions.find("right");
    if (left_it == side_positions.end() || right_it == side_positions.end() ||
        left_it->second.size() != left->size() || right_it->second.size() != right->size()) {
        std::cerr << "[dual-replay][WARN] invalid home pose file: " << path << std::endl;
        return false;
    }

    for (size_t i = 0; i < left->size(); ++i) {
        (*left)[i].position = left_it->second[i];
        (*left)[i].velocity = 0.0;
        (*left)[i].effort = 0.0;
    }
    for (size_t i = 0; i < right->size(); ++i) {
        (*right)[i].position = right_it->second[i];
        (*right)[i].velocity = 0.0;
        (*right)[i].effort = 0.0;
    }
    return true;
}

bool load_home_pose_file(const std::string& path, std::vector<JointState>* left,
                         std::vector<JointState>* right) {
    return load_pose_file(path, "follower", left, right);
}

bool load_startup_pose_file(const std::string& path, std::vector<JointState>* left,
                            std::vector<JointState>* right) {
    return load_pose_file(path, "leader", left, right);
}

std::vector<JointState> slice_joints(const std::vector<JointState>& joints, size_t offset,
                                     size_t count) {
    if (offset + count > joints.size()) {
        throw std::runtime_error("merged replay frame does not contain enough joints");
    }
    return {joints.begin() + static_cast<std::ptrdiff_t>(offset),
            joints.begin() + static_cast<std::ptrdiff_t>(offset + count)};
}

std::vector<JointState> concat_joints(const std::vector<JointState>& left,
                                      const std::vector<JointState>& right) {
    std::vector<JointState> combined;
    combined.reserve(left.size() + right.size());
    combined.insert(combined.end(), left.begin(), left.end());
    combined.insert(combined.end(), right.begin(), right.end());
    return combined;
}

std::vector<JointState> zero_velocity_hold_frame(const std::vector<JointState>& joints) {
    std::vector<JointState> hold = joints;
    for (auto& joint : hold) {
        joint.velocity = 0.0;
        joint.effort = 0.0;
    }
    return hold;
}

double smoothstep5(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) + 6.0 * std::pow(s, 5);
}

std::vector<double> scaled_vector(const std::vector<double>& values, double scale) {
    std::vector<double> scaled = values;
    for (auto& value : scaled) value *= scale;
    return scaled;
}

std::vector<JointState> read_openarm_joint_feedback(
    openarm::can::socket::OpenArm* openarm, size_t arm_motor_num, size_t hand_motor_num) {
    if (openarm == nullptr) return {};
    openarm->recv_all(200);

    std::vector<JointState> joints;
    joints.reserve(arm_motor_num + hand_motor_num);
    for (const auto& motor : openarm->get_arm().get_motors()) {
        joints.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    for (const auto& motor : openarm->get_gripper().get_motors()) {
        joints.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    return joints;
}

void capture_and_hold_current_pose(Control& control, openarm::can::socket::OpenArm* openarm,
                                   std::shared_ptr<RobotSystemState> state,
                                   size_t arm_motor_num, size_t hand_motor_num) {
    const std::vector<JointState> current =
        read_openarm_joint_feedback(openarm, arm_motor_num, hand_motor_num);
    if (current.size() != arm_motor_num + hand_motor_num) {
        throw std::runtime_error("failed to read current OpenArm joint feedback");
    }
    const std::vector<JointState> hold = zero_velocity_hold_frame(current);
    state->set_all_responses(current);
    state->set_all_references(hold);
    control.ResetSafetyFilter(hold);
}

std::vector<JointState> capture_and_hold_both_current_pose(
    Control& left_control, Control& right_control,
    openarm::can::socket::OpenArm* left_openarm,
    openarm::can::socket::OpenArm* right_openarm,
    std::shared_ptr<RobotSystemState> left_state,
    std::shared_ptr<RobotSystemState> right_state,
    size_t left_arm_motor_num, size_t left_hand_motor_num,
    size_t right_arm_motor_num, size_t right_hand_motor_num,
    double hold_duration) {
    capture_and_hold_current_pose(left_control, left_openarm, left_state, left_arm_motor_num,
                                  left_hand_motor_num);
    capture_and_hold_current_pose(right_control, right_openarm, right_state,
                                  right_arm_motor_num, right_hand_motor_num);

    const std::vector<JointState> hold_frame =
        concat_joints(left_state->get_all_references(), right_state->get_all_references());
    const int period_us = static_cast<int>(1e6 / FREQUENCY);
    const auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    while (keep_running) {
        left_state->set_all_references(slice_joints(hold_frame, 0, left_state->get_total_joint_count()));
        right_state->set_all_references(slice_joints(hold_frame, left_state->get_total_joint_count(),
                                                     right_state->get_total_joint_count()));
        left_control.unilateral_step();
        right_control.unilateral_step();
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= std::max(0.0, hold_duration)) break;
        next_time += std::chrono::microseconds(period_us);
        std::this_thread::sleep_until(next_time);
    }
    return hold_frame;
}

void ramp_both_followers_to_nominal_gains(
    Control& left_control, Control& right_control,
    std::shared_ptr<RobotSystemState> left_state,
    std::shared_ptr<RobotSystemState> right_state,
    const std::vector<double>& kp, const std::vector<double>& kd,
    const std::vector<double>& fc, const std::vector<double>& k,
    const std::vector<double>& fv, const std::vector<double>& fo,
    double duration, double start_scale) {
    const double safe_duration = std::max(0.0, duration);
    const double safe_start_scale = std::clamp(start_scale, 0.0, 1.0);
    const int period_us = static_cast<int>(1e6 / FREQUENCY);
    const auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;

    while (keep_running) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        const double alpha = safe_duration > 1e-6 ? smoothstep5(elapsed / safe_duration) : 1.0;
        const double scale = safe_start_scale + (1.0 - safe_start_scale) * alpha;
        const std::vector<double> ramp_kp = scaled_vector(kp, scale);
        const std::vector<double> ramp_kd = scaled_vector(kd, scale);
        left_control.SetParameter(ramp_kp, ramp_kd, fc, k, fv, fo);
        right_control.SetParameter(ramp_kp, ramp_kd, fc, k, fv, fo);

        // During soft takeover, track the latest feedback as the reference so gravity
        // sag does not accumulate position error that gets pulled back as gains rise.
        const std::vector<JointState> left_hold =
            zero_velocity_hold_frame(left_state->get_all_responses());
        const std::vector<JointState> right_hold =
            zero_velocity_hold_frame(right_state->get_all_responses());
        left_state->set_all_references(left_hold);
        right_state->set_all_references(right_hold);
        left_control.ResetSafetyFilter(left_hold);
        right_control.ResetSafetyFilter(right_hold);
        left_control.unilateral_step();
        right_control.unilateral_step();
        if (elapsed >= safe_duration) break;
        next_time += std::chrono::microseconds(period_us);
        std::this_thread::sleep_until(next_time);
    }

    left_control.SetParameter(kp, kd, fc, k, fv, fo);
    right_control.SetParameter(kp, kd, fc, k, fv, fo);
    const std::vector<JointState> left_final =
        zero_velocity_hold_frame(left_state->get_all_responses());
    const std::vector<JointState> right_final =
        zero_velocity_hold_frame(right_state->get_all_responses());
    left_state->set_all_references(left_final);
    right_state->set_all_references(right_final);
    left_control.ResetSafetyFilter(left_final);
    right_control.ResetSafetyFilter(right_final);
}

std::vector<std::string> split_text(const std::string& text, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, delimiter)) {
        parts.push_back(part);
    }
    return parts;
}

struct ReplayCommand {
    std::string path;
    uint16_t telemetry_port = 0;
    std::string label;
    std::string side;
    double speed = 1.0;
};

std::optional<ReplayCommand> parse_replay_command(const std::string& text) {
    const auto parts = split_text(text, '|');
    if (parts.size() < 4 || parts[0] != "play") return std::nullopt;
    try {
        const int port = std::stoi(parts[2]);
        if (port < 0 || port > 65535) {
            std::cerr << "[dual-replay-server][WARN] invalid telemetry port: "
                      << parts[2] << std::endl;
            return std::nullopt;
        }
        ReplayCommand command;
        command.path = parts[1];
        command.telemetry_port = static_cast<uint16_t>(port);
        command.label = parts[3];
        if (parts.size() >= 5) command.side = parts[4];
        if (command.side != "" && command.side != "left" && command.side != "right") {
            std::cerr << "[dual-replay-server][WARN] invalid side: "
                      << command.side << std::endl;
            return std::nullopt;
        }
        if (parts.size() >= 6 && !parts[5].empty()) {
            command.speed = std::stod(parts[5]);
            if (!std::isfinite(command.speed) || command.speed <= 0.0 ||
                command.speed > 20.0) {
                std::cerr << "[dual-replay-server][WARN] invalid speed: "
                          << parts[5] << std::endl;
                return std::nullopt;
            }
        }
        return command;
    } catch (const std::exception& e) {
        std::cerr << "[dual-replay-server][WARN] invalid command ignored: "
                  << text << " (" << e.what() << ")" << std::endl;
        return std::nullopt;
    }
}

bool write_dual_position_dump(const std::string& path,
                              const std::vector<JointState>& left,
                              const std::vector<JointState>& right,
                              size_t left_total,
                              size_t right_total) {
    if (path.empty()) return false;
    std::filesystem::path output(path);
    if (output.has_parent_path()) {
        std::filesystem::create_directories(output.parent_path());
    }
    std::ofstream out(output, std::ios::app);
    if (!out) {
        std::cerr << "[dual-replay-server][dump] failed to open: " << path << std::endl;
        return false;
    }
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()).count();

    auto write_side = [&](const std::string& side,
                          const std::vector<JointState>& joints,
                          size_t total) {
        out << "[positions] timestamp=" << seconds
            << " arm_side=" << side
            << " leader={";
        for (size_t i = 0; i < 7 && i < total; ++i) {
            if (i > 0) out << ", ";
            out << "arm_m" << (i + 1) << "=" << joints[i].position;
        }
        const double gripper = total > 7 ? joints[7].position : 0.0;
        out << ", gripper_m1=" << gripper << "} follower={";
        for (size_t i = 0; i < 7 && i < total; ++i) {
            if (i > 0) out << ", ";
            out << "arm_m" << (i + 1) << "=" << joints[i].position;
        }
        out << ", gripper_m1=" << gripper << "}\n";
    };

    write_side("left_arm", left, left_total);
    write_side("right_arm", right, right_total);
    std::cout << "[dual-replay-server][dump] wrote feedback pose to " << path << std::endl;
    return true;
}

bool write_single_arm_as_merged_csv(const std::string& input_path, const std::string& output_path,
                                    const std::string& side,
                                    const std::vector<JointState>& hold_left,
                                    const std::vector<JointState>& hold_right) {
    if (side != "left" && side != "right") return false;
    std::ifstream in(input_path);
    if (!in) return false;
    std::ofstream out(output_path);
    if (!out) return false;

    std::string line;
    std::getline(in, line);
    out << "time";
    for (size_t i = 0; i < 16; ++i) out << ",q" << i << ",dq" << i << ",tau" << i;
    out << "\n";

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream stream(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(stream, cell, ',')) {
            try {
                values.push_back(std::stod(cell));
            } catch (...) {
                values.clear();
                break;
            }
        }
        const bool has_effort = values.size() == 25;
        if (values.size() != 17 && !has_effort) continue;
        out << values[0];
        for (size_t i = 0; i < 8; ++i) {
            if (side == "left") {
                const size_t stride = has_effort ? 3 : 2;
                const double effort = has_effort ? values[1 + stride * i + 2] : 0.0;
                out << "," << values[1 + stride * i] << "," << values[1 + stride * i + 1]
                    << "," << effort;
            } else {
                out << "," << hold_left[i].position << ",0,0";
            }
        }
        for (size_t i = 0; i < 8; ++i) {
            if (side == "right") {
                const size_t stride = has_effort ? 3 : 2;
                const double effort = has_effort ? values[1 + stride * i + 2] : 0.0;
                out << "," << values[1 + stride * i] << "," << values[1 + stride * i + 1]
                    << "," << effort;
            } else {
                out << "," << hold_right[i].position << ",0,0";
            }
        }
        out << "\n";
    }
    return true;
}

bool feedback_safety_limit_exceeded(const std::vector<JointState>& left,
                                    const std::vector<JointState>& right,
                                    std::string* reason) {
    std::vector<std::string> triggers;
    const auto inspect = [&triggers](const char* side,
                                     const std::vector<JointState>& joints) {
        for (size_t i = 0; i < joints.size(); ++i) {
            const size_t idx = std::min(i, static_cast<size_t>(NJOINTS - 1));
            const double effort_threshold = effort_limit_F[idx];
            const double effort = joints[i].effort;
            if (effort_threshold > 0.0 && std::isfinite(effort) &&
                std::abs(effort) > effort_threshold) {
                std::ostringstream oss;
                oss << side << "-J" << (i + 1) << " feedback effort " << effort
                    << " Nm > " << effort_threshold << " Nm";
                triggers.push_back(oss.str());
            }

            const double velocity_threshold = velocity_limit_F[idx];
            const double velocity = joints[i].velocity;
            if (velocity_threshold > 0.0 && std::isfinite(velocity) &&
                std::abs(velocity) > velocity_threshold) {
                std::ostringstream oss;
                oss << side << "-J" << (i + 1) << " feedback velocity " << velocity
                    << " rad/s > " << velocity_threshold << " rad/s";
                triggers.push_back(oss.str());
            }
        }
    };
    inspect("L", left);
    inspect("R", right);
    if (triggers.empty()) return false;

    if (reason != nullptr) {
        std::ostringstream oss;
        oss << "feedback safety triggered by ";
        for (size_t i = 0; i < triggers.size(); ++i) {
            if (i > 0) oss << "; ";
            oss << triggers[i];
        }
        *reason = oss.str();
    }
    return true;
}

double compute_recovery_duration(const std::vector<JointState>& start,
                                 const std::vector<JointState>& goal,
                                 double speed, double min_duration) {
    double max_delta = 0.0;
    const size_t count = std::min(start.size(), goal.size());
    for (size_t i = 0; i < count; ++i) {
        max_delta = std::max(max_delta, std::abs(goal[i].position - start[i].position));
    }
    const double safe_speed = std::max(speed, 1e-3);
    return std::max(min_duration, max_delta / safe_speed);
}

std::vector<JointState> interpolate_recovery_frame(const std::vector<JointState>& start,
                                                   const std::vector<JointState>& goal,
                                                   double elapsed,
                                                   double duration) {
    std::vector<JointState> frame = goal;
    const double s = std::clamp(elapsed / std::max(duration, 1e-6), 0.0, 1.0);
    const double blend = 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) +
                         6.0 * std::pow(s, 5);
    const double blend_dot = (30.0 * std::pow(s, 2) - 60.0 * std::pow(s, 3) +
                              30.0 * std::pow(s, 4)) /
                             std::max(duration, 1e-6);
    const size_t count = std::min(start.size(), goal.size());
    for (size_t i = 0; i < count; ++i) {
        const double delta = goal[i].position - start[i].position;
        frame[i].position = start[i].position + delta * blend;
        frame[i].velocity = delta * blend_dot;
        frame[i].effort = 0.0;
    }
    return frame;
}

void move_both_arms_smoothly(Control& left_control, Control& right_control,
                             std::shared_ptr<RobotSystemState> left_state,
                             std::shared_ptr<RobotSystemState> right_state,
                             const std::vector<JointState>& goal_frame,
                             size_t left_total, size_t right_total,
                             double speed, double min_duration,
                             bool allow_during_shutdown = false) {
    const std::vector<JointState> start_frame =
        concat_joints(left_state->get_all_responses(), right_state->get_all_responses());
    const double duration = compute_recovery_duration(start_frame, goal_frame, speed, min_duration);
    std::cout << "[dual-replay] smooth home pose duration: " << duration << "s" << std::endl;

    left_control.ResetSafetyFilter(slice_joints(start_frame, 0, left_total));
    right_control.ResetSafetyFilter(slice_joints(start_frame, left_total, right_total));

    const int period_us = static_cast<int>(1e6 / FREQUENCY);
    const auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    while (keep_running || allow_during_shutdown) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_time).count();
        const std::vector<JointState> frame =
            interpolate_recovery_frame(start_frame, goal_frame, elapsed, duration);
        left_state->set_all_references(slice_joints(frame, 0, left_total));
        right_state->set_all_references(slice_joints(frame, left_total, right_total));
        left_control.unilateral_step();
        right_control.unilateral_step();
        if (elapsed >= duration) break;
        next_time += std::chrono::microseconds(period_us);
        std::this_thread::sleep_until(next_time);
    }
}

void send_kd_only_release(openarm::can::socket::OpenArm* left_openarm,
                          openarm::can::socket::OpenArm* right_openarm,
                          const std::vector<double>& kd,
                          double duration_sec = -1.0) {
    // Kp=0, tau_ff=0, hold measured q with Kd damping so gravity lowers the arm slowly.
    double duration = duration_sec;
    if (duration < 0.0) {
        const char* env = std::getenv("OPENARM_REPLAY_DAMP_SEC");
        if (env != nullptr && *env != '\0') {
            try {
                duration = std::stod(env);
            } catch (...) {
                duration = 6.0;
            }
        } else {
            duration = 6.0;
        }
    }
    duration = std::clamp(duration, 0.2, 60.0);
    const int period_ms = 5;
    const int cycles = std::max(1, static_cast<int>(std::lround(duration * 1000.0 / period_ms)));

    // Optionally scale Kd for a softer/firmer fall.
    double kd_scale = 1.0;
    if (const char* env = std::getenv("OPENARM_REPLAY_DAMP_KD_SCALE"); env && *env) {
        try {
            kd_scale = std::stod(env);
        } catch (...) {
            kd_scale = 1.0;
        }
    }
    kd_scale = std::clamp(kd_scale, 0.05, 5.0);

    const auto send_one_arm = [&kd, kd_scale](openarm::can::socket::OpenArm* openarm) {
        if (openarm == nullptr) return;
        openarm->recv_all(200);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(openarm->get_arm().get_motors().size());
        size_t idx = 0;
        for (const auto& motor : openarm->get_arm().get_motors()) {
            const double damping = (idx < kd.size() ? kd[idx] : 0.35) * kd_scale;
            arm_cmds.push_back({0.0, damping, motor.get_position(), 0.0, 0.0});
            ++idx;
        }

        std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
        gripper_cmds.reserve(openarm->get_gripper().get_motors().size());
        for (const auto& motor : openarm->get_gripper().get_motors()) {
            const double damping = (idx < kd.size() ? kd[idx] : 0.35) * kd_scale;
            gripper_cmds.push_back({0.0, damping, motor.get_position(), 0.0, 0.0});
            ++idx;
        }

        openarm->get_arm().mit_control_all(arm_cmds);
        openarm->get_gripper().mit_control_all(gripper_cmds);
    };

    std::cout << "[dual-replay] Kd-only damping release for " << duration
              << " s (Kp=0, tau_ff=0, kd_scale=" << kd_scale
              << "); motors stay enabled so arms settle under gravity" << std::endl;
    for (int i = 0; i < cycles; ++i) {
        send_one_arm(left_openarm);
        send_one_arm(right_openarm);
        if (left_openarm != nullptr) left_openarm->recv_all(200);
        if (right_openarm != nullptr) right_openarm->recv_all(200);
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
}

void sleep_until_next(std::chrono::steady_clock::time_point* next_time, int period_us) {
    *next_time += std::chrono::microseconds(period_us);
    std::this_thread::sleep_until(*next_time);
}

std::string format_action_event(const CsvReplay::ActionEvent& event) {
    std::ostringstream detail;
    detail << "label=" << event.label;
    if (!event.face_emoji.empty()) {
        detail << "|face_emoji=" << event.face_emoji;
    }
    return detail.str();
}

enum class MotorTelemetryKind {
    ErrorCode,
    MosTemp,
    RotorTemp,
};

int motor_telemetry_value(const openarm::damiao_motor::Motor& motor,
                          MotorTelemetryKind kind) {
    switch (kind) {
        case MotorTelemetryKind::ErrorCode:
            return motor.get_error_code();
        case MotorTelemetryKind::MosTemp:
            return motor.get_state_tmos();
        case MotorTelemetryKind::RotorTemp:
            return motor.get_state_trotor();
    }
    return 0;
}

std::vector<int> collect_arm_motor_telemetry(openarm::can::socket::OpenArm* openarm,
                                             MotorTelemetryKind kind) {
    std::vector<int> values;
    values.reserve(8);
    if (openarm != nullptr) {
        for (const auto& motor : openarm->get_arm().get_motors()) {
            values.push_back(motor_telemetry_value(motor, kind));
        }
        for (const auto& motor : openarm->get_gripper().get_motors()) {
            values.push_back(motor_telemetry_value(motor, kind));
        }
    }
    if (values.size() < 8) values.resize(8, 0);
    if (values.size() > 8) values.resize(8);
    return values;
}

std::vector<int> collect_dual_motor_telemetry(openarm::can::socket::OpenArm* left_openarm,
                                              openarm::can::socket::OpenArm* right_openarm,
                                              MotorTelemetryKind kind) {
    std::vector<int> values = collect_arm_motor_telemetry(left_openarm, kind);
    const auto right = collect_arm_motor_telemetry(right_openarm, kind);
    values.insert(values.end(), right.begin(), right.end());
    return values;
}

const char* motor_fault_text(int code) {
    switch (code) {
        case 8:
            return "over-voltage";
        case 9:
            return "under-voltage";
        case 10:
            return "over-current";
        case 11:
            return "MOS over-temperature";
        case 12:
            return "rotor over-temperature";
        case 13:
            return "communication lost";
        case 14:
            return "overload";
        default:
            return "unknown";
    }
}

std::string motor_channel_name(size_t idx) {
    const char side = idx < 8 ? 'L' : 'R';
    const size_t local = idx < 8 ? idx : idx - 8;
    std::ostringstream name;
    name << side << "-";
    if (local < 7) {
        name << "J" << (local + 1);
    } else {
        name << "Grip";
    }
    return name.str();
}

bool is_motor_fault_code(int code) {
    return code >= 8 && code <= 14;
}

class MotorFaultPrinter {
public:
    void check(openarm::can::socket::OpenArm* left_openarm,
               openarm::can::socket::OpenArm* right_openarm) {
        const auto codes = collect_dual_motor_telemetry(left_openarm, right_openarm,
                                                        MotorTelemetryKind::ErrorCode);
        const auto mos_temps = collect_dual_motor_telemetry(left_openarm, right_openarm,
                                                            MotorTelemetryKind::MosTemp);
        const auto rotor_temps = collect_dual_motor_telemetry(left_openarm, right_openarm,
                                                              MotorTelemetryKind::RotorTemp);
        const auto now = std::chrono::steady_clock::now();
        const bool periodic =
            last_print_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_print_).count() >= 2.0;
        bool printed = false;
        if (last_codes_.size() != codes.size()) last_codes_.assign(codes.size(), 0);
        for (size_t i = 0; i < codes.size(); ++i) {
            if (!is_motor_fault_code(codes[i])) continue;
            if (!periodic && codes[i] == last_codes_[i]) continue;
            std::cout << "[motor-fault] " << motor_channel_name(i)
                      << " code=" << codes[i]
                      << " (" << motor_fault_text(codes[i]) << ")"
                      << " t_mos=" << mos_temps[i]
                      << "C t_rotor=" << rotor_temps[i] << "C"
                      << std::endl;
            printed = true;
        }
        last_codes_ = codes;
        if (printed) last_print_ = now;
    }

private:
    std::vector<int> last_codes_;
    std::chrono::steady_clock::time_point last_print_;
};

class MotorTelemetryCsvLogger {
public:
    bool open(const std::string& path, double hz) {
        close();
        if (path.empty() || path == "0" || path == "off" || path == "false") {
            return false;
        }
        try {
            const auto parent = std::filesystem::path(path).parent_path();
            if (!parent.empty()) {
                std::filesystem::create_directories(parent);
            }
        } catch (const std::exception& e) {
            std::cerr << "[motor-log][ERROR] cannot create directory for " << path << ": "
                      << e.what() << std::endl;
            return false;
        }
        file_.open(path);
        if (!file_) {
            std::cerr << "[motor-log][ERROR] cannot open " << path << std::endl;
            return false;
        }
        hz_ = std::clamp(hz, 1.0, static_cast<double>(FREQUENCY));
        period_ = 1.0 / hz_;
        next_t_ = 0.0;
        rows_ = 0;
        file_ << "time,cycle";
        for (size_t i = 0; i < 16; ++i) {
            file_ << "," << motor_channel_name(i) << "_tau";
        }
        for (size_t i = 0; i < 16; ++i) {
            file_ << "," << motor_channel_name(i) << "_t_mos_c";
        }
        for (size_t i = 0; i < 16; ++i) {
            file_ << "," << motor_channel_name(i) << "_t_rotor_c";
        }
        for (size_t i = 0; i < 16; ++i) {
            file_ << "," << motor_channel_name(i) << "_error_code";
        }
        for (size_t i = 0; i < 16; ++i) {
            file_ << "," << motor_channel_name(i) << "_q";
        }
        file_ << "\n";
        file_.flush();
        path_ = path;
        std::cout << "[motor-log] writing torque+temp CSV @ " << hz_ << " Hz -> " << path_
                  << std::endl;
        return true;
    }

    void set_cycle(int cycle) { cycle_ = std::max(0, cycle); }

    void maybe_append(double t, const std::vector<JointState>& left,
                      const std::vector<JointState>& right,
                      const std::vector<int>& mos_temps, const std::vector<int>& rotor_temps,
                      const std::vector<int>& error_codes) {
        if (!file_) return;
        if (t + 1e-9 < next_t_) return;
        next_t_ = t + period_;

        auto pad16 = [](std::vector<int> v) {
            if (v.size() < 16) v.resize(16, 0);
            if (v.size() > 16) v.resize(16);
            return v;
        };
        const auto mos = pad16(mos_temps);
        const auto rotor = pad16(rotor_temps);
        const auto err = pad16(error_codes);

        std::vector<double> tau(16, 0.0);
        std::vector<double> q(16, 0.0);
        for (size_t i = 0; i < 8 && i < left.size(); ++i) {
            tau[i] = left[i].effort;
            q[i] = left[i].position;
        }
        for (size_t i = 0; i < 8 && i < right.size(); ++i) {
            tau[8 + i] = right[i].effort;
            q[8 + i] = right[i].position;
        }

        file_ << std::fixed << std::setprecision(6) << t << "," << cycle_;
        for (double v : tau) file_ << "," << v;
        for (int v : mos) file_ << "," << v;
        for (int v : rotor) file_ << "," << v;
        for (int v : err) file_ << "," << v;
        for (double v : q) file_ << "," << v;
        file_ << "\n";
        ++rows_;
        if ((rows_ % 50) == 0) {
            file_.flush();
        }
    }

    void close() {
        if (file_) {
            file_.flush();
            file_.close();
            if (!path_.empty()) {
                std::cout << "[motor-log] closed " << path_ << " (" << rows_ << " rows)"
                          << std::endl;
            }
        }
        path_.clear();
        rows_ = 0;
        cycle_ = 0;
    }

    bool active() const { return static_cast<bool>(file_); }
    const std::string& path() const { return path_; }

    ~MotorTelemetryCsvLogger() { close(); }

private:
    std::ofstream file_;
    std::string path_;
    double hz_ = 100.0;
    double period_ = 0.01;
    double next_t_ = 0.0;
    size_t rows_ = 0;
    int cycle_ = 0;
};

std::string resolve_motor_log_path(const Options& options) {
    std::string path = options.motor_log_path;
    if (path.empty()) {
        const char* env = std::getenv("OPENARM_REPLAY_MOTOR_LOG");
        if (env != nullptr && *env != '\0') {
            path = env;
        }
    }
    if (path.empty() || path == "1" || path == "true" || path == "auto" || path == "on") {
        const auto now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm {};
        localtime_r(&tt, &tm);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
        path = std::string("recordings/motor_logs/") + stamp + "_motor_torque_temp.csv";
    }
    if (path == "0" || path == "off" || path == "false" || path == "no") {
        return {};
    }
    return path;
}

double resolve_motor_log_hz(const Options& options) {
    double hz = options.motor_log_hz;
    if (const char* env = std::getenv("OPENARM_REPLAY_MOTOR_LOG_HZ"); env && *env) {
        try {
            hz = std::stod(env);
        } catch (...) {
        }
    }
    return std::clamp(hz, 1.0, static_cast<double>(FREQUENCY));
}

class TorqueTelemetrySender {
public:
    bool open(uint16_t port) {
        close();
        if (port == 0) return false;
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::cerr << "[torque-telemetry] socket failed: " << std::strerror(errno)
                      << std::endl;
            return false;
        }
        addr_ = {};
        addr_.sin_family = AF_INET;
        addr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr_.sin_port = htons(port);
        std::cout << "[torque-telemetry] sending 16 motor torques to 127.0.0.1:"
                  << port << std::endl;
        return true;
    }

    void send(double t, bool left_gripper_contact_hold, bool right_gripper_contact_hold,
              bool left_gripper_slip_boost, bool right_gripper_slip_boost,
              const std::vector<JointState>& left,
              const std::vector<JointState>& right, uint32_t reserved_alert_mask = 0,
              const std::vector<int>& error_codes = {},
              const std::vector<int>& mos_temps = {},
              const std::vector<int>& rotor_temps = {}) {
        if (fd_ < 0 || left.empty() || right.empty()) return;
        std::ostringstream payload;
        payload << t << "," << (left_gripper_contact_hold ? 1 : 0)
                << "," << (right_gripper_contact_hold ? 1 : 0)
                << "," << (left_gripper_slip_boost ? 1 : 0)
                << "," << (right_gripper_slip_boost ? 1 : 0);
        for (const auto& joint : left) payload << "," << joint.effort;
        for (const auto& joint : right) payload << "," << joint.effort;
        payload << "," << reserved_alert_mask;
        for (const int code : error_codes) payload << "," << code;
        for (const int temp : mos_temps) payload << "," << temp;
        for (const int temp : rotor_temps) payload << "," << temp;
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

    ~TorqueTelemetrySender() { close(); }

private:
    int fd_ = -1;
    sockaddr_in addr_ {};
};

class StatusSender {
public:
    bool open(uint16_t port) {
        close();
        if (port == 0) return false;
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::cerr << "[dual-replay-status] socket failed: " << std::strerror(errno)
                      << std::endl;
            return false;
        }
        addr_ = {};
        addr_.sin_family = AF_INET;
        addr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr_.sin_port = htons(port);
        std::cout << "[dual-replay-status] sending status to 127.0.0.1:"
                  << port << std::endl;
        return true;
    }

    void send(const std::string& status, const std::string& detail = "") {
        if (fd_ < 0) return;
        const std::string text = detail.empty() ? status : status + "|" + detail;
        ::sendto(fd_, text.data(), text.size(), 0,
                 reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~StatusSender() { close(); }

private:
    int fd_ = -1;
    sockaddr_in addr_ {};
};

}  // namespace

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        Options options;
        if (!parse_args(argc, argv, &options)) {
            return 1;
        }
        if (!std::filesystem::exists(options.urdf_path)) {
            std::cerr << "[ERROR] URDF not found: " << options.urdf_path << std::endl;
            return 1;
        }
        if (options.command_port == 0 && !std::filesystem::exists(options.replay_path)) {
            std::cerr << "[ERROR] replay CSV not found: " << options.replay_path << std::endl;
            return 1;
        }

        std::cout << "=== OpenArm Dual Real Replay Control ===" << std::endl;
        std::cout << "URDF              : " << options.urdf_path << std::endl;
        std::cout << "Replay merged CSV : " << options.replay_path << std::endl;
        std::cout << "Left follower CAN : " << options.left_follower_can << std::endl;
        std::cout << "Right follower CAN: " << options.right_follower_can << std::endl;
        std::cout << "Auto home         : " << (options.auto_home ? "on" : "off") << std::endl;
        std::cout << "Start delay       : " << options.start_delay << "s" << std::endl;
        std::cout << "Replay speed      : " << options.speed << "x" << std::endl;
        std::cout << "Transition duration: " << options.transition_duration << "s" << std::endl;
        std::cout << "Torque recovery   : " << options.torque_recovery_speed
                  << " rad/s, min " << options.torque_recovery_min_duration << "s"
                  << std::endl;
        std::cout << "Startup pose      : "
                  << (options.startup_pose_path.empty() ? std::string("INITIAL_POSITION")
                                                        : options.startup_pose_path)
                  << std::endl;
        std::cout << "Home pose         : "
                  << (options.home_pose_path.empty() ? std::string("INITIAL_POSITION")
                                                     : options.home_pose_path)
                  << std::endl;
        std::cout << "Torque telemetry : "
                  << (options.torque_telemetry_port == 0
                          ? std::string("off")
                          : std::to_string(options.torque_telemetry_port))
                  << std::endl;
        const std::string motor_log_path = resolve_motor_log_path(options);
        const double motor_log_hz = resolve_motor_log_hz(options);
        std::cout << "Motor CSV log    : "
                  << (motor_log_path.empty() ? std::string("off") : motor_log_path)
                  << (motor_log_path.empty() ? "" : (" @ " + std::to_string(motor_log_hz) + " Hz"))
                  << std::endl;

        const std::string root_link = "openarm_body_link0";
        Dynamics left_dynamics(options.urdf_path, root_link, select_leaf_link(options.urdf_path, "left"));
        Dynamics right_dynamics(options.urdf_path, root_link, select_leaf_link(options.urdf_path, "right"));
        left_dynamics.Init();
        right_dynamics.Init();

        YamlLoader follower_loader("config/follower.yaml");
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        std::cout << "=== Initializing left follower OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm* left_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(options.left_follower_can, true);
        std::cout << "=== Initializing right follower OpenArm ===" << std::endl;
        openarm::can::socket::OpenArm* right_openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(options.right_follower_can, true);

        const size_t left_arm_motor_num = left_openarm->get_arm().get_motors().size();
        const size_t left_hand_motor_num = left_openarm->get_gripper().get_motors().size();
        const size_t right_arm_motor_num = right_openarm->get_arm().get_motors().size();
        const size_t right_hand_motor_num = right_openarm->get_gripper().get_motors().size();
        const size_t left_total = left_arm_motor_num + left_hand_motor_num;
        const size_t right_total = right_arm_motor_num + right_hand_motor_num;
        if (left_total != 8 || right_total != 8) {
            std::cerr << "[ERROR] expected 8 joints per arm, got left=" << left_total
                      << " right=" << right_total << std::endl;
            return 1;
        }

        auto left_state = std::make_shared<RobotSystemState>(left_arm_motor_num, left_hand_motor_num);
        auto right_state =
            std::make_shared<RobotSystemState>(right_arm_motor_num, right_hand_motor_num);

        Control left_control(left_openarm, &left_dynamics, &left_dynamics, left_state,
                             1.0 / FREQUENCY, ROLE_FOLLOWER, "left_arm",
                             left_arm_motor_num, left_hand_motor_num);
        Control right_control(right_openarm, &right_dynamics, &right_dynamics, right_state,
                              1.0 / FREQUENCY, ROLE_FOLLOWER, "right_arm",
                              right_arm_motor_num, right_hand_motor_num);

        left_control.SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv,
                                  follower_Fo);
        right_control.SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv,
                                   follower_Fo);
        left_control.SetSafetyFilter(std::make_unique<SafetyFilter>(make_default_safety_config(left_total)));
        right_control.SetSafetyFilter(std::make_unique<SafetyFilter>(make_default_safety_config(right_total)));

        std::cout << "[dual-replay] clearing stale motor targets by holding current feedback pose for "
                  << options.init_hold_duration << "s" << std::endl;
        capture_and_hold_both_current_pose(left_control, right_control, left_openarm,
                                           right_openarm, left_state, right_state,
                                           left_arm_motor_num, left_hand_motor_num,
                                           right_arm_motor_num, right_hand_motor_num,
                                           options.init_hold_duration);
        std::cout << "[dual-replay] ramping follower startup gains from "
                  << options.startup_gain_ramp_start_scale << " to 1.0 over "
                  << options.startup_gain_ramp_duration << "s" << std::endl;
        ramp_both_followers_to_nominal_gains(
            left_control, right_control, left_state, right_state,
            follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv, follower_Fo,
            options.startup_gain_ramp_duration, options.startup_gain_ramp_start_scale);

        std::vector<JointState> left_zero(left_total);
        std::vector<JointState> right_zero(right_total);
        for (size_t i = 0; i < left_total; ++i) {
            const size_t idx = std::min(i, static_cast<size_t>(NJOINTS - 1));
            left_zero[i].position = INITIAL_POSITION[idx];
            left_zero[i].velocity = 0.0;
            left_zero[i].effort = 0.0;
            right_zero[i].position = INITIAL_POSITION[idx];
            right_zero[i].velocity = 0.0;
            right_zero[i].effort = 0.0;
        }
        const std::vector<JointState> zero_home_frame = concat_joints(left_zero, right_zero);

        std::vector<JointState> left_startup_ready = left_zero;
        std::vector<JointState> right_startup_ready = right_zero;
        const bool startup_pose_loaded =
            load_startup_pose_file(options.startup_pose_path, &left_startup_ready,
                                   &right_startup_ready);
        const std::vector<JointState> startup_ready_frame =
            concat_joints(left_startup_ready, right_startup_ready);

        std::vector<JointState> left_action_ready = left_zero;
        std::vector<JointState> right_action_ready = right_zero;
        const bool home_pose_loaded =
            load_home_pose_file(options.home_pose_path, &left_action_ready, &right_action_ready);
        const std::vector<JointState> action_ready_frame =
            concat_joints(left_action_ready, right_action_ready);

        if (options.auto_home) {
            if (startup_pose_loaded) {
                std::cout << "[dual-replay] startup: smoothly moving both followers to startup pose"
                          << std::endl;
                move_both_arms_smoothly(left_control, right_control, left_state, right_state,
                                        startup_ready_frame, left_total, right_total, 0.25, 4.0);
            } else {
                std::cout << "[dual-replay] no startup pose file; using zero home"
                          << std::endl;
                move_both_arms_smoothly(left_control, right_control, left_state, right_state,
                                        zero_home_frame, left_total, right_total, 0.25, 4.0);
            }
            if (home_pose_loaded) {
                std::cout << "[dual-replay] preparing action-ready pose from home pose file"
                          << std::endl;
                move_both_arms_smoothly(left_control, right_control, left_state, right_state,
                                        action_ready_frame, left_total, right_total, 0.25, 4.0);
            } else {
                std::cout << "[dual-replay] no action-ready home pose file; using zero pose"
                          << std::endl;
            }
        } else {
            std::cout << "[dual-replay] --no-auto-home: keeping current follower positions"
                      << std::endl;
            left_state->arm_state().set_all_references(left_state->arm_state().get_all_responses());
            left_state->hand_state().set_all_references(left_state->hand_state().get_all_responses());
            right_state->arm_state().set_all_references(
                right_state->arm_state().get_all_responses());
            right_state->hand_state().set_all_references(
                right_state->hand_state().get_all_responses());
        }
        left_control.ResetSafetyFilter(left_state->get_all_references());
        right_control.ResetSafetyFilter(right_state->get_all_references());
        const std::vector<JointState> initial_home_frame =
            concat_joints(left_state->get_all_references(), right_state->get_all_references());

        if (options.command_port != 0) {
            UdpTextReceiver command_receiver;
            if (!command_receiver.open(options.command_port, "dual-replay-command")) {
                return 1;
            }
            std::cout << "[dual-replay-server] ready on UDP port "
                      << options.command_port << std::endl;
            std::cout << "[dual-replay-server] concurrent-sides=1 "
                         "(left/right plays may overlap; full dual remains exclusive)"
                      << std::endl;
            std::cout << "[dual-replay-server] approach stream: approach_on|side, "
                         "tcp_vel|side|vx|vy|vz, approach_stop|side, approach_get_pose|side"
                      << std::endl;

            // Per-side replays allow left and right CSVs to run at the same time.
            // Full dual (side="") still requires both sides idle.
            // Approach stream is mutually exclusive with play on the same side.
            openarm_approach::TcpApproachStream approach_stream;
            CsvReplay left_replay;
            CsvReplay right_replay;
            CsvReplay dual_replay;
            bool left_active = false;
            bool right_active = false;
            bool dual_active = false;
            std::vector<JointState> last_frame =
                concat_joints(left_state->get_all_references(), right_state->get_all_references());
            TorqueTelemetrySender torque_sender;
            MotorFaultPrinter motor_fault_printer;
            MotorTelemetryCsvLogger motor_logger;
            motor_logger.open(motor_log_path, motor_log_hz);
            StatusSender status_sender;
            status_sender.open(options.status_port);
            status_sender.send("ready", std::to_string(options.command_port));
            if (motor_logger.active()) {
                status_sender.send("motor_log", motor_logger.path() + "|hz=" +
                                                    std::to_string(motor_log_hz));
            }
            auto replay_wall_start = std::chrono::steady_clock::now();
            const auto motor_log_wall_start = std::chrono::steady_clock::now();
            const int period_us = static_cast<int>(1e6 / FREQUENCY);
            const double control_dt = 1.0 / FREQUENCY;
            auto next_time = std::chrono::steady_clock::now();
            int play_cycle = 0;

            auto any_replay_active = [&]() {
                return left_active || right_active || dual_active;
            };

            while (keep_running) {
                for (const auto& message : command_receiver.receive_all()) {
                    if (message == "shutdown") {
                        std::cout << "[dual-replay-server] shutdown command received"
                                  << std::endl;
                        left_active = false;
                        right_active = false;
                        dual_active = false;
                        torque_sender.close();
                        status_sender.send("shutdown_received");
                        keep_running = false;
                        break;
                    }
                    if (message.rfind("dump|", 0) == 0) {
                        const std::string dump_path = message.substr(5);
                        const bool ok = write_dual_position_dump(
                            dump_path,
                            left_state->get_all_responses(),
                            right_state->get_all_responses(),
                            left_total,
                            right_total);
                        status_sender.send(ok ? "dumped" : "dump_failed", dump_path);
                        continue;
                    }

                    {
                        const std::vector<JointState> hold_left_cmd =
                            slice_joints(last_frame, 0, left_total);
                        const std::vector<JointState> hold_right_cmd =
                            slice_joints(last_frame, left_total, right_total);
                        const openarm_approach::ApproachStatus approach_reply =
                            approach_stream.handle_command(
                                message, left_active, right_active, dual_active,
                                hold_left_cmd, hold_right_cmd, &left_dynamics,
                                &right_dynamics);
                        if (approach_reply.handled) {
                            status_sender.send(approach_reply.status, approach_reply.detail);
                            continue;
                        }
                    }

                    auto command = parse_replay_command(message);
                    if (!command.has_value()) continue;

                    const bool want_left = command->side == "left";
                    const bool want_right = command->side == "right";
                    const bool want_dual = command->side.empty();

                    if (approach_stream.blocks_play(command->side)) {
                        std::cout << "[dual-replay-server] busy (approach), ignoring play: "
                                  << command->path << std::endl;
                        status_sender.send("busy", "approach|" + command->path);
                        continue;
                    }
                    if (want_dual && (left_active || right_active || dual_active)) {
                        std::cout << "[dual-replay-server] busy (dual), ignoring: "
                                  << command->path << std::endl;
                        status_sender.send("busy", "dual|" + command->path);
                        continue;
                    }
                    if (want_left && (left_active || dual_active)) {
                        std::cout << "[dual-replay-server] busy (left), ignoring: "
                                  << command->path << std::endl;
                        status_sender.send("busy", "left|" + command->path);
                        continue;
                    }
                    if (want_right && (right_active || dual_active)) {
                        std::cout << "[dual-replay-server] busy (right), ignoring: "
                                  << command->path << std::endl;
                        status_sender.send("busy", "right|" + command->path);
                        continue;
                    }

                    std::string replay_input = command->path;
                    // Start C++ prepend from last COMMANDED hold (last_frame), not measured
                    // feedback. Using responses accepted gravity sag into the next segment
                    // and looked like the arm "dropped" between replay and visual grasp.
                    const std::vector<JointState> transition_start_frame =
                        zero_velocity_hold_frame(last_frame);
                    const std::vector<JointState> current_left =
                        slice_joints(transition_start_frame, 0, left_total);
                    const std::vector<JointState> current_right =
                        slice_joints(transition_start_frame, left_total, right_total);

                    CsvReplay next_replay;
                    bool loaded = false;
                    size_t load_joint_count = left_total + right_total;
                    std::vector<JointState> transition_start = transition_start_frame;

                    if ((want_left || want_right) &&
                        std::filesystem::is_regular_file(replay_input)) {
                        // Prefer native 8-joint load so the other side can keep moving.
                        loaded = next_replay.load(replay_input, left_total);
                        if (loaded) {
                            load_joint_count = left_total;
                            transition_start = want_left ? current_left : current_right;
                        } else {
                            // Fallback freezes the other arm — only safe when that side is idle.
                            if ((want_left && right_active) || (want_right && left_active)) {
                                std::cout << "[dual-replay-server] cannot use merged fallback while "
                                             "the other side is active; rejecting "
                                          << command->path << std::endl;
                                status_sender.send("busy", "fallback_blocked|" + command->path);
                                continue;
                            }
                            const std::filesystem::path tmp =
                                std::filesystem::temp_directory_path() /
                                ("openarm_single_arm_replay_" + command->side + ".csv");
                            if (write_single_arm_as_merged_csv(replay_input, tmp.string(),
                                                               command->side, current_left,
                                                               current_right)) {
                                replay_input = tmp.string();
                                loaded = next_replay.load(replay_input, left_total + right_total);
                                load_joint_count = left_total + right_total;
                                transition_start = transition_start_frame;
                            }
                        }
                    } else if (std::filesystem::is_directory(replay_input)) {
                        loaded = next_replay.load_directory(replay_input, left_total + right_total,
                                                           options.generate_transitions,
                                                           options.transition_duration);
                        load_joint_count = left_total + right_total;
                    } else {
                        loaded = next_replay.load(replay_input, left_total + right_total);
                        load_joint_count = left_total + right_total;
                    }
                    if (!loaded) {
                        std::cout << "[dual-replay-server] failed to load: "
                                  << replay_input << std::endl;
                        status_sender.send("load_failed", replay_input);
                        continue;
                    }
                    const std::string side_tag =
                        want_left ? "left" : (want_right ? "right" : "dual");
                    status_sender.send("accepted",
                                       replay_input + "|side=" + side_tag + "|speed=" +
                                           std::to_string(command->speed) +
                                           "|telemetry_port=" +
                                           std::to_string(command->telemetry_port) +
                                           "|label=" +
                                           (command->label.empty() ? replay_input
                                                                   : command->label));
                    next_replay.set_time_scale(command->speed);
                    const bool single_file = std::filesystem::is_regular_file(replay_input);
                    // Default was 3.0s and stacked on every UDP play; keep caller override.
                    const double initial_transition_duration =
                        options.initial_transition_duration > 0.0
                            ? options.initial_transition_duration
                            : 1.2;
                    const CsvReplay::TransitionResult transition_result =
                        next_replay.prepend_transition_from_with_result(
                            transition_start, load_joint_count,
                            initial_transition_duration);
                    if (single_file) {
                        if (transition_result == CsvReplay::TransitionResult::Added) {
                            std::cout << "[dual-replay-server] single-file initial transition "
                                         "generated from held reference side="
                                      << side_tag << std::endl;
                            status_sender.send("transition_added", side_tag + "|" + replay_input);
                        } else if (transition_result == CsvReplay::TransitionResult::SkippedNoMotion) {
                            std::cout << "[dual-replay-server] single-file initial transition "
                                         "skipped: held reference already matches first frame"
                                      << std::endl;
                            status_sender.send("transition_skipped", "already_matches_first_frame");
                        } else {
                            std::cout << "[dual-replay-server][WARN] single-file initial transition "
                                         "failed; replay continues without prepended transition"
                                      << std::endl;
                            status_sender.send("transition_failed", replay_input);
                        }
                    }
                    next_replay.reset();
                    if (want_left && load_joint_count == left_total) {
                        left_replay = std::move(next_replay);
                        left_active = true;
                    } else if (want_right && load_joint_count == left_total) {
                        right_replay = std::move(next_replay);
                        right_active = true;
                    } else if (want_left) {
                        // 16-joint merged fallback for left: treat as dual-exclusive on this slot.
                        dual_replay = std::move(next_replay);
                        dual_active = true;
                        left_active = false;
                        right_active = false;
                    } else if (want_right) {
                        dual_replay = std::move(next_replay);
                        dual_active = true;
                        left_active = false;
                        right_active = false;
                    } else {
                        dual_replay = std::move(next_replay);
                        dual_active = true;
                        left_active = false;
                        right_active = false;
                    }
                    if (!any_replay_active()) {
                        // Should not happen; keep compiler calm.
                    }
                    replay_wall_start = std::chrono::steady_clock::now();
                    ++play_cycle;
                    motor_logger.set_cycle(play_cycle);
                    torque_sender.open(command->telemetry_port);
                    std::cout << "[dual-replay-server] started side=" << side_tag << ": "
                              << (command->label.empty() ? replay_input : command->label)
                              << " speed=" << command->speed << "x"
                              << " joints=" << load_joint_count
                              << std::endl;
                    status_sender.send("started",
                                       std::string("side=") + side_tag + "|" +
                                           (command->label.empty() ? replay_input : command->label) +
                                           "|speed=" + std::to_string(command->speed) +
                                           "|telemetry_port=" +
                                           std::to_string(command->telemetry_port));
                }

                std::vector<JointState> hold_left = slice_joints(last_frame, 0, left_total);
                std::vector<JointState> hold_right =
                    slice_joints(last_frame, left_total, right_total);

                if (dual_active) {
                    if (auto frame = dual_replay.sample()) {
                        last_frame = frame.value();
                        hold_left = slice_joints(last_frame, 0, left_total);
                        hold_right = slice_joints(last_frame, left_total, right_total);
                    }
                    for (const auto& event : dual_replay.consume_action_events()) {
                        std::cout << "[dual-replay-server] action started: "
                                  << format_action_event(event) << std::endl;
                        status_sender.send("action_started", format_action_event(event));
                    }
                } else {
                    if (left_active && !approach_stream.side_active("left")) {
                        if (auto frame = left_replay.sample()) {
                            hold_left = frame.value();
                        }
                        for (const auto& event : left_replay.consume_action_events()) {
                            std::cout << "[dual-replay-server] action started (left): "
                                      << format_action_event(event) << std::endl;
                            status_sender.send("action_started",
                                               "side=left|" + format_action_event(event));
                        }
                    }
                    if (right_active && !approach_stream.side_active("right")) {
                        if (auto frame = right_replay.sample()) {
                            hold_right = frame.value();
                        }
                        for (const auto& event : right_replay.consume_action_events()) {
                            std::cout << "[dual-replay-server] action started (right): "
                                      << format_action_event(event) << std::endl;
                            status_sender.send("action_started",
                                               "side=right|" + format_action_event(event));
                        }
                    }
                }

                approach_stream.tick(control_dt, &left_dynamics, &right_dynamics, &hold_left,
                                     &hold_right);
                last_frame = concat_joints(hold_left, hold_right);

                left_state->set_all_references(slice_joints(last_frame, 0, left_total));
                right_state->set_all_references(slice_joints(last_frame, left_total, right_total));

                left_control.unilateral_step();
                right_control.unilateral_step();

                const auto now = std::chrono::steady_clock::now();
                if (motor_logger.active()) {
                    const double t_log =
                        std::chrono::duration<double>(now - motor_log_wall_start).count();
                    motor_logger.maybe_append(
                        t_log, left_state->get_all_responses(), right_state->get_all_responses(),
                        collect_dual_motor_telemetry(left_openarm, right_openarm,
                                                     MotorTelemetryKind::MosTemp),
                        collect_dual_motor_telemetry(left_openarm, right_openarm,
                                                     MotorTelemetryKind::RotorTemp),
                        collect_dual_motor_telemetry(left_openarm, right_openarm,
                                                     MotorTelemetryKind::ErrorCode));
                }
                if (any_replay_active() || approach_stream.any_active()) {
                    const double t = std::chrono::duration<double>(now - replay_wall_start).count();
                    if (any_replay_active()) {
                        torque_sender.send(t, left_control.GripperContactHoldActive(),
                                           right_control.GripperContactHoldActive(),
                                           left_control.GripperSlipBoostActive(),
                                           right_control.GripperSlipBoostActive(),
                                           left_state->get_all_responses(),
                                           right_state->get_all_responses(),
                                           0,
                                           collect_dual_motor_telemetry(
                                               left_openarm, right_openarm,
                                               MotorTelemetryKind::ErrorCode),
                                           collect_dual_motor_telemetry(
                                               left_openarm, right_openarm,
                                               MotorTelemetryKind::MosTemp),
                                           collect_dual_motor_telemetry(
                                               left_openarm, right_openarm,
                                               MotorTelemetryKind::RotorTemp));
                    }
                    motor_fault_printer.check(left_openarm, right_openarm);
                    std::string safety_reason;
                    if (feedback_safety_limit_exceeded(left_state->get_all_responses(),
                                                       right_state->get_all_responses(),
                                                       &safety_reason)) {
                        std::cout << "[dual-replay-server][SAFETY] " << safety_reason
                                  << std::endl;
                        status_sender.send("safety", safety_reason);
                        std::cout << "[dual-replay-server][SAFETY] switching both arms to "
                                     "Kd-only damping release"
                                  << std::endl;
                        left_active = false;
                        right_active = false;
                        dual_active = false;
                        torque_sender.close();
                        send_kd_only_release(left_openarm, right_openarm, follower_kd);
                        last_frame = concat_joints(left_state->get_all_responses(),
                                                   right_state->get_all_responses());
                        next_time = std::chrono::steady_clock::now();
                        continue;
                    }
                    if (dual_active && dual_replay.finished()) {
                        std::cout << "[dual-replay-server] dual replay finished; holding"
                                  << std::endl;
                        dual_active = false;
                        status_sender.send("finished", "side=dual");
                        last_frame = concat_joints(left_state->get_all_references(),
                                                   right_state->get_all_references());
                    }
                    if (left_active && left_replay.finished()) {
                        std::cout << "[dual-replay-server] left replay finished; holding left"
                                  << std::endl;
                        left_active = false;
                        status_sender.send("finished", "side=left");
                        last_frame = concat_joints(left_state->get_all_references(),
                                                   right_state->get_all_references());
                    }
                    if (right_active && right_replay.finished()) {
                        std::cout << "[dual-replay-server] right replay finished; holding right"
                                  << std::endl;
                        right_active = false;
                        status_sender.send("finished", "side=right");
                        last_frame = concat_joints(left_state->get_all_references(),
                                                   right_state->get_all_references());
                    }
                    if (!any_replay_active()) {
                        torque_sender.close();
                    }
                }

                sleep_until_next(&next_time, period_us);
            }

            std::cout << "[dual-replay-server] shutting down; sending Kd-only damping release"
                      << std::endl;
            send_kd_only_release(left_openarm, right_openarm, follower_kd);
            status_sender.send("shutdown_done");
            return 0;
        }

        CsvReplay replay;
        if (options.clean_transitions && std::filesystem::is_directory(options.replay_path)) {
            CsvReplay::clean_transitions(options.replay_path);
        }

        bool loaded = false;
        if (std::filesystem::is_directory(options.replay_path)) {
            loaded = replay.load_directory(options.replay_path, left_total + right_total,
                                          options.generate_transitions,
                                          options.transition_duration);
        } else {
            loaded = replay.load(options.replay_path, left_total + right_total);
        }
        if (!loaded) {
            return 1;
        }
        replay.set_time_scale(options.speed);
        replay.prepend_transition_from(
            concat_joints(left_state->get_all_references(), right_state->get_all_references()),
            left_total + right_total,
            options.initial_transition_duration);

        if (options.start_delay > 0.0) {
            std::cout << "[dual-replay] waiting " << options.start_delay
                      << "s before starting replay" << std::endl;
            std::this_thread::sleep_for(std::chrono::duration<double>(options.start_delay));
        }
        replay.reset();
        std::cout << "[dual-replay] started with one shared replay clock" << std::endl;

        const int period_us = static_cast<int>(1e6 / FREQUENCY);
        auto next_time = std::chrono::steady_clock::now();
        auto replay_wall_start = std::chrono::steady_clock::now();
        std::vector<JointState> last_frame =
            concat_joints(left_state->get_all_references(), right_state->get_all_references());
        TorqueTelemetrySender torque_sender;
        torque_sender.open(options.torque_telemetry_port);
        MotorFaultPrinter motor_fault_printer;
        bool recovery_requested = false;
        std::string recovery_reason;

        while (keep_running) {
            if (auto frame = replay.sample()) {
                last_frame = frame.value();
            }
            for (const auto& event : replay.consume_action_events()) {
                std::cout << "[dual-replay] action started: "
                          << format_action_event(event) << std::endl;
            }

            left_state->set_all_references(slice_joints(last_frame, 0, left_total));
            right_state->set_all_references(slice_joints(last_frame, left_total, right_total));

            left_control.unilateral_step();
            right_control.unilateral_step();

            const auto now = std::chrono::steady_clock::now();
            if (options.torque_telemetry_port != 0) {
                const double t = std::chrono::duration<double>(now - replay_wall_start).count();
                torque_sender.send(t, left_control.GripperContactHoldActive(),
                                   right_control.GripperContactHoldActive(),
                                   left_control.GripperSlipBoostActive(),
                                   right_control.GripperSlipBoostActive(),
                                   left_state->get_all_responses(),
                                   right_state->get_all_responses(),
                                   0,
                                   collect_dual_motor_telemetry(
                                       left_openarm, right_openarm,
                                       MotorTelemetryKind::ErrorCode),
                                   collect_dual_motor_telemetry(
                                       left_openarm, right_openarm,
                                       MotorTelemetryKind::MosTemp),
                                   collect_dual_motor_telemetry(
                                       left_openarm, right_openarm,
                                       MotorTelemetryKind::RotorTemp));
            }
            motor_fault_printer.check(left_openarm, right_openarm);

            if (feedback_safety_limit_exceeded(left_state->get_all_responses(),
                                               right_state->get_all_responses(),
                                               &recovery_reason)) {
                recovery_requested = true;
                break;
            }

            if (replay.finished()) {
                std::cout << "[dual-replay] replay finished" << std::endl;
                break;
            }
            sleep_until_next(&next_time, period_us);
        }

        if (recovery_requested && keep_running) {
            std::cout << "[dual-replay][SAFETY] " << recovery_reason << std::endl;
            std::cout << "[dual-replay][SAFETY] stopping replay and switching both arms to "
                         "Kd-only damping release"
                      << std::endl;
            send_kd_only_release(left_openarm, right_openarm, follower_kd);
        }

        if (options.damping_release_on_exit) {
            std::cout << "[dual-replay] shutting down; sending Kd-only damping release"
                      << std::endl;
            send_kd_only_release(left_openarm, right_openarm, follower_kd);
        } else {
            std::cout << "[dual-replay] shutting down without Kd-only damping release"
                      << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
