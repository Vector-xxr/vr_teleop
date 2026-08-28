// Copyright 2026 Enactic, Inc.
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

#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm_port/openarm_init.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <io/teleop_io.hpp>

namespace {

volatile std::sig_atomic_t keep_running = 1;

void signal_handler(int) { keep_running = 0; }

struct Options {
    std::string arm_side = "right_arm";
    std::string follower_can = "follower_right";
    std::string record_path;
    std::string position_dump_path;
    uint16_t record_start_port = 47920;
    double hz = 1000.0;
    bool disable_on_exit = true;
};

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " --follower-can IFACE --record CSV [options]\n"
              << "Options:\n"
              << "  --arm-side right_arm|left_arm       default: right_arm\n"
              << "  --record-start-port PORT            UDP start/stop/dump port\n"
              << "  --position-dump PATH                append current motor positions on dump\n"
              << "  --hz HZ                             sampling rate, default: 1000\n"
              << "  --keep-enabled-on-exit              do not disable motors at shutdown\n";
}

bool parse_args(int argc, char** argv, Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--arm-side" && i + 1 < argc) {
            options->arm_side = argv[++i];
        } else if (arg == "--follower-can" && i + 1 < argc) {
            options->follower_can = argv[++i];
        } else if (arg == "--record" && i + 1 < argc) {
            options->record_path = argv[++i];
        } else if (arg == "--record-start-port" && i + 1 < argc) {
            options->record_start_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--position-dump" && i + 1 < argc) {
            options->position_dump_path = argv[++i];
        } else if (arg == "--hz" && i + 1 < argc) {
            options->hz = std::stod(argv[++i]);
        } else if (arg == "--keep-enabled-on-exit") {
            options->disable_on_exit = false;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "[follower-record][ERROR] unknown or incomplete option: "
                      << arg << std::endl;
            print_usage(argv[0]);
            return false;
        }
    }

    if (options->arm_side != "right_arm" && options->arm_side != "left_arm") {
        std::cerr << "[follower-record][ERROR] invalid arm side: "
                  << options->arm_side << std::endl;
        return false;
    }
    if (options->record_path.empty()) {
        std::cerr << "[follower-record][ERROR] --record is required" << std::endl;
        print_usage(argv[0]);
        return false;
    }
    if (options->hz <= 0.0) {
        std::cerr << "[follower-record][ERROR] --hz must be positive" << std::endl;
        return false;
    }
    return true;
}

std::vector<JointState> read_follower_joints(openarm::can::socket::OpenArm* openarm) {
    std::vector<JointState> joints;
    if (openarm == nullptr) return joints;
    joints.reserve(openarm->get_arm().get_motors().size() +
                   openarm->get_gripper().get_motors().size());
    for (const auto& motor : openarm->get_arm().get_motors()) {
        joints.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    for (const auto& motor : openarm->get_gripper().get_motors()) {
        joints.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    return joints;
}

std::string next_record_segment_path(const std::string& record_path, int segment_index) {
    const std::filesystem::path base(record_path);
    std::ostringstream suffix;
    suffix << "_" << std::setw(2) << std::setfill('0') << segment_index;
    return (base.parent_path() /
            (base.stem().string() + suffix.str() + base.extension().string()))
        .string();
}

std::string format_joints(const std::vector<JointState>& joints) {
    std::ostringstream out;
    out << "{";
    for (size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) out << ", ";
        if (i + 1 == joints.size()) {
            out << "gripper_m1=";
        } else {
            out << "arm_m" << (i + 1) << "=";
        }
        out << joints[i].position;
    }
    out << "}";
    return out.str();
}

void dump_positions(const Options& options, const std::vector<JointState>& follower_joints) {
    if (options.position_dump_path.empty()) return;
    std::ofstream out(options.position_dump_path, std::ios::app);
    if (!out) {
        std::cerr << "[positions] failed to open dump file: "
                  << options.position_dump_path << std::endl;
        return;
    }
    const std::time_t now = std::time(nullptr);
    char timestamp[64] = {};
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", std::localtime(&now));
    out << "[positions] timestamp=" << timestamp
        << " arm_side=" << options.arm_side
        << " follower=" << format_joints(follower_joints) << "\n";
    std::cout << "[positions] dumped follower pose to "
              << options.position_dump_path << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Options options;
    if (!parse_args(argc, argv, &options)) return 1;

    try {
        std::filesystem::create_directories(
            std::filesystem::path(options.record_path).parent_path());
        if (!options.position_dump_path.empty()) {
            std::filesystem::create_directories(
                std::filesystem::path(options.position_dump_path).parent_path());
        }

        std::cout << "=== OpenArm follower-only record ===" << std::endl;
        std::cout << "Arm side       : " << options.arm_side << std::endl;
        std::cout << "Follower CAN   : " << options.follower_can << std::endl;
        std::cout << "Record path    : " << options.record_path << std::endl;
        std::cout << "Start port     : " << options.record_start_port << std::endl;
        std::cout << "Sample rate    : " << options.hz << " Hz" << std::endl;

        std::unique_ptr<openarm::can::socket::OpenArm> follower_openarm(
            openarm_init::OpenArmInitializer::initialize_openarm(options.follower_can, true));
        const size_t joint_count = follower_openarm->get_arm().get_motors().size() +
                                   follower_openarm->get_gripper().get_motors().size();
        if (joint_count == 0) {
            std::cerr << "[follower-record][ERROR] no follower motors initialized" << std::endl;
            return 1;
        }

        UdpTextReceiver trigger_receiver;
        if (!trigger_receiver.open(options.record_start_port, "follower-record")) {
            return 1;
        }

        CsvRecorder recorder;
        int segment_index = 0;
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / options.hz));
        auto next_time = std::chrono::steady_clock::now();

        std::cout << "[follower-record] waiting for UDP start/stop/dump on port "
                  << options.record_start_port << std::endl;
        while (keep_running) {
            follower_openarm->refresh_all();
            follower_openarm->recv_all(200);
            const std::vector<JointState> follower_joints =
                read_follower_joints(follower_openarm.get());

            for (const auto& message : trigger_receiver.receive_all()) {
                if (message.find("start") != std::string::npos && !recorder.active()) {
                    ++segment_index;
                    const std::string path =
                        next_record_segment_path(options.record_path, segment_index);
                    if (recorder.open(path, joint_count)) {
                        std::cout << "[follower-record] started: " << path << std::endl;
                    }
                }
                if (message.find("stop") != std::string::npos && recorder.active()) {
                    recorder.close();
                    std::cout << "[follower-record] stopped" << std::endl;
                }
                if (message.find("dump") != std::string::npos) {
                    dump_positions(options, follower_joints);
                }
            }

            if (recorder.active() && follower_joints.size() == joint_count) {
                recorder.append(follower_joints);
            }

            next_time += period;
            std::this_thread::sleep_until(next_time);
            if (next_time < std::chrono::steady_clock::now() - period) {
                next_time = std::chrono::steady_clock::now();
            }
        }

        recorder.close();
        if (options.disable_on_exit) {
            follower_openarm->disable_all();
        }
        std::cout << "[follower-record] shutdown complete" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[follower-record][ERROR] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
