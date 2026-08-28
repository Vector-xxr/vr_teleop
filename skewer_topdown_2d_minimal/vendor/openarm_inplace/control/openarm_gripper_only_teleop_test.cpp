#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm_constants.hpp>
#include <string>
#include <thread>

namespace {

std::atomic_bool keep_running {true};

struct Options {
    std::string leader_can = "leader_right";
    std::string follower_can = "follower_right";
    uint32_t gripper_send_can_id = DEFAULT_MOTOR_CONFIG.gripper_send_can_id;
    uint32_t gripper_recv_can_id = DEFAULT_MOTOR_CONFIG.gripper_recv_can_id;
    double hz = 200.0;
    double kp = 8.0;
    double kd = 0.3;
    double offset = 0.0;
    double min_position = position_limit_min[NJOINTS - 1];
    double max_position = position_limit_max[NJOINTS - 1];
};

enum class ParseResult {
    Ok,
    Help,
    Error,
};

void signal_handler(int) {
    keep_running = false;
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " [--leader-can IFACE] [--follower-can IFACE]"
                 " [--send-id HEX_OR_DEC] [--recv-id HEX_OR_DEC]"
                 " [--hz HZ] [--kp K] [--kd D] [--offset RAD]"
                 " [--min-position RAD] [--max-position RAD]\n";
}

uint32_t parse_u32(const std::string& value) {
    return static_cast<uint32_t>(std::stoul(value, nullptr, 0));
}

ParseResult parse_args(int argc, char** argv, Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--leader-can" && i + 1 < argc) {
            options->leader_can = argv[++i];
        } else if (arg == "--follower-can" && i + 1 < argc) {
            options->follower_can = argv[++i];
        } else if (arg == "--send-id" && i + 1 < argc) {
            options->gripper_send_can_id = parse_u32(argv[++i]);
        } else if (arg == "--recv-id" && i + 1 < argc) {
            options->gripper_recv_can_id = parse_u32(argv[++i]);
        } else if (arg == "--hz" && i + 1 < argc) {
            options->hz = std::stod(argv[++i]);
        } else if (arg == "--kp" && i + 1 < argc) {
            options->kp = std::stod(argv[++i]);
        } else if (arg == "--kd" && i + 1 < argc) {
            options->kd = std::stod(argv[++i]);
        } else if (arg == "--offset" && i + 1 < argc) {
            options->offset = std::stod(argv[++i]);
        } else if (arg == "--min-position" && i + 1 < argc) {
            options->min_position = std::stod(argv[++i]);
        } else if (arg == "--max-position" && i + 1 < argc) {
            options->max_position = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return ParseResult::Help;
        } else {
            std::cerr << "[gripper-only-teleop][ERROR] unknown or incomplete option: "
                      << arg << std::endl;
            print_usage(argv[0]);
            return ParseResult::Error;
        }
    }

    options->hz = std::clamp(options->hz, 20.0, FREQUENCY);
    options->kp = std::clamp(options->kp, 0.0, 30.0);
    options->kd = std::clamp(options->kd, 0.0, 5.0);
    if (options->min_position > options->max_position) {
        std::swap(options->min_position, options->max_position);
    }
    return ParseResult::Ok;
}

double current_gripper_position(openarm::can::socket::OpenArm& openarm);
void send_gripper_mit(openarm::can::socket::OpenArm& openarm, double position,
                      double kp, double kd);

void init_gripper_only(openarm::can::socket::OpenArm& openarm, const Options& options) {
    openarm.init_gripper_motor(DEFAULT_MOTOR_CONFIG.gripper_motor_type,
                               options.gripper_send_can_id,
                               options.gripper_recv_can_id);
    openarm.get_gripper().set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    openarm.get_gripper().enable_all();
    openarm.recv_all(1000);
    const double q = current_gripper_position(openarm);
    send_gripper_mit(openarm, q, 0.0, 0.0);
    openarm.recv_all(200);
}

double current_gripper_position(openarm::can::socket::OpenArm& openarm) {
    openarm.recv_all(200);
    const auto motors = openarm.get_gripper().get_motors();
    if (motors.empty()) return 0.0;
    return motors.front().get_position();
}

void send_gripper_mit(openarm::can::socket::OpenArm& openarm, double position,
                      double kp, double kd) {
    openarm.get_gripper().mit_control_one(
        0, openarm::damiao_motor::MITParam{kp, kd, position, 0.0, 0.0});
}

void send_gripper_neutral(openarm::can::socket::OpenArm& openarm) {
    const double q = current_gripper_position(openarm);
    send_gripper_mit(openarm, q, 0.0, 0.0);
    openarm.recv_all(200);
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    const ParseResult parse_result = parse_args(argc, argv, &options);
    if (parse_result == ParseResult::Help) return 0;
    if (parse_result == ParseResult::Error) return 2;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "[gripper-only-teleop] leader_can=" << options.leader_can
              << " follower_can=" << options.follower_can
              << " send_id=0x" << std::hex << options.gripper_send_can_id
              << " recv_id=0x" << options.gripper_recv_can_id << std::dec
              << " hz=" << options.hz
              << " kp=" << options.kp
              << " kd=" << options.kd
              << " offset=" << options.offset
              << " min=" << options.min_position
              << " max=" << options.max_position << std::endl;

    openarm::can::socket::OpenArm leader_openarm(options.leader_can, true);
    openarm::can::socket::OpenArm follower_openarm(options.follower_can, true);

    std::cout << "[gripper-only-teleop] initializing leader gripper only" << std::endl;
    init_gripper_only(leader_openarm, options);
    std::cout << "[gripper-only-teleop] initializing follower gripper only" << std::endl;
    init_gripper_only(follower_openarm, options);

    const double period_sec = 1.0 / options.hz;
    const auto period = std::chrono::duration<double>(period_sec);
    auto next_time = std::chrono::steady_clock::now();
    size_t tick = 0;

    while (keep_running) {
        const double leader_q = current_gripper_position(leader_openarm);
        const double target_q =
            std::clamp(leader_q + options.offset, options.min_position, options.max_position);

        send_gripper_mit(leader_openarm, leader_q, 0.0, 0.0);
        send_gripper_mit(follower_openarm, target_q, options.kp, options.kd);
        const double follower_q = current_gripper_position(follower_openarm);

        if (++tick % static_cast<size_t>(std::max(1.0, options.hz)) == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "[gripper-only-teleop] leader_q=" << leader_q
                      << " target_q=" << target_q
                      << " follower_q=" << follower_q
                      << " error=" << (target_q - follower_q) << std::endl;
        }

        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_time);
    }

    std::cout << "[gripper-only-teleop] stopping grippers" << std::endl;
    send_gripper_neutral(follower_openarm);
    send_gripper_neutral(leader_openarm);
    follower_openarm.get_gripper().disable_all();
    leader_openarm.get_gripper().disable_all();
    std::cout << "[gripper-only-teleop] stopped" << std::endl;
    return 0;
}
