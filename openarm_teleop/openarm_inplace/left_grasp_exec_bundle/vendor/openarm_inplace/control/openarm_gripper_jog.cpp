#include <openarm_port/openarm_init.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::string can_interface = "follower_right";
    double delta = 0.08;
    double step_duration = 0.8;
    double settle_duration = 0.3;
    double kp = 8.0;
    double kd = 0.3;
};

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " [--can IFACE] [--delta RAD] [--step-duration SEC]"
                 " [--settle-duration SEC] [--kp K] [--kd D]\n";
}

bool parse_args(int argc, char** argv, Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--can" && i + 1 < argc) {
            options->can_interface = argv[++i];
        } else if (arg == "--delta" && i + 1 < argc) {
            options->delta = std::stod(argv[++i]);
        } else if (arg == "--step-duration" && i + 1 < argc) {
            options->step_duration = std::stod(argv[++i]);
        } else if (arg == "--settle-duration" && i + 1 < argc) {
            options->settle_duration = std::stod(argv[++i]);
        } else if (arg == "--kp" && i + 1 < argc) {
            options->kp = std::stod(argv[++i]);
        } else if (arg == "--kd" && i + 1 < argc) {
            options->kd = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "[gripper-jog][ERROR] unknown or incomplete option: " << arg
                      << std::endl;
            print_usage(argv[0]);
            return false;
        }
    }

    options->delta = std::clamp(options->delta, 0.0, 0.5);
    options->step_duration = std::clamp(options->step_duration, 0.1, 5.0);
    options->settle_duration = std::clamp(options->settle_duration, 0.0, 3.0);
    options->kp = std::clamp(options->kp, 0.0, 30.0);
    options->kd = std::clamp(options->kd, 0.0, 5.0);
    return true;
}

double smoothstep(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 10.0 * std::pow(s, 3) - 15.0 * std::pow(s, 4) + 6.0 * std::pow(s, 5);
}

std::vector<openarm::damiao_motor::MITParam> hold_current_arm_commands(
    openarm::can::socket::OpenArm* openarm) {
    std::vector<openarm::damiao_motor::MITParam> cmds;
    cmds.reserve(openarm->get_arm().get_motors().size());
    for (const auto& motor : openarm->get_arm().get_motors()) {
        cmds.emplace_back(openarm::damiao_motor::MITParam{
            0.0, 0.0, motor.get_position(), 0.0, 0.0});
    }
    return cmds;
}

void send_pose(openarm::can::socket::OpenArm* openarm, double gripper_position,
               double kp, double kd) {
    openarm->recv_all(200);
    auto arm_cmds = hold_current_arm_commands(openarm);
    std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
    gripper_cmds.reserve(openarm->get_gripper().get_motors().size());
    for (const auto& motor : openarm->get_gripper().get_motors()) {
        (void)motor;
        gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
            kp, kd, gripper_position, 0.0, 0.0});
    }
    openarm->get_arm().mit_control_all(arm_cmds);
    openarm->get_gripper().mit_control_all(gripper_cmds);
    openarm->recv_all(200);
}

void move_gripper_smooth(openarm::can::socket::OpenArm* openarm, double start,
                         double target, double duration, double kp, double kd) {
    const double dt = 0.002;
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int i = 0; i <= steps; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(steps);
        const double q = start + (target - start) * smoothstep(s);
        send_pose(openarm, q, kp, kd);
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
}

double current_gripper_position(openarm::can::socket::OpenArm* openarm) {
    openarm->recv_all(200);
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) return 0.0;
    return motors.front().get_position();
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_args(argc, argv, &options)) return 2;

    std::cout << "[gripper-jog] can=" << options.can_interface
              << " delta=" << options.delta
              << " step_duration=" << options.step_duration
              << " kp=" << options.kp << " kd=" << options.kd << std::endl;

    auto* openarm =
        openarm_init::OpenArmInitializer::initialize_openarm(options.can_interface, true);
    if (openarm == nullptr) {
        std::cerr << "[gripper-jog][ERROR] failed to initialize OpenArm" << std::endl;
        return 1;
    }

    const double q0 = current_gripper_position(openarm);
    const double q_plus = q0 + options.delta;
    const double q_minus = q0 - options.delta;
    std::cout << "[gripper-jog] current gripper q=" << q0 << std::endl;
    std::cout << "[gripper-jog] move q0 -> q0+delta -> q0-delta -> q0" << std::endl;

    move_gripper_smooth(openarm, q0, q_plus, options.step_duration, options.kp, options.kd);
    std::this_thread::sleep_for(std::chrono::duration<double>(options.settle_duration));
    move_gripper_smooth(openarm, q_plus, q_minus, options.step_duration * 2.0, options.kp,
                        options.kd);
    std::this_thread::sleep_for(std::chrono::duration<double>(options.settle_duration));
    move_gripper_smooth(openarm, q_minus, q0, options.step_duration, options.kp, options.kd);

    openarm_init::send_neutral_current_position_commands(openarm, 10);
    delete openarm;
    std::cout << "[gripper-jog] done" << std::endl;
    return 0;
}
