#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <openarm_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <robot_state.hpp>
#include <safety/safety_filter.hpp>
#include <thread>
#include <yamlloader.hpp>
#include <vector>

namespace {

std::atomic_bool keep_running {true};

struct Options {
    std::string urdf_path;
    std::string arm_side = "right_arm";
    std::string leader_can = "leader_right";
    std::string follower_can = "follower_right";
    double hz = FREQUENCY;
    double prezero_duration = 2.0;
    double prezero_target = 0.0;
};

struct GripperState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
};

void signal_handler(int) {
    keep_running = false;
}

SafetyConfig make_real_safety_config(size_t total_joints) {
    SafetyConfig config;
    config.min_position.resize(total_joints);
    config.max_position.resize(total_joints);
    config.max_velocity.resize(total_joints);
    config.max_acceleration.resize(total_joints);
    config.max_effort.resize(total_joints);
    for (size_t i = 0; i < total_joints; ++i) {
        const size_t idx = i < NJOINTS ? i : NJOINTS - 1;
        config.min_position[i] = position_limit_min_F[idx];
        config.max_position[i] = position_limit_max_F[idx];
        config.max_velocity[i] = std::min(velocity_limit_F[idx], 2.0);
        config.max_acceleration[i] = 8.0;
        config.max_effort[i] = effort_limit_F[idx];
    }
    return config;
}

std::string select_leaf_link(const std::string& arm_side) {
    return arm_side == "left_arm" ? "openarm_left_hand_tcp" : "openarm_right_hand_tcp";
}

double smoothstep(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + x * 6.0));
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " URDF [--arm-side right_arm|left_arm]"
                 " [--leader-can IFACE] [--follower-can IFACE] [--hz HZ]"
                 " [--prezero-duration SEC] [--prezero-target Q]\n";
}

bool parse_args(int argc, char** argv, Options* options) {
    int i = 1;
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
        options->urdf_path = argv[i++];
    }
    for (; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--arm-side" && i + 1 < argc) {
            options->arm_side = argv[++i];
        } else if (arg == "--leader-can" && i + 1 < argc) {
            options->leader_can = argv[++i];
        } else if (arg == "--follower-can" && i + 1 < argc) {
            options->follower_can = argv[++i];
        } else if (arg == "--hz" && i + 1 < argc) {
            options->hz = std::stod(argv[++i]);
        } else if (arg == "--prezero-duration" && i + 1 < argc) {
            options->prezero_duration = std::stod(argv[++i]);
        } else if (arg == "--prezero-target" && i + 1 < argc) {
            options->prezero_target = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "[gripper-teleop-test][ERROR] unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return false;
        }
    }
    if (options->urdf_path.empty()) {
        std::cerr << "[gripper-teleop-test][ERROR] missing URDF path" << std::endl;
        print_usage(argv[0]);
        return false;
    }
    if (options->arm_side != "right_arm" && options->arm_side != "left_arm") {
        std::cerr << "[gripper-teleop-test][ERROR] unsupported arm side: "
                  << options->arm_side << std::endl;
        return false;
    }
    options->hz = std::max(20.0, std::min(options->hz, FREQUENCY));
    options->prezero_duration = std::max(0.0, std::min(options->prezero_duration, 10.0));
    return true;
}

GripperState last_gripper_state(const std::shared_ptr<RobotSystemState>& state) {
    const auto hand = state->hand_state().get_all_responses();
    if (hand.empty()) return {};
    return {hand.back().position, hand.back().velocity, hand.back().effort};
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_args(argc, argv, &options)) return 2;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "[gripper-teleop-test] full teleop control path, gripper reference only"
              << std::endl;
    std::cout << "[gripper-teleop-test] urdf=" << options.urdf_path
              << " arm_side=" << options.arm_side
              << " leader_can=" << options.leader_can
              << " follower_can=" << options.follower_can
              << " hz=" << options.hz
              << " prezero_duration=" << options.prezero_duration
              << " prezero_target=" << options.prezero_target << std::endl;

    YamlLoader leader_loader("config/leader.yaml");
    YamlLoader follower_loader("config/follower.yaml");

    const std::string root_link = "openarm_body_link0";
    const std::string leaf_link = select_leaf_link(options.arm_side);
    Dynamics leader_arm_dynamics(options.urdf_path, root_link, leaf_link);
    Dynamics follower_arm_dynamics(options.urdf_path, root_link, leaf_link);
    leader_arm_dynamics.Init();
    follower_arm_dynamics.Init();

    std::cout << "=== Initializing Leader OpenArm ===" << std::endl;
    std::unique_ptr<openarm::can::socket::OpenArm> leader_openarm(
        openarm_init::OpenArmInitializer::initialize_openarm(options.leader_can, true));
    std::cout << "=== Initializing Follower OpenArm ===" << std::endl;
    std::unique_ptr<openarm::can::socket::OpenArm> follower_openarm(
        openarm_init::OpenArmInitializer::initialize_openarm(options.follower_can, true));

    const size_t leader_arm_motor_num = leader_openarm->get_arm().get_motors().size();
    const size_t follower_arm_motor_num = follower_openarm->get_arm().get_motors().size();
    const size_t leader_hand_motor_num = leader_openarm->get_gripper().get_motors().size();
    const size_t follower_hand_motor_num = follower_openarm->get_gripper().get_motors().size();
    std::cout << "leader arm motor num : " << leader_arm_motor_num << std::endl;
    std::cout << "follower arm motor num : " << follower_arm_motor_num << std::endl;
    std::cout << "leader hand motor num : " << leader_hand_motor_num << std::endl;
    std::cout << "follower hand motor num : " << follower_hand_motor_num << std::endl;

    auto leader_state =
        std::make_shared<RobotSystemState>(leader_arm_motor_num, leader_hand_motor_num);
    auto follower_state =
        std::make_shared<RobotSystemState>(follower_arm_motor_num, follower_hand_motor_num);

    Control control_leader(leader_openarm.get(), &leader_arm_dynamics, &follower_arm_dynamics,
                           leader_state, 1.0 / FREQUENCY, ROLE_LEADER, options.arm_side,
                           leader_arm_motor_num, leader_hand_motor_num);
    Control control_follower(follower_openarm.get(), &leader_arm_dynamics,
                             &follower_arm_dynamics, follower_state, 1.0 / FREQUENCY,
                             ROLE_FOLLOWER, options.arm_side, follower_arm_motor_num,
                             follower_hand_motor_num);

    control_leader.SetParameter(
        leader_loader.get_vector("LeaderArmParam", "Kp"),
        leader_loader.get_vector("LeaderArmParam", "Kd"),
        leader_loader.get_vector("LeaderArmParam", "Fc"),
        leader_loader.get_vector("LeaderArmParam", "k"),
        leader_loader.get_vector("LeaderArmParam", "Fv"),
        leader_loader.get_vector("LeaderArmParam", "Fo"));
    control_follower.SetParameter(
        follower_loader.get_vector("FollowerArmParam", "Kp"),
        follower_loader.get_vector("FollowerArmParam", "Kd"),
        follower_loader.get_vector("FollowerArmParam", "Fc"),
        follower_loader.get_vector("FollowerArmParam", "k"),
        follower_loader.get_vector("FollowerArmParam", "Fv"),
        follower_loader.get_vector("FollowerArmParam", "Fo"));

    control_leader.SetSafetyFilter(std::make_unique<SafetyFilter>(
        make_real_safety_config(leader_arm_motor_num + leader_hand_motor_num)));
    control_follower.SetSafetyFilter(std::make_unique<SafetyFilter>(
        make_real_safety_config(follower_arm_motor_num + follower_hand_motor_num)));

    control_leader.unilateral_step();
    control_follower.unilateral_step();
    auto follower_arm_hold = follower_state->arm_state().get_all_responses();
    auto follower_hand_ref = follower_state->hand_state().get_all_responses();
    for (auto& joint : follower_arm_hold) {
        joint.velocity = 0.0;
        joint.effort = 0.0;
    }
    for (auto& joint : follower_hand_ref) {
        joint.velocity = 0.0;
        joint.effort = 0.0;
    }
    follower_state->arm_state().set_all_references(follower_arm_hold);
    follower_state->hand_state().set_all_references(follower_hand_ref);
    control_leader.ResetSafetyFilter(leader_state->get_all_references());
    control_follower.ResetSafetyFilter(follower_state->get_all_references());

    const auto period = std::chrono::duration<double>(1.0 / options.hz);

    if (options.prezero_duration > 0.0 && !follower_hand_ref.empty()) {
        std::cout << "[gripper-teleop-test] prezero follower gripper to q="
                  << options.prezero_target << " over " << options.prezero_duration
                  << "s before teleop follow" << std::endl;
        const std::vector<JointState> prezero_start = follower_hand_ref;
        const size_t prezero_ticks = static_cast<size_t>(
            std::max(1.0, std::ceil(options.prezero_duration * options.hz)));
        auto prezero_next_time = std::chrono::steady_clock::now();
        for (size_t i = 0; keep_running && i <= prezero_ticks; ++i) {
            control_leader.unilateral_step();

            const double alpha =
                smoothstep(static_cast<double>(i) / static_cast<double>(prezero_ticks));
            auto prezero_ref = prezero_start;
            for (auto& joint : prezero_ref) {
                joint.position =
                    joint.position + (options.prezero_target - joint.position) * alpha;
                joint.velocity = 0.0;
                joint.effort = 0.0;
            }
            follower_state->arm_state().set_all_references(follower_arm_hold);
            follower_state->hand_state().set_all_references(prezero_ref);

            control_follower.unilateral_step();

            if (i % static_cast<size_t>(std::max(1.0, options.hz / 2.0)) == 0 ||
                i == prezero_ticks) {
                const GripperState follower_grip = last_gripper_state(follower_state);
                std::cout << std::fixed << std::setprecision(6)
                          << "[gripper-teleop-test][prezero] follower_q="
                          << follower_grip.position
                          << " target_q=" << prezero_ref.back().position
                          << " error="
                          << (prezero_ref.back().position - follower_grip.position)
                          << std::endl;
            }

            prezero_next_time +=
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(prezero_next_time);
        }

        for (auto& joint : follower_hand_ref) {
            joint.position = options.prezero_target;
            joint.velocity = 0.0;
            joint.effort = 0.0;
        }
        follower_state->arm_state().set_all_references(follower_arm_hold);
        follower_state->hand_state().set_all_references(follower_hand_ref);
        control_follower.ResetSafetyFilter(follower_state->get_all_references());
    }

    std::cout << "[gripper-teleop-test] follower arm joints are held at current position"
              << std::endl;
    std::cout << "[gripper-teleop-test] follower gripper reference = leader gripper q,"
                 " dq scaled to 0, tau forced to 0"
              << std::endl;

    auto next_time = std::chrono::steady_clock::now();
    size_t tick = 0;
    while (keep_running) {
        control_leader.unilateral_step();
        const auto leader_hand = leader_state->hand_state().get_all_responses();
        if (!leader_hand.empty()) {
            auto hand_ref = leader_hand;
            for (auto& joint : hand_ref) {
                joint.velocity = 0.0;
                joint.effort = 0.0;
            }
            follower_state->arm_state().set_all_references(follower_arm_hold);
            follower_state->hand_state().set_all_references(hand_ref);
        }

        control_follower.unilateral_step();

        const GripperState leader_grip = last_gripper_state(leader_state);
        const GripperState follower_grip = last_gripper_state(follower_state);

        if (++tick % static_cast<size_t>(std::max(1.0, options.hz)) == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "[gripper-teleop-test] leader_q=" << leader_grip.position
                      << " follower_q=" << follower_grip.position
                      << " follower_dq=" << follower_grip.velocity
                      << " follower_tau=" << follower_grip.effort
                      << " target_q=" << leader_grip.position
                      << " error=" << (leader_grip.position - follower_grip.position)
                      << std::endl;
        }
        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_time);
    }

    openarm_init::send_neutral_current_position_commands(follower_openarm.get(), 10);
    openarm_init::send_neutral_current_position_commands(leader_openarm.get(), 10);
    std::cout << "[gripper-teleop-test] stopped" << std::endl;
    return 0;
}
