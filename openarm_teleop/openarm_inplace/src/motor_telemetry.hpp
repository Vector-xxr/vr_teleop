// Motor feedback helpers: Damiao MIT status error codes + MOS/rotor temps.
// Used by Quest teleop for fault visibility and continuous motor_status.log.

#pragma once

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

inline const char* motor_fault_text(int code) {
    switch (code) {
        case 0:
            return "ok";
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

inline bool is_motor_fault_code(int code) { return code >= 8 && code <= 14; }

inline int motor_gripper_error_code(openarm::can::socket::OpenArm* openarm) {
    if (openarm == nullptr) {
        return 0;
    }
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) {
        return 0;
    }
    return motors.front().get_error_code();
}

inline int motor_gripper_tmos(openarm::can::socket::OpenArm* openarm) {
    if (openarm == nullptr) {
        return 0;
    }
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) {
        return 0;
    }
    return motors.front().get_state_tmos();
}

inline int motor_gripper_trotor(openarm::can::socket::OpenArm* openarm) {
    if (openarm == nullptr) {
        return 0;
    }
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) {
        return 0;
    }
    return motors.front().get_state_trotor();
}

inline double motor_gripper_tau(openarm::can::socket::OpenArm* openarm) {
    if (openarm == nullptr) {
        return 0.0;
    }
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) {
        return 0.0;
    }
    return motors.front().get_torque();
}

inline double motor_gripper_q(openarm::can::socket::OpenArm* openarm) {
    if (openarm == nullptr) {
        return 0.0;
    }
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) {
        return 0.0;
    }
    return motors.front().get_position();
}

inline int motor_gripper_enabled(openarm::can::socket::OpenArm* openarm) {
    if (openarm == nullptr) {
        return 0;
    }
    const auto& motors = openarm->get_gripper().get_motors();
    if (motors.empty()) {
        return 0;
    }
    return motors.front().is_enabled() ? 1 : 0;
}

// Collect arm(7)+gripper(1) error codes for one OpenArm (size 8).
inline std::vector<int> collect_arm_error_codes(openarm::can::socket::OpenArm* openarm) {
    std::vector<int> values;
    values.reserve(8);
    if (openarm != nullptr) {
        for (const auto& motor : openarm->get_arm().get_motors()) {
            values.push_back(motor.get_error_code());
        }
        for (const auto& motor : openarm->get_gripper().get_motors()) {
            values.push_back(motor.get_error_code());
        }
    }
    if (values.size() < 8) {
        values.resize(8, 0);
    }
    if (values.size() > 8) {
        values.resize(8);
    }
    return values;
}

inline std::vector<int> collect_arm_tmos(openarm::can::socket::OpenArm* openarm) {
    std::vector<int> values;
    values.reserve(8);
    if (openarm != nullptr) {
        for (const auto& motor : openarm->get_arm().get_motors()) {
            values.push_back(motor.get_state_tmos());
        }
        for (const auto& motor : openarm->get_gripper().get_motors()) {
            values.push_back(motor.get_state_tmos());
        }
    }
    if (values.size() < 8) {
        values.resize(8, 0);
    }
    if (values.size() > 8) {
        values.resize(8);
    }
    return values;
}

inline std::string motor_channel_name(const char* side, size_t local_idx) {
    std::ostringstream name;
    name << side << "-";
    if (local_idx < 7) {
        name << "J" << (local_idx + 1);
    } else {
        name << "Grip";
    }
    return name.str();
}

// Continuous motor_status.log + event motor_fault.log.
class MotorFaultMonitor {
public:
    // Open both logs under the session directory (creates files immediately).
    void set_log_dir(const std::string& dir) {
        dir_ = dir;
        open_files();
    }

    // Backward-compatible single-path setter (fault events only).
    void set_log_path(const std::string& path) {
        path_fault_ = path;
        open_files();
    }

    void set_status_hz(double hz) {
        status_period_s_ = (hz > 1e-6) ? (1.0 / hz) : 0.1;
    }

    void check_arm(const char* side, openarm::can::socket::OpenArm* openarm) {
        if (openarm == nullptr) {
            return;
        }
        const auto codes = collect_arm_error_codes(openarm);
        const auto tmos = collect_arm_tmos(openarm);
        const auto& arm_motors = openarm->get_arm().get_motors();
        const auto& grip_motors = openarm->get_gripper().get_motors();

        auto trotor_at = [&](size_t i) -> int {
            if (i < arm_motors.size()) {
                return arm_motors[i].get_state_trotor();
            }
            if (!grip_motors.empty()) {
                return grip_motors.front().get_state_trotor();
            }
            return 0;
        };

        auto& last = (side[0] == 'l' || side[0] == 'L') ? last_left_ : last_right_;
        if (last.size() != codes.size()) {
            last.assign(codes.size(), -1);
        }

        const auto now = std::chrono::steady_clock::now();
        const bool periodic =
            last_fault_print_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_fault_print_).count() >= 1.0;

        bool printed = false;
        for (size_t i = 0; i < codes.size(); ++i) {
            const int code = codes[i];
            if (!is_motor_fault_code(code)) {
                if (last[i] >= 8 && last[i] <= 14 && code == 0) {
                    emit_fault(side, i, 0, tmos[i], trotor_at(i), /*recovered=*/true);
                    printed = true;
                }
                last[i] = code;
                continue;
            }
            if (!periodic && code == last[i]) {
                continue;
            }
            emit_fault(side, i, code, tmos[i], trotor_at(i), /*recovered=*/false);
            printed = true;
            last[i] = code;
        }
        if (printed) {
            last_fault_print_ = now;
        }
    }

    // Sample both arms into motor_status.log (periodic) and check faults.
    void sample_bimanual(openarm::can::socket::OpenArm* left,
                         openarm::can::socket::OpenArm* right,
                         double left_grip_tgt = 0.0, double right_grip_tgt = 0.0,
                         int left_grip_mot = 0, int right_grip_mot = 0) {
        check_arm("left", left);
        check_arm("right", right);

        const auto now = std::chrono::steady_clock::now();
        if (last_status_write_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_status_write_).count() < status_period_s_) {
            return;
        }
        last_status_write_ = now;
        ensure_status_file();
        if (!status_file_.is_open()) {
            return;
        }

        const auto wall = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(wall);
        std::tm tm{};
        localtime_r(&t, &tm);

        const auto l_err = collect_arm_error_codes(left);
        const auto r_err = collect_arm_error_codes(right);
        const auto l_mos = collect_arm_tmos(left);
        const auto r_mos = collect_arm_tmos(right);

        status_file_ << std::put_time(&tm, "%H:%M:%S");
        // Left 8 then right 8 error codes
        for (int c : l_err) {
            status_file_ << "," << c;
        }
        for (int c : r_err) {
            status_file_ << "," << c;
        }
        // Gripper detail
        status_file_ << "," << motor_gripper_q(left) << "," << left_grip_tgt << ","
                     << motor_gripper_tau(left) << "," << left_grip_mot << ","
                     << motor_gripper_enabled(left) << "," << l_mos[7]
                     << "," << motor_gripper_q(right) << "," << right_grip_tgt << ","
                     << motor_gripper_tau(right) << "," << right_grip_mot << ","
                     << motor_gripper_enabled(right) << "," << r_mos[7] << "\n";
        status_file_.flush();
    }

    void check_bimanual(openarm::can::socket::OpenArm* left,
                        openarm::can::socket::OpenArm* right) {
        sample_bimanual(left, right);
    }

    void log_event(const std::string& line) {
        ensure_fault_file();
        std::cout << line << std::endl;
        if (!fault_file_.is_open()) {
            return;
        }
        const auto wall = std::chrono::system_clock::now();
        const std::time_t t = std::chrono::system_clock::to_time_t(wall);
        std::tm tm{};
        localtime_r(&t, &tm);
        fault_file_ << std::put_time(&tm, "%H:%M:%S") << " " << line << std::endl;
        fault_file_.flush();
    }

    const std::string& status_path() const { return path_status_; }
    const std::string& fault_path() const { return path_fault_; }

private:
    void open_files() {
        if (!dir_.empty()) {
            path_status_ = dir_ + "/motor_status.log";
            if (path_fault_.empty()) {
                path_fault_ = dir_ + "/motor_fault.log";
            }
        }
        // Force recreate handles on next write.
        if (status_file_.is_open()) {
            status_file_.close();
        }
        if (fault_file_.is_open()) {
            fault_file_.close();
        }
        status_header_written_ = false;
        fault_header_written_ = false;
        ensure_status_file();
        ensure_fault_file();
    }

    void ensure_status_file() {
        if (status_file_.is_open() || path_status_.empty()) {
            return;
        }
        status_file_.open(path_status_, std::ios::out | std::ios::trunc);
        if (!status_file_.is_open()) {
            std::cerr << "[motor-status] failed to open log: " << path_status_ << std::endl;
            return;
        }
        if (!status_header_written_) {
            status_file_
                << "# time,"
                << "L_J1_err,L_J2_err,L_J3_err,L_J4_err,L_J5_err,L_J6_err,L_J7_err,L_Grip_err,"
                << "R_J1_err,R_J2_err,R_J3_err,R_J4_err,R_J5_err,R_J6_err,R_J7_err,R_Grip_err,"
                << "L_grip_q,L_grip_tgt,L_grip_tau,L_grip_mot,L_grip_en,L_grip_tmos,"
                << "R_grip_q,R_grip_tgt,R_grip_tau,R_grip_mot,R_grip_en,R_grip_tmos\n"
                << "# err codes: 0=ok 8=OV 9=UV 10=OC 11=MOS_OT 12=rotor_OT 13=comm 14=overload\n";
            status_header_written_ = true;
            status_file_.flush();
            std::cout << "[quest-cartesian] motor status log -> " << path_status_ << std::endl;
        }
    }

    void ensure_fault_file() {
        if (fault_file_.is_open() || path_fault_.empty()) {
            return;
        }
        fault_file_.open(path_fault_, std::ios::out | std::ios::app);
        if (!fault_file_.is_open()) {
            std::cerr << "[motor-fault] failed to open log: " << path_fault_ << std::endl;
            return;
        }
        if (!fault_header_written_) {
            fault_file_ << "# motor fault / gripper-slip events\n";
            fault_header_written_ = true;
            fault_file_.flush();
            std::cout << "[quest-cartesian] motor fault log -> " << path_fault_ << std::endl;
        }
    }

    void emit_fault(const char* side, size_t local_idx, int code, int t_mos, int t_rotor,
                    bool recovered) {
        std::ostringstream oss;
        oss << "[motor-fault] " << motor_channel_name(side, local_idx);
        if (recovered) {
            oss << " recovered (was fault)";
        } else {
            oss << " code=" << code << " (" << motor_fault_text(code) << ")"
                << " t_mos=" << t_mos << "C t_rotor=" << t_rotor << "C";
        }
        log_event(oss.str());
    }

    std::string dir_;
    std::string path_status_;
    std::string path_fault_;
    std::ofstream status_file_;
    std::ofstream fault_file_;
    bool status_header_written_ = false;
    bool fault_header_written_ = false;
    double status_period_s_ = 0.1;  // 10 Hz continuous CSV
    std::vector<int> last_left_;
    std::vector<int> last_right_;
    std::chrono::steady_clock::time_point last_fault_print_{};
    std::chrono::steady_clock::time_point last_status_write_{};
};
