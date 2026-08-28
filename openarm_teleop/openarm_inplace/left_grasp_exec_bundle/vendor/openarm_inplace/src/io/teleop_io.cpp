#include <io/teleop_io.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>

bool CsvRecorder::open(const std::string& path, size_t joint_count) {
    file_.open(path);
    if (!file_) {
        std::cerr << "[recorder] failed to open: " << path << std::endl;
        return false;
    }
    start_ = std::chrono::steady_clock::now();
    file_ << "time";
    for (size_t i = 0; i < joint_count; ++i) {
        file_ << ",q" << i << ",dq" << i << ",tau" << i;
    }
    file_ << "\n";
    return true;
}

void CsvRecorder::append(const std::vector<JointState>& joints) {
    if (!file_) return;
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - start_).count();
    file_ << t;
    for (const auto& joint : joints) {
        file_ << "," << joint.position << "," << joint.velocity << "," << joint.effort;
    }
    file_ << "\n";
}

void CsvRecorder::close() {
    if (file_) file_.close();
}

bool CsvRecorder::active() const { return file_.is_open(); }

bool CsvReplay::load(const std::string& path, size_t joint_count) {
    std::string face_emoji;
    frames_ = load_frames_from_csv(path, joint_count, &face_emoji);
    if (frames_.empty()) return false;
    action_events_.clear();
    action_events_.push_back({0.0, std::filesystem::path(path).filename().string(), face_emoji});
    start_ = std::chrono::steady_clock::now();
    index_ = 0;
    action_event_index_ = 0;
    std::cout << "[replay] loaded " << frames_.size() << " frames from " << path << std::endl;
    if (!face_emoji.empty()) {
        std::cout << "[replay] face_emoji=" << face_emoji << " for "
                  << std::filesystem::path(path).filename().string() << std::endl;
    }
    return true;
}

void CsvReplay::reset() {
    start_ = std::chrono::steady_clock::now();
    index_ = 0;
    action_event_index_ = 0;
}

void CsvReplay::set_time_scale(double time_scale) {
    if (!std::isfinite(time_scale) || time_scale <= 0.0) {
        std::cerr << "[replay] invalid time scale " << time_scale
                  << ", keeping " << time_scale_ << std::endl;
        return;
    }
    time_scale_ = time_scale;
    std::cout << "[replay] time scale set to " << time_scale_ << "x" << std::endl;
}

void CsvReplay::pause_for(double seconds) {
    if (seconds <= 0.0) return;
    start_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds));
}

std::vector<CsvReplay::ActionEvent> CsvReplay::consume_action_events() {
    std::vector<ActionEvent> events;
    if (action_event_index_ >= action_events_.size()) return events;
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - start_).count() * time_scale_;
    while (action_event_index_ < action_events_.size() &&
           action_events_[action_event_index_].t <= t + 1e-9) {
        events.push_back(action_events_[action_event_index_]);
        ++action_event_index_;
    }
    return events;
}

bool CsvReplay::prepend_transition_from(const std::vector<JointState>& start_joints,
                                        size_t joint_count, double transition_duration) {
    return prepend_transition_from_with_result(start_joints, joint_count, transition_duration) !=
           TransitionResult::Failed;
}

CsvReplay::TransitionResult CsvReplay::prepend_transition_from_with_result(
    const std::vector<JointState>& start_joints, size_t joint_count,
    double transition_duration) {
    if (frames_.empty()) return TransitionResult::Failed;
    if (start_joints.size() != joint_count || frames_.front().joints.size() != joint_count) {
        std::cerr << "[replay] cannot prepend initial transition: joint count mismatch" << std::endl;
        return TransitionResult::Failed;
    }

    auto transition = generate_transition(start_joints, frames_.front().joints, 0.0, joint_count,
                                             transition_duration);
    if (transition.empty()) {
        std::cout << "[replay] initial transition skipped: current pose already matches first frame" << std::endl;
        reset();
        return TransitionResult::SkippedNoMotion;
    }

    const double duration = transition.back().t;
    for (auto& frame : frames_) {
        frame.t += duration;
    }
    for (auto& event : action_events_) {
        event.t += duration;
    }

    std::vector<Frame> merged;
    merged.reserve(transition.size() + frames_.size());
    for (auto& frame : transition) {
        merged.push_back(std::move(frame));
    }
    for (auto& frame : frames_) {
        merged.push_back(std::move(frame));
    }
    frames_ = std::move(merged);
    reset();

    std::cout << "[replay] prepended current-to-first transition " << transition.size()
              << " frames (" << duration << "s)" << std::endl;
    return TransitionResult::Added;
}

namespace {
std::string trim_copy(const std::string& value) {
    size_t begin = 0;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }
    size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }
    return value.substr(begin, end - begin);
}

bool parse_face_emoji_metadata_line(const std::string& line, std::string* face_emoji) {
    if (face_emoji == nullptr) return false;
    std::string text = trim_copy(line);
    if (text.rfind("#", 0) == 0) {
        text = trim_copy(text.substr(1));
    } else if (text.rfind("//", 0) == 0) {
        text = trim_copy(text.substr(2));
    } else {
        return false;
    }

    const std::vector<std::string> keys = {"face_emoji", "face-emoji", "emoji"};
    for (const auto& key : keys) {
        if (text.rfind(key, 0) != 0) continue;
        std::string value = trim_copy(text.substr(key.size()));
        if (!value.empty() && (value[0] == '=' || value[0] == ':')) {
            value = trim_copy(value.substr(1));
        }
        if (!value.empty()) {
            *face_emoji = value;
            return true;
        }
    }
    return false;
}
}  // namespace

std::vector<CsvReplay::Frame> CsvReplay::load_frames_from_csv(
    const std::string& path, size_t joint_count, std::string* face_emoji) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "[replay] failed to open: " << path << std::endl;
        return {};
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '#' || (line.size() >= 2 && line[0] == '/' && line[1] == '/')) {
            parse_face_emoji_metadata_line(line, face_emoji);
            continue;
        }
        break;  // header
    }

    std::vector<Frame> file_frames;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(ss, cell, ',')) {
            try {
                values.push_back(std::stod(cell));
            } catch (const std::exception&) {
                values.clear();
                break;
            }
        }
        const bool has_effort = values.size() == 1 + 3 * joint_count;
        if (values.size() != 1 + 2 * joint_count && !has_effort) continue;

        Frame frame;
        frame.t = values[0];
        frame.joints.resize(joint_count);
        for (size_t i = 0; i < joint_count; ++i) {
            const size_t stride = has_effort ? 3 : 2;
            frame.joints[i].position = values[1 + stride * i];
            frame.joints[i].velocity = values[1 + stride * i + 1];
            frame.joints[i].effort = has_effort ? values[1 + stride * i + 2] : 0.0;
        }
        file_frames.push_back(frame);
    }
    return file_frames;
}

std::vector<CsvReplay::Frame> CsvReplay::generate_transition(
    const std::vector<JointState>& start_joints,
    const std::vector<JointState>& end_joints,
    double time_offset, size_t joint_count, double transition_duration) {
    // Check if positions differ significantly
    double max_disp = 0.0;
    for (size_t i = 0; i < joint_count; ++i) {
        max_disp = std::max(max_disp, std::abs(end_joints[i].position - start_joints[i].position));
    }

    if (max_disp < 1e-6) return {};  // No significant position gap

    // Calculate max boundary velocity for duration estimation
    double max_vel = 0.0;
    for (size_t i = 0; i < joint_count; ++i) {
        max_vel = std::max(max_vel, std::abs(start_joints[i].velocity));
        max_vel = std::max(max_vel, std::abs(end_joints[i].velocity));
    }

    // Duration: enough for both displacement and velocity changes
    const double v_ref = 1.0;   // reference velocity (rad/s)
    const double a_ref = 2.0;   // reference acceleration (rad/s^2)
    double duration = transition_duration > 0.0
                          ? transition_duration
                          : std::max({max_disp / v_ref, max_vel / a_ref, 0.5});
    duration = std::min(std::max(duration, 0.05), 10.0);

    const double dt = 0.001;  // 1000 Hz
    const size_t num_steps = static_cast<size_t>(std::round(duration / dt));
    const double T = duration;

    std::vector<Frame> transition_frames;
    transition_frames.reserve(num_steps + 1);

    for (size_t step = 0; step <= num_steps; ++step) {
        const double tau = static_cast<double>(step) / static_cast<double>(num_steps);

        Frame frame;
        frame.t = time_offset + tau * duration;
        frame.joints.resize(joint_count);

        for (size_t i = 0; i < joint_count; ++i) {
            const double p0 = start_joints[i].position;
            const double p1 = end_joints[i].position;
            const double v0 = start_joints[i].velocity;
            const double v1 = end_joints[i].velocity;
            const double dp = p1 - p0;

            // Quintic polynomial: q(tau) = p0 + v0*T*tau + a3*tau^3 + a4*tau^4 + a5*tau^5
            // Boundary conditions:
            //   q(0)=p0, q(1)=p1, q'(0)=v0*T, q'(1)=v1*T, q''(0)=0, q''(1)=0
            const double a3 = 10.0 * dp - 6.0 * v0 * T - 4.0 * v1 * T;
            const double a4 = -15.0 * dp + 8.0 * v0 * T + 7.0 * v1 * T;
            const double a5 = 6.0 * dp - 3.0 * (v0 + v1) * T;

            const double tau2 = tau * tau;
            const double tau3 = tau2 * tau;
            const double tau4 = tau3 * tau;
            const double tau5 = tau4 * tau;

            frame.joints[i].position = p0 + v0 * T * tau + a3 * tau3 + a4 * tau4 + a5 * tau5;
            frame.joints[i].velocity =
                v0 + (3.0 * a3 * tau2 + 4.0 * a4 * tau3 + 5.0 * a5 * tau4) / T;
            frame.joints[i].effort = (1.0 - tau) * start_joints[i].effort + tau * end_joints[i].effort;
        }

        transition_frames.push_back(frame);
    }

    return transition_frames;
}

bool CsvReplay::is_transition_file(const std::string& filename) {
    // Match pattern: {number}_{number}.csv  (e.g. 0_1.csv, 1_2.csv)
    size_t last_slash = filename.find_last_of('/');
    std::string basename =
        (last_slash != std::string::npos) ? filename.substr(last_slash + 1) : filename;
    if (basename.size() < 5) return false;  // minimum: "0_1.csv"
    // Must end with .csv
    if (basename.compare(basename.size() - 4, 4, ".csv") != 0) return false;
    // Remove .csv suffix
    std::string stem = basename.substr(0, basename.size() - 4);
    // Must contain exactly one underscore, with digits on both sides
    size_t us = stem.find('_');
    if (us == std::string::npos || us == 0 || us == stem.size() - 1) return false;
    // Check for second underscore (not allowed)
    if (stem.find('_', us + 1) != std::string::npos) return false;
    // Both parts must be all digits
    for (size_t i = 0; i < us; ++i) {
        if (stem[i] < '0' || stem[i] > '9') return false;
    }
    for (size_t i = us + 1; i < stem.size(); ++i) {
        if (stem[i] < '0' || stem[i] > '9') return false;
    }
    return true;
}

bool CsvReplay::save_frames_to_csv(const std::string& path,
                                    const std::vector<Frame>& frames,
                                    size_t joint_count) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "[replay] failed to create transition file: " << path << std::endl;
        return false;
    }

    // Header
    out << "time";
    for (size_t i = 0; i < joint_count; ++i) {
        out << ",q" << i << ",dq" << i << ",tau" << i;
    }
    out << "\n";

    // Data rows (time starts from 0 in the file)
    double t_start = frames.empty() ? 0.0 : frames.front().t;
    for (const auto& frame : frames) {
        out << (frame.t - t_start);
        for (const auto& joint : frame.joints) {
            out << "," << joint.position << "," << joint.velocity << "," << joint.effort;
        }
        out << "\n";
    }

    std::cout << "[replay] saved transition file: " << path << std::endl;
    return true;
}

int CsvReplay::clean_transitions(const std::string& dir_path) {
    namespace fs = std::filesystem;
    if (!fs::is_directory(dir_path)) return 0;

    int removed = 0;
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            if (is_transition_file(entry.path().filename().string())) {
                fs::remove(entry.path());
                std::cout << "[replay] deleted " << entry.path().filename().string() << std::endl;
                ++removed;
            }
        }
    }
    std::cout << "[replay] cleaned " << removed << " transition file(s)" << std::endl;
    return removed;
}

bool CsvReplay::load_directory(const std::string& dir_path, size_t joint_count,
                                bool generate_transitions, double transition_duration) {
    namespace fs = std::filesystem;

    if (!fs::is_directory(dir_path)) {
        std::cerr << "[replay] not a directory: " << dir_path << std::endl;
        return false;
    }

    // Collect only non-transition CSV files
    std::vector<fs::path> csv_files;
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            if (!is_transition_file(entry.path().filename().string())) {
                csv_files.push_back(entry.path());
            }
        }
    }

    if (csv_files.empty()) {
        std::cerr << "[replay] no CSV files found in: " << dir_path << std::endl;
        return false;
    }

    // Sort by leading number in filename
    auto extract_leading_number = [](const fs::path& p) -> int {
        std::string name = p.stem().string();
        size_t i = 0;
        while (i < name.size() && (name[i] < '0' || name[i] > '9')) ++i;
        if (i >= name.size()) return 0;
        size_t start = i;
        while (i < name.size() && name[i] >= '0' && name[i] <= '9') ++i;
        try {
            return std::stoi(name.substr(start, i - start));
        } catch (...) {
            return 0;
        }
    };

    std::sort(csv_files.begin(), csv_files.end(),
              [&extract_leading_number](const fs::path& a, const fs::path& b) {
                  return extract_leading_number(a) < extract_leading_number(b);
              });

    // Load all action files into a struct to preserve metadata
    struct ActionFile {
        fs::path path;
        int seq;  // leading number from filename
        std::string face_emoji;
        std::vector<Frame> frames;
    };
    std::vector<ActionFile> actions;
    actions.reserve(csv_files.size());

    for (const auto& csv_path : csv_files) {
        std::string face_emoji;
        auto file_frames = load_frames_from_csv(csv_path.string(), joint_count, &face_emoji);
        if (file_frames.empty()) {
            std::cerr << "[replay] no valid frames in: " << csv_path << std::endl;
            continue;
        }
        int seq = extract_leading_number(csv_path);
        std::cout << "[replay] loaded " << file_frames.size() << " frames from "
                  << csv_path.filename().string() << " (seq=" << seq << ")" << std::endl;
        if (!face_emoji.empty()) {
            std::cout << "[replay] face_emoji=" << face_emoji << " for "
                      << csv_path.filename().string() << std::endl;
        }
        actions.push_back({csv_path, seq, face_emoji, std::move(file_frames)});
    }

    if (actions.empty()) return false;

    frames_.clear();
    action_events_.clear();
    double time_offset = 0.0;

    // Initial current-to-first transition is prepended after loading, using the real current
    // follower pose. Do not generate a zero-pose transition here for real hardware replay.

    // Process each action file with transitions between them
    for (size_t idx = 0; idx < actions.size(); ++idx) {
        auto& action = actions[idx];

        // Save first frame info before moving (needed for transition)
        auto first_joints = action.frames.front().joints;
        double file_duration = action.frames.back().t;

        // Generate transition from previous action to this one
        if (generate_transitions && idx > 0) {
            // Use the last frame already in frames_ (from previous action)
            const auto& prev_last = frames_.back();

            auto transition = generate_transition(
                prev_last.joints, first_joints,
                time_offset, joint_count, transition_duration);

            if (!transition.empty()) {
                double transition_duration = transition.back().t - time_offset;
                std::cout << "[replay] transition " << transition.size()
                          << " frames (" << transition_duration << "s) "
                          << actions[idx - 1].path.filename().string()
                          << " -> " << action.path.filename().string() << std::endl;

                // Save transition file: {prev_seq}_{curr_seq}.csv
                std::string trans_name =
                    std::to_string(actions[idx - 1].seq) + "_" +
                    std::to_string(action.seq) + ".csv";
                fs::path trans_path = fs::path(dir_path) / trans_name;
                save_frames_to_csv(trans_path.string(), transition, joint_count);

                time_offset += transition_duration;
                for (auto& frame : transition) {
                    frames_.push_back(std::move(frame));
                }
            }
        }

        // Add action frames with time offset
        action_events_.push_back(
            {time_offset, action.path.filename().string(), action.face_emoji});
        for (auto& frame : action.frames) {
            frame.t += time_offset;
            frames_.push_back(std::move(frame));
        }
        time_offset += file_duration;
    }

    start_ = std::chrono::steady_clock::now();
    index_ = 0;
    action_event_index_ = 0;
    std::cout << "[replay] total " << frames_.size() << " frames from " << actions.size()
              << " file(s), duration " << time_offset << "s" << std::endl;
    return !frames_.empty();
}

std::optional<std::vector<JointState>> CsvReplay::sample() {
    if (frames_.empty() || index_ >= frames_.size()) return std::nullopt;
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - start_).count() * time_scale_;
    while (index_ + 1 < frames_.size() && frames_[index_ + 1].t <= t) {
        ++index_;
    }
    if (frames_[index_].t <= t) {
        auto joints = frames_[index_].joints;
        for (auto& joint : joints) {
            joint.velocity *= time_scale_;
        }
        return joints;
    }
    return std::nullopt;
}

bool CsvReplay::finished() const { return !frames_.empty() && index_ + 1 >= frames_.size(); }

UdpPanelReceiver::UdpPanelReceiver() = default;

UdpPanelReceiver::~UdpPanelReceiver() { close(); }

bool UdpPanelReceiver::open(uint16_t port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "[panel] socket failed: " << std::strerror(errno) << std::endl;
        return false;
    }
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[panel] bind failed on 127.0.0.1:" << port << ": "
                  << std::strerror(errno) << std::endl;
        close();
        return false;
    }
    const int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    std::cout << "[panel] listening on 127.0.0.1:" << port << std::endl;
    return true;
}

std::optional<std::vector<JointState>> UdpPanelReceiver::receive_latest(size_t joint_count) {
    if (fd_ < 0) return std::nullopt;
    std::optional<std::vector<JointState>> latest;
    char buffer[2048];
    while (true) {
        const ssize_t n = ::recv(fd_, buffer, sizeof(buffer) - 1, 0);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "[panel] recv failed: " << std::strerror(errno) << std::endl;
            }
            break;
        }
        buffer[n] = '\0';
        std::stringstream ss(buffer);
        std::string cell;
        std::vector<double> values;
        while (std::getline(ss, cell, ',')) {
            try {
                values.push_back(std::stod(cell));
            } catch (const std::exception&) {
                values.clear();
                break;
            }
        }
        if (values.size() != joint_count && values.size() != 2 * joint_count) continue;
        std::vector<JointState> joints(joint_count);
        if (values.size() == joint_count) {
            for (size_t i = 0; i < joint_count; ++i) {
                joints[i].position = values[i];
                joints[i].velocity = 0.0;
                joints[i].effort = 0.0;
            }
        } else {
            for (size_t i = 0; i < joint_count; ++i) {
                joints[i].position = values[2 * i];
                joints[i].velocity = values[2 * i + 1];
                joints[i].effort = 0.0;
            }
        }
        latest = joints;
    }
    return latest;
}

void UdpPanelReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

UdpTextReceiver::UdpTextReceiver() = default;

UdpTextReceiver::~UdpTextReceiver() { close(); }

bool UdpTextReceiver::open(uint16_t port, const std::string& label) {
    close();
    label_ = label;
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "[" << label_ << "] socket failed: " << std::strerror(errno)
                  << std::endl;
        return false;
    }
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[" << label_ << "] bind failed on 127.0.0.1:" << port << ": "
                  << std::strerror(errno) << std::endl;
        close();
        return false;
    }
    const int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    std::cout << "[" << label_ << "] listening on 127.0.0.1:" << port << std::endl;
    return true;
}

std::optional<std::string> UdpTextReceiver::receive_latest() {
    auto messages = receive_all();
    if (messages.empty()) {
        return std::nullopt;
    }
    return messages.back();
}

std::vector<std::string> UdpTextReceiver::receive_all() {
    std::vector<std::string> messages;
    if (fd_ < 0) return messages;

    char buffer[512];
    while (true) {
        const ssize_t n = ::recv(fd_, buffer, sizeof(buffer) - 1, 0);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "[" << label_ << "] recv failed: " << std::strerror(errno)
                          << std::endl;
            }
            break;
        }
        buffer[n] = '\0';
        messages.emplace_back(buffer);
    }
    return messages;
}

void UdpTextReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}
