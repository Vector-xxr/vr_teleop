#include <io/skeleton_pose_receiver.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool parse_arm(const std::vector<std::string>& fields, size_t i, SkeletonPoseArm* arm) {
    if (arm == nullptr || i + 8 >= fields.size()) {
        return false;
    }
    try {
        arm->u_upper = Eigen::Vector3d(std::stod(fields[i]), std::stod(fields[i + 1]),
                                       std::stod(fields[i + 2]));
        arm->u_fore = Eigen::Vector3d(std::stod(fields[i + 3]), std::stod(fields[i + 4]),
                                      std::stod(fields[i + 5]));
        arm->L1 = std::stod(fields[i + 6]);
        arm->L2 = std::stod(fields[i + 7]);
        arm->conf = std::stod(fields[i + 8]);
        return arm->u_upper.allFinite() && arm->u_fore.allFinite() && std::isfinite(arm->L1) &&
               std::isfinite(arm->L2) && std::isfinite(arm->conf);
    } catch (...) {
        return false;
    }
}
}  // namespace

SkeletonPoseReceiver::SkeletonPoseReceiver() = default;
SkeletonPoseReceiver::~SkeletonPoseReceiver() { close(); }

bool SkeletonPoseReceiver::open(uint16_t port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "[skel-pose] socket failed: " << std::strerror(errno) << std::endl;
        return false;
    }
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[skel-pose] bind failed on port " << port << ": " << std::strerror(errno)
                  << std::endl;
        close();
        return false;
    }
    std::cout << "[skel-pose] listening on UDP port " << port << " (SEW dirs for skeleton_proximal)"
              << std::endl;
    return true;
}

void SkeletonPoseReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SkeletonPoseReceiver::parse_payload(const char* data, size_t length,
                                         SkeletonPoseSample* sample) {
    if (sample == nullptr || data == nullptr || length == 0) {
        return false;
    }
    std::string text(data, length);
    std::stringstream stream(text);
    std::string token;
    std::vector<std::string> fields;
    while (std::getline(stream, token, ',')) {
        fields.push_back(token);
    }
    // ts,valid + 9 left + 9 right = 20 fields
    if (fields.size() < 20) {
        return false;
    }
    try {
        sample->timestamp_ms = std::stoll(fields[0]);
        sample->valid = std::stoi(fields[1]) != 0;
        if (!parse_arm(fields, 2, &sample->left) || !parse_arm(fields, 11, &sample->right)) {
            return false;
        }
        return true;
    } catch (...) {
        return false;
    }
}

std::optional<SkeletonPoseSample> SkeletonPoseReceiver::receive_latest() {
    if (fd_ < 0) {
        return std::nullopt;
    }
    char buffer[1024];
    sockaddr_in sender {};
    socklen_t sender_len = sizeof(sender);
    const int64_t now_ns = steady_now_ns();
    while (true) {
        const ssize_t nbytes = recvfrom(fd_, buffer, sizeof(buffer) - 1, 0,
                                        reinterpret_cast<sockaddr*>(&sender), &sender_len);
        if (nbytes < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                break;
            }
            std::cerr << "[skel-pose] recv failed: " << std::strerror(errno) << std::endl;
            break;
        }
        buffer[nbytes] = '\0';
        SkeletonPoseSample parsed;
        if (parse_payload(buffer, static_cast<size_t>(nbytes), &parsed)) {
            std::lock_guard<std::mutex> lock(mutex_);
            parsed.receive_timestamp_ns = now_ns;
            parsed.sequence_id = ++sequence_counter_;
            latest_ = parsed;
            has_latest_ = true;
            ++packets_total_;
            ++packets_window_;
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_) {
        return std::nullopt;
    }
    return latest_;
}

std::optional<SkeletonPoseSample> SkeletonPoseReceiver::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_) {
        return std::nullopt;
    }
    return latest_;
}

double SkeletonPoseReceiver::latest_age_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_ || latest_.receive_timestamp_ns <= 0) {
        return 1.0e9;
    }
    return static_cast<double>(steady_now_ns() - latest_.receive_timestamp_ns) * 1e-6;
}

SkeletonPoseReceiver::Stats SkeletonPoseReceiver::snapshot_stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats stats;
    stats.packets_total = packets_total_;
    stats.packets_window = packets_window_;
    stats.has_latest = has_latest_;
    if (has_latest_) {
        stats.last_valid = latest_.valid;
        stats.last_conf_left = latest_.left.conf;
        stats.last_conf_right = latest_.right.conf;
        if (latest_.receive_timestamp_ns > 0) {
            stats.last_age_ms =
                static_cast<double>(steady_now_ns() - latest_.receive_timestamp_ns) * 1e-6;
        }
    }
    packets_window_ = 0;
    return stats;
}
