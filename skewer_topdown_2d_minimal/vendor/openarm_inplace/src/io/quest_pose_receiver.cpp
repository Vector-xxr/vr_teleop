#include <io/quest_pose_receiver.hpp>

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
#include <vector>

QuestPoseReceiver::QuestPoseReceiver() = default;

QuestPoseReceiver::~QuestPoseReceiver() { close(); }

bool QuestPoseReceiver::open(uint16_t port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "[quest-pose] socket failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[quest-pose] bind failed on port " << port << ": "
                  << std::strerror(errno) << std::endl;
        close();
        return false;
    }

    std::cout << "[quest-pose] listening on UDP port " << port << std::endl;
    return true;
}

void QuestPoseReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool QuestPoseReceiver::parse_payload(const char* data, size_t length, QuestPoseSample* sample) {
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
    if (fields.size() < 10) {
        return false;
    }

    try {
        sample->timestamp_ms = std::stoll(fields[0]);
        sample->grip_active = std::stoi(fields[1]) != 0;
        sample->trigger = std::stod(fields[2]);
        sample->position = Eigen::Vector3d(std::stod(fields[3]), std::stod(fields[4]),
                                         std::stod(fields[5]));
        sample->orientation = Eigen::Quaterniond(
            std::stod(fields[9]), std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8]));
        sample->orientation.normalize();
        sample->button_close = false;
        sample->button_open = false;
        if (fields.size() >= 12) {
            sample->button_close = std::stoi(fields[10]) != 0;
            sample->button_open = std::stoi(fields[11]) != 0;
        }
        sample->valid = sample->orientation.coeffs().allFinite() && sample->position.allFinite();
        return sample->valid;
    } catch (...) {
        return false;
    }
}

std::optional<QuestPoseSample> QuestPoseReceiver::receive_latest() {
    if (fd_ < 0) {
        return std::nullopt;
    }

    char buffer[1024];
    sockaddr_in sender {};
    socklen_t sender_len = sizeof(sender);
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();

    while (true) {
        const ssize_t nbytes = recvfrom(fd_, buffer, sizeof(buffer) - 1, 0,
                                        reinterpret_cast<sockaddr*>(&sender), &sender_len);
        if (nbytes < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                break;
            }
            std::cerr << "[quest-pose] recv failed: " << std::strerror(errno) << std::endl;
            break;
        }
        buffer[nbytes] = '\0';
        QuestPoseSample parsed;
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

std::optional<QuestPoseSample> QuestPoseReceiver::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_) {
        return std::nullopt;
    }
    return latest_;
}

QuestPoseReceiver::Stats QuestPoseReceiver::snapshot_stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats stats;
    stats.packets_total = packets_total_;
    stats.packets_window = packets_window_;
    stats.has_latest = has_latest_;
    if (has_latest_) {
        stats.last_grip = latest_.grip_active;
        stats.last_trigger = latest_.trigger;
        if (latest_.receive_timestamp_ns > 0) {
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
            stats.last_age_ms =
                static_cast<double>(now_ns - latest_.receive_timestamp_ns) * 1e-6;
        }
    }
    packets_window_ = 0;
    return stats;
}
