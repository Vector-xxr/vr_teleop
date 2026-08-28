#include <io/skeleton_hint_receiver.hpp>

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

bool parse_vec3(const std::vector<std::string>& fields, size_t i0, Eigen::Vector3d* out) {
    if (out == nullptr || fields.size() < i0 + 3) {
        return false;
    }
    try {
        (*out)(0) = std::stod(fields[i0]);
        (*out)(1) = std::stod(fields[i0 + 1]);
        (*out)(2) = std::stod(fields[i0 + 2]);
        return out->allFinite();
    } catch (...) {
        return false;
    }
}
}  // namespace

SkeletonHintReceiver::SkeletonHintReceiver() = default;

SkeletonHintReceiver::~SkeletonHintReceiver() { close(); }

bool SkeletonHintReceiver::open(uint16_t port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "[skel-hint] socket failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[skel-hint] bind failed on port " << port << ": " << std::strerror(errno)
                  << std::endl;
        close();
        return false;
    }

    std::cout << "[skel-hint] listening on UDP port " << port
              << " (elbow-swivel hints, v1/v2)" << std::endl;
    return true;
}

void SkeletonHintReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SkeletonHintReceiver::parse_payload(const char* data, size_t length,
                                         SkeletonHintSample* sample) {
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
    if (fields.size() < 6) {
        return false;
    }

    try {
        sample->timestamp_ms = std::stoll(fields[0]);
        sample->valid = std::stoi(fields[1]) != 0;
        sample->phi_left = std::stod(fields[2]);
        sample->phi_right = std::stod(fields[3]);
        sample->conf_left = std::stod(fields[4]);
        sample->conf_right = std::stod(fields[5]);
        sample->has_bones = false;
        sample->u_upper_left.setZero();
        sample->u_fore_left.setZero();
        sample->u_upper_right.setZero();
        sample->u_fore_right.setZero();

        const bool finite = std::isfinite(sample->phi_left) && std::isfinite(sample->phi_right) &&
                            std::isfinite(sample->conf_left) && std::isfinite(sample->conf_right);
        if (!finite) {
            return false;
        }

        // Optional v2 bone directions.
        if (fields.size() >= 18) {
            Eigen::Vector3d ul, fl, ur, fr;
            if (parse_vec3(fields, 6, &ul) && parse_vec3(fields, 9, &fl) &&
                parse_vec3(fields, 12, &ur) && parse_vec3(fields, 15, &fr)) {
                sample->u_upper_left = ul;
                sample->u_fore_left = fl;
                sample->u_upper_right = ur;
                sample->u_fore_right = fr;
                sample->has_bones = (ul.norm() > 1e-6 || fl.norm() > 1e-6 || ur.norm() > 1e-6 ||
                                     fr.norm() > 1e-6);
            }
        }
        return true;
    } catch (...) {
        return false;
    }
}

std::optional<SkeletonHintSample> SkeletonHintReceiver::receive_latest() {
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
            std::cerr << "[skel-hint] recv failed: " << std::strerror(errno) << std::endl;
            break;
        }
        buffer[nbytes] = '\0';
        SkeletonHintSample parsed;
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

std::optional<SkeletonHintSample> SkeletonHintReceiver::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_) {
        return std::nullopt;
    }
    return latest_;
}

double SkeletonHintReceiver::latest_age_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_ || latest_.receive_timestamp_ns <= 0) {
        return 1.0e9;
    }
    return static_cast<double>(steady_now_ns() - latest_.receive_timestamp_ns) * 1e-6;
}

SkeletonHintReceiver::Stats SkeletonHintReceiver::snapshot_stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats stats;
    stats.packets_total = packets_total_;
    stats.packets_window = packets_window_;
    stats.has_latest = has_latest_;
    if (has_latest_) {
        stats.last_valid = latest_.valid;
        stats.last_phi_left = latest_.phi_left;
        stats.last_phi_right = latest_.phi_right;
        stats.last_conf_left = latest_.conf_left;
        stats.last_conf_right = latest_.conf_right;
        stats.last_has_bones = latest_.has_bones;
        if (latest_.receive_timestamp_ns > 0) {
            stats.last_age_ms =
                static_cast<double>(steady_now_ns() - latest_.receive_timestamp_ns) * 1e-6;
        }
    }
    packets_window_ = 0;
    return stats;
}
