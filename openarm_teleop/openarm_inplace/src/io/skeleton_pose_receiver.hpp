#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include <Eigen/Dense>

// SEW direction stream for --teleop-mode skeleton_proximal (UDP, default 47951).
// Independent of the phi-only SkeletonHintReceiver on 47950.
struct SkeletonPoseArm {
    Eigen::Vector3d u_upper = Eigen::Vector3d::Zero();  // normalize(E - S)
    Eigen::Vector3d u_fore = Eigen::Vector3d::Zero();   // normalize(W - E)
    double L1 = 0.0;
    double L2 = 0.0;
    double conf = 0.0;
};

struct SkeletonPoseSample {
    int64_t timestamp_ms = 0;
    bool valid = false;
    SkeletonPoseArm left;
    SkeletonPoseArm right;
    int64_t receive_timestamp_ns = 0;
    uint64_t sequence_id = 0;
};

class SkeletonPoseReceiver {
public:
    struct Stats {
        uint64_t packets_total = 0;
        uint64_t packets_window = 0;
        bool has_latest = false;
        bool last_valid = false;
        double last_conf_left = 0.0;
        double last_conf_right = 0.0;
        double last_age_ms = -1.0;
    };

    SkeletonPoseReceiver();
    ~SkeletonPoseReceiver();

    bool open(uint16_t port);
    std::optional<SkeletonPoseSample> receive_latest();
    std::optional<SkeletonPoseSample> latest() const;
    double latest_age_ms() const;
    Stats snapshot_stats();
    void close();

private:
    int fd_ = -1;
    mutable std::mutex mutex_;
    SkeletonPoseSample latest_;
    bool has_latest_ = false;
    uint64_t packets_total_ = 0;
    uint64_t packets_window_ = 0;
    uint64_t sequence_counter_ = 0;

    static bool parse_payload(const char* data, size_t length, SkeletonPoseSample* sample);
};
