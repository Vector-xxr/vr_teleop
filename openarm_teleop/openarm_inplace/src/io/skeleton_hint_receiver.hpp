#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <mutex>
#include <optional>

// Elbow-swivel hint from scripts/skeleton_swivel_bridge.py (UDP, default 47950).
//
// Packet v1 (6 fields): timestamp_ms,valid,phi_L,phi_R,conf_L,conf_R
// Packet v2 (18 fields): v1 + unit upper/forearm dirs for L then R
//   ux_l,uy_l,uz_l,fx_l,fy_l,fz_l, ux_r,uy_r,uz_r,fx_r,fy_r,fz_r
struct SkeletonHintSample {
    int64_t timestamp_ms = 0;
    bool valid = false;
    double phi_left = 0.0;
    double phi_right = 0.0;
    double conf_left = 0.0;
    double conf_right = 0.0;
    Eigen::Vector3d u_upper_left = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_fore_left = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_upper_right = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_fore_right = Eigen::Vector3d::Zero();
    bool has_bones = false;  // true when v2 fields present and finite
    int64_t receive_timestamp_ns = 0;
    uint64_t sequence_id = 0;
};

class SkeletonHintReceiver {
public:
    struct Stats {
        uint64_t packets_total = 0;
        uint64_t packets_window = 0;
        bool has_latest = false;
        bool last_valid = false;
        double last_phi_left = 0.0;
        double last_phi_right = 0.0;
        double last_conf_left = 0.0;
        double last_conf_right = 0.0;
        double last_age_ms = -1.0;
        bool last_has_bones = false;
    };

    SkeletonHintReceiver();
    ~SkeletonHintReceiver();

    bool open(uint16_t port);
    std::optional<SkeletonHintSample> receive_latest();
    std::optional<SkeletonHintSample> latest() const;
    double latest_age_ms() const;
    Stats snapshot_stats();
    void close();

private:
    int fd_ = -1;
    mutable std::mutex mutex_;
    SkeletonHintSample latest_;
    bool has_latest_ = false;
    uint64_t packets_total_ = 0;
    uint64_t packets_window_ = 0;
    uint64_t sequence_counter_ = 0;

    static bool parse_payload(const char* data, size_t length, SkeletonHintSample* sample);
};
