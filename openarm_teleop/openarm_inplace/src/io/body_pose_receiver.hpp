#pragma once

#include <controller/body_skeleton_types.hpp>

#include <cstdint>
#include <mutex>
#include <optional>

// Body keypoints stream for body_skeleton_retarget (UDP, default 47952).
// Packet (CSV):
//   ts_ms,valid,
//   lsx,lsy,lsz,lsc, lex,ley,lez,lec, lwx,lwy,lwz,lwc,
//   rsx,rsy,rsz,rsc, rex,rey,rez,rec, rwx,rwy,rwz,rwc,
//   lhx,lhy,lhz,lhc, rhx,rhy,rhz,rhc
class BodyPoseReceiver {
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

    BodyPoseReceiver();
    ~BodyPoseReceiver();

    bool open(uint16_t port);
    std::optional<BodyPoseSample> receive_latest();
    std::optional<BodyPoseSample> latest() const;
    double latest_age_ms() const;
    Stats snapshot_stats();
    void close();

private:
    int fd_ = -1;
    mutable std::mutex mutex_;
    BodyPoseSample latest_;
    bool has_latest_ = false;
    uint64_t packets_total_ = 0;
    uint64_t packets_window_ = 0;
    uint64_t sequence_counter_ = 0;

    static bool parse_payload(const char* data, size_t length, BodyPoseSample* sample);
};
