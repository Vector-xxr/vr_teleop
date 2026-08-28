#pragma once

#include <controller/cartesian_types.hpp>

#include <cstdint>
#include <mutex>
#include <optional>

class QuestPoseReceiver {
public:
    struct Stats {
        uint64_t packets_total = 0;
        uint64_t packets_window = 0;
        bool has_latest = false;
        bool last_grip = false;
        double last_trigger = 0.0;
        double last_age_ms = -1.0;
    };

    QuestPoseReceiver();
    ~QuestPoseReceiver();

    bool open(uint16_t port);
    // Drains UDP socket; returns latest sample (may be unchanged). Always stamps receive time.
    std::optional<QuestPoseSample> receive_latest();
    // Cached sample only (no socket I/O). Used by high-rate DLS thread.
    std::optional<QuestPoseSample> latest() const;
    Stats snapshot_stats();
    void close();

private:
    int fd_ = -1;
    mutable std::mutex mutex_;
    QuestPoseSample latest_;
    bool has_latest_ = false;
    uint64_t packets_total_ = 0;
    uint64_t packets_window_ = 0;
    uint64_t sequence_counter_ = 0;

    static bool parse_payload(const char* data, size_t length, QuestPoseSample* sample);
};
