#pragma once

#include <robot_state.hpp>

#include <chrono>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

class CsvRecorder {
public:
    bool open(const std::string& path, size_t joint_count);
    void append(const std::vector<JointState>& joints);
    void close();
    bool active() const;

private:
    std::ofstream file_;
    std::chrono::steady_clock::time_point start_;
};

class CsvReplay {
public:
    struct ActionEvent {
        double t = 0.0;
        std::string label;
        std::string face_emoji;
    };

    enum class TransitionResult {
        Added,
        SkippedNoMotion,
        Failed,
    };

    bool load(const std::string& path, size_t joint_count);
    bool load_directory(const std::string& dir_path, size_t joint_count,
                        bool generate_transitions = true, double transition_duration = 0.0);
    std::optional<std::vector<JointState>> sample();
    bool prepend_transition_from(const std::vector<JointState>& start_joints, size_t joint_count,
                                 double transition_duration = 0.0);
    TransitionResult prepend_transition_from_with_result(
        const std::vector<JointState>& start_joints, size_t joint_count,
        double transition_duration = 0.0);
    bool finished() const;
    void reset();
    void set_time_scale(double time_scale);
    void pause_for(double seconds);
    std::vector<ActionEvent> consume_action_events();

    static int clean_transitions(const std::string& dir_path);

private:
    struct Frame {
        double t = 0.0;
        std::vector<JointState> joints;
    };

    std::vector<Frame> load_frames_from_csv(const std::string& path, size_t joint_count,
                                            std::string* face_emoji = nullptr);
    static std::vector<Frame> generate_transition(
        const std::vector<JointState>& start_joints,
        const std::vector<JointState>& end_joints,
        double time_offset, size_t joint_count, double transition_duration = 0.0);
    static bool is_transition_file(const std::string& filename);
    static bool save_frames_to_csv(const std::string& path,
                                    const std::vector<Frame>& frames,
                                    size_t joint_count);

    std::vector<Frame> frames_;
    std::vector<ActionEvent> action_events_;
    std::chrono::steady_clock::time_point start_;
    size_t index_ = 0;
    size_t action_event_index_ = 0;
    double time_scale_ = 1.0;
};

class UdpPanelReceiver {
public:
    UdpPanelReceiver();
    ~UdpPanelReceiver();

    bool open(uint16_t port);
    std::optional<std::vector<JointState>> receive_latest(size_t joint_count);
    void close();

private:
    int fd_ = -1;
};

class UdpTextReceiver {
public:
    UdpTextReceiver();
    ~UdpTextReceiver();

    bool open(uint16_t port, const std::string& label = "udp-text");
    std::optional<std::string> receive_latest();
    std::vector<std::string> receive_all();
    void close();

private:
    int fd_ = -1;
    std::string label_ = "udp-text";
};
