// Shared body-pose types for BODY_SKELETON_RETARGET (phase 1+).
// Controller must depend only on these samples — never on a concrete vision model.
#pragma once

#include <algorithm>
#include <cstdint>

#include <Eigen/Dense>

struct BodyJoint3d {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double conf = 0.0;
};

// One person, camera / model frame (exact axis convention is adapter-defined;
// phase 2 builds a torso frame from these keypoints).
struct BodyPoseSample {
    int64_t timestamp_ms = 0;
    bool valid = false;

    BodyJoint3d left_shoulder;
    BodyJoint3d left_elbow;
    BodyJoint3d left_wrist;
    BodyJoint3d right_shoulder;
    BodyJoint3d right_elbow;
    BodyJoint3d right_wrist;
    BodyJoint3d left_hip;
    BodyJoint3d right_hip;

    int64_t receive_timestamp_ns = 0;
    uint64_t sequence_id = 0;

    double arm_confidence_left() const {
        return std::min({left_shoulder.conf, left_elbow.conf, left_wrist.conf});
    }
    double arm_confidence_right() const {
        return std::min({right_shoulder.conf, right_elbow.conf, right_wrist.conf});
    }
};
