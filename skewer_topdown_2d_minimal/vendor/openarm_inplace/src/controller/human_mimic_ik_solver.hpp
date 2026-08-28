// Whole-arm inverse kinematics for HUMAN_MIMIC.
//
// Backends:
//   LM       — position-level Levenberg-Marquardt (legacy)
//   DIFF_QP  — velocity-level weighted least squares with per-step + URDF box
//              constraints on dq (differential QP via projection, no external solver)
#pragma once

#include <Eigen/Dense>
#include <vector>

#include "controller/human_mimic_kinematics.hpp"
#include "controller/human_mimic_types.hpp"

class HumanMimicIkSolver {
public:
    HumanMimicIkSolver(HumanMimicKinematics* kin, const HumanMimicConfig& config);

    void SetJointLimits(const std::vector<double>& q_min, const std::vector<double>& q_max);

    HumanMimicIkResult Solve(const std::vector<double>& q_seed,
                             const Eigen::Vector3d& wrist_pos_des, const Eigen::Matrix3d& hand_R_des,
                             const Eigen::Vector3d& elbow_pos_des,
                             const std::vector<double>& q_nominal, double elbow_w_scale = 1.0,
                             double ori_w_scale = 1.0, double cont_w_scale = 1.0,
                             double nom_w_scale = 1.0) const;

private:
    void clampToLimits(std::vector<double>* q) const;
    double perJointMaxStep(int joint_index) const;

    HumanMimicIkResult SolveLm(const std::vector<double>& q_seed,
                                 const Eigen::Vector3d& wrist_pos_des,
                                 const Eigen::Matrix3d& hand_R_des,
                                 const Eigen::Vector3d& elbow_pos_des,
                                 const std::vector<double>& q_nominal, double elbow_w_scale,
                                 double ori_w_scale, double cont_w_scale,
                                 double nom_w_scale) const;

    HumanMimicIkResult SolveDiffQp(const std::vector<double>& q_seed,
                                   const Eigen::Vector3d& wrist_pos_des,
                                   const Eigen::Matrix3d& hand_R_des,
                                   const Eigen::Vector3d& elbow_pos_des,
                                   const std::vector<double>& q_nominal, double elbow_w_scale,
                                   double ori_w_scale, double cont_w_scale,
                                   double nom_w_scale) const;

    void applySoftJointLimitScale(const std::vector<double>& q, Eigen::VectorXd* dq) const;
    void projectDqBox(const std::vector<double>& q, Eigen::VectorXd* dq) const;
    void fillTaskErrors(const std::vector<double>& q, const std::vector<double>& q_seed,
                        const Eigen::Vector3d& wrist_pos_des, const Eigen::Matrix3d& hand_R_des,
                        const Eigen::Vector3d& elbow_pos_des, const std::vector<double>& q_nominal,
                        double elbow_w_scale, double ori_w_scale, double cont_w_scale,
                        double nom_w_scale, Eigen::VectorXd* r, HumanMimicIkResult* result) const;

    HumanMimicKinematics* kin_;
    HumanMimicConfig config_;
    size_t dof_ = 0;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    bool limits_valid_ = false;
};
