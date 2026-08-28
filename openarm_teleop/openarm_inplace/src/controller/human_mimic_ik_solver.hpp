// Whole-arm inverse kinematics for HUMAN_MIMIC.
//
// Backends:
//   LM       — position-level Levenberg-Marquardt (legacy)
//   DIFF_QP  — velocity-level weighted least squares with per-step + URDF box
//              constraints on dq (differential QP via projection, no external solver)
//
// Elbow redundancy (default PHI):
//   Constraint is the SEW swivel angle on the human elbow circle about S→W,
//   not a full 3D elbow position — avoids over-constraining 7-DoF with wrist+ori.
#pragma once

#include <Eigen/Dense>
#include <vector>

#include "controller/human_mimic_kinematics.hpp"
#include "controller/human_mimic_types.hpp"
#include "controller/virtual_arm_skeleton.hpp"

class HumanMimicIkSolver {
public:
    HumanMimicIkSolver(HumanMimicKinematics* kin, const HumanMimicConfig& config);

    void SetJointLimits(const std::vector<double>& q_min, const std::vector<double>& q_max);
    void SetWristToTcp(const Eigen::Vector3d& wrist_to_tcp) {
        if (wrist_to_tcp.allFinite()) {
            config_.wrist_to_tcp = wrist_to_tcp;
        }
    }

    // elbow_pos_des used when ElbowTaskMode::XYZ.
    // elbow_phi_des + shoulder + skeleton used when ElbowTaskMode::PHI.
    HumanMimicIkResult Solve(const std::vector<double>& q_seed,
                             const Eigen::Vector3d& wrist_pos_des, const Eigen::Matrix3d& hand_R_des,
                             const Eigen::Vector3d& elbow_pos_des,
                             const std::vector<double>& q_nominal, double elbow_w_scale = 1.0,
                             double ori_w_scale = 1.0, double cont_w_scale = 1.0,
                             double nom_w_scale = 1.0, double elbow_phi_des = 0.0,
                             const Eigen::Vector3d& shoulder = Eigen::Vector3d::Zero(),
                             const VirtualArmSkeleton* skeleton = nullptr) const;

private:
    void clampToLimits(std::vector<double>* q) const;
    double perJointMaxStep(int joint_index) const;
    int elbowTaskRows() const;
    int residualRows() const;

    bool computeElbowPhiResidual(const VirtualArmSkeleton* skeleton,
                                 const Eigen::Vector3d& shoulder, const Eigen::Vector3d& wrist,
                                 const Eigen::Vector3d& elbow, const Eigen::MatrixXd& Jp_elbow,
                                 double phi_des, double* e_phi, Eigen::RowVectorXd* J_phi) const;

    HumanMimicIkResult SolveLm(const std::vector<double>& q_seed,
                               const Eigen::Vector3d& wrist_pos_des,
                               const Eigen::Matrix3d& hand_R_des,
                               const Eigen::Vector3d& elbow_pos_des,
                               const std::vector<double>& q_nominal, double elbow_w_scale,
                               double ori_w_scale, double cont_w_scale, double nom_w_scale,
                               double elbow_phi_des, const Eigen::Vector3d& shoulder,
                               const VirtualArmSkeleton* skeleton) const;

    HumanMimicIkResult SolveDiffQp(const std::vector<double>& q_seed,
                                   const Eigen::Vector3d& wrist_pos_des,
                                   const Eigen::Matrix3d& hand_R_des,
                                   const Eigen::Vector3d& elbow_pos_des,
                                   const std::vector<double>& q_nominal, double elbow_w_scale,
                                   double ori_w_scale, double cont_w_scale, double nom_w_scale,
                                   double elbow_phi_des, const Eigen::Vector3d& shoulder,
                                   const VirtualArmSkeleton* skeleton) const;

    void applySoftJointLimitScale(const std::vector<double>& q, Eigen::VectorXd* dq) const;
    void projectDqBox(const std::vector<double>& q, Eigen::VectorXd* dq) const;
    void fillTaskErrors(const std::vector<double>& q, const std::vector<double>& q_seed,
                        const Eigen::Vector3d& wrist_pos_des, const Eigen::Matrix3d& hand_R_des,
                        const Eigen::Vector3d& elbow_pos_des, const std::vector<double>& q_nominal,
                        double elbow_w_scale, double ori_w_scale, double cont_w_scale,
                        double nom_w_scale, double elbow_phi_des, const Eigen::Vector3d& shoulder,
                        const VirtualArmSkeleton* skeleton, Eigen::VectorXd* r,
                        HumanMimicIkResult* result) const;

    HumanMimicKinematics* kin_;
    HumanMimicConfig config_;
    size_t dof_ = 0;
    std::vector<double> q_min_;
    std::vector<double> q_max_;
    bool limits_valid_ = false;
};
