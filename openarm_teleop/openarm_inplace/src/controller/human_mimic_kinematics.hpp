// Whole-arm kinematics helper for the HUMAN_MIMIC mode.
//
// Owns dedicated KDL sub-chains (root -> shoulder, root -> elbow, root -> wrist)
// so the mode can query the positions of intermediate frames and their position/
// orientation Jacobians independently of the DLS path.
#pragma once

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <memory>
#include <string>
#include <vector>

struct TcpState {
    Eigen::Vector3d p_wrist = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_tcp = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_hand = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd Jp_wrist;
    Eigen::MatrixXd Jp_tcp;
    Eigen::MatrixXd Jw_hand;
};

class HumanMimicKinematics {
public:
    HumanMimicKinematics(std::string urdf_path, std::string root_link, std::string shoulder_link,
                         std::string elbow_link, std::string wrist_link);

    bool Init();

    size_t Dof() const { return wrist_dof_; }

    // Shoulder frame origin (base frame). Effectively constant, but computed from q.
    bool ShoulderPos(const std::vector<double>& q, Eigen::Vector3d* p) const;

    // Elbow frame origin + position Jacobian (3 x dof, base frame).
    bool ElbowPosJac(const std::vector<double>& q, Eigen::Vector3d* p, Eigen::MatrixXd* Jp) const;

    // Wrist center origin, hand orientation (= wrist_link rotation), and the wrist
    // position Jacobian (3 x dof) + hand angular Jacobian (3 x dof), all base frame.
    bool WristState(const std::vector<double>& q, Eigen::Vector3d* p_wrist, Eigen::Matrix3d* R_hand,
                    Eigen::MatrixXd* Jp_wrist, Eigen::MatrixXd* Jw_hand) const;

    // Wrist + fixed hand_tcp state. For r=R_hand*wrist_to_tcp:
    //   p_tcp = p_wrist + r
    //   Jp_tcp = Jp_wrist - [r]x Jw_hand
    bool ComputeTcpState(const std::vector<double>& q, const Eigen::Vector3d& wrist_to_tcp,
                         TcpState* state) const;

private:
    static bool BuildChain(const KDL::Tree& tree, const std::string& root, const std::string& tip,
                           KDL::Chain* chain);

    std::string urdf_path_;
    std::string root_link_;
    std::string shoulder_link_;
    std::string elbow_link_;
    std::string wrist_link_;

    KDL::Chain shoulder_chain_;
    KDL::Chain elbow_chain_;
    KDL::Chain wrist_chain_;

    size_t shoulder_dof_ = 0;
    size_t elbow_dof_ = 0;
    size_t wrist_dof_ = 0;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> shoulder_fk_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> elbow_fk_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> wrist_fk_;
    std::unique_ptr<KDL::ChainJntToJacSolver> elbow_jac_;
    std::unique_ptr<KDL::ChainJntToJacSolver> wrist_jac_;

    bool initialized_ = false;
};
