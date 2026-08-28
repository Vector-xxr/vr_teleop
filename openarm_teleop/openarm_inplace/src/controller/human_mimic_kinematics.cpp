#include "controller/human_mimic_kinematics.hpp"

#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <utility>

HumanMimicKinematics::HumanMimicKinematics(std::string urdf_path, std::string root_link,
                                           std::string shoulder_link, std::string elbow_link,
                                           std::string wrist_link)
    : urdf_path_(std::move(urdf_path)),
      root_link_(std::move(root_link)),
      shoulder_link_(std::move(shoulder_link)),
      elbow_link_(std::move(elbow_link)),
      wrist_link_(std::move(wrist_link)) {}

bool HumanMimicKinematics::BuildChain(const KDL::Tree& tree, const std::string& root,
                                      const std::string& tip, KDL::Chain* chain) {
    if (!tree.getChain(root, tip, *chain)) {
        std::cerr << "[human-mimic-kin] failed to build chain " << root << " -> " << tip
                  << std::endl;
        return false;
    }
    return true;
}

bool HumanMimicKinematics::Init() {
    std::ifstream file(urdf_path_);
    if (!file.good()) {
        std::cerr << "[human-mimic-kin] cannot open URDF: " << urdf_path_ << std::endl;
        return false;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string urdf_xml = buffer.str();

    auto model = urdf::parseURDF(urdf_xml);
    if (!model) {
        std::cerr << "[human-mimic-kin] failed to parse URDF" << std::endl;
        return false;
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(*model, tree)) {
        std::cerr << "[human-mimic-kin] failed to build KDL tree" << std::endl;
        return false;
    }

    if (!BuildChain(tree, root_link_, shoulder_link_, &shoulder_chain_) ||
        !BuildChain(tree, root_link_, elbow_link_, &elbow_chain_) ||
        !BuildChain(tree, root_link_, wrist_link_, &wrist_chain_)) {
        return false;
    }

    shoulder_dof_ = shoulder_chain_.getNrOfJoints();
    elbow_dof_ = elbow_chain_.getNrOfJoints();
    wrist_dof_ = wrist_chain_.getNrOfJoints();

    shoulder_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(shoulder_chain_);
    elbow_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(elbow_chain_);
    wrist_fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(wrist_chain_);
    elbow_jac_ = std::make_unique<KDL::ChainJntToJacSolver>(elbow_chain_);
    wrist_jac_ = std::make_unique<KDL::ChainJntToJacSolver>(wrist_chain_);

    std::cout << "[human-mimic-kin] chains ready: shoulder(" << shoulder_dof_ << ") elbow("
              << elbow_dof_ << ") wrist(" << wrist_dof_ << ")" << std::endl;
    initialized_ = true;
    return true;
}

bool HumanMimicKinematics::ShoulderPos(const std::vector<double>& q, Eigen::Vector3d* p) const {
    if (!initialized_ || q.size() < shoulder_dof_) {
        return false;
    }
    KDL::JntArray ja(shoulder_dof_);
    for (size_t i = 0; i < shoulder_dof_; ++i) {
        ja(i) = q[i];
    }
    KDL::Frame frame;
    if (shoulder_fk_->JntToCart(ja, frame) < 0) {
        return false;
    }
    (*p) = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
    return true;
}

bool HumanMimicKinematics::ElbowPosJac(const std::vector<double>& q, Eigen::Vector3d* p,
                                       Eigen::MatrixXd* Jp) const {
    if (!initialized_ || q.size() < wrist_dof_) {
        return false;
    }
    KDL::JntArray ja(elbow_dof_);
    for (size_t i = 0; i < elbow_dof_; ++i) {
        ja(i) = q[i];
    }
    KDL::Frame frame;
    if (elbow_fk_->JntToCart(ja, frame) < 0) {
        return false;
    }
    (*p) = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());

    KDL::Jacobian jac(elbow_dof_);
    if (elbow_jac_->JntToJac(ja, jac) < 0) {
        return false;
    }
    Jp->setZero(3, static_cast<int>(wrist_dof_));
    for (size_t c = 0; c < elbow_dof_; ++c) {
        (*Jp)(0, static_cast<int>(c)) = jac(0, c);
        (*Jp)(1, static_cast<int>(c)) = jac(1, c);
        (*Jp)(2, static_cast<int>(c)) = jac(2, c);
    }
    return true;
}

bool HumanMimicKinematics::WristState(const std::vector<double>& q, Eigen::Vector3d* p_wrist,
                                      Eigen::Matrix3d* R_hand, Eigen::MatrixXd* Jp_wrist,
                                      Eigen::MatrixXd* Jw_hand) const {
    if (!initialized_ || q.size() < wrist_dof_) {
        return false;
    }
    KDL::JntArray ja(wrist_dof_);
    for (size_t i = 0; i < wrist_dof_; ++i) {
        ja(i) = q[i];
    }
    KDL::Frame frame;
    if (wrist_fk_->JntToCart(ja, frame) < 0) {
        return false;
    }
    (*p_wrist) = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R(r, c) = frame.M(r, c);
        }
    }
    (*R_hand) = R;

    KDL::Jacobian jac(wrist_dof_);
    if (wrist_jac_->JntToJac(ja, jac) < 0) {
        return false;
    }
    Jp_wrist->setZero(3, static_cast<int>(wrist_dof_));
    Jw_hand->setZero(3, static_cast<int>(wrist_dof_));
    for (size_t c = 0; c < wrist_dof_; ++c) {
        (*Jp_wrist)(0, static_cast<int>(c)) = jac(0, c);
        (*Jp_wrist)(1, static_cast<int>(c)) = jac(1, c);
        (*Jp_wrist)(2, static_cast<int>(c)) = jac(2, c);
        (*Jw_hand)(0, static_cast<int>(c)) = jac(3, c);
        (*Jw_hand)(1, static_cast<int>(c)) = jac(4, c);
        (*Jw_hand)(2, static_cast<int>(c)) = jac(5, c);
    }
    return true;
}

bool HumanMimicKinematics::ComputeTcpState(const std::vector<double>& q,
                                           const Eigen::Vector3d& wrist_to_tcp,
                                           TcpState* state) const {
    if (state == nullptr || !wrist_to_tcp.allFinite()) {
        return false;
    }
    if (!WristState(q, &state->p_wrist, &state->R_hand, &state->Jp_wrist,
                    &state->Jw_hand)) {
        return false;
    }

    const Eigen::Vector3d r = state->R_hand * wrist_to_tcp;
    state->p_tcp = state->p_wrist + r;
    Eigen::Matrix3d r_cross;
    r_cross << 0.0, -r.z(), r.y(),
               r.z(), 0.0, -r.x(),
              -r.y(), r.x(), 0.0;
    state->Jp_tcp = state->Jp_wrist - r_cross * state->Jw_hand;
    return state->p_tcp.allFinite() && state->Jp_tcp.allFinite();
}
