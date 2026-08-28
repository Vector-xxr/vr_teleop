// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <controller/dynamics.hpp>

#include <algorithm>
#include <cmath>

Dynamics::Dynamics(std::string urdf_path, std::string start_link, std::string end_link) {
    this->urdf_path = urdf_path;
    this->start_link = start_link;
    this->end_link = end_link;
}

Dynamics::~Dynamics() {}

bool Dynamics::Init() {
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        fprintf(stderr, "Failed to open URDF file: %s\n", urdf_path.c_str());
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    urdf_model_interface = urdf::parseURDF(buffer.str());
    if (!urdf_model_interface) {
        fprintf(stderr, "Failed to parse URDF: %s\n", urdf_path.c_str());
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(*urdf_model_interface, kdl_tree)) {
        fprintf(stderr, "Failed to extract KDL tree: %s\n", urdf_path.c_str());
        return false;
    }

    if (!kdl_tree.getChain(start_link, end_link, kdl_chain)) {
        fprintf(stderr, "Failed to get KDL chain\n");
        return false;
    }

    std::cout << "[GetGravity] kdl_chain.getNrOfJoints() = " << kdl_chain.getNrOfJoints()
              << std::endl;

    coriolis_forces.resize(kdl_chain.getNrOfJoints());
    gravity_forces.resize(kdl_chain.getNrOfJoints());
    inertia_matrix.resize(kdl_chain.getNrOfJoints());

    coriolis_forces.data.setZero();
    gravity_forces.data.setZero();
    inertia_matrix.data.setZero();

    solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, KDL::Vector(0, 0.0, -9.81));

    Eigen::Matrix<double, 6, 1> ik_weights_;
    ik_weights_ << 1.0, 1.0, 1.0, 0.2, 0.2, 0.2;
    pos_ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain, ik_weights_);

    q_min_.assign(kdl_chain.getNrOfJoints(), -M_PI);
    q_max_.assign(kdl_chain.getNrOfJoints(), M_PI);
    joint_limits_valid_ = false;
    size_t joint_idx = 0;
    for (unsigned int seg = 0; seg < kdl_chain.getNrOfSegments(); ++seg) {
        const KDL::Joint& joint = kdl_chain.getSegment(seg).getJoint();
        if (joint.getType() == KDL::Joint::None) {
            continue;
        }
        if (joint_idx >= q_min_.size()) {
            break;
        }
        const std::string name = joint.getName();
        const auto urdf_joint = urdf_model_interface->getJoint(name);
        if (urdf_joint && urdf_joint->limits) {
            q_min_[joint_idx] = urdf_joint->limits->lower;
            q_max_[joint_idx] = urdf_joint->limits->upper;
            joint_limits_valid_ = true;
        }
        ++joint_idx;
    }

    return true;
}

bool Dynamics::GetJointPositionLimits(std::vector<double> *q_min, std::vector<double> *q_max) const {
    if (q_min == nullptr || q_max == nullptr || !joint_limits_valid_) {
        return false;
    }
    *q_min = q_min_;
    *q_max = q_max_;
    return true;
}

void Dynamics::ClampJointPositions(double *q_inout) const {
    if (q_inout == nullptr || q_min_.empty()) {
        return;
    }
    for (size_t i = 0; i < q_min_.size(); ++i) {
        q_inout[i] = std::clamp(q_inout[i], q_min_[i], q_max_[i]);
    }
}

void Dynamics::GetGravity(const double *motor_position, double *gravity) {
    const auto njoints = kdl_chain.getNrOfJoints();

    KDL::JntArray q_(kdl_chain.getNrOfJoints());

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
    }

    solver->JntToGravity(q_, gravity_forces);
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        gravity[i] = gravity_forces(i);
    }
}

void Dynamics::GetCoriolis(const double *motor_position, const double *motor_velocity,
                           double *coriolis) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    KDL::JntArray q_dot(kdl_chain.getNrOfJoints());

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
        q_dot(i) = motor_velocity[i];
    }

    solver->JntToCoriolis(q_, q_dot, coriolis_forces);

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        coriolis[i] = coriolis_forces(i);
    }
}

void Dynamics::GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    KDL::JntSpaceInertiaMatrix inertia_matrix(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        q_(i) = motor_position[i];
    }

    solver->JntToMass(q_, inertia_matrix);

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        inertia_diag[i] = inertia_matrix(i, i);
    }
}

void Dynamics::GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::Jacobian kdl_jac(kdl_chain.getNrOfJoints());
    KDL::ChainJntToJacSolver jac_solver(kdl_chain);
    jac_solver.JntToJac(q_, kdl_jac);

    jacobian = Eigen::MatrixXd(6, kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < kdl_chain.getNrOfJoints(); ++j) {
            jacobian(i, j) = kdl_jac(i, j);
        }
    }
}

void Dynamics::GetNullSpace(const double *motor_position, Eigen::MatrixXd &nullspace) {
    const size_t dof = kdl_chain.getNrOfJoints();

    bool use_stable_svd = false;

    Eigen::MatrixXd J;
    GetJacobian(motor_position, J);

    Eigen::MatrixXd J_pinv;

    if (use_stable_svd) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol =
            1e-6 * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValuesInv = svd.singularValues();
        for (int i = 0; i < singularValuesInv.size(); ++i) {
            singularValuesInv(i) = (singularValuesInv(i) > tol) ? 1.0 / singularValuesInv(i) : 0.0;
        }
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    } else {
        J_pinv = J.transpose() * (J * J.transpose()).inverse();
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
    nullspace = I - J_pinv * J;

    //        std::cout << "[INFO] Null space projector computed.\n";
}

void Dynamics::GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T) {
    const size_t dof = kdl_chain.getNrOfJoints();
    bool use_stable_svd = false;

    Eigen::MatrixXd J;
    GetJacobian(motor_position, J);

    Eigen::MatrixXd J_pinv;

    if (use_stable_svd) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol =
            1e-6 * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValuesInv = svd.singularValues();
        for (int i = 0; i < singularValuesInv.size(); ++i) {
            singularValuesInv(i) = (singularValuesInv(i) > tol) ? 1.0 / singularValuesInv(i) : 0.0;
        }
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    } else {
        J_pinv = J.transpose() * (J * J.transpose()).inverse();
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
    Eigen::MatrixXd N = I - J_pinv * J;

    nullspace_T = N.transpose();
}

void Dynamics::GetEECordinate(const double *motor_position, Eigen::Matrix3d &R,
                              Eigen::Vector3d &p) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;

    if (fk_solver.JntToCart(q_, kdl_frame) < 0) {
        //  std::cerr << "[KDL] FK failed in GetEECordinate!" << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);

    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
}

void Dynamics::GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R,
                                 Eigen::Vector3d &p) {
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;

    if (fk_solver.JntToCart(q_, kdl_frame, kdl_chain.getNrOfSegments() - 1) < 0) {
        //        std::cerr << "[KDL] FK failed in GetPreEECordinate!" << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);

    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
}

bool Dynamics::GetSegmentCordinate(const double *motor_position, int segment_index,
                                   Eigen::Matrix3d &R, Eigen::Vector3d &p) {
    const int nseg = static_cast<int>(kdl_chain.getNrOfSegments());
    if (segment_index < 0 || segment_index > nseg) {
        return false;
    }
    KDL::JntArray q_(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_(i) = motor_position[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::Frame kdl_frame;
    if (fk_solver.JntToCart(q_, kdl_frame, segment_index) < 0) {
        return false;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i, j) = kdl_frame.M(i, j);
    p << kdl_frame.p[0], kdl_frame.p[1], kdl_frame.p[2];
    return true;
}

double Dynamics::ComputeMinSingularValue(const double *motor_position) {
    Eigen::MatrixXd jacobian;
    GetJacobian(motor_position, jacobian);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues().size() == 0) {
        return 0.0;
    }
    return svd.singularValues().tail(1)(0);
}

bool Dynamics::SolveDlsIk(const double *motor_position, const Eigen::VectorXd &v_cart,
                          double lambda, Eigen::VectorXd *dq_out, double *sigma_min_out,
                          const std::vector<double> *joint_mobility_weights) {
    if (dq_out == nullptr || v_cart.size() != 6) {
        return false;
    }

    const size_t dof = kdl_chain.getNrOfJoints();
    Eigen::MatrixXd jacobian;
    GetJacobian(motor_position, jacobian);

    // Mobility diagonal W: higher weight => joint is preferred in the solution.
    // q_dot = W J^T (J W J^T + λ² I)^{-1} v
    Eigen::VectorXd w = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(dof));
    if (joint_mobility_weights != nullptr && !joint_mobility_weights->empty()) {
        for (size_t i = 0; i < dof && i < joint_mobility_weights->size(); ++i) {
            const double wi = (*joint_mobility_weights)[i];
            w[static_cast<Eigen::Index>(i)] = (wi > 1e-6 && std::isfinite(wi)) ? wi : 1.0;
        }
    }
    const Eigen::MatrixXd jw = jacobian * w.asDiagonal();  // J * W

    const Eigen::MatrixXd A =
        jw * jacobian.transpose() + (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    if (ldlt.info() != Eigen::Success) {
        dq_out->setZero(static_cast<Eigen::Index>(dof));
        if (sigma_min_out != nullptr) {
            *sigma_min_out = ComputeMinSingularValue(motor_position);
        }
        return false;
    }
    const Eigen::VectorXd y = ldlt.solve(v_cart);
    if (ldlt.info() != Eigen::Success || !y.allFinite()) {
        dq_out->setZero(static_cast<Eigen::Index>(dof));
        if (sigma_min_out != nullptr) {
            *sigma_min_out = ComputeMinSingularValue(motor_position);
        }
        return false;
    }
    *dq_out = w.asDiagonal() * jacobian.transpose() * y;

    if (sigma_min_out != nullptr) {
        *sigma_min_out = ComputeMinSingularValue(motor_position);
    }
    return dq_out->size() == static_cast<Eigen::Index>(dof) && dq_out->allFinite();
}

bool Dynamics::SolvePosIk(const double *motor_position, const Eigen::Matrix3d &R_des,
                          const Eigen::Vector3d &p_des, double *q_out) {
    if (q_out == nullptr) {
        return false;
    }

    KDL::JntArray q_seed(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_seed(i) = motor_position[i];
    }

    KDL::Rotation target_rot;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            target_rot(i, j) = R_des(i, j);
        }
    }
    const KDL::Frame target_frame(target_rot, KDL::Vector(p_des.x(), p_des.y(), p_des.z()));

    if (!pos_ik_solver_) {
        return false;
    }
    KDL::JntArray q_result(kdl_chain.getNrOfJoints());
    const int ret = pos_ik_solver_->CartToJnt(q_seed, target_frame, q_result);
    if (ret < 0) {
        return false;
    }

    for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i) {
        q_out[i] = q_result(i);
    }
    return true;
}

bool Dynamics::SolvePosIkPosition(const double *motor_position, const Eigen::Vector3d &p_des,
                                  double *q_out) {
    if (q_out == nullptr) {
        return false;
    }
    Eigen::Matrix3d r_curr = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_curr = Eigen::Vector3d::Zero();
    GetEECordinate(motor_position, r_curr, p_curr);
    return SolvePosIk(motor_position, r_curr, p_des, q_out);
}

bool Dynamics::RefineIkOrientation(const double *motor_position,
                                   const Eigen::Quaterniond &q_des, double dt, double *q_out) {
    if (q_out == nullptr) {
        return false;
    }

    Eigen::Matrix3d r_fk = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_fk = Eigen::Vector3d::Zero();
    GetEECordinate(motor_position, r_fk, p_fk);

    const Eigen::Quaterniond q_fk(r_fk);
    Eigen::Quaterniond q_err =
        (q_des.normalized() * q_fk.normalized().inverse()).normalized();
    if (q_err.w() < 0.0) {
        q_err.coeffs() = -q_err.coeffs();
    }
    Eigen::AngleAxisd axis_angle(q_err);
    double angle = axis_angle.angle();
    if (!std::isfinite(angle) || angle < 0.03) {
        return true;
    }

    Eigen::VectorXd v_cart(6);
    v_cart.setZero();
    v_cart.tail<3>() = axis_angle.axis() * (angle / std::max(dt, 1e-3));

    Eigen::VectorXd dq_step;
    if (!SolveDlsIk(motor_position, v_cart, 0.08, &dq_step)) {
        return false;
    }

    const size_t dof = kdl_chain.getNrOfJoints();
    for (size_t i = 0; i < dof; ++i) {
        q_out[i] = motor_position[i] + dq_step[static_cast<Eigen::Index>(i)] * dt;
    }
    return true;
}
