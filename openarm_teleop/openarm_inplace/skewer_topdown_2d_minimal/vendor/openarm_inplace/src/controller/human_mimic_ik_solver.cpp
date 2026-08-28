#include "controller/human_mimic_ik_solver.hpp"

#include <algorithm>
#include <cmath>

namespace {
Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R, const Eigen::Matrix3d& R_des) {
    Eigen::Matrix3d Re = R_des * R.transpose();
    Eigen::AngleAxisd aa(Eigen::Quaterniond(Re).normalized());
    double angle = aa.angle();
    if (!std::isfinite(angle)) {
        return Eigen::Vector3d::Zero();
    }
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    return aa.axis() * angle;
}
}  // namespace

HumanMimicIkSolver::HumanMimicIkSolver(HumanMimicKinematics* kin, const HumanMimicConfig& config)
    : kin_(kin), config_(config), dof_(kin != nullptr ? kin->Dof() : 0) {}

void HumanMimicIkSolver::SetJointLimits(const std::vector<double>& q_min,
                                        const std::vector<double>& q_max) {
    q_min_ = q_min;
    q_max_ = q_max;
    limits_valid_ = q_min_.size() >= dof_ && q_max_.size() >= dof_;
}

void HumanMimicIkSolver::clampToLimits(std::vector<double>* q) const {
    if (!limits_valid_ || q == nullptr) {
        return;
    }
    for (size_t i = 0; i < dof_ && i < q->size(); ++i) {
        (*q)[i] = std::clamp((*q)[i], q_min_[i], q_max_[i]);
    }
}

double HumanMimicIkSolver::perJointMaxStep(int joint_index) const {
    if (joint_index < 4) {
        return std::max(1e-4, config_.ik_proximal_max_step);
    }
    return std::max(1e-4, config_.ik_distal_max_step);
}

HumanMimicIkResult HumanMimicIkSolver::Solve(const std::vector<double>& q_seed,
                                             const Eigen::Vector3d& wrist_pos_des,
                                             const Eigen::Matrix3d& hand_R_des,
                                             const Eigen::Vector3d& elbow_pos_des,
                                             const std::vector<double>& q_nominal,
                                             double elbow_w_scale, double ori_w_scale,
                                             double cont_w_scale, double nom_w_scale) const {
    if (config_.ik_backend == HumanMimicIkBackend::DIFF_QP) {
        return SolveDiffQp(q_seed, wrist_pos_des, hand_R_des, elbow_pos_des, q_nominal,
                           elbow_w_scale, ori_w_scale, cont_w_scale, nom_w_scale);
    }
    return SolveLm(q_seed, wrist_pos_des, hand_R_des, elbow_pos_des, q_nominal, elbow_w_scale,
                   ori_w_scale, cont_w_scale, nom_w_scale);
}

void HumanMimicIkSolver::fillTaskErrors(const std::vector<double>& q,
                                        const std::vector<double>& q_seed,
                                        const Eigen::Vector3d& wrist_pos_des,
                                        const Eigen::Matrix3d& hand_R_des,
                                        const Eigen::Vector3d& elbow_pos_des,
                                        const std::vector<double>& q_nominal,
                                        double elbow_w_scale, double ori_w_scale,
                                        double cont_w_scale, double nom_w_scale,
                                        Eigen::VectorXd* r, HumanMimicIkResult* result) const {
    const int n = static_cast<int>(dof_);
    const double sw_wrist = std::sqrt(std::max(0.0, config_.w_wrist_pos));
    const double sw_ori =
        std::sqrt(std::max(0.0, config_.w_hand_ori * std::max(0.0, ori_w_scale)));
    const double sw_elbow =
        std::sqrt(std::max(0.0, config_.w_elbow * std::max(0.0, elbow_w_scale)));
    const double sw_cont =
        std::sqrt(std::max(0.0, config_.w_continuity * std::max(0.0, cont_w_scale)));
    const double sw_nom =
        std::sqrt(std::max(0.0, config_.w_nominal * std::max(0.0, nom_w_scale)));
    const bool use_nominal = q_nominal.size() >= dof_ && sw_nom > 0.0;

    Eigen::Vector3d p_wrist, p_elbow;
    Eigen::Matrix3d R_hand;
    Eigen::MatrixXd Jp_wrist, Jw_hand, Jp_elbow;
    if (!kin_->WristState(q, &p_wrist, &R_hand, &Jp_wrist, &Jw_hand) ||
        !kin_->ElbowPosJac(q, &p_elbow, &Jp_elbow)) {
        if (result != nullptr) {
            result->ok = false;
        }
        return;
    }

    const Eigen::Vector3d e_wrist = p_wrist - wrist_pos_des;
    const Eigen::Vector3d e_ori = orientation_error(R_hand, hand_R_des);
    const Eigen::Vector3d e_elbow = p_elbow - elbow_pos_des;

    if (result != nullptr) {
        result->wrist_pos_err = e_wrist.norm();
        result->hand_ori_err = e_ori.norm();
        result->elbow_pos_err = e_elbow.norm();
        result->elbow_phi_err = 0.0;
    }

    if (r == nullptr) {
        return;
    }

    r->segment<3>(0) = sw_wrist * e_wrist;
    r->segment<3>(3) = sw_ori * e_ori;
    r->segment<3>(6) = sw_elbow * e_elbow;
    for (int i = 0; i < n; ++i) {
        (*r)(9 + i) = sw_cont * (q[static_cast<size_t>(i)] - q_seed[static_cast<size_t>(i)]);
        const double target =
            use_nominal ? q_nominal[static_cast<size_t>(i)] : q_seed[static_cast<size_t>(i)];
        (*r)(9 + n + i) = sw_nom * (q[static_cast<size_t>(i)] - target);
    }
}

void HumanMimicIkSolver::applySoftJointLimitScale(const std::vector<double>& q,
                                                 Eigen::VectorXd* dq) const {
    if (!limits_valid_ || dq == nullptr) {
        return;
    }
    const double margin_frac =
        std::clamp(config_.ik_joint_limit_margin_frac, 0.01, 0.45);
    for (size_t i = 0; i < dof_ && static_cast<Eigen::Index>(i) < dq->size(); ++i) {
        const double span = q_max_[i] - q_min_[i];
        if (!(span > 1e-6)) {
            continue;
        }
        const double soft = span * margin_frac;
        double& dqi = (*dq)[static_cast<Eigen::Index>(i)];
        if (dqi > 0.0) {
            const double room = q_max_[i] - q[static_cast<size_t>(i)];
            if (room <= 0.0) {
                dqi = 0.0;
            } else if (room < soft) {
                dqi *= room / soft;
            }
        }
        if (dqi < 0.0) {
            const double room = q[static_cast<size_t>(i)] - q_min_[i];
            if (room <= 0.0) {
                dqi = 0.0;
            } else if (room < soft) {
                dqi *= room / soft;
            }
        }
    }
}

void HumanMimicIkSolver::projectDqBox(const std::vector<double>& q, Eigen::VectorXd* dq) const {
    if (dq == nullptr) {
        return;
    }
    const int n = static_cast<int>(dof_);
    for (int i = 0; i < n; ++i) {
        const double step_max = perJointMaxStep(i);
        double lo = -step_max;
        double hi = step_max;
        if (limits_valid_ && static_cast<size_t>(i) < q.size()) {
            lo = std::max(lo, q_min_[static_cast<size_t>(i)] - q[static_cast<size_t>(i)]);
            hi = std::min(hi, q_max_[static_cast<size_t>(i)] - q[static_cast<size_t>(i)]);
        }
        (*dq)(i) = std::clamp((*dq)(i), lo, hi);
    }
}

HumanMimicIkResult HumanMimicIkSolver::SolveDiffQp(
    const std::vector<double>& q_seed, const Eigen::Vector3d& wrist_pos_des,
    const Eigen::Matrix3d& hand_R_des, const Eigen::Vector3d& elbow_pos_des,
    const std::vector<double>& q_nominal, double elbow_w_scale, double ori_w_scale,
    double cont_w_scale, double nom_w_scale) const {
    HumanMimicIkResult result;
    result.q = q_seed;
    result.accepted = true;
    if (kin_ == nullptr || dof_ == 0 || q_seed.size() < dof_) {
        return result;
    }

    const int n = static_cast<int>(dof_);
    const int m = 9 + 2 * n;
    const double sw_wrist = std::sqrt(std::max(0.0, config_.w_wrist_pos));
    const double sw_ori =
        std::sqrt(std::max(0.0, config_.w_hand_ori * std::max(0.0, ori_w_scale)));
    const double sw_elbow =
        std::sqrt(std::max(0.0, config_.w_elbow * std::max(0.0, elbow_w_scale)));
    const double sw_cont =
        std::sqrt(std::max(0.0, config_.w_continuity * std::max(0.0, cont_w_scale)));
    const double sw_nom =
        std::sqrt(std::max(0.0, config_.w_nominal * std::max(0.0, nom_w_scale)));
    const bool use_nominal = q_nominal.size() >= dof_ && sw_nom > 0.0;
    const double kp_pos = std::max(0.1, config_.ik_kp_pos);
    const double kp_ori = std::max(0.1, config_.ik_kp_ori);

    std::vector<double> q = q_seed;
    clampToLimits(&q);

    for (int iter = 0; iter < config_.ik_max_iters; ++iter) {
        result.iters = iter + 1;

        Eigen::Vector3d p_wrist, p_elbow;
        Eigen::Matrix3d R_hand;
        Eigen::MatrixXd Jp_wrist, Jw_hand, Jp_elbow;
        if (!kin_->WristState(q, &p_wrist, &R_hand, &Jp_wrist, &Jw_hand) ||
            !kin_->ElbowPosJac(q, &p_elbow, &Jp_elbow)) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        const Eigen::Vector3d e_wrist = p_wrist - wrist_pos_des;
        const Eigen::Vector3d e_ori = orientation_error(R_hand, hand_R_des);
        const Eigen::Vector3d e_elbow = p_elbow - elbow_pos_des;

        result.wrist_pos_err = e_wrist.norm();
        result.hand_ori_err = e_ori.norm();
        result.elbow_pos_err = e_elbow.norm();
        result.elbow_phi_err = 0.0;

        // Desired task velocities (move opposite to weighted error).
        Eigen::VectorXd v(m);
        v.segment<3>(0) = -kp_pos * sw_wrist * e_wrist;
        v.segment<3>(3) = -kp_ori * sw_ori * e_ori;
        v.segment<3>(6) = -kp_pos * sw_elbow * e_elbow;
        for (int i = 0; i < n; ++i) {
            v(9 + i) = -sw_cont * (q[static_cast<size_t>(i)] - q_seed[static_cast<size_t>(i)]);
            const double target =
                use_nominal ? q_nominal[static_cast<size_t>(i)] : q_seed[static_cast<size_t>(i)];
            v(9 + n + i) = -sw_nom * (q[static_cast<size_t>(i)] - target);
        }

        Eigen::MatrixXd J(m, n);
        J.setZero();
        J.block(0, 0, 3, n) = sw_wrist * Jp_wrist;
        J.block(3, 0, 3, n) = -sw_ori * Jw_hand;
        J.block(6, 0, 3, n) = sw_elbow * Jp_elbow;
        for (int i = 0; i < n; ++i) {
            J(9 + i, i) = sw_cont;
            J(9 + n + i, i) = sw_nom;
        }

        // min ||J dq - v||^2 + λ||dq||^2  =>  dq = (J^T J + λI)^{-1} J^T v
        Eigen::MatrixXd H = J.transpose() * J;
        H.diagonal().array() += config_.ik_lambda * config_.ik_lambda;
        const Eigen::VectorXd g = J.transpose() * v;
        Eigen::VectorXd dq = H.ldlt().solve(g);
        if (!dq.allFinite()) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        applySoftJointLimitScale(q, &dq);
        projectDqBox(q, &dq);

        for (int i = 0; i < n; ++i) {
            q[static_cast<size_t>(i)] += dq(i);
        }
        clampToLimits(&q);

        if (dq.norm() < config_.ik_tol) {
            break;
        }
    }

    fillTaskErrors(q, q_seed, wrist_pos_des, hand_R_des, elbow_pos_des, q_nominal, elbow_w_scale,
                   ori_w_scale, cont_w_scale, nom_w_scale, nullptr, &result);

    result.q = q;
    result.ok = true;
    result.accepted = true;
    return result;
}

HumanMimicIkResult HumanMimicIkSolver::SolveLm(const std::vector<double>& q_seed,
                                               const Eigen::Vector3d& wrist_pos_des,
                                               const Eigen::Matrix3d& hand_R_des,
                                               const Eigen::Vector3d& elbow_pos_des,
                                               const std::vector<double>& q_nominal,
                                               double elbow_w_scale, double ori_w_scale,
                                               double cont_w_scale, double nom_w_scale) const {
    HumanMimicIkResult result;
    result.q = q_seed;
    result.accepted = true;
    if (kin_ == nullptr || dof_ == 0 || q_seed.size() < dof_) {
        return result;
    }

    const int n = static_cast<int>(dof_);
    const double sw_wrist = std::sqrt(std::max(0.0, config_.w_wrist_pos));
    const double sw_ori =
        std::sqrt(std::max(0.0, config_.w_hand_ori * std::max(0.0, ori_w_scale)));
    const double sw_elbow =
        std::sqrt(std::max(0.0, config_.w_elbow * std::max(0.0, elbow_w_scale)));
    const double sw_cont =
        std::sqrt(std::max(0.0, config_.w_continuity * std::max(0.0, cont_w_scale)));
    const double sw_nom =
        std::sqrt(std::max(0.0, config_.w_nominal * std::max(0.0, nom_w_scale)));
    const bool use_nominal = q_nominal.size() >= dof_ && sw_nom > 0.0;

    std::vector<double> q = q_seed;
    clampToLimits(&q);

    const int m = 9 + 2 * n;

    for (int iter = 0; iter < config_.ik_max_iters; ++iter) {
        result.iters = iter + 1;

        Eigen::Vector3d p_wrist, p_elbow;
        Eigen::Matrix3d R_hand;
        Eigen::MatrixXd Jp_wrist, Jw_hand, Jp_elbow;
        if (!kin_->WristState(q, &p_wrist, &R_hand, &Jp_wrist, &Jw_hand) ||
            !kin_->ElbowPosJac(q, &p_elbow, &Jp_elbow)) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        const Eigen::Vector3d e_wrist = p_wrist - wrist_pos_des;
        const Eigen::Vector3d e_ori = orientation_error(R_hand, hand_R_des);
        const Eigen::Vector3d e_elbow = p_elbow - elbow_pos_des;

        result.wrist_pos_err = e_wrist.norm();
        result.hand_ori_err = e_ori.norm();
        result.elbow_pos_err = e_elbow.norm();
        result.elbow_phi_err = 0.0;

        Eigen::VectorXd r(m);
        Eigen::MatrixXd J(m, n);
        J.setZero();
        r.setZero();

        r.segment<3>(0) = sw_wrist * e_wrist;
        J.block(0, 0, 3, n) = sw_wrist * Jp_wrist;

        r.segment<3>(3) = sw_ori * e_ori;
        J.block(3, 0, 3, n) = -sw_ori * Jw_hand;

        r.segment<3>(6) = sw_elbow * e_elbow;
        J.block(6, 0, 3, n) = sw_elbow * Jp_elbow;

        for (int i = 0; i < n; ++i) {
            r(9 + i) = sw_cont * (q[static_cast<size_t>(i)] - q_seed[static_cast<size_t>(i)]);
            J(9 + i, i) = sw_cont;
        }
        for (int i = 0; i < n; ++i) {
            const double target = use_nominal ? q_nominal[static_cast<size_t>(i)]
                                              : q_seed[static_cast<size_t>(i)];
            r(9 + n + i) = sw_nom * (q[static_cast<size_t>(i)] - target);
            J(9 + n + i, i) = sw_nom;
        }

        Eigen::MatrixXd H = J.transpose() * J;
        H.diagonal().array() += config_.ik_lambda * config_.ik_lambda;
        const Eigen::VectorXd g = J.transpose() * r;
        Eigen::VectorXd dq = H.ldlt().solve(-g);
        if (!dq.allFinite()) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        for (int i = 0; i < n; ++i) {
            dq(i) = std::clamp(dq(i), -config_.ik_max_step, config_.ik_max_step);
            q[static_cast<size_t>(i)] += dq(i);
        }
        clampToLimits(&q);

        if (dq.norm() < config_.ik_tol) {
            break;
        }
    }

    fillTaskErrors(q, q_seed, wrist_pos_des, hand_R_des, elbow_pos_des, q_nominal, elbow_w_scale,
                   ori_w_scale, cont_w_scale, nom_w_scale, nullptr, &result);

    result.q = q;
    result.ok = true;
    result.accepted = true;
    return result;
}
