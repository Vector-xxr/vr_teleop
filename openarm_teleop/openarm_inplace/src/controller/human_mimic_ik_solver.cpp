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

double wrap_to_pi(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
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

int HumanMimicIkSolver::elbowTaskRows() const {
    return (config_.elbow_task_mode == ElbowTaskMode::PHI) ? 1 : 3;
}

int HumanMimicIkSolver::residualRows() const {
    return 6 + elbowTaskRows() + 2 * static_cast<int>(dof_);
}

bool HumanMimicIkSolver::computeElbowPhiResidual(
    const VirtualArmSkeleton* skeleton, const Eigen::Vector3d& shoulder,
    const Eigen::Vector3d& wrist, const Eigen::Vector3d& elbow, const Eigen::MatrixXd& Jp_elbow,
    double phi_des, double* e_phi, Eigen::RowVectorXd* J_phi) const {
    if (skeleton == nullptr || e_phi == nullptr) {
        return false;
    }
    double phi_meas = 0.0;
    double h = 0.0;
    if (!skeleton->PhiFromPose(shoulder, elbow, wrist, &phi_meas, &h) || h < 1e-4) {
        *e_phi = 0.0;
        if (J_phi != nullptr) {
            J_phi->setZero(1, Jp_elbow.cols());
        }
        return false;
    }
    *e_phi = wrap_to_pi(phi_meas - phi_des);

    if (J_phi == nullptr) {
        return true;
    }

    Eigen::Vector3d center, axis, e1, e2;
    double hh = 0.0;
    if (!skeleton->SwivelCircle(shoulder, wrist, skeleton->L1(), skeleton->L2(), &center, &axis,
                                &e1, &e2, &hh) ||
        hh < 1e-4) {
        J_phi->setZero(1, Jp_elbow.cols());
        return true;
    }
    // phi = atan2(e2·r, e1·r), r = E - center (center fixed for this wrist target).
    const Eigen::Vector3d r = elbow - center;
    const double c = e1.dot(r);
    const double s = e2.dot(r);
    const double den = c * c + s * s;
    if (den < 1e-10) {
        J_phi->setZero(1, Jp_elbow.cols());
        return true;
    }
    // dphi/dr = (c e2 - s e1) / (c^2 + s^2)
    const Eigen::Vector3d dphi_dr = (c * e2 - s * e1) / den;
    J_phi->resize(1, Jp_elbow.cols());
    *J_phi = dphi_dr.transpose() * Jp_elbow;
    return true;
}

HumanMimicIkResult HumanMimicIkSolver::Solve(const std::vector<double>& q_seed,
                                             const Eigen::Vector3d& wrist_pos_des,
                                             const Eigen::Matrix3d& hand_R_des,
                                             const Eigen::Vector3d& elbow_pos_des,
                                             const std::vector<double>& q_nominal,
                                             double elbow_w_scale, double ori_w_scale,
                                             double cont_w_scale, double nom_w_scale,
                                             double elbow_phi_des, const Eigen::Vector3d& shoulder,
                                             const VirtualArmSkeleton* skeleton) const {
    if (config_.ik_backend == HumanMimicIkBackend::DIFF_QP) {
        return SolveDiffQp(q_seed, wrist_pos_des, hand_R_des, elbow_pos_des, q_nominal,
                           elbow_w_scale, ori_w_scale, cont_w_scale, nom_w_scale, elbow_phi_des,
                           shoulder, skeleton);
    }
    return SolveLm(q_seed, wrist_pos_des, hand_R_des, elbow_pos_des, q_nominal, elbow_w_scale,
                   ori_w_scale, cont_w_scale, nom_w_scale, elbow_phi_des, shoulder, skeleton);
}

void HumanMimicIkSolver::fillTaskErrors(
    const std::vector<double>& q, const std::vector<double>& q_seed,
    const Eigen::Vector3d& wrist_pos_des, const Eigen::Matrix3d& hand_R_des,
    const Eigen::Vector3d& elbow_pos_des, const std::vector<double>& q_nominal,
    double elbow_w_scale, double ori_w_scale, double cont_w_scale, double nom_w_scale,
    double elbow_phi_des, const Eigen::Vector3d& shoulder, const VirtualArmSkeleton* skeleton,
    Eigen::VectorXd* r, HumanMimicIkResult* result) const {
    const int n = static_cast<int>(dof_);
    const int elbow_rows = elbowTaskRows();
    const bool phi_mode =
        config_.elbow_task_mode == ElbowTaskMode::PHI && skeleton != nullptr;
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

    TcpState tip;
    Eigen::Vector3d p_elbow;
    Eigen::MatrixXd Jp_elbow;
    if (!kin_->ComputeTcpState(q, config_.wrist_to_tcp, &tip) ||
        !kin_->ElbowPosJac(q, &p_elbow, &Jp_elbow)) {
        if (result != nullptr) {
            result->ok = false;
        }
        return;
    }

    const Eigen::Vector3d p_task = config_.ik_track_tcp ? tip.p_tcp : tip.p_wrist;
    const Eigen::Vector3d e_wrist = p_task - wrist_pos_des;
    const Eigen::Vector3d e_ori = orientation_error(tip.R_hand, hand_R_des);
    const Eigen::Vector3d wrist_pos_for_elbow =
        config_.ik_track_tcp ? wrist_pos_des - hand_R_des * config_.wrist_to_tcp
                             : wrist_pos_des;

    double e_phi = 0.0;
    Eigen::Vector3d e_elbow = Eigen::Vector3d::Zero();
    if (phi_mode) {
        computeElbowPhiResidual(skeleton, shoulder, wrist_pos_for_elbow, p_elbow, Jp_elbow,
                                elbow_phi_des, &e_phi, nullptr);
    } else {
        e_elbow = p_elbow - elbow_pos_des;
    }

    if (result != nullptr) {
        result->wrist_pos_err = e_wrist.norm();
        result->hand_ori_err = e_ori.norm();
        result->elbow_pos_err = phi_mode ? 0.0 : e_elbow.norm();
        result->elbow_phi_err = phi_mode ? std::abs(e_phi) : 0.0;
    }

    if (r == nullptr) {
        return;
    }

    r->segment<3>(0) = sw_wrist * e_wrist;
    r->segment<3>(3) = sw_ori * e_ori;
    if (phi_mode) {
        (*r)(6) = sw_elbow * e_phi;
    } else {
        r->segment<3>(6) = sw_elbow * e_elbow;
    }
    const int off = 6 + elbow_rows;
    for (int i = 0; i < n; ++i) {
        (*r)(off + i) = sw_cont * (q[static_cast<size_t>(i)] - q_seed[static_cast<size_t>(i)]);
        const double target =
            use_nominal ? q_nominal[static_cast<size_t>(i)] : q_seed[static_cast<size_t>(i)];
        (*r)(off + n + i) = sw_nom * (q[static_cast<size_t>(i)] - target);
    }
}

void HumanMimicIkSolver::applySoftJointLimitScale(const std::vector<double>& q,
                                                  Eigen::VectorXd* dq) const {
    if (dq == nullptr || !limits_valid_) {
        return;
    }
    const double margin_frac = std::clamp(config_.ik_joint_limit_margin_frac, 0.01, 0.45);
    for (int i = 0; i < dq->size() && static_cast<size_t>(i) < q.size(); ++i) {
        const double span = q_max_[static_cast<size_t>(i)] - q_min_[static_cast<size_t>(i)];
        if (!(span > 1e-6)) {
            continue;
        }
        const double soft = span * margin_frac;
        const double qi = q[static_cast<size_t>(i)];
        double& dqi = (*dq)(i);
        if (dqi > 0.0) {
            const double room = q_max_[static_cast<size_t>(i)] - qi;
            if (room <= 0.0) {
                dqi = 0.0;
            } else if (room < soft) {
                dqi *= room / soft;
            }
        } else if (dqi < 0.0) {
            const double room = qi - q_min_[static_cast<size_t>(i)];
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
    for (int i = 0; i < dq->size(); ++i) {
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
    double cont_w_scale, double nom_w_scale, double elbow_phi_des,
    const Eigen::Vector3d& shoulder, const VirtualArmSkeleton* skeleton) const {
    HumanMimicIkResult result;
    result.q = q_seed;
    result.accepted = true;
    if (kin_ == nullptr || dof_ == 0 || q_seed.size() < dof_) {
        return result;
    }

    const int n = static_cast<int>(dof_);
    const bool phi_mode =
        config_.elbow_task_mode == ElbowTaskMode::PHI && skeleton != nullptr;
    const bool elbow_ns = config_.elbow_nullspace_enabled && config_.use_elbow_skeleton &&
                          (phi_mode || config_.elbow_task_mode == ElbowTaskMode::XYZ);
    // When elbow is in nullspace: primary is tip 6D + light continuity (scaled).
    // Full-weight cont would steal the 1D swivel DOF; zero cont lets joint steps
    // explode (stutter with perfect e_wrist). Elbow remains secondary via N.
    const int elbow_rows_pri = elbow_ns ? 0 : elbowTaskRows();
    const bool cont_in_pri = !elbow_ns || config_.elbow_nullspace_cont_scale > 1e-9;
    const int cont_nom_rows = cont_in_pri ? (2 * n) : 0;
    const int m = 6 + elbow_rows_pri + cont_nom_rows;
    const double sw_wrist = std::sqrt(std::max(0.0, config_.w_wrist_pos));
    const double sw_ori =
        std::sqrt(std::max(0.0, config_.w_hand_ori * std::max(0.0, ori_w_scale)));
    const double sw_elbow =
        std::sqrt(std::max(0.0, config_.w_elbow * std::max(0.0, elbow_w_scale)));
    const double cont_ns =
        elbow_ns ? std::clamp(config_.elbow_nullspace_cont_scale, 0.0, 1.0) : 1.0;
    const double sw_cont =
        std::sqrt(std::max(0.0, config_.w_continuity * std::max(0.0, cont_w_scale))) * cont_ns;
    const double sw_nom =
        std::sqrt(std::max(0.0, config_.w_nominal * std::max(0.0, nom_w_scale))) * cont_ns;
    const bool use_nominal = q_nominal.size() >= dof_ && sw_nom > 0.0;
    const double kp_pos = std::max(0.1, config_.ik_kp_pos);
    const double kp_ori = std::max(0.1, config_.ik_kp_ori);
    const double kp_phi = std::max(0.1, config_.ik_kp_phi);
    const double lambda = std::max(1e-6, config_.ik_lambda);

    std::vector<double> q = q_seed;
    clampToLimits(&q);

    for (int iter = 0; iter < config_.ik_max_iters; ++iter) {
        result.iters = iter + 1;

        TcpState tip;
        Eigen::Vector3d p_elbow;
        Eigen::MatrixXd Jp_elbow;
        if (!kin_->ComputeTcpState(q, config_.wrist_to_tcp, &tip) ||
            !kin_->ElbowPosJac(q, &p_elbow, &Jp_elbow)) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        const Eigen::Vector3d p_task = config_.ik_track_tcp ? tip.p_tcp : tip.p_wrist;
        const Eigen::MatrixXd& Jp_task = config_.ik_track_tcp ? tip.Jp_tcp : tip.Jp_wrist;
        const Eigen::Vector3d e_wrist = p_task - wrist_pos_des;
        const Eigen::Vector3d e_ori = orientation_error(tip.R_hand, hand_R_des);
        const Eigen::Vector3d wrist_pos_for_elbow =
            config_.ik_track_tcp ? wrist_pos_des - hand_R_des * config_.wrist_to_tcp
                                 : wrist_pos_des;

        double e_phi = 0.0;
        Eigen::RowVectorXd J_phi;
        Eigen::Vector3d e_elbow = Eigen::Vector3d::Zero();
        bool elbow_jac_ok = false;
        if (phi_mode) {
            elbow_jac_ok = computeElbowPhiResidual(skeleton, shoulder, wrist_pos_for_elbow,
                                                   p_elbow, Jp_elbow, elbow_phi_des, &e_phi,
                                                   &J_phi);
        } else if (config_.use_elbow_skeleton) {
            e_elbow = p_elbow - elbow_pos_des;
            elbow_jac_ok = Jp_elbow.rows() == 3 && Jp_elbow.cols() == n;
        }

        result.wrist_pos_err = e_wrist.norm();
        result.hand_ori_err = e_ori.norm();
        result.elbow_pos_err = phi_mode ? 0.0 : e_elbow.norm();
        result.elbow_phi_err = phi_mode ? std::abs(e_phi) : 0.0;

        Eigen::VectorXd v(m);
        v.setZero();
        v.segment<3>(0) = -kp_pos * sw_wrist * e_wrist;
        v.segment<3>(3) = -kp_ori * sw_ori * e_ori;
        int off = 6;
        if (!elbow_ns) {
            if (phi_mode) {
                v(6) = -kp_phi * sw_elbow * e_phi;
            } else {
                v.segment<3>(6) = -kp_pos * sw_elbow * e_elbow;
            }
            off = 6 + elbow_rows_pri;
        }
        if (cont_in_pri) {
            for (int i = 0; i < n; ++i) {
                v(off + i) =
                    -sw_cont * (q[static_cast<size_t>(i)] - q_seed[static_cast<size_t>(i)]);
                const double target =
                    use_nominal ? q_nominal[static_cast<size_t>(i)]
                                : q_seed[static_cast<size_t>(i)];
                v(off + n + i) = -sw_nom * (q[static_cast<size_t>(i)] - target);
            }
        }

        Eigen::MatrixXd J(m, n);
        J.setZero();
        J.block(0, 0, 3, n) = sw_wrist * Jp_task;
        J.block(3, 0, 3, n) = -sw_ori * tip.Jw_hand;
        if (!elbow_ns) {
            if (phi_mode) {
                if (J_phi.size() == n) {
                    J.block(6, 0, 1, n) = sw_elbow * J_phi;
                }
            } else {
                J.block(6, 0, 3, n) = sw_elbow * Jp_elbow;
            }
        }
        if (cont_in_pri) {
            for (int i = 0; i < n; ++i) {
                J(off + i, i) = sw_cont;
                J(off + n + i, i) = sw_nom;
            }
        }

        Eigen::MatrixXd H = J.transpose() * J;
        H.diagonal().array() += lambda * lambda;
        const Eigen::VectorXd g = J.transpose() * v;
        Eigen::VectorXd dq = H.ldlt().solve(g);
        if (!dq.allFinite()) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        // Secondary: elbow in nullspace of unweighted 6D wrist+ori Jacobian.
        if (elbow_ns && elbow_jac_ok && sw_elbow > 1e-9 && elbow_w_scale > 1e-9) {
            Eigen::MatrixXd Jt(6, n);
            Jt.topRows(3) = Jp_task;
            Jt.bottomRows(3) = -tip.Jw_hand;
            Eigen::MatrixXd JJt = Jt * Jt.transpose();
            JJt.diagonal().array() += lambda * lambda;
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jt, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const double sigma_min =
                (svd.singularValues().size() > 0) ? svd.singularValues()(svd.singularValues().size() - 1)
                                                  : 0.0;
            if (sigma_min >= std::max(1e-6, config_.elbow_nullspace_sigma_min)) {
                const Eigen::MatrixXd Jsharp =
                    Jt.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));
                const Eigen::MatrixXd N =
                    Eigen::MatrixXd::Identity(n, n) - Jsharp * Jt;
                const double gain = std::max(0.0, config_.elbow_nullspace_gain);
                Eigen::VectorXd dq_sec = Eigen::VectorXd::Zero(n);
                if (phi_mode && J_phi.size() == n) {
                    // dq_sec = N * (-k J_φᵀ e_φ); k includes task weight.
                    const double k = gain * kp_phi * config_.w_elbow * std::max(0.0, elbow_w_scale);
                    dq_sec = N * (-k * J_phi.transpose() * e_phi);
                } else {
                    const double k = gain * kp_pos * config_.w_elbow * std::max(0.0, elbow_w_scale);
                    dq_sec = N * (-k * Jp_elbow.transpose() * e_elbow);
                }
                if (dq_sec.allFinite()) {
                    const double cap = std::max(1e-4, config_.elbow_nullspace_dq_max);
                    const double ns = dq_sec.norm();
                    if (ns > cap) {
                        dq_sec *= cap / ns;
                    }
                    dq += dq_sec;
                }
            }
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
                   ori_w_scale, cont_w_scale, nom_w_scale, elbow_phi_des, shoulder, skeleton,
                   nullptr, &result);

    result.q = q;
    result.ok = true;
    result.accepted = true;
    return result;
}

HumanMimicIkResult HumanMimicIkSolver::SolveLm(
    const std::vector<double>& q_seed, const Eigen::Vector3d& wrist_pos_des,
    const Eigen::Matrix3d& hand_R_des, const Eigen::Vector3d& elbow_pos_des,
    const std::vector<double>& q_nominal, double elbow_w_scale, double ori_w_scale,
    double cont_w_scale, double nom_w_scale, double elbow_phi_des,
    const Eigen::Vector3d& shoulder, const VirtualArmSkeleton* skeleton) const {
    HumanMimicIkResult result;
    result.q = q_seed;
    result.accepted = true;
    if (kin_ == nullptr || dof_ == 0 || q_seed.size() < dof_) {
        return result;
    }

    const int n = static_cast<int>(dof_);
    const int elbow_rows = elbowTaskRows();
    const int m = residualRows();
    const bool phi_mode =
        config_.elbow_task_mode == ElbowTaskMode::PHI && skeleton != nullptr;
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

    for (int iter = 0; iter < config_.ik_max_iters; ++iter) {
        result.iters = iter + 1;

        TcpState tip;
        Eigen::Vector3d p_elbow;
        Eigen::MatrixXd Jp_elbow;
        if (!kin_->ComputeTcpState(q, config_.wrist_to_tcp, &tip) ||
            !kin_->ElbowPosJac(q, &p_elbow, &Jp_elbow)) {
            result.ok = false;
            result.q = q_seed;
            result.accepted = false;
            return result;
        }

        const Eigen::Vector3d p_task = config_.ik_track_tcp ? tip.p_tcp : tip.p_wrist;
        const Eigen::MatrixXd& Jp_task = config_.ik_track_tcp ? tip.Jp_tcp : tip.Jp_wrist;
        const Eigen::Vector3d e_wrist = p_task - wrist_pos_des;
        const Eigen::Vector3d e_ori = orientation_error(tip.R_hand, hand_R_des);
        const Eigen::Vector3d wrist_pos_for_elbow =
            config_.ik_track_tcp ? wrist_pos_des - hand_R_des * config_.wrist_to_tcp
                                 : wrist_pos_des;

        double e_phi = 0.0;
        Eigen::RowVectorXd J_phi;
        Eigen::Vector3d e_elbow = Eigen::Vector3d::Zero();
        if (phi_mode) {
            computeElbowPhiResidual(skeleton, shoulder, wrist_pos_for_elbow, p_elbow, Jp_elbow,
                                    elbow_phi_des, &e_phi, &J_phi);
        } else {
            e_elbow = p_elbow - elbow_pos_des;
        }

        result.wrist_pos_err = e_wrist.norm();
        result.hand_ori_err = e_ori.norm();
        result.elbow_pos_err = phi_mode ? 0.0 : e_elbow.norm();
        result.elbow_phi_err = phi_mode ? std::abs(e_phi) : 0.0;

        Eigen::VectorXd r(m);
        Eigen::MatrixXd J(m, n);
        J.setZero();
        r.setZero();

        r.segment<3>(0) = sw_wrist * e_wrist;
        J.block(0, 0, 3, n) = sw_wrist * Jp_task;

        r.segment<3>(3) = sw_ori * e_ori;
        J.block(3, 0, 3, n) = -sw_ori * tip.Jw_hand;

        if (phi_mode) {
            r(6) = sw_elbow * e_phi;
            if (J_phi.size() == n) {
                J.block(6, 0, 1, n) = sw_elbow * J_phi;
            }
        } else {
            r.segment<3>(6) = sw_elbow * e_elbow;
            J.block(6, 0, 3, n) = sw_elbow * Jp_elbow;
        }

        const int off = 6 + elbow_rows;
        for (int i = 0; i < n; ++i) {
            r(off + i) = sw_cont * (q[static_cast<size_t>(i)] - q_seed[static_cast<size_t>(i)]);
            J(off + i, i) = sw_cont;
        }
        for (int i = 0; i < n; ++i) {
            const double target = use_nominal ? q_nominal[static_cast<size_t>(i)]
                                              : q_seed[static_cast<size_t>(i)];
            r(off + n + i) = sw_nom * (q[static_cast<size_t>(i)] - target);
            J(off + n + i, i) = sw_nom;
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
                   ori_w_scale, cont_w_scale, nom_w_scale, elbow_phi_des, shoulder, skeleton,
                   nullptr, &result);

    result.q = q;
    result.ok = true;
    result.accepted = true;
    return result;
}
