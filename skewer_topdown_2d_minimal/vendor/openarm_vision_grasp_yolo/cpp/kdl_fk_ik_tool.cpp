#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string urdf;
    std::string base = "openarm_body_link0";
    std::string ee = "openarm_right_hand_tcp";
    std::vector<double> seed;
    bool has_target = false;
    std::vector<double> target16;
    std::string output;
    double eps = 1e-5;
    int max_iter = 500;
};

void usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --urdf PATH --base LINK --ee LINK --seed q1,...,q7 "
        << "[--target m00,...,m33] --output result.json [--eps 1e-5] [--max-iter 500]\n";
}

std::vector<double> parse_csv_doubles(const std::string& text) {
    std::vector<double> values;
    std::stringstream ss(text);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        if (cell.empty()) continue;
        char* end = nullptr;
        const double value = std::strtod(cell.c_str(), &end);
        if (end == cell.c_str() || !std::isfinite(value)) {
            throw std::runtime_error("invalid numeric value: " + cell);
        }
        values.push_back(value);
    }
    return values;
}

Options parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + name);
            }
            return argv[++i];
        };
        if (arg == "--urdf") {
            options.urdf = require_value(arg);
        } else if (arg == "--base") {
            options.base = require_value(arg);
        } else if (arg == "--ee") {
            options.ee = require_value(arg);
        } else if (arg == "--seed") {
            options.seed = parse_csv_doubles(require_value(arg));
        } else if (arg == "--target") {
            options.target16 = parse_csv_doubles(require_value(arg));
            options.has_target = true;
        } else if (arg == "--output") {
            options.output = require_value(arg);
        } else if (arg == "--eps") {
            options.eps = std::stod(require_value(arg));
        } else if (arg == "--max-iter") {
            options.max_iter = std::stoi(require_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    if (options.urdf.empty() || options.output.empty() || options.seed.empty()) {
        usage(argv[0]);
        throw std::runtime_error("--urdf, --seed and --output are required");
    }
    if (options.has_target && options.target16.size() != 16) {
        throw std::runtime_error("--target must contain 16 row-major values");
    }
    return options;
}

std::shared_ptr<urdf::ModelInterface> load_urdf(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open URDF: " + path);
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    auto model = urdf::parseURDF(buffer.str());
    if (!model) {
        throw std::runtime_error("failed to parse URDF: " + path);
    }
    return model;
}

std::vector<std::string> chain_joint_names(const KDL::Chain& chain) {
    std::vector<std::string> names;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const auto& joint = chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
            names.push_back(joint.getName());
        }
    }
    return names;
}

void build_joint_limits(const urdf::ModelInterface& model,
                        const std::vector<std::string>& joint_names,
                        KDL::JntArray& q_min,
                        KDL::JntArray& q_max) {
    q_min.resize(joint_names.size());
    q_max.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const auto joint = model.getJoint(joint_names[i]);
        if (!joint) {
            throw std::runtime_error("joint not found in URDF: " + joint_names[i]);
        }
        if (joint->type == urdf::Joint::CONTINUOUS) {
            q_min(i) = -M_PI;
            q_max(i) = M_PI;
        } else if (joint->limits) {
            q_min(i) = joint->limits->lower;
            q_max(i) = joint->limits->upper;
        } else {
            q_min(i) = -M_PI;
            q_max(i) = M_PI;
        }
    }
}

KDL::JntArray make_q(const std::vector<double>& values, unsigned int dof) {
    if (values.size() != dof) {
        std::ostringstream msg;
        msg << "expected " << dof << " joint values, got " << values.size();
        throw std::runtime_error(msg.str());
    }
    KDL::JntArray q(dof);
    for (unsigned int i = 0; i < dof; ++i) {
        q(i) = values[i];
    }
    return q;
}

KDL::Frame frame_from_row_major(const std::vector<double>& values) {
    KDL::Rotation R(
        values[0], values[1], values[2],
        values[4], values[5], values[6],
        values[8], values[9], values[10]);
    KDL::Vector p(values[3], values[7], values[11]);
    return KDL::Frame(R, p);
}

void write_matrix_json(std::ostream& out, const char* key, const KDL::Frame& frame,
                       int indent = 2) {
    const std::string sp(indent, ' ');
    out << sp << "\"" << key << "\": [\n";
    for (int r = 0; r < 4; ++r) {
        out << sp << "  [";
        for (int c = 0; c < 4; ++c) {
            double value = 0.0;
            if (r < 3 && c < 3) value = frame.M(r, c);
            if (r < 3 && c == 3) value = frame.p[r];
            if (r == 3 && c == 3) value = 1.0;
            out << std::fixed << std::setprecision(9) << value;
            if (c != 3) out << ", ";
        }
        out << "]";
        if (r != 3) out << ",";
        out << "\n";
    }
    out << sp << "]";
}

void write_vector_json(std::ostream& out, const char* key, const KDL::JntArray& q,
                       int indent = 2) {
    const std::string sp(indent, ' ');
    out << sp << "\"" << key << "\": [";
    for (unsigned int i = 0; i < q.rows(); ++i) {
        out << std::fixed << std::setprecision(9) << q(i);
        if (i + 1 != q.rows()) out << ", ";
    }
    out << "]";
}

void write_string_vector_json(std::ostream& out, const char* key,
                              const std::vector<std::string>& values, int indent = 2) {
    const std::string sp(indent, ' ');
    out << sp << "\"" << key << "\": [";
    for (size_t i = 0; i < values.size(); ++i) {
        out << "\"" << values[i] << "\"";
        if (i + 1 != values.size()) out << ", ";
    }
    out << "]";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_args(argc, argv);
        auto urdf_model = load_urdf(options.urdf);

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree)) {
            throw std::runtime_error("failed to build KDL tree");
        }

        KDL::Chain chain;
        if (!tree.getChain(options.base, options.ee, chain)) {
            throw std::runtime_error("failed to get KDL chain: " + options.base + " -> " + options.ee);
        }
        const unsigned int dof = chain.getNrOfJoints();
        const auto joint_names = chain_joint_names(chain);
        KDL::JntArray q_seed = make_q(options.seed, dof);

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::Frame fk_frame;
        if (fk_solver.JntToCart(q_seed, fk_frame) < 0) {
            throw std::runtime_error("FK failed");
        }

        bool ik_success = false;
        int ik_ret = 0;
        std::string ik_method = "none";
        KDL::JntArray q_solution(dof);
        if (options.has_target) {
            KDL::JntArray q_min, q_max;
            build_joint_limits(*urdf_model, joint_names, q_min, q_max);
            const KDL::Frame target = frame_from_row_major(options.target16);

            // 1) Newton-Raphson with joint limits (local, seed-sensitive).
            {
                KDL::ChainIkSolverVel_pinv vel_solver(chain);
                KDL::ChainIkSolverPos_NR_JL ik_solver(
                    chain, q_min, q_max, fk_solver, vel_solver, options.max_iter, options.eps);
                ik_ret = ik_solver.CartToJnt(q_seed, target, q_solution);
                ik_success = ik_ret >= 0;
                if (ik_success) {
                    ik_method = "nr_jl";
                }
            }

            // 2) Levenberg-Marquardt often converges from the same seed when NR fails.
            if (!ik_success) {
                KDL::ChainIkSolverPos_LMA lma_solver(chain, options.eps, options.max_iter);
                KDL::JntArray q_lma(dof);
                const int lma_ret = lma_solver.CartToJnt(q_seed, target, q_lma);
                if (lma_ret >= 0) {
                    bool within_limits = true;
                    for (unsigned int i = 0; i < dof; ++i) {
                        if (q_lma(i) < q_min(i) - 1e-6 || q_lma(i) > q_max(i) + 1e-6) {
                            within_limits = false;
                            break;
                        }
                    }
                    if (within_limits) {
                        q_solution = q_lma;
                        ik_ret = lma_ret;
                        ik_success = true;
                        ik_method = "lma";
                    } else {
                        ik_ret = lma_ret;
                        ik_method = "lma_out_of_limits";
                    }
                } else {
                    ik_ret = lma_ret;
                    ik_method = "lma_failed";
                }
            }
        }

        std::ofstream out(options.output);
        if (!out) {
            throw std::runtime_error("failed to open output: " + options.output);
        }
        out << "{\n";
        out << "  \"urdf\": \"" << options.urdf << "\",\n";
        out << "  \"base_frame\": \"" << options.base << "\",\n";
        out << "  \"ee_frame\": \"" << options.ee << "\",\n";
        out << "  \"dof\": " << dof << ",\n";
        write_string_vector_json(out, "joint_names", joint_names);
        out << ",\n";
        write_vector_json(out, "seed_joints", q_seed);
        out << ",\n";
        write_matrix_json(out, "T_base_ee_fk", fk_frame);
        if (options.has_target) {
            out << ",\n";
            out << "  \"ik_success\": " << (ik_success ? "true" : "false") << ",\n";
            out << "  \"ik_return_code\": " << ik_ret << ",\n";
            out << "  \"ik_method\": \"" << ik_method << "\",\n";
            if (ik_success) {
                write_vector_json(out, "ik_solution_joints", q_solution);
                out << ",\n";
                std::vector<double> delta(dof);
                KDL::JntArray q_delta(dof);
                for (unsigned int i = 0; i < dof; ++i) q_delta(i) = q_solution(i) - q_seed(i);
                write_vector_json(out, "ik_delta_from_seed", q_delta);
            } else {
                out << "  \"ik_solution_joints\": null";
            }
        }
        out << "\n}\n";

        std::cout << "[kdl_fk_ik_tool] wrote " << options.output << std::endl;
        if (options.has_target) {
            std::cout << "[kdl_fk_ik_tool] IK " << (ik_success ? "success" : "failed")
                      << " code=" << ik_ret << " method=" << ik_method << std::endl;
        }
        return ik_success || !options.has_target ? 0 : 2;
    } catch (const std::exception& exc) {
        std::cerr << "[kdl_fk_ik_tool][ERROR] " << exc.what() << std::endl;
        return 1;
    }
}
