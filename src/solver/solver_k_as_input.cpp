//
// Created by ljn on 20-1-27.
//

#include "path_optimizer/solver/solver_k_as_input.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/data_struct/vehicle_state_frenet.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/config/planning_flags.hpp"

namespace PathOptimizationNS {

SolverKAsInput::SolverKAsInput(const ReferencePath &reference_path,
                               const VehicleState &vehicle_state,
                               const size_t &horizon) :
    OsqpSolver(reference_path, vehicle_state, horizon) {
    num_of_variables_ = 4 * horizon_ - 1;
    num_of_constraints_ = 11 * horizon_ - 1;
}

void SolverKAsInput::getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                      std::vector<PathOptimizationNS::State> *optimized_path) const {
    CHECK_EQ(optimization_result.size(), num_of_variables_);
    optimized_path->clear();
    const auto &ref_states = reference_path_.getReferenceStates();
    double tmp_s = 0;
    for (size_t i = 0; i != horizon_; ++i) {
        double angle = ref_states[i].z;
        double new_angle = constraintAngle(angle + M_PI_2);
        double tmp_x = ref_states[i].x + optimization_result(2 * i + 1) * cos(new_angle);
        double tmp_y = ref_states[i].y + optimization_result(2 * i + 1) * sin(new_angle);
        double k = 0;
        if (i != horizon_ - 1) {
            k = optimization_result(2 * horizon_ + i);
        } else {
            k = optimization_result(3 * horizon_ - 2);
        }
        if (i != 0) {
            tmp_s += sqrt(pow(tmp_x - optimized_path->back().x, 2) + pow(tmp_y - optimized_path->back().y, 2));
        }
        optimized_path->emplace_back(tmp_x, tmp_y, angle + optimization_result(2 * i), k, tmp_s);
    }
}

void SolverKAsInput::setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t state_size = 2 * horizon_;
    const size_t control_size = horizon_ - 1;
    const size_t slack_size = horizon_;
    const size_t matrix_size = state_size + control_size + slack_size;
    double w_c = FLAGS_K_curvature_weight;
    double w_cr = FLAGS_K_curvature_rate_weight;
    double w_pq = FLAGS_K_deviation_weight;
    double w_e = FLAGS_KP_slack_weight;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    // Populate hessian matrix
    // Matrix Q is for state variables, only related to e_y.
    Eigen::Matrix2d matrix_Q;
    matrix_Q << 0, 0,
        0, w_pq;
    // Matrix R is for control variables.
    Eigen::SparseMatrix<double> matrix_R;
    matrix_R.resize(control_size, control_size);
    for (size_t i = 0; i != control_size; ++i) {
        for (size_t j = 0; j != control_size; ++j) {
            if (i == j) {
                if (i == 0 || i == control_size - 1) {
                    matrix_R.insert(i, j) = w_c + w_cr;
                } else {
                    matrix_R.insert(i, j) = w_cr * 2 + w_c;
                }
            } else if (i == j - 1 || i == j + 1) {
                matrix_R.insert(i, j) = -w_cr;
            }
        }
    }
    // Matrix S is for slack variables.
    Eigen::MatrixXd matrix_S = Eigen::MatrixXd::Identity(slack_size, slack_size) * w_e;

    // Fill Q, R and S into hessian matrix.
    for (size_t i = 0; i != horizon_; ++i) {
        hessian.block(2 * i, 2 * i, 2, 2) = matrix_Q;
    }
    hessian.block(2 * horizon_, 2 * horizon_, control_size, control_size) = matrix_R;
    hessian.block(3 * horizon_ - 1, 3 * horizon_ - 1, slack_size, slack_size) = matrix_S;
    *matrix_h = hessian.sparseView();
}

void SolverKAsInput::setDynamicMatrix(size_t i,
                                      Eigen::Matrix<double, 2, 2> *matrix_a,
                                      Eigen::Matrix<double, 2, 1> *matrix_b) const {
    const auto &ref_states = reference_path_.getReferenceStates();
    double ref_k = ref_states[i].k;
    double ref_s = ref_states[i + 1].s - ref_states[i].s;
    double ref_delta = atan(ref_k * FLAGS_wheel_base);
    Eigen::Matrix2d a;
    a << 1, -ref_s * pow(ref_k, 2),
        ref_s, 1;
    Eigen::Matrix<double, 2, 1> b;
    b << ref_s / FLAGS_wheel_base / pow(cos(ref_delta), 2), 0;
    *matrix_a = a;
    *matrix_b = b;
}

void SolverKAsInput::setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                         Eigen::VectorXd *lower_bound,
                                         Eigen::VectorXd *upper_bound) const {
    const auto &ref_states = reference_path_.getReferenceStates();
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(11 * horizon_ - 1, 4 * horizon_ - 1);

    // Set trans part.
    for (size_t i = 0; i != 2 * horizon_; ++i) {
        cons(i, i) = -1;
    }
    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        setDynamicMatrix(i, &a, &b);
        cons.block(2 * (i + 1), 2 * i, 2, 2) = a;
        cons.block(2 * (i + 1), 2 * horizon_ + i, 2, 1) = b;
    }

    // Set variable constraint part.
    for (size_t i = 0; i != 4 * horizon_ - 1; ++i) {
        cons(2 * horizon_ + i, i) = 1;
    }

    // Set collision avoidance part 1. This part does not include the second circle.
    Eigen::Matrix<double, 3, 2> collision;
    collision << FLAGS_d1, 1,
//        FLAGS_d2, 1,
        FLAGS_d3, 1,
        FLAGS_d4, 1;
    for (size_t i = 0; i != horizon_; ++i) {
        cons.block(6 * horizon_ - 1 + 3 * i, 2 * i, 3, 2) = collision;
    }

    // Set collison avoidance part 2, This part contains the second circle only.
    // The purpose for this is to shrink the drivable corridor and then add a slack variable on it.
    Eigen::Matrix<double, 1, 2> collision1;
    collision1 << FLAGS_d2, 1;
    for (size_t i = 0; i != horizon_; ++i) {
        cons.block(9 * horizon_ - 1 + i, 2 * i, 1, 2) = collision1;
        cons.block(10 * horizon_ - 1 + i, 2 * i, 1, 2) = collision1;
    }
    cons.block(9 * horizon_ - 1, 3 * horizon_ - 1, horizon_, horizon_) = -Eigen::MatrixXd::Identity(horizon_, horizon_);
    cons.block(10 * horizon_ - 1, 3 * horizon_ - 1, horizon_, horizon_) = Eigen::MatrixXd::Identity(horizon_, horizon_);
    // Finished.
    *matrix_constraints = cons.sparseView();

    // Set initial state bounds.
    *lower_bound = Eigen::MatrixXd::Zero(11 * horizon_ - 1, 1);
    *upper_bound = Eigen::MatrixXd::Zero(11 * horizon_ - 1, 1);
    Eigen::Matrix<double, 2, 1> x0;
    auto init_error = vehicle_state_.getInitError();
    x0 << init_error[1], init_error[0];
    lower_bound->block(0, 0, 2, 1) = -x0;
    upper_bound->block(0, 0, 2, 1) = -x0;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        double ds = ref_states[i + 1].s - ref_states[i].s;
        double steer = atan(ref_states[i].k * FLAGS_wheel_base);
        Eigen::Vector2d c;
        c << ds * steer / FLAGS_wheel_base / pow(cos(steer), 2), 0;
        lower_bound->block(2 + 2 * i, 0, 2, 1) = c;
        upper_bound->block(2 + 2 * i, 0, 2, 1) = c;
    }
    // State variables bounds.
    lower_bound->block(2 * horizon_, 0, 2 * horizon_, 1) = Eigen::VectorXd::Constant(2 * horizon_, -OsqpEigen::INFTY);
    upper_bound->block(2 * horizon_, 0, 2 * horizon_, 1) = Eigen::VectorXd::Constant(2 * horizon_, OsqpEigen::INFTY);
    // Add end state bounds.
    if (FLAGS_constraint_end_heading) {
        double end_psi = constraintAngle(vehicle_state_.getEndState().z - ref_states.back().z);
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(2 * horizon_ + 2 * horizon_ - 2) = end_psi - 5 * M_PI / 180;
            (*upper_bound)(2 * horizon_ + 2 * horizon_ - 2) = end_psi + 5 * M_PI / 180;
        }
    }
    // Control variables bounds.
    lower_bound->block(4 * horizon_, 0, horizon_ - 1, 1) =
        Eigen::VectorXd::Constant(horizon_ - 1, -FLAGS_max_steering_angle);
    upper_bound->block(4 * horizon_, 0, horizon_ - 1, 1) =
        Eigen::VectorXd::Constant(horizon_ - 1, FLAGS_max_steering_angle);
    // Slack variables bounds.
    lower_bound->block(5 * horizon_ - 1, 0, horizon_, 1) = Eigen::VectorXd::Constant(horizon_, 0);
    upper_bound->block(5 * horizon_ - 1, 0, horizon_, 1) =
        Eigen::VectorXd::Constant(horizon_, FLAGS_expected_safety_margin);
    // Set collision bound part 1.
    const auto &bounds = reference_path_.getBounds();
    for (size_t i = 0; i != horizon_; ++i) {
        Eigen::Vector3d ld, ud;
        ud
            << bounds[i].c0.ub, bounds[i].c2.ub, bounds[i].c3.ub;
        ld
            << bounds[i].c0.lb, bounds[i].c2.lb, bounds[i].c3.lb;
        lower_bound->block(6 * horizon_ - 1 + 3 * i, 0, 3, 1) = ld;
        upper_bound->block(6 * horizon_ - 1 + 3 * i, 0, 3, 1) = ud;
    }
    // Set collision bound part 2.
    upper_bound->block(10 * horizon_ - 1, 0, horizon_, 1) = Eigen::VectorXd::Constant(horizon_, OsqpEigen::INFTY);
    lower_bound->block(9 * horizon_ - 1, 0, horizon_, 1) = Eigen::VectorXd::Constant(horizon_, -OsqpEigen::INFTY);
    for (size_t i = 0; i != horizon_; ++i) {
        double ud = bounds[i].c1.ub - FLAGS_expected_safety_margin;
        double ld = bounds[i].c1.lb + FLAGS_expected_safety_margin;
        (*upper_bound)(9 * horizon_ - 1 + i, 0) = ud;
        (*lower_bound)(10 * horizon_ - 1 + i, 0) = ld;
    }
}
}