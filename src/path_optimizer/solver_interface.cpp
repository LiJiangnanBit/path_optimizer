//
// Created by ljn on 20-1-27.
//

#include "path_optimizer/solver_interface.hpp"
#include "tools/tools.hpp"

namespace PathOptimizationNS {

SolverInterface::SolverInterface(const Config &config,
                                 const ReferencePath &reference_path,
                                 const VehicleState &vehicle_state,
                                 const size_t &horizon) :
    config_(config),
    horizon_(horizon),
    reference_path_(reference_path),
    vehicle_state_(vehicle_state),
    solver_for_sampling_initialized_flag_(false),
    solver_for_dynamic_initialized_flag_(false),
    offset_error_allowed_(0) {
    std::cout << "optimization horizon: " << horizon_ << std::endl;
}

bool SolverInterface::solve(Eigen::VectorXd *solution) {
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(4 * horizon_ - 1);
    solver_.data()->setNumberOfConstraints(11 * horizon_ - 1);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(4 * horizon_ - 1); // TODO: check this.
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    // Set Hessian matrix.
    setHessianMatrix(&hessian);
    // Set state transition matrix, constraint matrix and bound vector.
    setConstraintMatrix(
        &linearMatrix,
        &lowerBound,
        &upperBound);
    // Input to solver.
    if (!solver_.data()->setHessianMatrix(hessian)) return false;
    if (!solver_.data()->setGradient(gradient)) return false;
    if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver_.data()->setLowerBound(lowerBound)) return false;
    if (!solver_.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver_.initSolver()) return false;
    if (!solver_.solve()) return false;
    *solution = solver_.getSolution();
    return true;
}

bool SolverInterface::initializeSampling(double target_angle, double angle_error_allowed, double offset_error_allowed) {
    offset_error_allowed_ = offset_error_allowed;
    solver_for_sampling_.settings()->setVerbosity(false);
    solver_for_sampling_.settings()->setWarmStart(true);
    solver_for_sampling_.settings()->setMaxIteraction(250);
    solver_for_sampling_.data()->setNumberOfVariables(3 * horizon_ - 1);
    solver_for_sampling_.data()->setNumberOfConstraints(9 * horizon_ - 1);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * horizon_ - 1);
    Eigen::SparseMatrix<double> linearMatrix;
    // Set Hessian matrix.
    setHessianMatrix(&hessian);
    // Set state transition matrix, constraint matrix and bound vector.
    setConstraintMatrixWithOffset(
        0,
        target_angle,
        angle_error_allowed,
        offset_error_allowed,
        &linearMatrix,
        &lowerBound_,
        &upperBound_);
    // Input to solver.
    if (!solver_for_sampling_.data()->setHessianMatrix(hessian)) return false;
    if (!solver_for_sampling_.data()->setGradient(gradient)) return false;
    if (!solver_for_sampling_.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver_for_sampling_.data()->setLowerBound(lowerBound_)) return false;
    if (!solver_for_sampling_.data()->setUpperBound(upperBound_)) return false;
    // Initialization.
    if (solver_for_sampling_.initSolver()) {
        solver_for_sampling_initialized_flag_ = true;
        return true;
    } else {
        return false;
    }
}

bool SolverInterface::solveSampling(Eigen::VectorXd *solution,
                                    double offset) {
    if (!solver_for_sampling_initialized_flag_) {
        std::cout << "sampling solver is uninitialized!" << std::endl;
        return false;
    } else {
        lowerBound_(2 * horizon_ + 2 * horizon_ - 1) = offset - offset_error_allowed_;
        upperBound_(2 * horizon_ + 2 * horizon_ - 1) = offset + offset_error_allowed_;
        if (!solver_for_sampling_.updateBounds(lowerBound_, upperBound_)) return false;
        // Solve.
        bool ok = solver_for_sampling_.solve();
        if (!ok) {
            return false;
        }
        // Get single path.
        *solution = solver_for_sampling_.getSolution();
        return true;
    }
}

bool SolverInterface::solveDynamic(Eigen::VectorXd *solution) {
    solver_for_dynamic_env_.settings()->setVerbosity(false);
    solver_for_dynamic_env_.settings()->setWarmStart(true);
//        solver_dynamic.settings()->setMaxIteraction(250);
    solver_for_dynamic_env_.data()->setNumberOfVariables(3 * horizon_ - 1);
    solver_for_dynamic_env_.data()->setNumberOfConstraints(9 * horizon_ - 1);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * horizon_ - 1);
    Eigen::SparseMatrix<double> linearMatrix;
    setHessianMatrix(&hessian);
    setConstraintMatrix(&linearMatrix,
                        &lowerBound_,
                        &upperBound_);
    if (!solver_for_dynamic_env_.data()->setHessianMatrix(hessian)) return false;
    if (!solver_for_dynamic_env_.data()->setGradient(gradient)) return false;
    if (!solver_for_dynamic_env_.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver_for_dynamic_env_.data()->setLowerBound(lowerBound_)) return false;
    if (!solver_for_dynamic_env_.data()->setUpperBound(upperBound_)) return false;
    if (!solver_for_dynamic_env_.initSolver()) return false;
    solver_for_dynamic_initialized_flag_ = true;
    bool ok = solver_for_dynamic_env_.solve();
    if (!ok) {
        printf("dynamic solver failed\n");
        return false;
    }
    *solution = solver_for_dynamic_env_.getSolution();
    solver_for_dynamic_initialized_flag_ = true;
    return true;
}

bool SolverInterface::solveDynamicUpdate(Eigen::VectorXd *solution, const std::vector<std::vector<double>> &clearance) {
    if (!solver_for_dynamic_initialized_flag_) {
        std::cout << "solver for dynamic env is not initialized!" << std::endl;
    } else {
        // Update bounds.
        for (size_t i = 0; i != horizon_; ++i) {
            Eigen::Vector4d ld, ud;
            ud
                << clearance[i][0], clearance[i][2], clearance[i][4], clearance[i][6];
            ld
                << clearance[i][1], clearance[i][3], clearance[i][5], clearance[i][7];
            lowerBound_.block(5 * horizon_ - 1 + 4 * i, 0, 4, 1) = ld;
            upperBound_.block(5 * horizon_ - 1 + 4 * i, 0, 4, 1) = ud;
        }
        if (!solver_for_dynamic_env_.updateBounds(lowerBound_, upperBound_)) return false;
        if (!solver_for_dynamic_env_.solve()) {
            printf("dynamic solver failed\n");
            return false;
        }
        *solution = solver_for_dynamic_env_.getSolution();
        return true;
    }
}

void SolverInterface::setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t state_size = 2 * horizon_;
    const size_t control_size = horizon_ - 1;
    const size_t slack_size = horizon_;
    const size_t matrix_size = state_size + control_size + slack_size;
    double w_c = config_.opt_curvature_w_;
    double w_cr = config_.opt_curvature_rate_w_;
    double w_pq = config_.opt_deviation_w_;
    double w_e = config_.opt_slack_w_;
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

void SolverInterface::setDynamicMatrix(size_t i,
                                       const std::vector<double> &seg_s_list,
                                       const std::vector<double> &seg_k_list,
                                       Eigen::Matrix<double, 2, 2> *matrix_a,
                                       Eigen::Matrix<double, 2, 1> *matrix_b) const {
    double ref_k = seg_k_list[i];
    double ref_s = seg_s_list[i + 1] - seg_s_list[i];
    double ref_delta = atan(ref_k * config_.wheel_base_);
    Eigen::Matrix2d a;
    a << 1, -ref_s * pow(ref_k, 2),
        ref_s, 1;
    Eigen::Matrix<double, 2, 1> b;
    b << ref_s / config_.wheel_base_ / pow(cos(ref_delta), 2), 0;
    *matrix_a = a;
    *matrix_b = b;
}

void SolverInterface::setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                          Eigen::VectorXd *lower_bound,
                                          Eigen::VectorXd *upper_bound) const {
    const auto &seg_s_list = reference_path_.seg_s_list_;
    const auto &seg_k_list = reference_path_.seg_k_list_;
    const auto &seg_angle_list = reference_path_.seg_angle_list_;
    const auto &seg_clearance_list = reference_path_.seg_clearance_list_;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(11 * horizon_ - 1, 4 * horizon_ - 1);

    // Set trans part.
    for (size_t i = 0; i != 2 * horizon_; ++i) {
        cons(i, i) = -1;
    }
    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        setDynamicMatrix(i, seg_s_list, seg_k_list, &a, &b);
        cons.block(2 * (i + 1), 2 * i, 2, 2) = a;
        cons.block(2 * (i + 1), 2 * horizon_ + i, 2, 1) = b;
    }

    // Set variable constraint part.
    for (size_t i = 0; i != 4 * horizon_ - 1; ++i) {
        cons(2 * horizon_ + i, i) = 1;
    }

    // Set collision avoidance part 1. This part does not include the second circle.
    Eigen::Matrix<double, 3, 2> collision;
    collision << config_.d1_ + config_.rear_axle_to_center_distance_, 1,
//        config_.d2_ + config_.rear_axle_to_center_distance_, 1,
        config_.d3_ + config_.rear_axle_to_center_distance_, 1,
        config_.d4_ + config_.rear_axle_to_center_distance_, 1;
    for (size_t i = 0; i != horizon_; ++i) {
        cons.block(6 * horizon_ - 1 + 3 * i, 2 * i, 3, 2) = collision;
    }

    // Set collison avoidance part 2, This part contains the second circle only.
    // The purpose for this is to shrink the drivable corridor and then add a slack variable on it.
    Eigen::Matrix<double, 1, 2> collision1;
    collision1 << config_.d2_ + config_.rear_axle_to_center_distance_, 1;
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
    x0 << vehicle_state_.initial_heading_error_, vehicle_state_.initial_offset_;
    lower_bound->block(0, 0, 2, 1) = -x0;
    upper_bound->block(0, 0, 2, 1) = -x0;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        double ds = seg_s_list[i + 1] - seg_s_list[i];
        double steer = atan(seg_k_list[i] * config_.wheel_base_);
        Eigen::Vector2d c;
        c << ds * steer / config_.wheel_base_ / pow(cos(steer), 2), 0;
        lower_bound->block(2 + 2 * i, 0, 2, 1) = c;
        upper_bound->block(2 + 2 * i, 0, 2, 1) = c;
    }
    // State variables bounds.
    lower_bound->block(2 * horizon_, 0, 2 * horizon_, 1) = Eigen::VectorXd::Constant(2 * horizon_, -OsqpEigen::INFTY);
    upper_bound->block(2 * horizon_, 0, 2 * horizon_, 1) = Eigen::VectorXd::Constant(2 * horizon_, OsqpEigen::INFTY);
    // Add end state bounds.
    if (config_.constraint_end_heading_) {
        double end_psi = constraintAngle(vehicle_state_.end_state_.z - seg_angle_list.back());
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(2 * horizon_ + 2 * horizon_ - 2) = end_psi - 5 * M_PI / 180;
            (*upper_bound)(2 * horizon_ + 2 * horizon_ - 2) = end_psi + 5 * M_PI / 180;
        }
    }
    // Control variables bounds.
    lower_bound->block(4 * horizon_, 0, horizon_ - 1, 1) = Eigen::VectorXd::Constant(horizon_ - 1, -MAX_STEER_ANGLE);
    upper_bound->block(4 * horizon_, 0, horizon_ - 1, 1) = Eigen::VectorXd::Constant(horizon_ - 1, MAX_STEER_ANGLE);
    // Slack variables bounds.
    lower_bound->block(5 * horizon_ - 1, 0, horizon_, 1) = Eigen::VectorXd::Constant(horizon_, 0);
    upper_bound->block(5 * horizon_ - 1, 0, horizon_, 1) =
        Eigen::VectorXd::Constant(horizon_, config_.expected_safety_margin_);
    // Set collision bound part 1.
    for (size_t i = 0; i != horizon_; ++i) {
        Eigen::Vector3d ld, ud;
        ud
            << seg_clearance_list[i][0], seg_clearance_list[i][4], seg_clearance_list[i][6];
        ld
            << seg_clearance_list[i][1], seg_clearance_list[i][5], seg_clearance_list[i][7];
        lower_bound->block(6 * horizon_ - 1 + 3 * i, 0, 3, 1) = ld;
        upper_bound->block(6 * horizon_ - 1 + 3 * i, 0, 3, 1) = ud;
    }
    // Set collision bound part 2.
    upper_bound->block(10 * horizon_ - 1, 0, horizon_, 1) = Eigen::VectorXd::Constant(horizon_, OsqpEigen::INFTY);
    lower_bound->block(9 * horizon_ - 1, 0, horizon_, 1) = Eigen::VectorXd::Constant(horizon_, -OsqpEigen::INFTY);
    for (size_t i = 0; i != horizon_; ++i) {
        double ud = seg_clearance_list[i][2] - config_.expected_safety_margin_;
        double ld = seg_clearance_list[i][3] + config_.expected_safety_margin_;
        (*upper_bound)(9 * horizon_ - 1 + i, 0) = ud;
        (*lower_bound)(10 * horizon_ - 1 + i, 0) = ld;
    }
}

void SolverInterface::setConstraintMatrixWithOffset(double offset,
                                                    double target_angle,
                                                    double angle_error_allowed,
                                                    double offset_error_allowed,
                                                    Eigen::SparseMatrix<double> *matrix_constraints,
                                                    Eigen::VectorXd *lower_bound,
                                                    Eigen::VectorXd *upper_bound) const {
    assert(angle_error_allowed >= 0 && offset_error_allowed >= 0);
    const auto &seg_s_list = reference_path_.seg_s_list_;
    const auto &seg_k_list = reference_path_.seg_k_list_;
    const auto &seg_angle_list = reference_path_.seg_angle_list_;
    const auto &seg_clearance_list = reference_path_.seg_clearance_list_;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(9 * horizon_ - 1, 3 * horizon_ - 1);
    for (size_t i = 0; i != 2 * horizon_; ++i) {
        cons(i, i) = -1;
    }
    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        setDynamicMatrix(i, seg_s_list, seg_k_list, &a, &b);
        cons.block(2 * (i + 1), 2 * i, 2, 2) = a;
        cons.block(2 * (i + 1), 2 * horizon_ + i, 2, 1) = b;
    }
    for (size_t i = 0; i != 3 * horizon_ - 1; ++i) {
        cons(2 * horizon_ + i, i) = 1;
    }
    Eigen::Matrix<double, 4, 2> collision;
    collision << config_.d1_ + config_.rear_axle_to_center_distance_, 1,
        config_.d2_ + config_.rear_axle_to_center_distance_, 1,
        config_.d3_ + config_.rear_axle_to_center_distance_, 1,
        config_.d4_ + config_.rear_axle_to_center_distance_, 1;
    for (size_t i = 0; i != horizon_; ++i) {
        cons.block(5 * horizon_ - 1 + 4 * i, 2 * i, 4, 2) = collision;
    }
    *matrix_constraints = cons.sparseView();

    *lower_bound = Eigen::MatrixXd::Zero(9 * horizon_ - 1, 1);
    *upper_bound = Eigen::MatrixXd::Zero(9 * horizon_ - 1, 1);
    Eigen::Matrix<double, 2, 1> x0;
    x0 << vehicle_state_.initial_heading_error_, vehicle_state_.initial_offset_;
    lower_bound->block(0, 0, 2, 1) = -x0;
    upper_bound->block(0, 0, 2, 1) = -x0;
    for (size_t i = 0; i != horizon_ - 1; ++i) {
        double ds = seg_s_list[i + 1] - seg_s_list[i];
        double steer = atan(seg_k_list[i] * config_.wheel_base_);
        Eigen::Vector2d c;
        c << ds * steer / config_.wheel_base_ / pow(cos(steer), 2), 0;
        lower_bound->block(2 + 2 * i, 0, 2, 1) = c;
        upper_bound->block(2 + 2 * i, 0, 2, 1) = c;
    }
    lower_bound->block(2 * horizon_, 0, 2 * horizon_, 1) = Eigen::VectorXd::Constant(2 * horizon_, -OsqpEigen::INFTY);
    upper_bound->block(2 * horizon_, 0, 2 * horizon_, 1) = Eigen::VectorXd::Constant(2 * horizon_, OsqpEigen::INFTY);
    // Add end state constraint.
    double end_psi = constraintAngle(target_angle - seg_angle_list.back());
    (*lower_bound)(2 * horizon_ + 2 * horizon_ - 2) = end_psi - angle_error_allowed / 2;
    (*upper_bound)(2 * horizon_ + 2 * horizon_ - 2) = end_psi + angle_error_allowed / 2;
    (*lower_bound)(2 * horizon_ + 2 * horizon_ - 1) = offset - offset_error_allowed / 2;
    (*upper_bound)(2 * horizon_ + 2 * horizon_ - 1) = offset + offset_error_allowed / 2;
    lower_bound->block(4 * horizon_, 0, horizon_ - 1, 1) = Eigen::VectorXd::Constant(horizon_ - 1, -30 * M_PI / 180);
    upper_bound->block(4 * horizon_, 0, horizon_ - 1, 1) = Eigen::VectorXd::Constant(horizon_ - 1, 30 * M_PI / 180);
    for (size_t i = 0; i != horizon_; ++i) {
        Eigen::Vector4d ld, ud;
        ud
            << seg_clearance_list[i][0], seg_clearance_list[i][2], seg_clearance_list[i][4], seg_clearance_list[i][6];
        ld
            << seg_clearance_list[i][1], seg_clearance_list[i][3], seg_clearance_list[i][5], seg_clearance_list[i][7];
        lower_bound->block(5 * horizon_ - 1 + 4 * i, 0, 4, 1) = ld;
        upper_bound->block(5 * horizon_ - 1 + 4 * i, 0, 4, 1) = ud;
    }
}

}