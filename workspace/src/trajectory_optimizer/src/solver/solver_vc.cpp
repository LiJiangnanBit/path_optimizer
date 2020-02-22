//
// Created by ljn on 20-2-22.
//

#include "trajectory_optimizer/solver/solver_vc.hpp"

namespace TrajOptNS {

SolverVC::SolverVC() :
Solver()
{}

void SolverVC::init(const TrajOptNS::SolverInput &input) {
    horizon_ = input.horizon;
    setHessianAndGradient(input);


}

bool SolverVC::solve() {


}

void SolverVC::setHessianAndGradient(const SolverInput &input) {
    double w_c{100}; // TODO: use config.
    double w_cr{200};
    double w_j{10};
    double w_e{1};
    double w_s{10};
    double w_f(1);

    // Calculate hessian matrix.
    const int state_size{3 * horizon_};
    const int control_size{2 * (horizon_ - 1)};
    const int slack_size{horizon_};
    const auto hessian_size = state_size + control_size + slack_size;
    Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(hessian_size, hessian_size, 0)};
    // Curvature part.
    Eigen::MatrixXd curvature_part{Eigen::MatrixXd::Constant(horizon_ - 1, horizon_ - 1, 0)};
    for (int i = 0; i != horizon_ - 1; ++i) {
        if (i == 0 || i == horizon_ - 2) {
            curvature_part(i, i) = w_cr + w_c;
        } else {
            curvature_part(i, i) = 2 * w_cr;
        }
        if (i != horizon_ - 2)
            curvature_part(i, i + 1) = -w_cr;
        if (i != 0)
            curvature_part(i, i - 1) = -w_cr;
    }
    // Velocity part.
    Eigen::MatrixXd velocity_part{Eigen::MatrixXd::Constant(horizon_ - 1, horizon_ - 1, 0)};
    Eigen::Matrix3d block;
    block <<
        1, -2, 1,
        -2, 4, -2,
        1, -2, 1;
    block *= w_j;
    for (int i = 0; i != horizon_ - 3; ++i) {
        velocity_part.block(i, i, 3, 3) += block;
    }
    // Fill in hessian;
    hessian.block(state_size, state_size, horizon_ - 1, horizon_ - 1) = curvature_part;
    hessian.block(state_size + horizon_ - 1, state_size + horizon_ - 1, horizon_ - 1, horizon_ - 1) = velocity_part;
    // Error and slack variable part.
    Eigen::MatrixXd error_part{Eigen::MatrixXd::Identity(state_size + control_size, state_size + control_size)};
    error_part *= w_e;
    Eigen::MatrixXd slack_part{Eigen::MatrixXd::Identity(slack_size, slack_size)};
    slack_part *= w_s;
    hessian.block(0, 0, error_part.rows(), error_part.cols()) += error_part;
    hessian.block(error_part.rows(), error_part.cols(), slack_part.rows(), slack_part.cols()) += slack_part;

    // Calculate gradient.
    Eigen::VectorXd ref_state;
    ref_state.resize(hessian_size);
    for (int i = 0; i != horizon_; ++i) {
        ref_state(i) = input.reference_states[i].ey;
        ref_state(i + horizon_) = input.reference_states[i].ephi;
        ref_state(i + 2 * horizon_) = input.reference_states[i].s;
        if (i != horizon_ - 1) {
            ref_state(i + 3 * horizon_) = input.reference_states[i].k;
            ref_state(i + 4 * horizon_ - 1) = input.reference_states[i].v;
        }
    }
    gradient_ = -2 * w_e * ref_state;
}

void SolverVC::setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_cons) {

}

}