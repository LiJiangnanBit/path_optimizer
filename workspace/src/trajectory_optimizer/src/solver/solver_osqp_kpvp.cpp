#include "trajectory_optimizer/solver/solver_osqp_kpvp.hpp"

namespace TrajOptNS {

void SolverOsqpKpvp::init(const std::shared_ptr<SolverInput> &input) {
    solver_input_ptr_ = input;
    state_horizon_ = input->state_horizon;
    control_horizon_ = input->contorl_horizon;
    state_size_ = 5;
    control_size_ = 2;
    state_vars_num_ = 5 * state_horizon_;
    control_vars_num_ = 2 * control_horizon_;
    vars_num_ = state_vars_num_ + control_vars_num_;
    cons_num_ = 12 * state_horizon_ + 2 * control_horizon_;
    setHessianAndGradient();
    setConstraintMatrix();
}

bool SolverOsqpKpvp::solve(std::vector<State> *result_trajectory) {

}

void SolverOsqpKpvp::setHessianAndGradient() {
    const double weight_ey{TrajOptConfig::weight_ey};
    const double weight_v{TrajOptConfig::weight_v};
    const double weight_k{TrajOptConfig::weight_k};
    const double weight_kp{TrajOptConfig::weight_kp};
    const double weight_vp{TrajOptConfig::weight_vp};
    const double weight_vpp{TrajOptConfig::weight_vpp};
    Eigen::MatrixXd hessian{Eigen::MatrixXd::Constant(vars_num_, vars_num_, 0)};
    // Hessian.
    // State part.
    for (size_t i = 0; i != state_horizon_; ++i) {
        hessian(5 * i, 5 * i) += weight_ey;
        hessian(5 * i + 3, 5 * i + 3) += weight_k;
        hessian(5 * i + 4, 5 * i + 4) += weight_v;
    }
    // Control part.
    Eigen::Matrix<double, 4, 1> vec{0, -1, 0, 1};
    auto vpp_part{vec * vec.transpose() * weight_vpp};
    for (size_t i = 0; i != control_horizon_; ++i) {
        // * keep_control_steps: make it the same as the ipopt version.
        hessian(state_vars_num_ + 2 * i, state_vars_num_ + 2 * i) +=
            weight_kp * TrajOptConfig::keep_control_steps_;
        hessian(state_vars_num_ + 2 * i + 1, state_vars_num_ + 2 * i + 1) +=
            weight_vp * TrajOptConfig::keep_control_steps_;
        if (i != control_horizon_ - 1) {
            hessian.block(state_vars_num_ + 2 * i, state_vars_num_ + 2 * i, 4, 4) += vpp_part;
        }
    }
    hessian_ = hessian.sparseView();

    // Gradient.
    gradient_ = Eigen::VectorXd::Constant(vars_num_, 0);
    for (size_t i = 0; i != state_horizon_; ++i) {
        gradient_(5 * i) = -weight_v * 2 * solver_input_ptr_->reference_trajectory->state_list[i].v;
    }
}

void SolverOsqpKpvp::setConstraintMatrix() {
    // Constraint matrix.
    const size_t trans_range_begin{0};
    const size_t vars_range_begin{trans_range_begin + 5 * state_horizon_};
    const size_t collision_range_begin{vars_range_begin + 3 * state_horizon_};
    const size_t acc_lower_range_begin{collision_range_begin + state_horizon_};
    const size_t acc_upper_range_begin{acc_lower_range_begin + control_horizon_};
    Eigen::MatrixXd cons_matrix{Eigen::MatrixXd::Constant(cons_num_, vars_num_, 0)};
    // Transition part.
    std::vector<Eigen::Vector3d> c_list; // Needed in bounds.
    Eigen::MatrixXd a{Eigen::MatrixXd::Constant(5, 5, 0)};
    a(0, 1) = 1;
    a(1, 3) = 1;
    Eigen::MatrixXd b{Eigen::MatrixXd::Constant(5, 2, 0)};
    b(3, 0) = 1;
    b(4, 1) = 1;
    size_t control_index{0};
    for (size_t i = 0; i != state_horizon_ - 1; ++i) {
        const auto &ref_state{solver_input_ptr_->reference_trajectory->state_list[i]};
        a(1, 0) = -pow(ref_state.k, 2);
        a(2, 3) = 2 * ref_state.v * ref_state.vp;
        a(2, 4) = 2 * (ref_state.k * ref_state.vp, +ref_state.kp * ref_state.v);
        b(2, 0) = pow(ref_state.v, 2);
        b(2, 1) = 2 * ref_state.v * ref_state.k;
        control_index = i / TrajOptConfig::keep_control_steps_;
        std::cout << "control index: " << control_index << ", control horizon: " << control_horizon_ << std::endl;
        auto A{a * TrajOptConfig::spacing_ + Eigen::MatrixXd::Identity(5, 5)};
        auto B{b * TrajOptConfig::spacing_};
        cons_matrix.block(5 * (i + 1), 5 * i, 5, 5) += A;
        cons_matrix.block(5 * (i + 1), 5 * (i + 1), 5, 5) += -Eigen::MatrixXd::Identity(5, 5);
        cons_matrix.block(5 * (i + 1), state_vars_num_ + 2 * control_index, 5, 2) = B;
        // Calculate c.
        Eigen::Matrix<double, 5, 1> c;
        c << 0, 0,
            2 * ref_state.v * ref_state.k * ref_state.vp+ pow(ref_state.v, 2) * ref_state.kp,
            ref_state.kp,
            ref_state.vp;
        Eigen::Matrix<double, 5, 1> ref_state_vector;
        ref_state_vector << ref_state.ey, ref_state.ephi, ref_state.ay, ref_state.k, ref_state.v;
        Eigen::Matrix<double, 2, 1> ref_control_vector;
        ref_control_vector << ref_state.kp, ref_state.vp;
        c_list.emplace_back(TrajOptConfig::spacing_ * c - a * ref_state_vector - b * ref_control_vector);
    }
    cons_matrix.block(0, 0, 5, 5) += -Eigen::MatrixXd::Identity(5, 5);
    // vars part.
    Eigen::MatrixXd vars_part{Eigen::MatrixXd::Constant(3, 5, 0)};
    vars_part.block(0, 2, 3, 3) = Eigen::Matrix3d::Identity();
    for (size_t i = 0; i != state_horizon_; ++i) {
        cons_matrix.block(vars_range_begin + 3 * i, 5 * i, 3, 5) += vars_part;
    }
    // Collision part.
    Eigen::Matrix<double, 4, 2> collision_part;
    collision_part <<
        1, TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d1_,
        1, TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d2_,
        1, TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d3_,
        1, TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d4_;
    for (size_t i = 0; i != state_horizon_; ++i) {
        cons_matrix.block(collision_range_begin + 4 * i, 5 * i, 4, 2) += collision_part;
    }
    // Acc part.
    for (size_t i = 0; i != control_horizon_; ++i) {
        const auto &ref_state{solver_input_ptr_->reference_trajectory->state_list[i * TrajOptConfig::keep_control_steps_]};
        Eigen::Matrix<double, 1, 5> acc_lower, acc_upper;
        acc_lower << ref_state.k * ref_state.v * TrajOptConfig::max_lon_dacc_, 0, 0, 0, TrajOptConfig::max_lon_dacc_;
        acc_upper << ref_state.k * ref_state.v * TrajOptConfig::max_lon_acc_, 0, 0, 0, TrajOptConfig::max_lon_acc_;
        cons_matrix.block(acc_lower_range_begin + i, 5 * i * TrajOptConfig::keep_control_steps_, 1, 5) += acc_lower;
        cons_matrix.block(acc_upper_range_begin + i, 5 * i * TrajOptConfig::keep_control_steps_, 1, 5) += acc_upper;
        cons_matrix(acc_lower_range_begin + i, state_vars_num_ + 2 * i + 1) += pow(ref_state.vp, 2);
    }


}

}