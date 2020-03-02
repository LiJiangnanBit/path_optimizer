//
// Created by ljn on 20-2-27.
//
#include <cfloat>
#include <path_optimizer/tools/tools.hpp>
#include "trajectory_optimizer/solver/solver_ipopt_kpvp.hpp"

using PathOptimizationNS::constraintAngle;
using PathOptimizationNS::getHeading;
using PathOptimizationNS::getCurvature;
using PathOptimizationNS::distance;

namespace TrajOptNS {

void FgEvalKpvp::init(const std::shared_ptr<SolverInput> &input) {
    state_horizon_ = input->state_horizon;
    control_horizon_ = input->contorl_horizon;
    input_ = input;
}

void FgEvalKpvp::operator()(TrajOptNS::FgEvalKpvp::ADvector &fg, const TrajOptNS::FgEvalKpvp::ADvector &vars) {
    const size_t ey_range_begin{0};
    const size_t ephi_range_begin{ey_range_begin + state_horizon_};
    const size_t ay_range_begin{ephi_range_begin + state_horizon_};
    const size_t k_range_begin{ay_range_begin + state_horizon_};
    const size_t v_range_begin{k_range_begin + state_horizon_};
    const size_t kp_range_begin{v_range_begin + state_horizon_};
    const size_t vp_range_begin{kp_range_begin + control_horizon_};
    const size_t cons_ey_range_begin{1};
    const size_t cons_ephi_range_begin{cons_ey_range_begin + state_horizon_};
    const size_t cons_ay_range_begin{cons_ephi_range_begin + state_horizon_};
    const size_t cons_k_range_begin{cons_ay_range_begin + state_horizon_};
    const size_t cons_v_range_begin{cons_k_range_begin + state_horizon_};
    const size_t cons_c0_range_begin{cons_v_range_begin + state_horizon_};
    const size_t cons_c1_range_begin{cons_c0_range_begin + state_horizon_};
    const size_t cons_c2_range_begin{cons_c1_range_begin + state_horizon_};
    const size_t cons_c3_range_begin{cons_c2_range_begin + state_horizon_};
    const size_t cons_acc_lower_range_begin{cons_c3_range_begin + state_horizon_};
    const size_t cons_acc_upper_range_begin{cons_acc_lower_range_begin + control_horizon_};
    const ADd spacing{TrajOptConfig::spacing_};
    const double weight_ey{TrajOptConfig::weight_ey};
    const double weight_v{TrajOptConfig::weight_v};
    const double weight_k{TrajOptConfig::weight_k};
    const double weight_kp{TrajOptConfig::weight_kp};
    const double weight_vp{TrajOptConfig::weight_vp};
    const double weight_vpp{TrajOptConfig::weight_vpp};

    fg[0] = 0;
    size_t control_variable_index{0};
    for (size_t i = 0; i != state_horizon_; ++i) {
        ADd ey{vars[ey_range_begin + i]};
        ADd ephi{vars[ephi_range_begin + i]};
        ADd ay{vars[ay_range_begin + i]};
        ADd k{vars[k_range_begin + i]};
        ADd v{vars[v_range_begin + i]};
        ADd ref_v{input_->reference_trajectory->state_list[i].v};
        ADd ref_k{input_->reference_trajectory->state_list[i].k};
        // Cost function.
        fg[0] += weight_ey * pow(ey, 2);
        fg[0] += weight_v * pow(v - ref_v, 2);
        fg[0] += weight_k * pow(k, 2);
        if (i % TrajOptConfig::keep_control_steps_ == 0 && i != state_horizon_ - 1) {
            control_variable_index = i / TrajOptConfig::keep_control_steps_;
            if (control_variable_index != 0) {
                ADd vp{vars[vp_range_begin + control_variable_index]};
                ADd last_vp{vars[vp_range_begin + control_variable_index - 1]};
                fg[0] += weight_vpp * pow(vp - last_vp, 2);
            }
        }
        if (i != state_horizon_ - 1) {
            ADd kp{vars[kp_range_begin + control_variable_index]};
            ADd vp{vars[vp_range_begin + control_variable_index]};
            fg[0] += weight_kp * pow(kp, 2);
            fg[0] += weight_vp * pow(vp, 2);
        }

        // Constraints.
        // Transition constraints.
        if (i == 0) {
            fg[cons_ey_range_begin] = ey;
            fg[cons_ephi_range_begin] = ephi;
            fg[cons_ay_range_begin] = ay;
            fg[cons_k_range_begin] = k;
            fg[cons_v_range_begin] = v;
        } else {
            ADd last_kp{vars[kp_range_begin + (i - 1) / TrajOptConfig::keep_control_steps_]};
            ADd last_vp{vars[vp_range_begin + (i - 1) / TrajOptConfig::keep_control_steps_]};
            ADd last_ey{vars[ey_range_begin + i - 1]};
            ADd last_ephi{vars[ephi_range_begin + i - 1]};
            ADd last_ay{vars[ay_range_begin + i - 1]};
            ADd last_k{vars[k_range_begin + i - 1]};
            ADd last_v{vars[v_range_begin + i - 1]};
            ADd last_ref_k{input_->reference_trajectory->state_list[i - 1].k};
            ADd last_ref_v{input_->reference_trajectory->state_list[i - 1].v};
            ADd last_ref_kp{input_->reference_trajectory->state_list[i - 1].kp};
            ADd last_ref_vp{input_->reference_trajectory->state_list[i - 1].vp};
            ADd current_ey{last_ey + last_ephi * spacing};
            fg[cons_ey_range_begin + i] = ey - current_ey;
            ADd current_ephi{last_ephi + spacing * (-pow(last_ref_k, 2) * last_ey + last_k - last_ref_k)};
            fg[cons_ephi_range_begin + i] = ephi - current_ephi;
            ADd current_ay{(
                               2 * last_ref_v * last_ref_vp * (last_k - last_ref_k) +
                                   (2 * last_ref_k * last_ref_vp + 2 * last_ref_kp * last_ref_v) * (last_v - last_ref_v)
                                   +
                                       pow(last_ref_v, 2) * (last_kp - last_ref_kp) +
                                   2 * last_ref_v * last_ref_k * (last_vp - last_ref_vp) +
                                   2 * last_ref_v * last_ref_k * last_ref_vp +
                                   pow(last_ref_v, 2) * last_ref_kp
                           ) * spacing + last_ay};
            fg[cons_ay_range_begin + i] = ay - current_ay;
            fg[cons_k_range_begin + i] = k - (last_kp * spacing + last_k);
            fg[cons_v_range_begin + i] = v - (last_vp * spacing + last_v);
        }
        // Collision constraints.
        fg[cons_c0_range_begin + i] = ey + (TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d1_) * ephi;
        fg[cons_c1_range_begin + i] = ey + (TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d2_) * ephi;
        fg[cons_c2_range_begin + i] = ey + (TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d3_) * ephi;
        fg[cons_c3_range_begin + i] = ey + (TrajOptConfig::rear_axle_to_center_distance_ + TrajOptConfig::d4_) * ephi;
        // Acceleration constraints.
        if (i % TrajOptConfig::keep_control_steps_ == 0 && i != state_horizon_ - 1) {
            ADd vp{vars[vp_range_begin + control_variable_index]};
            ADd adjusted_ref_v{std::max<ADd>(ref_v, 0.1)};
            ADd m{(2 - v / adjusted_ref_v - ref_k * ey) / adjusted_ref_v};
            fg[cons_acc_lower_range_begin + control_variable_index] = vp - TrajOptConfig::max_lon_dacc_ * m;
            fg[cons_acc_upper_range_begin + control_variable_index] = vp - TrajOptConfig::max_lon_acc_ * m;
        }
    }
}

void SolverIpoptKpvp::init(const std::shared_ptr<SolverInput> &input) {
    state_horizon_ = input->state_horizon;
    control_horizon_ = input->contorl_horizon;
    LOG(INFO) << "[SolverIpoptKpvp] state_horizon: " << state_horizon_;
    solver_input_ptr_ = input;
    vars_.clear();
    vars_lowerbound_.clear();
    vars_upperbound_.clear();
    cons_lowerbound_.clear();
    cons_upperbound_.clear();
    const size_t state_size{5}, control_size{2};
    const size_t state_vars_num{state_size * state_horizon_},
        control_vars_num{control_size * (control_horizon_)};
    const auto vars_num{state_vars_num + control_vars_num};
    const size_t ey_range_begin{0};
    const size_t ephi_range_begin{ey_range_begin + state_horizon_};
    const size_t ay_range_begin{ephi_range_begin + state_horizon_};
    const size_t k_range_begin{ay_range_begin + state_horizon_};
    const size_t v_range_begin{k_range_begin + state_horizon_};
    const size_t kp_range_begin{v_range_begin + state_horizon_};
    const size_t vp_range_begin{kp_range_begin + control_horizon_};
    // Initial values of variables.
    vars_.resize(vars_num);
    for (size_t i = 0; i != state_horizon_; ++i) {
        const auto &ref_state{input->reference_trajectory->state_list[i]};
        vars_[ey_range_begin + i] = ref_state.ey;
        vars_[ephi_range_begin + i] = ref_state.ephi;
        vars_[ay_range_begin + i] = ref_state.ay;
        vars_[k_range_begin + i] = ref_state.k;
        vars_[v_range_begin + i] = ref_state.v;
    }
    for (size_t i = 0; i != control_horizon_; ++i) {
        const auto &ref_state{input->reference_trajectory->state_list[i * TrajOptConfig::keep_control_steps_]};
        vars_[kp_range_begin + i] = ref_state.kp;
        vars_[vp_range_begin + i] = ref_state.vp;
    }
    // Bounds for variables.
    vars_lowerbound_.resize(vars_num);
    vars_upperbound_.resize(vars_num);
    for (size_t i = ey_range_begin; i != ay_range_begin; ++i) {
        vars_lowerbound_[i] = -DBL_MAX;
        vars_upperbound_[i] = DBL_MAX;
    }
    for (size_t i = ay_range_begin; i != k_range_begin; ++i) {
        vars_lowerbound_[i] = -TrajOptConfig::max_lat_acc_;
        vars_upperbound_[i] = TrajOptConfig::max_lat_acc_;
    }
    for (size_t i = k_range_begin; i != v_range_begin; ++i) {
        vars_lowerbound_[i] = -tan(TrajOptConfig::max_steer_angle_) / TrajOptConfig::wheel_base_;
        vars_upperbound_[i] = tan(TrajOptConfig::max_steer_angle_) / TrajOptConfig::wheel_base_;
    }
    for (size_t i = v_range_begin; i != kp_range_begin; ++i) {
        vars_lowerbound_[i] = 0;
        vars_upperbound_[i] = TrajOptConfig::max_v_;
    }
    for (size_t i = kp_range_begin; i != vars_num; ++i) {
        vars_lowerbound_[i] = -DBL_MAX;
        vars_upperbound_[i] = DBL_MAX;
    }

    // Constrants.
    const size_t transition_cons_num{state_size * state_horizon_};
    const size_t collision_cons_num{4 * state_horizon_};
    const size_t acc_cons_num{2 * (control_horizon_)};
    const size_t cons_size{transition_cons_num + collision_cons_num + acc_cons_num};
    cons_lowerbound_.resize(cons_size);
    cons_upperbound_.resize(cons_size);
    const size_t cons_ey_range_begin{0};
    const size_t cons_ephi_range_begin{cons_ey_range_begin + state_horizon_};
    const size_t cons_ay_range_begin{cons_ephi_range_begin + state_horizon_};
    const size_t cons_k_range_begin{cons_ay_range_begin + state_horizon_};
    const size_t cons_v_range_begin{cons_k_range_begin + state_horizon_};
    const size_t cons_c0_range_begin{cons_v_range_begin + state_horizon_};
    const size_t cons_c1_range_begin{cons_c0_range_begin + state_horizon_};
    const size_t cons_c2_range_begin{cons_c1_range_begin + state_horizon_};
    const size_t cons_c3_range_begin{cons_c2_range_begin + state_horizon_};
    const size_t cons_acc_lower_range_begin{cons_c3_range_begin + state_horizon_};
    const size_t cons_acc_upper_range_begin{cons_acc_lower_range_begin + control_horizon_};
    // State transition constraints.
    for (size_t i = cons_ey_range_begin; i != cons_c0_range_begin; ++i) {
        cons_lowerbound_[i] = cons_upperbound_[i] = 0;
    }
    // Set initial state.
    double initial_ephi{
        constraintAngle(
            input->start_state.z - getHeading(input->reference_trajectory->x_s, input->reference_trajectory->y_s, 0))
    };
    std::cout << "initial ephi: " << initial_ephi << std::endl;
    cons_lowerbound_[cons_ephi_range_begin] = cons_upperbound_[cons_ephi_range_begin] = initial_ephi;
    cons_lowerbound_[cons_k_range_begin] = cons_upperbound_[cons_k_range_begin] = input->start_state.k;
    cons_lowerbound_[cons_v_range_begin] = cons_upperbound_[cons_v_range_begin] = input->start_state.v;
    cons_lowerbound_[cons_ay_range_begin] = cons_upperbound_[cons_ay_range_begin] =
        pow(input->start_state.v, 2) * input->start_state.k;
    // Collision constraints.
    for (size_t i = 0; i != state_horizon_; ++i) {
        cons_lowerbound_[cons_c0_range_begin + i] = input->bounds[i].c0.lb;
        cons_upperbound_[cons_c0_range_begin + i] = input->bounds[i].c0.ub;
        cons_lowerbound_[cons_c1_range_begin + i] = input->bounds[i].c1.lb;
        cons_upperbound_[cons_c1_range_begin + i] = input->bounds[i].c1.ub;
        cons_lowerbound_[cons_c2_range_begin + i] = input->bounds[i].c2.lb;
        cons_upperbound_[cons_c2_range_begin + i] = input->bounds[i].c2.ub;
        cons_lowerbound_[cons_c3_range_begin + i] = input->bounds[i].c3.lb;
        cons_upperbound_[cons_c3_range_begin + i] = input->bounds[i].c3.ub;
    }
    // Acceleration constraints.
    for (size_t i = 0; i != control_horizon_; ++i) {
        cons_lowerbound_[cons_acc_lower_range_begin + i] = 0;
        cons_upperbound_[cons_acc_lower_range_begin + i] = DBL_MAX;
        cons_lowerbound_[cons_acc_upper_range_begin + i] = -DBL_MAX;
        cons_upperbound_[cons_acc_upper_range_begin + i] = 0;
    }

    fg_eval_kpvp_.init(input);
    LOG(INFO) << "[SolverIpoptKpvp] init OK!";
}

bool SolverIpoptKpvp::solve(std::vector<State> *result_trajectory) {
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
//    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";
    options += "String  jac_c_constant      yes\n";
    options += "String  jac_d_constant      yes\n";
    options += "String  hessian_constant        yes\n";
    auto t1{std::clock()};
    CppAD::ipopt::solve<Dvector, FgEvalKpvp>(options,
                                             vars_,
                                             vars_lowerbound_,
                                             vars_upperbound_,
                                             cons_lowerbound_,
                                             cons_upperbound_,
                                             fg_eval_kpvp_,
                                             solution_);
    auto t2{std::clock()};
    std::cout << "[SolverIpoptKvpv] solving time: " << PathOptimizationNS::time_s(t1, t2) << std::endl;
    bool ok = true;
    ok &= solution_.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(WARNING) << "[SolverIpoptKpvp] optimization failed!";
        return false;
    }
    LOG(INFO) << "[SolverIpoptKpvp] optimization OK!";
    const size_t ey_range_begin{0};
    const size_t ephi_range_begin{ey_range_begin + state_horizon_};
    const size_t ay_range_begin{ephi_range_begin + state_horizon_};
    const size_t k_range_begin{ay_range_begin + state_horizon_};
    const size_t v_range_begin{k_range_begin + state_horizon_};
    const size_t kp_range_begin{v_range_begin + state_horizon_};
    const size_t vp_range_begin{kp_range_begin + control_horizon_};
    const auto &ref_x_s{solver_input_ptr_->reference_trajectory->x_s};
    const auto &ref_y_s{solver_input_ptr_->reference_trajectory->y_s};
    result_trajectory->clear();
    double s{0};
    for (size_t i = 0; i != state_horizon_; ++i) {
//        std::cout << "ay: " << solution_.x[ay_range_begin + i] << " actual ay: "
//                  << pow(solution_.x[v_range_begin + i], 2) * solution_.x[k_range_begin + i] << std::endl;
        double tmp_s{i * TrajOptConfig::spacing_};
        State ref_state{ref_x_s(tmp_s),
                        ref_y_s(tmp_s),
                        getHeading(ref_x_s, ref_y_s, tmp_s)};
        double offset_angle{ref_state.z + M_PI_2};
        double x{ref_state.x + solution_.x[ey_range_begin + i] * cos(offset_angle)};
        double y{ref_state.y + solution_.x[ey_range_begin + i] * sin(offset_angle)};
        double z{constraintAngle(ref_state.z + solution_.x[ephi_range_begin + i])};
        if (i != 0) {
            s += sqrt(pow(x - result_trajectory->back().x, 2) + pow(y - result_trajectory->back().y, 2));
        }
        result_trajectory->emplace_back(x, y, z, solution_.x[k_range_begin + i], s, solution_.x[v_range_begin + i]);
    }
    return true;
}
}
