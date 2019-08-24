//
// Created by ljn on 19-8-16.
//

#include "../include/MpcPathOptimizer.hpp"
namespace MpcSmoother {

MpcPathOptimizer::MpcPathOptimizer(const std::vector<double> &x_list,
                                   const std::vector<double> &y_list,
                                   const State &start_state,
                                   const State &end_state) :
    x_list(x_list),
    y_list(y_list),
    start_state(start_state),
    end_state(end_state),
    succeed_flag(false),
    large_init_psi_flag(false) {}

bool MpcPathOptimizer::solve() {
    CHECK(x_list.size() == y_list.size()) << "x and y list size not equal!";
    point_num = x_list.size();

    double s = 0;
    s_list.push_back(0);
    for (size_t i = 1; i != point_num; ++i) {
        double ds = sqrt(pow(x_list[i] - x_list[i - 1], 2)
                             + pow(y_list[i] - y_list[i - 1], 2));
        s += ds;
        s_list.push_back(s);
    }
    double max_s = s_list.back();
    std::cout << "ref path length: " << max_s << std::endl;
    x_spline.set_points(s_list, x_list);
    y_spline.set_points(s_list, y_list);

    // make the path dense, the interval being 0.3m
    x_list.clear();
    y_list.clear();
    s_list.clear();
    size_t new_points_count = 0;
    for (double new_s = 0; new_s <= max_s; new_s += 0.3) {
        double x = x_spline(new_s);
        double y = y_spline(new_s);
        x_list.push_back(x);
        y_list.push_back(y);
        s_list.push_back(new_s);
        ++new_points_count;
    }
    point_num = x_list.size();

    // check if there are points whose curvature or curvature change is too large. if such point does exist, the result
    // might be not natural.
    // todo: when large curvature or curvature change is detected, try to generate a shorter path instead of quiting this method.
    double max_curvature_abs;
    double max_curvature_change_abs;
    getCurvature(x_list, y_list, &k_list, &max_curvature_abs, &max_curvature_change_abs);
    if (max_curvature_abs > 0.4) {
        LOG(WARNING) << "the ref path has large curvature, quit mpc optimization!";
        return false;
    }
    if (max_curvature_change_abs > 0.08) {
        LOG(WARNING) << "the ref path has large curvature change, quit mpc optimization!";
        return false;
    }

    k_spline.set_points(s_list, k_list);

    // initial states
    // todo: consider the condition where the initial state is not on the path.
    cte = 0;
    double start_ref_angle = 0;
    // calculate the start angle of the reference path.
    if (x_spline.deriv(1, 0) == 0) {
        start_ref_angle = 0;
    } else {
        start_ref_angle = atan(y_spline.deriv(1, 0) / x_spline.deriv(1, 0));
    }
    if (x_spline.deriv(1, 0) < 0) {
        if (start_ref_angle > 0) {
            start_ref_angle -= M_PI;
        } else if (start_ref_angle < 0) {
            start_ref_angle += M_PI;
        }
    }
    // calculate the difference between the start angle of the reference path ande the angle of start state.
    epsi = start_state.z - start_ref_angle;
    // keep the angle between -π and π.
    if (epsi > M_PI) {
        epsi -= 2 * M_PI;
    } else if (epsi < -M_PI) {
        epsi += 2 * M_PI;
    }
    if (fabs(epsi) > 80 * M_PI / 180) {
        LOG(WARNING) << "initial epsi is larger than 80°, quit mpc path optimization!";
        return false;
    }

    // if the initial psi is large, use smaller step size(sampling time) at early stage.
    if (fabs(epsi) > M_PI / 4) {
        large_init_psi_flag = true;
    }
    //
    double delta_s = 1.3;
    size_t N = max_s / delta_s;
    if (large_init_psi_flag) {
        LOG(INFO) << "large initial psi mode";
        N += 6;
    }
    double length = 0;
    for (size_t i = 0; i != N; ++i) {
        if (large_init_psi_flag && i <= 8) {
            length += delta_s / 4;
        } else {
            length += delta_s;
        }
        seg_list.push_back(length);
    }
    if (max_s - length > delta_s * 0.2) {
        ++N;
        seg_list.push_back(max_s);
    }

    // initial states
    int state_size = 3;
    double curvature = start_state.k;
    double psi = epsi;
    double ps = seg_list[0] / cos(psi);
    double pq = seg_list[0] * tan(psi);
    psi += ps * curvature - seg_list[0] * k_spline(seg_list[0]);
    double end_ref_angle;
    if (x_spline.deriv(1, s_list.back()) == 0) {
        end_ref_angle = 0;
    } else {
        end_ref_angle = atan(y_spline.deriv(1, s_list.back()) / x_spline.deriv(1, s_list.back()));
    }
    if (x_spline.deriv(1, s_list.back()) < 0) {
        if (end_ref_angle > 0) {
            end_ref_angle -= M_PI;
        } else if (end_ref_angle < 0) {
            end_ref_angle += M_PI;
        }
    }
    double end_psi = end_state.z - end_ref_angle;
    if (end_psi > M_PI) {
        end_psi -= 2 * M_PI;
    } else if (end_psi < -M_PI) {
        end_psi += 2 * M_PI;
    }
    if (fabs(end_psi) > M_PI_2) {
        LOG(WARNING) << "end psi is larger than 90°, quit mpc path optimization!";
        return false;
    }

    typedef CPPAD_TESTVECTOR(double) Dvector;
    // n_vars: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = state_size * N + (N - 1);
    // Set the number of constraints
    size_t n_constraints = state_size * N;
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    const size_t ps_range_begin = 0;
    const size_t pq_range_begin = ps_range_begin + N;
    const size_t psi_range_begin = pq_range_begin + N;
    const size_t curvature_range_begin = psi_range_begin + N;
    vars[ps_range_begin] = ps;
    vars[pq_range_begin] = pq;
    vars[psi_range_begin] = psi;
    vars[curvature_range_begin] = curvature;

    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // state variables bounds
    for (size_t i = 0; i < curvature_range_begin; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // the calculated path should have the same heading with the end state.
    vars_lowerbound[psi_range_begin + N - 1] = end_psi;// - end_psi_error;
    vars_upperbound[psi_range_begin + N - 1] = end_psi;// + end_psi_error;
    // bound the lateral error of the end state .
    vars_lowerbound[pq_range_begin + N - 1] = -1.5;
    vars_upperbound[pq_range_begin + N - 1] = 1.5;
    // todo: set bounds for pq to ensure the path won't collide with obstacles!

    // set bounds for control variables
    for (size_t i = curvature_range_begin; i < n_vars; i++) {
        vars_lowerbound[i] = -MAX_CURVATURE;
        vars_upperbound[i] = MAX_CURVATURE;
    }
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0.0;
        constraints_upperbound[i] = 0.0;
    }
    // ... initial state constraints.
    constraints_lowerbound[ps_range_begin] = ps;
    constraints_upperbound[ps_range_begin] = ps;

    constraints_lowerbound[pq_range_begin] = pq;
    constraints_upperbound[pq_range_begin] = pq;

    constraints_lowerbound[psi_range_begin] = psi;
    constraints_upperbound[psi_range_begin] = psi;

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.02\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    // todo: use a config file
    std::vector<double> weights;
    weights.push_back(0.3); //cost_func_cte_weight
    weights.push_back(30); //cost_func_epsi_weight
    weights.push_back(80); //cost_func_curvature_weight
    weights.push_back(1500); //cost_func_curvature_rate_weight
    bool isback = false;

    FgEvalFrenet fg_eval_frenet(k_spline, isback, N, weights, seg_list);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalFrenet>(options, vars,
                                               vars_lowerbound, vars_upperbound,
                                               constraints_lowerbound, constraints_upperbound,
                                               fg_eval_frenet, solution);

    // Check if it works
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(WARNING) << "mpc path optimization solver failed!";
        return false;
    }

    // Cost
//    double cost = solution.obj_value;
//    std::cout << "cost: " << cost << std::endl;

    // output
    for (size_t i = 0; i < N; i++) {
        double tmp[4] = {solution.x[ps_range_begin + i], solution.x[pq_range_begin + i],
                         solution.x[psi_range_begin + i], double(i)};
        std::vector<double> v(tmp, tmp + sizeof tmp / sizeof tmp[0]);
        this->predicted_path_in_frenet.push_back(v);
    }


    // todo: consider the condition where the start state is not on the path
    predicted_path_x.push_back(start_state.x);
    predicted_path_y.push_back(start_state.y);
    for (size_t i = 0; i != seg_list.size(); ++i) {
        double length_on_ref_path = seg_list[i];
        double angle;
        if (x_spline.deriv(1, length_on_ref_path) == 0) {
            angle = 0;
        } else {
            angle = atan(y_spline.deriv(1, length_on_ref_path) / x_spline.deriv(1, length_on_ref_path));
        }
        if (x_spline.deriv(1, length_on_ref_path) < 0) {
            if (angle > 0) {
                angle -= M_PI;
            } else if (angle < 0) {
                angle += M_PI;
            }
        }
        double new_angle = angle + M_PI_2;
        double x = x_spline(length_on_ref_path) + predicted_path_in_frenet[i][1] * cos(new_angle);
        double y = y_spline(length_on_ref_path) + predicted_path_in_frenet[i][1] * sin(new_angle);
        if (std::isnan(x) || std::isnan(y)) {
            LOG(WARNING) << "output is not a number, mpc path opitmization failed!" << std::endl;
            return false;
        }
//        std::cout << "i: " << i << ", d: " << predicted_path_in_frenet[i][1] << std::endl;
        predicted_path_x.push_back(x);
        predicted_path_y.push_back(y);
    }

    return true;
}

std::vector<double> &MpcPathOptimizer::getXList() {
    return this->predicted_path_x;
}

std::vector<double> &MpcPathOptimizer::getYList() {
    return this->predicted_path_y;
}

double MpcPathOptimizer::getPointCurvature(const double &x1,
                                           const double &y1,
                                           const double &x2,
                                           const double &y2,
                                           const double &x3,
                                           const double &y3) {
    double_t a, b, c;
    double_t delta_x, delta_y;
    double_t s;
    double_t A;
    double_t curv;
    double_t rotate_direction;

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    a = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

    delta_x = x3 - x2;
    delta_y = y3 - y2;
    b = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

    delta_x = x1 - x3;
    delta_y = y1 - y3;
    c = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

    s = (a + b + c) / 2.0;
    A = sqrt(fabs(s * (s - a) * (s - b) * (s - c)));
    curv = 4 * A / (a * b * c);

    /* determine the sign, using cross product(叉乘)
     * 2维空间中的叉乘是： A x B = |A||B|Sin(\theta)
     * V1(x1, y1) X V2(x2, y2) = x1y2 – y1x2
     */
    rotate_direction = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    if (rotate_direction < 0) {
        curv = -curv;
    }
    return curv;
}

void MpcPathOptimizer::getCurvature(const std::vector<double> &local_x,
                                    const std::vector<double> &local_y,
                                    std::vector<double> *pt_curvature_out,
                                    double *max_curvature_abs,
                                    double *max_curvature_change_abs) {
    assert(local_x.size() == local_y.size());
    unsigned long size_n = local_x.size();
    std::vector<double> curvature = std::vector<double>(size_n);
    for (size_t i = 1; i < size_n - 1; ++i) {
        double x1 = local_x.at(i - 1);
        double x2 = local_x.at(i);
        double x3 = local_x.at(i + 1);
        double y1 = local_y.at(i - 1);
        double y2 = local_y.at(i);
        double y3 = local_y.at(i + 1);
        curvature.at(i) = getPointCurvature(x1, y1, x2, y2, x3, y3);
    }
    curvature.at(0) = curvature.at(1);
    curvature.at(size_n - 1) = curvature.at(size_n - 2);
    double final_curvature;
    double max_curvature = 0;
    double max_curvature_change = 0;
    for (size_t j = 0; j < size_n; ++j) {
        if (j == 0 || j == size_n - 1)
            final_curvature = curvature[j];
        else
            final_curvature = (curvature[j - 1] + curvature[j] + curvature[j + 1]) / 3;
        pt_curvature_out->push_back(final_curvature);
        if (fabs(final_curvature) > max_curvature) {
            max_curvature = fabs(final_curvature);
        }
        if (j != size_n - 1) {
            double curvature_change = fabs(curvature[j] - curvature[j + 1]);
            if (curvature_change > max_curvature_change) {
                max_curvature_change = curvature_change;
            }
        }
    }
    *max_curvature_abs = max_curvature;
    *max_curvature_change_abs = max_curvature_change;
}

}
