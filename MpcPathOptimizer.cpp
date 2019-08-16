//
// Created by ljn on 19-8-16.
//

#include "MpcPathOptimizer.hpp"
namespace MpcSmoother {

MpcPathOptimizer::MpcPathOptimizer(const std::vector<double> &x_list,
                                   const std::vector<double> &y_list,
                                   const State &start_state,
                                   const State &end_state) :
    x_list(x_list),
    y_list(y_list),
    start_state(start_state),
    end_state(end_state) {}

void MpcPathOptimizer::reset(const std::vector<double> &x,
                             const std::vector<double> &y,
                             const State &init_state,
                             const State &goal_state) {
    this->x_list = x;
    this->y_list = y;
    this->start_state = init_state;
    this->end_state = goal_state;
    // todo: clear other elements!
}
void MpcPathOptimizer::getCurvature(const std::vector<double> &local_x,
                                    const std::vector<double> &local_y,
                                    std::vector<double> *pt_curvature_out) {
    assert(local_x.size() == local_y.size());
    unsigned long size_n = local_x.size();
    std::vector<double> curvature = std::vector<double>(size_n);
    for (int i = 1; i < size_n - 1; ++i) {
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
    for (int j = 0; j < size_n; ++j) {
        if (j == 0 || j == size_n - 1)
            pt_curvature_out->push_back(curvature[j]);
        else
            pt_curvature_out->push_back((curvature[j - 1] + curvature[j] + curvature[j + 1]) / 3);
    }
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

void MpcPathOptimizer::solve() {
    CHECK(x_list.size() == y_list.size()) << "x and y list size not equal!";
    point_num = x_list.size();
    // todo: consider the condition where the initial state is not on the path.
    double s = 0;
    s_list.push_back(0);
    for (size_t i = 1; i != point_num; ++i) {
        double ds = sqrt(pow(x_list[i] - x_list[i - 1], 2)
                             + pow(y_list[i] - y_list[i - 1], 2));
        s += ds;
        s_list.push_back(s);
    }
    x_spline.set_points(s_list, x_list);
    y_spline.set_points(s_list, y_list);
    getCurvature(x_list, y_list, &k_list);
    k_spline.set_points(s_list, k_list);
    // initial states
    cte = 0;
    epsi = start_state.z - atan(y_spline.deriv(1, 0) / x_spline.deriv(1, 0));
    double curvature = start_state.k;
    // todo: delta_s should be changeable.
    double delta_s = 3;

    double psi = epsi;
    double ps = delta_s * cos(psi);
    double pq = delta_s * sin(psi);
    psi += delta_s * curvature - delta_s * cos(psi) * k_spline(ps);

    typedef CPPAD_TESTVECTOR(double) Dvector;
    // todo: these variables should be changeable.
    size_t N = 12;
    int state_size = 3;
    // n_vars: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = state_size * N + (N - 1);
    // Set the number of constraints
    size_t n_constraints = state_size * N;
    Dvector vars(n_vars);
    for (unsigned int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    const size_t ps_range_begin = 0;
    const size_t pq_range_begin = ps_range_begin + N;
    const size_t psi_range_begin = pq_range_begin + N;
    const size_t curvature_range_begin = psi_range_begin + N;
    vars[ps_range_begin] = ps;
    vars[pq_range_begin] = pq;
    vars[psi_range_begin] = psi;

    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // no bounds for state variables
    for (int i = 0; i < curvature_range_begin; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    // set bounds for control variables
    for (int i = curvature_range_begin; i < n_vars; i++) {
        vars_lowerbound[i] = -0.25;
        vars_upperbound[i] = 0.25;
    }
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (unsigned int i = 0; i < n_constraints; i++) {
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
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    std::vector<double> weights(4);
    weights.push_back(1);
    weights.push_back(11);
    weights.push_back(1);
    weights.push_back(1500);
    bool isback = false;

    FgEvalFrenet fg_eval_frenet(k_spline, isback, N, weights, delta_s);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalFrenet>(options, vars,
                                               vars_lowerbound, vars_upperbound,
                                               constraints_lowerbound, constraints_upperbound,
                                               fg_eval_frenet, solution);

    for (int i = 0; i < N; i++) {
        double tmp[3] = {solution.x[ps_range_begin + i], solution.x[pq_range_begin + i], double(i)};
        std::vector<double> v(tmp, tmp + sizeof tmp / sizeof tmp[0]);
        this->predicted_path_in_frenet.push_back(v);
    }

    size_t num = 0;
    for (const auto &point_in_frenet : predicted_path_in_frenet) {
        for (; num != s_list.size() - 1; ++num) {
            if (s_list[num] >= point_in_frenet[0]) {
                double angle = atan((y_list[num + 1] - y_list[num]) / (x_list[num + 1] - x_list[num]));
                double new_angle = angle + M_PI_2;
                double x = x_list[num] + point_in_frenet[1] * cos(new_angle);
                double y = y_list[num] + point_in_frenet[1] * sin(new_angle);
                predicted_path_x.push_back(x);
                predicted_path_y.push_back(y);
                break;
            }
        }
    }
}

std::vector<double> &MpcPathOptimizer::getXList() {
    return this->predicted_path_x;
}

std::vector<double> &MpcPathOptimizer::getYList() {
    return this->predicted_path_y;
}

}
