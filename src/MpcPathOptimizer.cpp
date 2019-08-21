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
    std::cout << "start state: " << start_state.x << ", " << start_state.y << std::endl;

//    double first_dis = sqrt(pow(y_list[1]-y_list[0],2) + pow(x_list[1]-x_list[0],2));
//    std::cout << "old first distance:  " << first_dis << std::endl;
//
//    tinyspline::BSpline bSpline(point_num);
//    std::vector<tinyspline::real> ctrlp = bSpline.controlPoints();
//    for (size_t i = 0; i != point_num; ++i) {
//        ctrlp[2 * i] = x_list[i];
//        ctrlp[2 * i+1] = y_list[i];
//    }
//    bSpline.setControlPoints(ctrlp);
//
//    x_list.clear();
//    y_list.clear();
//    for (double t = 0; t <= 1; t += 0.01) {
//        std::vector<tinyspline::real> result = bSpline.eval(t).result();
//        x_list.push_back(result[0]);
//        y_list.push_back(result[1]);
//    }
//    std::cout << "b spline num: " << x_list.size() << std::endl;
//    point_num = x_list.size();
//
//    double first_dis_new = sqrt(pow(y_list[1]-y_list[2],2) + pow(x_list[1]-x_list[2],2));
//    std::cout << "first distance:  " << first_dis_new << std::endl;
    double s = 0;
    s_list.push_back(0);
    for (size_t i = 1; i != point_num; ++i) {
        double ds = sqrt(pow(x_list[i] - x_list[i - 1], 2)
                             + pow(y_list[i] - y_list[i - 1], 2));
        s += ds;
        s_list.push_back(s);
    }
    double max_s = s_list.back();
    x_spline.set_points(s_list, x_list);
    y_spline.set_points(s_list, y_list);
    x_list.clear();
    y_list.clear();
    s_list.clear();
    // make the path dense
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

//    transformToLocal(x_list, y_list, &x_local, &y_local);

    // todo: consider the condition where the initial state is not on the path.
//    double s = 0;
//    s_list.push_back(0);
//    std::cout << "s list: ----" << std::endl;
//    for (size_t i = 1; i != point_num; ++i) {
//        double ds = sqrt(pow(x_local[i] - x_local[i - 1], 2)
//                             + pow(y_local[i] - y_local[i - 1], 2));
//        s += ds;
//        std::cout << "s: " << s << std::endl;
//        s_list.push_back(s);
//    }
//    x_spline.set_points(s_list, x_local);
//    y_spline.set_points(s_list, y_local);

    getCurvature(x_list, y_list, &k_list);
//    for (size_t n = 0; n != s_list.size(); ++n) {
//        std::cout << n << ", " << s_list[n] << " " << k_list[n] << std::endl;
//    }
    k_spline.set_points(s_list, k_list);
    // initial states
    cte = 0;
//    double start_ref_angle = atan((y_list[5] - y_list[0]) / (x_list[5] - x_list[0]));
    double start_ref_angle = 0;
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


//    start_ref_angle = start_ref_angle > M_PI ? start_ref_angle - M_PI : start_ref_angle;
//    start_ref_angle = start_ref_angle < -M_PI ? start_ref_angle + M_PI : start_ref_angle;
    epsi = start_state.z - start_ref_angle;
    if (epsi > M_PI) {
        epsi -= 2 * M_PI;
    } else if (epsi < - M_PI) {
        epsi += 2 * M_PI;
    }

//    std::cout << "y1y0x1x0: " << y_list[5] << " " << y_list[0] << " " << x_list[5] << " " << x_list[0] << std::endl;
//    std::cout << "end point y x: " << y_list.back() << " " << x_list.back() << std::endl;
//    epsi = start_state.z - start_ref_angle;
    std::cout<< "start state angle: " << start_state.z*180/M_PI << ",  initial ref angle: " << start_ref_angle*180/M_PI << ",  initial epsi: " << epsi*180/M_PI << std::endl;
    double curvature = start_state.k;
    // todo: delta_s should be changeable.
    double delta_s = 3;

    double psi = epsi;
    double ps = delta_s * cos(psi);
    double pq = delta_s * sin(psi);
    psi += delta_s * curvature - delta_s * cos(psi) * k_spline(ps);
    std::cout << "initial ps pq psi: " << ps << " " << pq << " " << psi*180/M_PI << std::endl;
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
    } else if (end_psi < - M_PI) {
        end_psi += 2 * M_PI;
    }
    std::cout << "end ref: " << end_ref_angle*180/M_PI << ", end z: " << end_state.z*180/M_PI <<  ", end psi: " << end_psi*180/M_PI << std::endl;

    // only for test----------------------
//    x_list_for_test = {-0.364482, -0.162103, 0.0402473, 0.242524, 0.444687, 0.646697, 0.848519, 1.05012, 1.25148, 1.45256, 1.65335, 1.85383, 2.05399, 2.25382, 2.45331, 2.65246, 2.85127, 3.04975, 3.2479, 3.44575, 3.64329, 3.84055, 4.03755, 4.2343, 4.43082, 4.62715, 4.8233, 5.01931, 5.21518, 5.41095};
//    y_list_for_test = {0.0288432, 0.0283601, 0.0248548, 0.0183767, 0.00898279, -0.00326293, -0.0182899, -0.0360215, -0.0563753, -0.079264, -0.104596, -0.132274, -0.162198, -0.194266, -0.228372, -0.264406, -0.30226, -0.34182, -0.382975, -0.42561, -0.469612, -0.514868, -0.561263, -0.608686, -0.657024, -0.706169, -0.756012, -0.806446, -0.85737, -0.908682};
//    s_list_for_test = {0, 0.20238, 0.40476, 0.607141, 0.809522, 1.0119, 1.21428, 1.41667, 1.61905, 1.82143, 2.02381, 2.22619, 2.42858, 2.63096, 2.83334, 3.03573, 3.23811, 3.44049, 3.64288, 3.84526, 4.04765, 4.25003, 4.45242, 4.6548, 4.85719, 5.05957, 5.26196, 5.46435, 5.66673, 5.86912};
//    k_list_for_test = {-0.0737908, -0.0733948, -0.0725463, -0.0711919, -0.0696773, -0.0680121, -0.0662057, -0.0642674, -0.0622067, -0.0600331, -0.057756, -0.0553848, -0.0529289, -0.0503978, -0.047801, -0.0451477, -0.0424476, -0.0397099, -0.0369442, -0.0341599, -0.0313663, -0.028573, -0.0257894, -0.0230248, -0.0202888, -0.0175908, -0.0149401, -0.0123462, -0.010637, -0.00979338};
//    k_spline_for_test.set_points(s_list_for_test, k_list_for_test);
//    ps = 1.27176;
//    pq = -0.00564891;
//    psi = 1.35341*M_PI/180;
    // -----------------------------------------------

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
    // no bounds for state variables
    for (size_t i = 0; i < curvature_range_begin; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    double end_psi_error = 2*M_PI/180;
    vars_lowerbound[psi_range_begin + N - 1] = end_psi - end_psi_error;
    vars_upperbound[psi_range_begin + N - 1] = end_psi + end_psi_error;
    std::cout << "target psi range: " << vars_lowerbound[psi_range_begin + N - 1]*180/M_PI
            << " " << vars_upperbound[psi_range_begin + N - 1]*180/M_PI << std::endl;
    // set bounds for control variables
    for (size_t i = curvature_range_begin; i < n_vars; i++) {
        vars_lowerbound[i] = -0.25;
        vars_upperbound[i] = 0.25;
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

    double end_psi_bound = 20*M_PI/180;
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
    // todo: use a config file
    std::vector<double> weights;
    weights.push_back(1); //cost_func_cte_weight
    weights.push_back(11); //cost_func_epsi_weight
    weights.push_back(400); //cost_func_curvature_weight
    weights.push_back(1500); //cost_func_curvature_rate_weight
    bool isback = false;

//    Eigen::VectorXd cost_func(4);
//    cost_func << 1, 11, 1, 1500;

    FgEvalFrenet fg_eval_frenet(k_spline, isback, N, weights, delta_s);
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalFrenet>(options, vars,
                                               vars_lowerbound, vars_upperbound,
                                               constraints_lowerbound, constraints_upperbound,
                                               fg_eval_frenet, solution);

//    std::cout << "test kspline: " << k_spline_for_test(0) << " " << k_spline_for_test(1) << " " << k_spline_for_test(2) << std::endl;

//    FgEvalFrenetForTest fg_eval_frenet_for_test(k_spline_for_test,isback,N,0.15,weights,2);
//    // solve the problem
//    CppAD::ipopt::solve<Dvector, FgEvalFrenetForTest>(options, vars,
//                                               vars_lowerbound, vars_upperbound,
//                                               constraints_lowerbound, constraints_upperbound,
//                                               fg_eval_frenet_for_test, solution);

    // Check some of the solution values
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    std::cout << "are you ok: " << ok << std::endl;

    // Cost
    double cost = solution.obj_value;
    std::cout << "cost: " << cost << std::endl;

    size_t nan_num = 0;
    for (size_t i = 0; i < N; i++) {
        double tmp[4] = {solution.x[ps_range_begin + i], solution.x[pq_range_begin + i],
                         solution.x[psi_range_begin + i], double(i)};
        std::vector<double> v(tmp, tmp + sizeof tmp / sizeof tmp[0]);
        this->predicted_path_in_frenet.push_back(v);
        std::cout << "calculated curvature: " << solution.x[curvature_range_begin + i] << std::endl;
    }
    std::cout << "solution end psi: " << solution.x[psi_range_begin + N -1]*180/M_PI << std::endl;
    std::cout << "predicted path size: " << predicted_path_in_frenet.size() << std::endl;

//    predicted_path_x.push_back(start_state.x);
//    predicted_path_y.push_back(start_state.y);
    predicted_path_x.push_back(x_list.front());
    predicted_path_y.push_back(y_list.front());
    // todo: use x_spline, y_spline and their derivative to calculate predicted path
    size_t num = 0;
    for (const auto &point_in_frenet : predicted_path_in_frenet) {
        for (; num != s_list.size() - 1; ++num) {
            if (s_list[num] >= point_in_frenet[0]) {
                double angle = atan((y_list[num + 1] - y_list[num]) / (x_list[num + 1] - x_list[num]));
                if (x_list[num + 1] - x_list[num] < 0) {
                    if (angle > 0) {
                        angle -= M_PI;
                    } else if (angle < 0) {
                        angle += M_PI;
                    }
                }
                double new_angle = angle + M_PI_2;
                double x = x_list[num] + point_in_frenet[1] * cos(new_angle);
                double y = y_list[num] + point_in_frenet[1] * sin(new_angle);
                std::cout << "num: " << num <<  ", target s: " << point_in_frenet[0] << ", original x: " << x_list[num] << ", original y: "
                        << y_list[num] << std::endl;

                std::cout << "original angle: " << angle*180/M_PI << ", new angle: "
                        << new_angle*180/M_PI << ", d: " << point_in_frenet[1] << ", psi: " << point_in_frenet[2]*180/M_PI << std::endl;
                if (std::isnan(x) || std::isnan(y)) {
                    ++nan_num;
                    std::cout << "not a number; " << x_list[num + 1] - x_list[num] << std::endl;
                }
                std::cout << "mpc smoothing: x: " << x << ", y: " << y << std::endl;
                predicted_path_x.push_back(x);
                predicted_path_y.push_back(y);
                break;
            }
        }
    }
    std::cout << "nan num: " << nan_num << std::endl;
}

std::vector<double> &MpcPathOptimizer::getXList() {
    return this->predicted_path_x;
}

std::vector<double> &MpcPathOptimizer::getYList() {
    return this->predicted_path_y;
}

void MpcPathOptimizer::transformToLocal(const std::vector<double> &x_before,
                                        const std::vector<double> &y_before,
                                        std::vector<double> *x_after,
                                        std::vector<double> *y_after) {
    double px = start_state.x;
    double py = start_state.y;
    double psi = start_state.z;
    int index_min = 0;
//    double min = 999;

//    // todo: check if the first point is on the path
//    for (unsigned int i = 0; i < x_before.size(); i++) {
//        double delta_x = x_before[i] - px;
//        double delta_y = y_before[i] - py;
//        double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
//        if (dist < min) {
//            min = dist;
//            index_min = i;
//        }
//    }

    x_after->clear();
    y_after->clear();
//    for (size_t j = index_min; j < x_before.size(); j++) {
//
//        double delta_x = x_before[j] - px;
//        double delta_y = y_before[j] - py;
//
//        double x = delta_x * cos(psi) + delta_y * sin(psi);
//        double y = delta_y * cos(psi) - delta_x * sin(psi);
//        x_after->push_back(x);
//        y_after->push_back(y);
//    }

    for (size_t j = index_min; j < x_before.size(); j++) {
        double delta_x = x_before[j] - px;
        double delta_y = y_before[j] - py;
        double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        double tmp_angle = atan(delta_y / delta_x);
        if (delta_x < 0) {
            if (tmp_angle > 0) {
                tmp_angle -= M_PI;
            } else if (tmp_angle < 0) {
                tmp_angle += M_PI;
            }
        }
        double delta_angle = tmp_angle - psi;
        double x = distance * cos(delta_angle);
        double y = distance * sin(delta_angle);
        x_after->push_back(x);
        y_after->push_back(y);
    }
}
}
