//
// Created by ljn on 19-8-16.
//

#include "../include/MpcPathOptimizer.hpp"
namespace MpcSmoother {

MpcPathOptimizer::MpcPathOptimizer(const std::vector<hmpl::State> &points_list,
                                   const hmpl::State &start_state,
                                   const hmpl::State &end_state,
                                   const hmpl::InternalGridMap &map) :
    grid_map_(map),
    collision_checker_(map),
    points_list_(points_list),
    point_num_(points_list.size()),
    start_state_(start_state),
    end_state_(end_state),
    car_type(ACKERMANN_STEERING),
    rear_axle_to_center_dis(1.3),
    best_sampling_index_(0),
    control_sampling_first_flag_(false) {}

bool MpcPathOptimizer::solve(std::vector<hmpl::State> *final_path) {
    //
    // TODO: the result path should be different for various initial velocity!
    //
    if (point_num_ == 0) {
        LOG(INFO) << "path input is empty!";
        return false;
    }
    // Set the car geometry. Use 3 circles to approximate the car.
    // TODO: use a config file.
    // TODO: consider back up situation
    double car_width = 2.0;
    double car_length = 5;
    double rear_l = 2.5;
    double front_l = 2.5;
    double rear_circle_distance = rear_l - car_width / 2;
    double front_circle_distance = front_l - car_width / 2;
    // Vector car_geo is for function getClearanceWithDirection.
    std::vector<double> car_geo;
    car_geo.push_back(rear_circle_distance);
    car_geo.push_back(front_circle_distance);
    double rear_front_r = sqrt(pow(car_width / 2, 2) + pow(car_width / 2, 2));
    double middle_r;
    if (car_length > 2 * car_width) {
        middle_r = sqrt(pow(std::max(rear_l, front_l) - car_width, 2) + pow(car_width / 2, 2));
    } else {
        middle_r = 0;
    }
    car_geo.push_back(rear_front_r);
    car_geo.push_back(middle_r);
    car_geo.push_back(1.3);

    auto original_start_state = start_state_;
    std::vector<hmpl::State> best_path;
    double min_distance_for_best_path = 0;
    size_t min_index_for_best_path = 0;
    grid_map::Position start_position(start_state_.x, start_state_.y);
    double start_ref_angle = hmpl::angle(points_list_[0], points_list_[1]);
    double start_angle_diff = fabs(constraintAngle(start_state_.z - start_ref_angle));
    if ((grid_map_.getObstacleDistance(start_position) < 2 && start_angle_diff > 20 * M_PI / 180)
        || start_angle_diff > 70 * M_PI / 180) {
        printf("start point is close to obstacle, control sampling first!\n");
        if (start_state_.k < -MAX_CURVATURE || start_state_.k > MAX_CURVATURE) goto normal_procedure;
        double dk = -0.1;
        while (dk <= 0.1) {
            double max_ds = 10.0;
            double max_turn_ds = 0;
            if (dk < 0) {
                max_turn_ds = (start_state_.k + MAX_CURVATURE) / fabs(dk);
            } else if (dk > 0) {
                max_turn_ds = (MAX_CURVATURE - start_state_.k) / fabs(dk);
            } else {
                max_turn_ds = 1;
            }
            G2lib::ClothoidCurve curve;
            curve.build(start_state_.x, start_state_.y, start_state_.z, start_state_.k, dk, max_turn_ds);
            hmpl::State turn_curve_end;
            curve.evaluate(max_turn_ds, turn_curve_end.z, turn_curve_end.k, turn_curve_end.x, turn_curve_end.y);
            G2lib::ClothoidCurve curve_keep;
            curve_keep.build(turn_curve_end.x,
                             turn_curve_end.y,
                             turn_curve_end.z,
                             turn_curve_end.k,
                             0,
                             std::max(4.0, max_ds - max_turn_ds));
            double sampling_length = 2;
            while (sampling_length <= max_ds) {
                hmpl::State tmp_state;
                double delta_s = 0.4;
                std::vector<hmpl::State> sampling_result;
                for (size_t i = 0; i * delta_s < sampling_length; ++i) {
                    if (i * delta_s < max_turn_ds) {
                        curve.evaluate(i * delta_s, tmp_state.z, tmp_state.k, tmp_state.x, tmp_state.y);
                    } else {
                        curve_keep.evaluate(i * delta_s - max_turn_ds,
                                            tmp_state.z,
                                            tmp_state.k,
                                            tmp_state.x,
                                            tmp_state.y);
                    }
                    if (collision_checker_.isSingleStateCollisionFreeImproved(tmp_state)) {
                        sampling_result.push_back(tmp_state);
                    } else {
                        failed_sampling_path_set_.push_back(sampling_result);
                        goto try_new_dk;
                    }
                }
                sampling_path_set_.push_back(sampling_result);
                sampling_length += 1;
            }
            try_new_dk :
            dk += 0.025;
        }
        printf("control sampling get %d paths\n", sampling_path_set_.size());
        auto min_angle_diff = DBL_MAX;
        if (!sampling_path_set_.empty()) {
//            std::vector<double> path_scoring;
            double min_score = DBL_MAX;
            for (size_t i = 0; i != sampling_path_set_.size(); ++i) {
                const auto &last_state = sampling_path_set_[i].back();
                size_t min_index = 0;
                auto min_distance = DBL_MAX;
                for (size_t i = 0; i != point_num_; ++i) {
                    double tmp_distance = hmpl::distance(last_state, points_list_[i]);
                    if (tmp_distance < min_distance) {
                        min_distance = tmp_distance;
                        min_index = i;
                    } else if (tmp_distance > 15 && min_distance < 15) {
                        break;
                    }
                }
                double ref_angle = hmpl::angle(points_list_[min_index], points_list_[min_index + 1]);
                double angle_diff = fabs(constraintAngle(ref_angle - last_state.z));
                if (fabs(angle_diff) > 45) continue;
                grid_map::Position sampling_end_position(last_state.x, last_state.y);
                double sampling_end_clearance = grid_map_.getObstacleDistance(sampling_end_position);
                // TODO: path choosing strategy should be improved!
                double score = 5 * angle_diff + min_distance + 4 / sampling_end_clearance;
                if (score < min_score) {
                    min_angle_diff = angle_diff;
                    best_sampling_index_ = i;
                    min_distance_for_best_path = min_distance;
                    min_index_for_best_path = min_index;
                    min_score = score;
                }
            }
            control_sampling_first_flag_ = true;
            best_path.clear();
            for (const auto &state : sampling_path_set_[best_sampling_index_]) {
                best_path.push_back(state);
            }
            start_state_ = best_path.back();
            printf("control sampling before path optimization succeeded!\n");
        } else {
            control_sampling_first_flag_ = false;
            printf("empty path set\n");
        }
    }

    normal_procedure:
    double cte = 0;  // lateral error
    double epsi = 0; // navigable error
    // Get the closest point on the ref path and erase the points before this point.
    auto min_distance = DBL_MAX;
    size_t min_index = 0;
    if (control_sampling_first_flag_) {
        min_index = min_index_for_best_path;
        min_distance = min_distance_for_best_path;
    } else if (hmpl::distance(points_list_.front(), start_state_) < 0.001) {
        min_distance = 0;
        min_index = 0;
    } else {
        for (size_t i = 0; i != point_num_; ++i) {
            double tmp_distance = hmpl::distance(points_list_[i], start_state_);
            if (tmp_distance < min_distance) {
                min_distance = tmp_distance;
                min_index = i;
            } else if (tmp_distance > 15 && min_distance < 15) {
                break;
            }
        }
    }
    if (min_distance != 0 || min_index != 0) {
        points_list_.erase(points_list_.begin(), points_list_.begin() + min_index);
        point_num_ = points_list_.size();
        // Car on the left: cte > 0; car on the right: cte < 0.
        auto first_point_local = hmpl::globalToLocal(start_state_, points_list_.front());
        if (first_point_local.y < 0) {
            cte = min_distance;
        } else {
            cte = -min_distance;
        }
    }

    double s = 0;
    for (size_t i = 0; i != point_num_; ++i) {
        if (i == 0) {
            s_list_.push_back(0);
        } else {
            double ds = hmpl::distance(points_list_[i], points_list_[i - 1]);
            s += ds;
            s_list_.push_back(s);
        }
        x_list_.push_back(points_list_[i].x);
        y_list_.push_back(points_list_[i].y);
    }
    double max_s = s_list_.back();
    std::cout << "ref path length: " << max_s << std::endl;
    x_spline_.set_points(s_list_, x_list_);
    y_spline_.set_points(s_list_, y_list_);
    // Make the path dense, the interval being 0.3m
    x_list_.clear();
    y_list_.clear();
    s_list_.clear();
    size_t new_points_count = 0;
    for (double new_s = 0; new_s <= max_s; new_s += 0.3) {
        double x = x_spline_(new_s);
        double y = y_spline_(new_s);
        x_list_.push_back(x);
        y_list_.push_back(y);
        s_list_.push_back(new_s);
        ++new_points_count;
    }
    point_num_ = x_list_.size();
    double max_curvature_abs;
    double max_curvature_change_abs;
    getCurvature(x_list_, y_list_, &k_list_, &max_curvature_abs, &max_curvature_change_abs);
    k_spline_.set_points(s_list_, k_list_);

    // If the start heading differs a lot with the ref path, quit.
    if (x_spline_.deriv(1, 0) == 0) {
        start_ref_angle = M_PI_2;
    } else {
        start_ref_angle = atan2(y_spline_.deriv(1, 0), x_spline_.deriv(1, 0));
    }
    // calculate the difference between the start angle of the reference path ande the angle of start state.
    epsi = constraintAngle(start_state_.z - start_ref_angle);
    if (fabs(epsi) > M_PI_2) {
        LOG(WARNING) << "initial epsi is larger than 90°, quit mpc path optimization!";
        return false;
    }
    // Calculate the second and the third state, which depends on the initial heading and curvature of the vehicle.
    // Therefore, the first 3 states are fixed. We only optimize the rest of the states.
    double fixed_length = 0.5;
    double second_x = start_state_.x + fixed_length * cos(start_state_.z);
    double second_y = start_state_.y + fixed_length * sin(start_state_.z);
    double second_heading = constraintAngle(start_state_.k * fixed_length + start_state_.z);
    double third_x = second_x + fixed_length * cos(second_heading);
    double third_y = second_y + fixed_length * sin(second_heading);
    hmpl::State second_state, third_state;
    second_state.x = second_x;
    second_state.y = second_y;
    second_state.s = hmpl::distance(start_state_, second_state);
    second_state.z = second_heading;
    third_state.x = third_x;
    third_state.y = third_y;
    third_state.s = hmpl::distance(second_state, third_state);
    second_third_point_.push_back(second_state);
    second_third_point_.push_back(third_state);

    // Divid the reference path. Intervals are smaller at the beginning.
    double delta_s_smaller = 0.5;
//    if (fabs(epsi) < 20 * M_PI / 180) delta_s_smaller = 1;
    double delta_s_larger = 1.5;
    seg_s_list_.push_back(0);
    double first_s_on_ref = fixed_length * cos(epsi);
    seg_s_list_.push_back(first_s_on_ref);
    double second_ref_heading = atan2(y_spline_.deriv(1, first_s_on_ref), x_spline_.deriv(1, first_s_on_ref));
    seg_s_list_.push_back(first_s_on_ref + fixed_length * cos(constraintAngle(second_state.z - second_ref_heading)));
    double tmp_max_s = seg_s_list_.back() + delta_s_smaller;
    while (tmp_max_s < max_s) {
        seg_s_list_.push_back(tmp_max_s);
        if (tmp_max_s < 4) {
            tmp_max_s += delta_s_smaller;
        } else {
            tmp_max_s += delta_s_larger;
        }
    }
    if (max_s - seg_s_list_.back() > 1) {
        seg_s_list_.push_back(max_s);
    }
    auto N = seg_s_list_.size();
    auto original_N = N;

    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != N; ++i) {
        double length_on_ref_path = seg_s_list_[i];
        double angle;
        if (x_spline_.deriv(1, length_on_ref_path) == 0) {
            angle = M_PI_2;
        } else {
            angle = atan2(y_spline_.deriv(1, length_on_ref_path), x_spline_.deriv(1, length_on_ref_path));
        }
        seg_angle_list_.push_back(angle);
        seg_x_list_.push_back(x_spline_(length_on_ref_path));
        seg_y_list_.push_back(y_spline_(length_on_ref_path));
        seg_k_list_.push_back(k_spline_(length_on_ref_path));
    }

    auto min_clearance = DBL_MAX;
    for (size_t i = 0; i != N; ++i) {
        hmpl::State center_state;
        center_state.x = seg_x_list_[i];
        center_state.y = seg_y_list_[i];
        center_state.s = seg_s_list_[i];
        center_state.z = seg_angle_list_[i];
        // Function getClearance uses the center position as input.
        if (car_type == ACKERMANN_STEERING) {
            center_state.x += rear_axle_to_center_dis * cos(center_state.z);
            center_state.y += rear_axle_to_center_dis * sin(center_state.z);
        }
        std::vector<double> clearance;
        printf("calculating clearance for %d of %d\n", i, N);
        clearance = getClearanceFor3Circles(center_state, car_geo);
        printf("got clearance for %d\n", i);
        if ((clearance[0] == clearance[1] || clearance[2] == clearance[3] || clearance[4] == clearance[5])
            && center_state.s > 0.75 * max_s) {
            N = i;
            seg_x_list_.erase(seg_x_list_.begin() + i, seg_x_list_.end());
            seg_y_list_.erase(seg_y_list_.begin() + i, seg_y_list_.end());
            seg_s_list_.erase(seg_s_list_.begin() + i, seg_s_list_.end());
            seg_k_list_.erase(seg_k_list_.begin() + i, seg_k_list_.end());
            seg_angle_list_.erase(seg_angle_list_.begin() + i, seg_angle_list_.end());
            break;
        }
        seg_clearance_list_.push_back(clearance);
//        std::vector<double> clearance_range = getClearance(center_state, seg_angle_list_[i], car_geo);
//        double clearance_left = clearance_range[0];
//        double clearance_right = clearance_range[1];
//        double clearance = clearance_left - clearance_right;
////        double clearance_left = getClearanceWithDirection(center_state, constraintAngle(seg_angle_list_[i] + M_PI_2));
////        double clearance_right = -getClearanceWithDirection(center_state, constraintAngle(seg_angle_list_[i] - M_PI_2));
////        double clearance = clearance_left - clearance_right;
//        min_clearance = std::min(clearance, min_clearance);
//        seg_clearance_left_list_.push_back(clearance_left);
//        seg_clearance_right_list_.push_back(clearance_right);
//        if ((clearance_left * clearance_right > 0 || clearance_left == clearance_right)
//            && center_state.s > 0.75 * max_s) {
//            std::cout << (center_state.s > 0.75 * max_s) << std::endl;
//            N = i;
//            seg_x_list_.erase(seg_x_list_.begin() + i, seg_x_list_.end());
//            seg_y_list_.erase(seg_y_list_.begin() + i, seg_y_list_.end());
//            seg_s_list_.erase(seg_s_list_.begin() + i, seg_s_list_.end());
//            seg_k_list_.erase(seg_k_list_.begin() + i, seg_k_list_.end());
//            seg_angle_list_.erase(seg_angle_list_.begin() + i, seg_angle_list_.end());
//            break;
//        }
//        std::cout << i << " upper & lower bound: " << clearance_left << ", " << clearance_right << std::endl;
    }
    printf("got all clearance\n");
//    hmpl::State left_bound, right_bound;
//    for (size_t i = 0; i != N; ++i) {
//        left_bound.x = seg_x_list_[i] + seg_clearance_left_list_[i] * cos(seg_angle_list_[i] + M_PI_2);
//        left_bound.y = seg_y_list_[i] + seg_clearance_left_list_[i] * sin(seg_angle_list_[i] + M_PI_2);
//        right_bound.x = seg_x_list_[i] + seg_clearance_right_list_[i] * cos(seg_angle_list_[i] + M_PI_2);
//        right_bound.y = seg_y_list_[i] + seg_clearance_right_list_[i] * sin(seg_angle_list_[i] + M_PI_2);
//        left_bound_.push_back(left_bound);
//        right_bound_.push_back(right_bound);
//    }

    typedef CPPAD_TESTVECTOR(double) Dvector;
    // n_vars: Set the number of model variables.
    // There are N - 3 pqs, 1 heading (the last heading), N -3 curvatures and N - 3 ps in the variables.
    // Note that there are supposed to be N states, but the first 3 points are fixed. So they are not optimized.
    size_t n_vars = (N - 3);
    // Set the number of constraints
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    // bounds of variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (size_t i = 0; i != n_vars; ++i) {
        vars_lowerbound[i] = -DBL_MAX;
        vars_upperbound[i] = DBL_MAX;
    }

    printf("setting cons\n");
    // Costraints inclued the end heading and N - 3 curvatures.
    size_t n_constraints = 1 + (N - 3) + 3 * (N - 3);
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    size_t cons_heading_range_begin = 0;
    size_t cons_curvature_range_begin = cons_heading_range_begin + 1;
    size_t cons_rear_range_begin = cons_curvature_range_begin + N - 3;
    size_t cons_center_range_begin = cons_rear_range_begin + N - 3;
    size_t cons_front_range_begin = cons_center_range_begin + N - 3;
    // heading constraint
//    double target_heading = end_state_.z;
//    if (seg_x_list_[N - 1] < seg_x_list_[N - 2]) {
//        target_heading = end_state_.z > 0 ? end_state_.z - M_PI : end_state_.z + M_PI;
//    }
//    if (N == original_N) {
//        constraints_lowerbound[cons_heading_range_begin] = target_heading;
//        constraints_upperbound[cons_heading_range_begin] = target_heading;
//    }
    constraints_lowerbound[cons_heading_range_begin] = -DBL_MAX;
    constraints_upperbound[cons_heading_range_begin] = DBL_MAX;
    // curvature constraints
    for (size_t i = cons_curvature_range_begin; i != cons_rear_range_begin; ++i) {
        constraints_lowerbound[i] = -MAX_CURVATURE;
        constraints_upperbound[i] = MAX_CURVATURE;
    }
    for (size_t i = 0; i != N - 3; ++i) {
        constraints_upperbound[cons_rear_range_begin + i] = seg_clearance_list_[i + 2][0];
        constraints_lowerbound[cons_rear_range_begin + i] = seg_clearance_list_[i + 2][1];
        constraints_upperbound[cons_center_range_begin + i] = seg_clearance_list_[i + 2][2];
        constraints_lowerbound[cons_center_range_begin + i] = seg_clearance_list_[i + 2][3];
        constraints_upperbound[cons_front_range_begin + i] = seg_clearance_list_[i + 2][4];
        constraints_lowerbound[cons_front_range_begin + i] = seg_clearance_list_[i + 2][5];
    }

    printf("solving\n");
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
//    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.02\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    // weights of the cost function
    // TODO: use a config file
    std::vector<double> weights;
    weights.push_back(2); //curvature weight
    weights.push_back(30); //curvature rate weight
    weights.push_back(0.01); //distance to boundary weight
    weights.push_back(0.05); //path length weight

    printf("constructing fgeval\n");
    FgEvalFrenet fg_eval_frenet(seg_x_list_,
                                seg_y_list_,
                                seg_angle_list_,
                                seg_k_list_,
                                seg_s_list_,
                                N,
                                weights,
                                start_state_,
                                second_state,
                                third_state,
                                seg_clearance_left_list_,
                                seg_clearance_right_list_,
                                car_geo);
    printf("into solver\n");
    // solve the problem
    CppAD::ipopt::solve<Dvector, FgEvalFrenet>(options, vars,
                                               vars_lowerbound, vars_upperbound,
                                               constraints_lowerbound, constraints_upperbound,
                                               fg_eval_frenet, solution);
    printf("solved\n");
    // Check if it works
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        LOG(WARNING) << "mpc path optimization solver failed!";
        return false;
    }
    LOG(INFO) << "mpc path optimization solver succeeded!";
    // output
    for (size_t i = 0; i != N - 3; i++) {
        double tmp[2] = {solution.x[i], double(i)};
        std::vector<double> v(tmp, tmp + sizeof tmp / sizeof tmp[0]);
        this->predicted_path_in_frenet_.push_back(v);
    }
    size_t control_points_num = N;
    if (control_sampling_first_flag_) {
        control_points_num = N + best_path.size() - 1;
    }
    tinyspline::BSpline b_spline(control_points_num);
    std::vector<tinyspline::real> ctrlp = b_spline.controlPoints();
    size_t control_sampling_point_count = 0;
    if (control_sampling_first_flag_) {
        for (; control_sampling_point_count != best_path.size() - 1; ++control_sampling_point_count) {
            ctrlp[2 * control_sampling_point_count] = best_path.at(control_sampling_point_count).x;
            ctrlp[2 * control_sampling_point_count + 1] = best_path.at(control_sampling_point_count).y;
        }
    }
    // The first two points are fixed. They are not in the optimized variables.
    size_t count = 0;
    if (control_sampling_first_flag_) {
        count = 2 * (control_sampling_point_count);
    };
    ctrlp[count + 0] = start_state_.x;
    ctrlp[count + 1] = start_state_.y;
    ctrlp[count + 2] = second_state.x;
    ctrlp[count + 3] = second_state.y;
    ctrlp[count + 4] = third_state.x;
    ctrlp[count + 5] = third_state.y;
    for (size_t i = 0; i != N - 3; ++i) {
        double length_on_ref_path = seg_s_list_[i + 3];
        double angle = seg_angle_list_[i + 3];
        double new_angle = constraintAngle(angle + M_PI_2);
        double tmp_x = x_spline_(length_on_ref_path) + predicted_path_in_frenet_[i][0] * cos(new_angle);
        double tmp_y = y_spline_(length_on_ref_path) + predicted_path_in_frenet_[i][0] * sin(new_angle);
        if (std::isnan(tmp_x) || std::isnan(tmp_y)) {
            LOG(WARNING) << "output is not a number, mpc path opitmization failed!" << std::endl;
            return false;
        }
        ctrlp[count + 2 * (i + 3)] = tmp_x;
        ctrlp[count + 2 * (i + 3) + 1] = tmp_y;
    }
    // B spline
    b_spline.setControlPoints(ctrlp);
    std::vector<hmpl::State> tmp_final_path;
    double total_s = 0;
    double step_t = 1.0 / (3.0 * N);
    for (size_t i = 0; i <= 3 * N; ++i) {
        double t = i * step_t;
        std::vector<tinyspline::real> result = b_spline.eval(t).result();
        hmpl::State state;
        state.x = result[0];
        state.y = result[1];
        if (i == 0) {
            state.z = original_start_state.z;
            state.s = 0;
        } else {
            double dx = result[0] - (tmp_final_path)[i - 1].x;
            double dy = result[1] - (tmp_final_path)[i - 1].y;
            state.z = atan2(dy, dx);
            total_s += sqrt(pow(dx, 2) + pow(dy, 2));
            state.s = total_s;
        }
        if (collision_checker_.isSingleStateCollisionFreeImproved(state)) {
            tmp_final_path.push_back(state);
        } else {
            printf("path optimization collision check failed at %d of %d\n", i, 3 * N);
            if (state.s > 30) {
                break;
            }
            return false;
//            tmp_final_path.push_back(state);
        }
    }
    final_path->clear();
    std::copy(tmp_final_path.begin(), tmp_final_path.end(), std::back_inserter(*final_path));
    return true;
}

double MpcPathOptimizer::getClearanceWithDirectionStrict(hmpl::State state, double angle, double radius) {
    double s = 0;
    double delta_s = 0.1;
    size_t n = 5.0 / delta_s;
    for (size_t i = 0; i != n; ++i) {
        s += delta_s;
        double x = state.x + s * cos(angle);
        double y = state.y + s * sin(angle);
        grid_map::Position new_position(x, y);
        double clearance = grid_map_.getObstacleDistance(new_position);
        if (clearance <= radius) {
            return s - delta_s;
        }
    }
    return s;
}

std::vector<double> MpcPathOptimizer::getClearanceFor3Circles(const hmpl::State &state,
                                                              const std::vector<double> &car_geometry) {
    double rear_front_radius = car_geometry[2];
    double middle_radius = car_geometry[3];
    hmpl::State front, center, rear;
    double center_x = state.x;
    double center_y = state.y;
    double rear_x = center_x - car_geometry[0] * cos(state.z);
    double rear_y = center_y - car_geometry[0] * sin(state.z);
    double front_x = center_x + car_geometry[1] * cos(state.z);
    double front_y = center_y + car_geometry[1] * sin(state.z);
    center.x = center_x;
    center.y = center_y;
    front.x = front_x;
    front.y = front_y;
    rear.x = rear_x;
    rear.y = rear_y;
    std::vector<double> result;
    double left_angle = constraintAngle(state.z + M_PI_2);
    double right_angle = constraintAngle(state.z - M_PI_2);
    result.push_back(getClearanceWithDirectionStrict(rear, left_angle, rear_front_radius));
    result.push_back(-getClearanceWithDirectionStrict(rear, right_angle, rear_front_radius));
    result.push_back(getClearanceWithDirectionStrict(center, left_angle, middle_radius));
    result.push_back(-getClearanceWithDirectionStrict(center, right_angle, middle_radius));
    result.push_back(getClearanceWithDirectionStrict(front, left_angle, rear_front_radius));
    result.push_back(-getClearanceWithDirectionStrict(front, right_angle, rear_front_radius));
    return result;

}

double MpcPathOptimizer::getClearanceWithDirection(const hmpl::State &state,
                                                   double angle,
                                                   const std::vector<double> &car_geometry) {
    double s = 0;
    double delta_s = 0.1;
    size_t n = 5.0 / delta_s;
    for (size_t i = 0; i != n; ++i) {
        s += delta_s;
        double x = state.x + s * cos(angle);
        double y = state.y + s * sin(angle);
        double rear_x = x - car_geometry[0] * cos(state.z);
        double rear_y = y - car_geometry[0] * sin(state.z);
        double front_x = x + car_geometry[1] * cos(state.z);
        double front_y = y + car_geometry[1] * sin(state.z);
        grid_map::Position new_position(x, y);
        grid_map::Position new_rear_position(rear_x, rear_y);
        grid_map::Position new_front_position(front_x, front_y);
        if (grid_map_.maps.isInside(new_position) && grid_map_.maps.isInside(new_rear_position)
            && grid_map_.maps.isInside(new_front_position)) {
            double new_rear_clearance = grid_map_.getObstacleDistance(new_rear_position);
            double new_front_clearance = grid_map_.getObstacleDistance(new_front_position);
            double new_middle_clearance = grid_map_.getObstacleDistance(new_position);
            if (std::min(new_rear_clearance, new_front_clearance) < car_geometry[2]
                || new_middle_clearance < car_geometry[3]) {
                return s - delta_s;
            }
        } else {
            return s - delta_s;
        }
    }
    return s;
}

std::vector<double> MpcPathOptimizer::getClearance(hmpl::State state,
                                                   double ref_angle,
                                                   const std::vector<double> &car_geometry) {
    // Check if the current state has collision.
    double rear_center_distance = car_geometry[0];
    double front_center_distance = car_geometry[1];
    double rear_front_radius = car_geometry[2];
    double middle_radius = car_geometry[3];
    // Calculate the position of the rear center, middle center and front center on current state.
    grid_map::Position
        rear_position(state.x - rear_center_distance * cos(state.z), state.y - rear_center_distance * sin(state.z));
    grid_map::Position
        middle_position(state.x, state.y);
    grid_map::Position
        front_position(state.x + front_center_distance * cos(state.z), state.y + front_center_distance * sin(state.z));
    double rear_clearance = grid_map_.getObstacleDistance(rear_position);
    double front_clearance = grid_map_.getObstacleDistance(front_position);
    double middle_clearance = grid_map_.getObstacleDistance(middle_position);
    // If the current state is collision free, then expand to left and right.
    if (std::min(rear_clearance, front_clearance) > rear_front_radius && middle_clearance > middle_radius) {
        double left_clearance = getClearanceWithDirection(state, constraintAngle(ref_angle + M_PI_2), car_geometry);
        double right_clearance = -getClearanceWithDirection(state, constraintAngle(ref_angle - M_PI_2), car_geometry);
        std::vector<double> clearance{left_clearance, right_clearance};
        return clearance;
    } else {
        // If the current state has collision, then search to left OR right until find a clear range.
        double left_limit, right_limit;
        bool left_exploration_succeed_flag = false;
        bool right_exploration_succeed_flag = false;
        // explore left:
        double s = 0;
        double delta_s = 0.1;
        size_t n = 5.0 / delta_s;
        for (size_t i = 0; i != n; ++i) {
            s += delta_s;
            double x = state.x + s * cos(constraintAngle(ref_angle + M_PI_2));
            double y = state.y + s * sin(constraintAngle(ref_angle + M_PI_2));
            double rear_x = x - rear_center_distance * cos(state.z);
            double rear_y = y - rear_center_distance * sin(state.z);
            double front_x = x + front_center_distance * cos(state.z);
            double front_y = y + front_center_distance * sin(state.z);
            grid_map::Position new_position(x, y);
            grid_map::Position new_rear_position(rear_x, rear_y);
            grid_map::Position new_front_position(front_x, front_y);
            // Search to left until a clear position is found, then continue searching to left until
            // a collision position is found.
            if (grid_map_.maps.isInside(new_position) && grid_map_.maps.isInside(new_rear_position)
                && grid_map_.maps.isInside(new_front_position)) {
                double new_rear_clearance = grid_map_.getObstacleDistance(new_rear_position);
                double new_front_clearance = grid_map_.getObstacleDistance(new_front_position);
                double new_middle_clearance = grid_map_.getObstacleDistance(new_position);
                if (std::min(new_rear_clearance, new_front_clearance) > rear_front_radius
                    && new_middle_clearance > middle_radius) {
                    left_exploration_succeed_flag = true;
                    right_limit = s;
                    hmpl::State new_state;
                    new_state.x = x;
                    new_state.y = y;
                    new_state.z = state.z;
                    left_limit = right_limit
                        + getClearanceWithDirection(new_state, constraintAngle(ref_angle + M_PI_2), car_geometry);
                    break;
                }
            }
        }
        if (!left_exploration_succeed_flag) {
            // explore right:
            double s = 0;
            double delta_s = 0.1;
            size_t n = 5.0 / delta_s;
            for (size_t i = 0; i != n; ++i) {
                s += delta_s;
                double x = state.x + s * cos(constraintAngle(ref_angle - M_PI_2));
                double y = state.y + s * sin(constraintAngle(ref_angle - M_PI_2));
                double rear_x = x - rear_center_distance * cos(state.z);
                double rear_y = y - rear_center_distance * sin(state.z);
                double front_x = x + front_center_distance * cos(state.z);
                double front_y = y + front_center_distance * sin(state.z);
                grid_map::Position new_position(x, y);
                grid_map::Position new_rear_position(rear_x, rear_y);
                grid_map::Position new_front_position(front_x, front_y);
                if (grid_map_.maps.isInside(new_position) && grid_map_.maps.isInside(new_rear_position)
                    && grid_map_.maps.isInside(new_front_position)) {
                    double new_rear_clearance = grid_map_.getObstacleDistance(new_rear_position);
                    double new_front_clearance = grid_map_.getObstacleDistance(new_front_position);
                    double new_middle_clearance = grid_map_.getObstacleDistance(new_position);
                    if (std::min(new_rear_clearance, new_front_clearance) > rear_front_radius
                        && new_middle_clearance > middle_radius) {
                        right_exploration_succeed_flag = true;
                        left_limit = -s;
                        hmpl::State new_state;
                        new_state.x = x;
                        new_state.y = y;
                        new_state.z = state.z;
                        right_limit = left_limit
                            - getClearanceWithDirection(new_state, constraintAngle(ref_angle - M_PI_2), car_geometry);
                        break;
                    }

                }
            }
        }
        if (left_exploration_succeed_flag || right_exploration_succeed_flag) {
            return std::vector<double>{left_limit, right_limit};
        } else {
            return std::vector<double>{0, 0};
        }
    }
}

double MpcPathOptimizer::getClearanceWithDirection(const hmpl::State &state, double angle) {
    double s = 0;
    double delta_s = 0.1;
    size_t n = 5.0 / delta_s;
    for (size_t i = 0; i != n; ++i) {
        s += delta_s;
        double x = state.x + s * cos(angle);
        double y = state.y + s * sin(angle);
        grid_map::Position new_position(x, y);
        if (grid_map_.maps.isInside(new_position)) {
            if (grid_map_.maps.atPosition("obstacle", new_position) == 0) {
//            if (grid_map_.getObstacleDistance(new_position) <= 1.45) {
                return s - delta_s;
            }
        } else {
            return s - delta_s;
        }
    }
    return s;
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
        final_curvature = curvature[j];
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

const std::vector<std::vector<hmpl::State> > &MpcPathOptimizer::getControlSamplingPathSet() {
    return this->sampling_path_set_;
};

const std::vector<std::vector<hmpl::State> > &MpcPathOptimizer::getControlSamplingFailedPathSet() {
    return this->failed_sampling_path_set_;
};

const std::vector<hmpl::State> &MpcPathOptimizer::getBestSamplingPath() {
    if (control_sampling_first_flag_) {
        return this->sampling_path_set_[best_sampling_index_];
    } else {
        return this->empty_;
    }
}

const std::vector<hmpl::State> &MpcPathOptimizer::getLeftBound() {
    return this->left_bound_;
}

const std::vector<hmpl::State> &MpcPathOptimizer::getRightBound() {
    return this->right_bound_;
}

const std::vector<hmpl::State> &MpcPathOptimizer::getSecondThirdPoint() {
    return this->second_third_point_;
}
}
