//
// Created by ljn on 19-8-16.
//

#include "path_optimizer/path_optimizer.hpp"
namespace PathOptimizationNS {

PathOptimizer::PathOptimizer(const std::vector<hmpl::State> &points_list,
                             const hmpl::State &start_state,
                             const hmpl::State &end_state,
                             const hmpl::InternalGridMap &map) :
    grid_map_(map),
    collision_checker_(map),
    vehicle_state_(start_state, end_state, 0, 0),
    points_list_(points_list),
    point_num_(points_list.size()),
    solver_dynamic_initialized(false) {
    setConfig();
}

void PathOptimizer::setConfig() {
    // TODO: read from config file.
    config_.car_type_ = ACKERMANN_STEERING;
    config_.car_width_ = 2.0;
    config_.car_length_ = 4.9;
    config_.circle_radius_ = sqrt(pow(config_.car_length_ / 8, 2) + pow(config_.car_width_ / 2, 2));
    double safety_margin = 0.1;
    config_.circle_radius_ += safety_margin;
    config_.wheel_base_ = 2.85;
    config_.rear_axle_to_center_distance_ = 1.45;
    config_.d1_ = -3.0 / 8.0 * config_.car_length_;
    config_.d2_ = -1.0 / 8.0 * config_.car_length_;
    config_.d3_ = 1.0 / 8.0 * config_.car_length_;
    config_.d4_ = 3.0 / 8.0 * config_.car_length_;
    config_.max_steer_angle_ = 30 * M_PI / 180;

    config_.smoothing_method_ = FRENET;
    config_.modify_input_points_ = true;
    config_.a_star_lateral_range_ = 10;
    config_.a_star_longitudinal_interval_ = 1.5;
    config_.a_star_lateral_interval_ = 0.6;

    //
    config_.frenet_curvature_rate_w_ = 1500;
    config_.frenet_curvature_w_ = 200;
    config_.frenet_deviation_w_ = 4;
    //
    config_.cartesian_curvature_w_ = 10;
    config_.cartesian_deviation_w_ = 0.001;
    //
    config_.opt_curvature_w_ = 10;
    config_.opt_curvature_rate_w_ = 1000;
    config_.opt_deviation_w_ = 0.0;
    config_.constraint_end_heading_ = true;
    // TODO: use this condition.
    config_.exact_end_position_ = false;

    //
    config_.raw_result_ = true;
    config_.output_interval_ = 0.5;
}

bool PathOptimizer::solve(std::vector<hmpl::State> *final_path) {
    auto t1 = std::clock();
    if (point_num_ == 0) {
        printf("empty input, quit path optimization\n");
        return false;
    }
    // Smooth reference path.
    ReferencePathSmoother
        reference_path_smoother(points_list_, vehicle_state_.start_state_, grid_map_, config_);
    bool smoothing_ok = false;
    if (config_.smoothing_method_ == FRENET) {
        smoothing_ok = reference_path_smoother.solve<FrenetReferencePathSmoother>(&reference_path_, &smoothed_path_);
    } else if (config_.smoothing_method_ == CARTESIAN) {
        smoothing_ok = reference_path_smoother.solve<CartesianReferencePathSmoother>(&reference_path_, &smoothed_path_);
    }
    a_star_display_ = reference_path_smoother.display();
    if (!smoothing_ok) {
        printf("smoothing stage failed, quit path optimization.\n");
        return false;
    }
    auto t2 = std::clock();
    // Divide reference path into segments and store infomation into vectors.
    if (!divideSmoothedPath(true)) {
        printf("divide stage failed, quit path optimization.\n");
        return false;
    };
    auto t3 = std::clock();
    // Optimize.
    bool optimization_ok = optimizePath(final_path);
    auto t4 = std::clock();
    printf("*********\n"
           "smooth phase t: %f\n"
           "divide phase t: %f\n"
           "optimize phase t: %f\n"
           "all t: %f\n"
           "*********\n\n\n",
           time_s(t1, t2),
           time_s(t2, t3),
           time_s(t3, t4),
           time_s(t1, t4));
    return optimization_ok;
}

bool PathOptimizer::samplePaths(const std::vector<double> &lon_set,
                                const std::vector<double> &lat_set,
                                std::vector<std::vector<hmpl::State>> *final_path_set) {
    if (point_num_ == 0) {
        printf("empty input, quit path optimization\n");
        return false;
    }
    // Smooth reference path.
    ReferencePathSmoother
        reference_path_smoother(points_list_, vehicle_state_.start_state_, grid_map_, config_);
    bool smoothing_ok = false;
    if (config_.smoothing_method_ == FRENET) {
        smoothing_ok = reference_path_smoother.solve<FrenetReferencePathSmoother>(&reference_path_, &smoothed_path_);
    } else if (config_.smoothing_method_ == CARTESIAN) {
        smoothing_ok = reference_path_smoother.solve<CartesianReferencePathSmoother>(&reference_path_, &smoothed_path_);
    }
    if (!smoothing_ok) {
        printf("smoothing stage failed, quit path optimization.\n");
        return false;
    }
    // Divide reference path into segments and store infomation into vectors.
    if (!divideSmoothedPath(false)) {
        printf("divide path failed!\n");
        return false;
    }
    // Sample paths for each longitudinal s.
    bool max_lon_flag = false;
    for (size_t i = 0; i != lon_set.size(); ++i) {
        if (i == lon_set.size() - 1) max_lon_flag = true;
        if (!sampleSingleLongitudinalPaths(lon_set[i], lat_set, final_path_set, max_lon_flag)) continue;
    }
    return !final_path_set->empty();
}

bool PathOptimizer::divideSmoothedPath(bool set_safety_margin) {
    if (reference_path_.max_s_ == 0) {
        LOG(INFO) << "Smoothed path is empty!";
        return false;
    }
    // Calculate the initial deviation and angle difference.
    hmpl::State first_point;
    first_point.x = reference_path_.x_s_(0);
    first_point.y = reference_path_.y_s_(0);
    first_point.z = atan2(reference_path_.y_s_.deriv(1, 0), reference_path_.x_s_.deriv(1, 0));
    auto first_point_local = hmpl::globalToLocal(vehicle_state_.start_state_, first_point);
    double min_distance = hmpl::distance(vehicle_state_.start_state_, first_point);
    if (first_point_local.y < 0) {
        vehicle_state_.initial_offset_ = min_distance;
    } else {
        vehicle_state_.initial_offset_ = -min_distance;
    }
    vehicle_state_.initial_heading_error_ = constraintAngle(vehicle_state_.start_state_.z - first_point.z);
    // If the start heading differs a lot with the ref path, quit.
    if (fabs(vehicle_state_.initial_heading_error_) > 75 * M_PI / 180) {
        LOG(WARNING) << "initial epsi is larger than 90°, quit mpc path optimization!";
        return false;
    }

    double end_distance =
        sqrt(pow(vehicle_state_.end_state_.x - reference_path_.x_s_(reference_path_.max_s_), 2) +
            pow(vehicle_state_.end_state_.y - reference_path_.y_s_(reference_path_.max_s_), 2));
    if (end_distance > 0.001) {
        // If the goal position is not the same as the end position of the reference line,
        // then find the closest point to the goal and change max_s of the reference line.
        double search_delta_s = 0;
        if (config_.exact_end_position_) {
            search_delta_s = 0.1;
        } else {
            search_delta_s = 0.3;
        }
        double tmp_s = reference_path_.max_s_ - search_delta_s;
        auto min_dis_to_goal = end_distance;
        double min_dis_s = reference_path_.max_s_;
        while (tmp_s > 0) {
            double x = reference_path_.x_s_(tmp_s);
            double y = reference_path_.y_s_(tmp_s);
            double tmp_dis = sqrt(pow(x - vehicle_state_.end_state_.x, 2) + pow(y - vehicle_state_.end_state_.y, 2));
            if (tmp_dis < min_dis_to_goal) {
                min_dis_to_goal = tmp_dis;
                min_dis_s = tmp_s;
            }
            tmp_s -= search_delta_s;
        }
        reference_path_.max_s_ = min_dis_s;
    }
    // TODO: adjust the interval according to the distance to the obstacles?
    // Divide the reference path. Intervals are smaller at the beginning.
    double delta_s_smaller = 0.3;
    // If we want to make the result path dense later, the interval here is 1.0m. This makes computation faster;
    // If we want to output the result directly, the interval is 3.0m.
    double delta_s_larger = config_.raw_result_ ? 0.3 : 1.0;
    // If the initial heading error with the reference path is small, then set intervals equal.
    if (fabs(vehicle_state_.initial_heading_error_) < 20 * M_PI / 180) delta_s_smaller = delta_s_larger;
    double tmp_max_s = delta_s_smaller;
    reference_path_.seg_s_list_.emplace_back(0);
    while (tmp_max_s < reference_path_.max_s_) {
        reference_path_.seg_s_list_.emplace_back(tmp_max_s);
        if (tmp_max_s <= 2) {
            tmp_max_s += delta_s_smaller;
        } else {
            tmp_max_s += delta_s_larger;
        }
    }
    if (reference_path_.max_s_ - reference_path_.seg_s_list_.back() > 1) {
        reference_path_.seg_s_list_.emplace_back(reference_path_.max_s_);
    }
    N_ = reference_path_.seg_s_list_.size();
    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != N_; ++i) {
        double length_on_ref_path = reference_path_.seg_s_list_[i];
        reference_path_.seg_x_list_.emplace_back(reference_path_.x_s_(length_on_ref_path));
        reference_path_.seg_y_list_.emplace_back(reference_path_.y_s_(length_on_ref_path));
        double x_d1 = reference_path_.x_s_.deriv(1, length_on_ref_path);
        double y_d1 = reference_path_.y_s_.deriv(1, length_on_ref_path);
        double x_d2 = reference_path_.x_s_.deriv(2, length_on_ref_path);
        double y_d2 = reference_path_.y_s_.deriv(2, length_on_ref_path);
        double tmp_h = atan2(y_d1, x_d1);
        double tmp_k = (x_d1 * y_d2 - y_d1 * x_d2) / pow(pow(x_d1, 2) + pow(y_d1, 2), 1.5);
        reference_path_.seg_angle_list_.emplace_back(tmp_h);
        reference_path_.seg_k_list_.emplace_back(tmp_k);
    }
    // Get clearance of covering circles.
    double rear_circle_d = config_.rear_axle_to_center_distance_ + config_.d1_;
    double front_circle_d = config_.rear_axle_to_center_distance_ + config_.d4_;
    grid_map::Position
        rear_circle_p(vehicle_state_.start_state_.x + rear_circle_d * cos(vehicle_state_.start_state_.z),
                      vehicle_state_.start_state_.y + rear_circle_d * sin(vehicle_state_.start_state_.z));
    grid_map::Position
        front_circle_p(vehicle_state_.start_state_.x + front_circle_d * cos(vehicle_state_.start_state_.z),
                       vehicle_state_.start_state_.y + front_circle_d * sin(vehicle_state_.start_state_.z));
    auto rear_circle_clearance = grid_map_.getObstacleDistance(rear_circle_p);
    auto front_circle_clearance = grid_map_.getObstacleDistance(front_circle_p);
    bool start_state_with_large_clearance =
        std::min(rear_circle_clearance, front_circle_clearance) >= config_.circle_radius_ + 0.5;
    for (size_t i = 0; i != N_; ++i) {
        hmpl::State center_state;
        center_state.x = reference_path_.seg_x_list_[i];
        center_state.y = reference_path_.seg_y_list_[i];
        center_state.s = reference_path_.seg_s_list_[i];
        center_state.z = reference_path_.seg_angle_list_[i];
        // Function getClearance uses the center position as input.
        if (config_.car_type_ == ACKERMANN_STEERING) {
            center_state.x += config_.rear_axle_to_center_distance_ * cos(center_state.z);
            center_state.y += config_.rear_axle_to_center_distance_ * sin(center_state.z);
        }
        std::vector<double> clearance;
        bool
        safety_margin_flag = set_safety_margin
            && (reference_path_.seg_s_list_[i] >= 10 || start_state_with_large_clearance);
        clearance = getClearanceFor4Circles(center_state, safety_margin_flag);
        // Terminate if collision is inevitable near the end.
        if ((clearance[0] == clearance[1] || clearance[2] == clearance[3] || clearance[4] == clearance[5]
            || clearance[6] == clearance[7])
            && center_state.s > 0.75 * reference_path_.max_s_) {
            printf("some states near end are not satisfying\n");
            N_ = i;
            config_.constraint_end_heading_ = false;
            reference_path_.seg_x_list_.erase(reference_path_.seg_x_list_.begin() + i,
                                              reference_path_.seg_x_list_.end());
            reference_path_.seg_y_list_.erase(reference_path_.seg_y_list_.begin() + i,
                                              reference_path_.seg_y_list_.end());
            reference_path_.seg_s_list_.erase(reference_path_.seg_s_list_.begin() + i,
                                              reference_path_.seg_s_list_.end());
            reference_path_.seg_k_list_.erase(reference_path_.seg_k_list_.begin() + i,
                                              reference_path_.seg_k_list_.end());
            reference_path_.seg_angle_list_.erase(reference_path_.seg_angle_list_.begin() + i,
                                                  reference_path_.seg_angle_list_.end());
            break;
        }
        reference_path_.seg_clearance_list_.emplace_back(clearance);
    }
    return true;
}

bool PathOptimizer::sampleSingleLongitudinalPaths(double lon,
                                                  const std::vector<double> &lat_set,
                                                  std::vector<std::vector<hmpl::State>> *final_path_set,
                                                  bool max_lon_flag) {
    // TODO: use provided lateral set.
    auto t1 = std::clock();
    size_t index = 0;
    for (; index != N_; ++index) {
        if (reference_path_.seg_s_list_[index] > lon) break;
    }
    ReferencePath divided_segments(reference_path_, index);
    auto N = divided_segments.seg_s_list_.size();

    auto t2 = std::clock();
    SolverInterface solver_interface(config_, reference_path_, vehicle_state_, N);
    double target_angle = divided_segments.seg_angle_list_.back();
    solver_interface.initializeSampling(target_angle, 0, 0.1);

    auto t3 = std::clock();
    size_t count = 0;
    double left_bound = divided_segments.seg_clearance_list_.back()[0];
    double right_bound = divided_segments.seg_clearance_list_.back()[1];
//    double range = (left_bound - right_bound) * 0.95;
    double range = (left_bound - right_bound);
    double reduced_range;
    // Total lateral sampling range is 6m.
    if (range >= 6) reduced_range = (range - 6) / 2;
    else reduced_range = 0;
    double interval = 0.3;
    std::vector<double> offset_set;
    for (size_t i = 0; i * interval <= range - 2 * reduced_range; ++i) {
        offset_set.emplace_back(right_bound + reduced_range + i * interval);
    }
    offset_set.emplace_back(0);
    hmpl::State sample_state;
    Eigen::VectorXd QPSolution;
//    for (size_t i = 0; i != lat_set.size(); ++i) {
    for (size_t i = 0; i != offset_set.size(); ++i) {
        sample_state.x = divided_segments.seg_x_list_.back() + offset_set[i] * cos(target_angle + M_PI_2);
        sample_state.y = divided_segments.seg_y_list_.back() + offset_set[i] * sin(target_angle + M_PI_2);
        sample_state.z = target_angle;
        if (!collision_checker_.isSingleStateCollisionFreeImproved(sample_state)) {
            printf("lon: %f, lat: %f is not feasible!\n", lon, offset_set[i]);
            continue;
        }
        if (!solver_interface.solveSampling(&QPSolution, offset_set[i])) {
            printf("solver failed at lon: %f, lat: %f!\n", lon, offset_set[i]);
            continue;
        }
        // Get single path.
        std::vector<double> result_x, result_y, result_s;
        double total_s = 0;
        double last_x = 0, last_y = 0;
        for (size_t j = 0; j != N; ++j) {
            double length_on_ref_path = divided_segments.seg_s_list_[j];
            double angle = divided_segments.seg_angle_list_[j];
            double new_angle = constraintAngle(angle + M_PI_2);
            double tmp_x = reference_path_.x_s_(length_on_ref_path) + QPSolution(2 * j + 1) * cos(new_angle);
            double tmp_y = reference_path_.y_s_(length_on_ref_path) + QPSolution(2 * j + 1) * sin(new_angle);
            result_x.emplace_back(tmp_x);
            result_y.emplace_back(tmp_y);
            if (j != 0) {
                total_s += sqrt(pow(tmp_x - last_x, 2) + pow(tmp_y - last_y, 2));
            }
            result_s.emplace_back(total_s);
            last_x = tmp_x;
            last_y = tmp_y;
        }
        tk::spline x_s, y_s;
        x_s.set_points(result_s, result_x);
        y_s.set_points(result_s, result_y);
        std::vector<hmpl::State> tmp_final_path;
        double delta_s = 0.3;
        double tmp_s = 0;
        hmpl::State tmp_state;
        for (int j = 0; tmp_s < result_s.back(); ++j) {
            tmp_s = j * delta_s;
            tmp_state.x = x_s(tmp_s);
            tmp_state.y = y_s(tmp_s);
            tmp_state.z = atan2(y_s.deriv(1, tmp_s), x_s.deriv(1, tmp_s));
            tmp_state.s = tmp_s;
            if (collision_checker_.isSingleStateCollisionFreeImproved(tmp_state)) {
                tmp_final_path.emplace_back(tmp_state);
            } else {
                printf("path optimization collision check failed at %f of %fm, lat: %f\n",
                       tmp_s,
                       result_s.back(),
                       offset_set[i]);
                break;
            }
            if ((j + 1) * delta_s > result_s.back()) {
                tmp_s = result_s.back();
                tmp_state.x = x_s(tmp_s);
                tmp_state.y = y_s(tmp_s);
                tmp_state.z = atan2(y_s.deriv(1, tmp_s), x_s.deriv(1, tmp_s));
                tmp_state.s = tmp_s;
                if (collision_checker_.isSingleStateCollisionFreeImproved(tmp_state)) {
                    tmp_final_path.emplace_back(tmp_state);
                }
            }
        }
        if (!tmp_final_path.empty()) {
            final_path_set->emplace_back(tmp_final_path);
            ++count;
        }
    }

    auto t4 = std::clock();
    printf("got %d paths at %fm\n", static_cast<int>(count), lon);
    printf("**********\n"
           "preprocess: %f\n"
           "solver init: %f\n"
           "solve: %f\n"
           "all: %f\n"
           "**********\n",
           time_s(t1, t2),
           time_s(t2, t3),
           time_s(t3, t4),
           time_s(t1, t4));
    return true;
}

bool PathOptimizer::optimizePath(std::vector<hmpl::State> *final_path) {
    if (reference_path_.max_s_ == 0) {
        LOG(INFO) << "path optimization input is empty!";
        return false;
    }

    // Solve problem.
    Eigen::VectorXd QPSolution;
    SolverInterface solver_interface(config_,
                                     reference_path_,
                                     vehicle_state_,
                                     N_);
    if (!solver_interface.solve(&QPSolution)) {
        return false;
    }

    // Output. Choose from:
    // 1. set the interval smaller and output the result directly.
    // 2. set the interval larger and use interpolation to make the result dense.
    std::vector<double> result_x, result_y, result_s;
    double total_s = 0;
    double last_x = 0, last_y = 0;
    std::vector<hmpl::State> tmp_raw_path;
    for (size_t i = 0; i != N_; ++i) {
//        std::cout << "cuvature contuol: " << QPSolution(2 * N_ + i) << std::endl;
        double length_on_ref_path = reference_path_.seg_s_list_[i];
        double angle = reference_path_.seg_angle_list_[i];
        double new_angle = constraintAngle(angle + M_PI_2);
        double tmp_x = reference_path_.x_s_(length_on_ref_path) + QPSolution(2 * i + 1) * cos(new_angle);
        double tmp_y = reference_path_.y_s_(length_on_ref_path) + QPSolution(2 * i + 1) * sin(new_angle);
        if (config_.raw_result_) {
            // If output raw path:
            hmpl::State state;
            state.x = tmp_x;
            state.y = tmp_y;
            state.z = angle + QPSolution(2 * i);
            if (collision_checker_.isSingleStateCollisionFreeImproved(state)) {
                tmp_raw_path.emplace_back(state);
            } else {
                tmp_raw_path.emplace_back(state);
                std::cout << "path optimization collision check failed at " << i << std::endl;
                break;
            }
        }
        result_x.emplace_back(tmp_x);
        result_y.emplace_back(tmp_y);
        if (i != 0) {
            total_s += sqrt(pow(tmp_x - last_x, 2) + pow(tmp_y - last_y, 2));
        }
        result_s.emplace_back(total_s);
        last_x = tmp_x;
        last_y = tmp_y;
    }
    std::vector<double> curvature_result;
    if (config_.raw_result_) {
        std::copy(tmp_raw_path.begin(), tmp_raw_path.end(), std::back_inserter(*final_path));
        LOG(INFO) << "output raw path!";
        return true;
    }
    // If output interpolated path:
    tk::spline x_s, y_s;
    x_s.set_points(result_s, result_x);
    y_s.set_points(result_s, result_y);
    std::vector<hmpl::State> tmp_final_path;
    double delta_s = 0.3;
    for (int i = 0; i * delta_s <= result_s.back(); ++i) {
        hmpl::State tmp_state;
        tmp_state.x = x_s(i * delta_s);
        tmp_state.y = y_s(i * delta_s);
        tmp_state.z = atan2(y_s.deriv(1, i * delta_s), x_s.deriv(1, i * delta_s));
        tmp_state.s = i * delta_s;
        if (collision_checker_.isSingleStateCollisionFreeImproved(tmp_state)) {
            tmp_final_path.emplace_back(tmp_state);
        } else {
            std::cout << "path optimization collision check failed at " << i << std::endl;
            break;
        }
    }
    final_path->clear();
    std::copy(tmp_final_path.begin(), tmp_final_path.end(), std::back_inserter(*final_path));
    return true;
}

bool PathOptimizer::optimizeDynamic(const std::vector<double> &sr_list,
                                    const std::vector<std::vector<double>> &clearance_list,
                                    std::vector<double> *x_list,
                                    std::vector<double> *y_list,
                                    std::vector<double> *s_list) {
    auto N = sr_list.size();
    if (!solver_dynamic_initialized) {
        std::vector<double> x_set, y_set, s_set;
        for (const auto &point : points_list_) {
            x_set.emplace_back(point.x);
            y_set.emplace_back(point.y);
            s_set.emplace_back(point.s);
        }
        xsr_.set_points(s_set, x_set);
        ysr_.set_points(s_set, y_set);
        std::vector<double> angle_list, k_list;
        for (size_t i = 0; i != sr_list.size(); ++i) {
            double tmp_s = sr_list[i];
            double x_d1 = xsr_.deriv(1, tmp_s);
            double y_d1 = ysr_.deriv(1, tmp_s);
            double x_d2 = xsr_.deriv(2, tmp_s);
            double y_d2 = ysr_.deriv(2, tmp_s);
            double h = atan2(y_d1, x_d1);
            double k = (x_d1 * y_d2 - y_d1 * x_d2) / pow(pow(x_d1, 2) + pow(y_d1, 2), 1.5);
            angle_list.emplace_back(h);
            k_list.emplace_back(k);
        }
        reference_path_.seg_angle_list_ = std::move(angle_list);
        reference_path_.seg_k_list_ = std::move(k_list);
        reference_path_.seg_s_list_.assign(sr_list.cbegin(), sr_list.cend());
        reference_path_.seg_clearance_list_.assign(clearance_list.cbegin(), clearance_list.cend());
        // Initialize solver.
        dynamic_solver_ptr = std::make_shared<SolverInterface>(config_, reference_path_, vehicle_state_, N);
        Eigen::VectorXd QPSolution;
        dynamic_solver_ptr->solveDynamic(&QPSolution);
        double total_s = 0;
        for (size_t j = 0; j != N; ++j) {
            double length_on_ref_path = sr_list[j];
            double angle = reference_path_.seg_angle_list_[j];
            double new_angle = constraintAngle(angle + M_PI_2);
            double tmp_x = xsr_(length_on_ref_path) + QPSolution(2 * j + 1) * cos(new_angle);
            double tmp_y = ysr_(length_on_ref_path) + QPSolution(2 * j + 1) * sin(new_angle);
            if (j != 0) {
                total_s += sqrt(pow(tmp_x - x_list->back(), 2) + pow(tmp_y - y_list->back(), 2));
            }
            s_list->emplace_back(total_s);
            x_list->emplace_back(tmp_x);
            y_list->emplace_back(tmp_y);
        }
        solver_dynamic_initialized = true;
        return true;
    } else {
        Eigen::VectorXd QPSolution;
        dynamic_solver_ptr->solveDynamicUpdate(&QPSolution, clearance_list);
        double total_s = 0;
        for (size_t j = 0; j != N; ++j) {
            double length_on_ref_path = sr_list[j];
            double x_d1 = xsr_.deriv(1, length_on_ref_path);
            double y_d1 = ysr_.deriv(1, length_on_ref_path);
            double angle = atan2(y_d1, x_d1);
            double new_angle = constraintAngle(angle + M_PI_2);
            double tmp_x = xsr_(length_on_ref_path) + QPSolution(2 * j + 1) * cos(new_angle);
            double tmp_y = ysr_(length_on_ref_path) + QPSolution(2 * j + 1) * sin(new_angle);
            if (j != 0) {
                total_s += sqrt(pow(tmp_x - x_list->back(), 2) + pow(tmp_y - y_list->back(), 2));
            }
            s_list->emplace_back(total_s);
            x_list->emplace_back(tmp_x);
            y_list->emplace_back(tmp_y);
        }
        return true;
    }
}

std::vector<double> PathOptimizer::getClearanceWithDirectionStrict(hmpl::State state,
                                                                   double radius,
                                                                   bool safety_margin_flag) const {
    double left_bound = 0;
    double right_bound = 0;
    double delta_s = 0.2;
    double left_angle = constraintAngle(state.z + M_PI_2);
    double right_angle = constraintAngle(state.z - M_PI_2);
    auto n = static_cast<size_t >(5.0 / delta_s);
    // Check if the original position is collision free.
    grid_map::Position original_position(state.x, state.y);
    auto original_clearance = grid_map_.getObstacleDistance(original_position);
    if (original_clearance > radius) {
        // Normal case:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = grid_map_.getObstacleDistance(new_position);
            if (clearance < radius) {
                break;
            }
        }
        double left_s = 0;
        for (size_t j = 0; j != n; ++j) {
            left_s += delta_s;
            double x = state.x + left_s * cos(left_angle);
            double y = state.y + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance = grid_map_.getObstacleDistance(new_position);
            if (clearance < radius) {
                break;
            }
        }
        right_bound = -(right_s - delta_s);
        left_bound = left_s - delta_s;
    } else {
        // Collision already; pick one side to expand:
        double right_s = 0;
        for (size_t j = 0; j != n; ++j) {
            right_s += delta_s;
            double x = state.x + right_s * cos(right_angle);
            double y = state.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = grid_map_.getObstacleDistance(new_position);
            if (clearance > radius) {
                break;
            }
        }
        double left_s = 0;
        for (size_t j = 0; j != n; ++j) {
            left_s += delta_s;
            double x = state.x + left_s * cos(left_angle);
            double y = state.y + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance = grid_map_.getObstacleDistance(new_position);
            if (clearance > radius) {
                break;
            }
        }
        // Compare
        if (left_s < right_s) {
            // Pick left side:
            right_bound = left_s;
            for (size_t j = 0; j != n; ++j) {
                left_s += delta_s;
                double x = state.x + left_s * cos(left_angle);
                double y = state.y + left_s * sin(left_angle);
                grid_map::Position new_position(x, y);
                double clearance = grid_map_.getObstacleDistance(new_position);
                if (clearance < radius) {
                    break;
                }
            }
            left_bound = left_s - delta_s;
        } else {
            // Pick right side:
            left_bound = -right_s;
            for (size_t j = 0; j != n; ++j) {
                right_s += delta_s;
                double x = state.x + right_s * cos(right_angle);
                double y = state.y + right_s * sin(right_angle);
                grid_map::Position new_position(x, y);
                double clearance = grid_map_.getObstacleDistance(new_position);
                if (clearance < radius) {
                    break;
                }
            }
            right_bound = -(right_s - delta_s);
        }
    }
    double base = std::max(left_bound - right_bound - 0.6, 0.0);
    if (safety_margin_flag) {
        double safety_margin = std::min(base * 0.2, 0.5);
        left_bound -= safety_margin;
        right_bound += safety_margin;
    }
    std::vector<double> result{left_bound, right_bound};
    return result;
}

std::vector<double> PathOptimizer::getClearanceFor4Circles(const hmpl::State &state, bool safety_margin_flag) {
    double circle_r = config_.circle_radius_;
    hmpl::State c0, c1, c2, c3;
    double center_x = state.x;
    double center_y = state.y;
    double c0_x = center_x + config_.d1_ * cos(state.z);
    double c0_y = center_y + config_.d1_ * sin(state.z);
    double c1_x = center_x + config_.d2_ * cos(state.z);
    double c1_y = center_y + config_.d2_ * sin(state.z);
    double c2_x = center_x + config_.d3_ * cos(state.z);
    double c2_y = center_y + config_.d3_ * sin(state.z);
    double c3_x = center_x + config_.d4_ * cos(state.z);
    double c3_y = center_y + config_.d4_ * sin(state.z);
    c0.x = c0_x;
    c0.y = c0_y;
    c0.z = state.z;
    c1.x = c1_x;
    c1.y = c1_y;
    c1.z = state.z;
    c2.x = c2_x;
    c2.y = c2_y;
    c2.z = state.z;
    c3.x = c3_x;
    c3.y = c3_y;
    c3.z = state.z;
    std::vector<double> result;
    std::vector<double> c0_bounds = getClearanceWithDirectionStrict(c0, circle_r, safety_margin_flag);
    std::vector<double> c1_bounds = getClearanceWithDirectionStrict(c1, circle_r, safety_margin_flag);
    std::vector<double> c2_bounds = getClearanceWithDirectionStrict(c2, circle_r, safety_margin_flag);
    std::vector<double> c3_bounds = getClearanceWithDirectionStrict(c3, circle_r, safety_margin_flag);
    result.emplace_back(c0_bounds[0]);
    result.emplace_back(c0_bounds[1]);
    result.emplace_back(c1_bounds[0]);
    result.emplace_back(c1_bounds[1]);
    result.emplace_back(c2_bounds[0]);
    result.emplace_back(c2_bounds[1]);
    result.emplace_back(c3_bounds[0]);
    result.emplace_back(c3_bounds[1]);
    // For visualization purpoose:
    hmpl::State rear_bound_l, center_bound_l, front_bound_l, rear_bound_r, center_bound_r, front_bound_r;
    rear_bound_l.x = c0_x + result[0] * cos(state.z + M_PI_2);
    rear_bound_l.y = c0_y + result[0] * sin(state.z + M_PI_2);
    rear_bound_r.x = c0_x + result[1] * cos(state.z + M_PI_2);
    rear_bound_r.y = c0_y + result[1] * sin(state.z + M_PI_2);
    center_bound_l.x = c2_x + result[4] * cos(state.z + M_PI_2);
    center_bound_l.y = c2_y + result[4] * sin(state.z + M_PI_2);
    center_bound_r.x = c2_x + result[5] * cos(state.z + M_PI_2);
    center_bound_r.y = c2_y + result[5] * sin(state.z + M_PI_2);
    front_bound_l.x = c3_x + result[6] * cos(state.z + M_PI_2);
    front_bound_l.y = c3_y + result[6] * sin(state.z + M_PI_2);
    front_bound_r.x = c3_x + result[7] * cos(state.z + M_PI_2);
    front_bound_r.y = c3_y + result[7] * sin(state.z + M_PI_2);
    rear_bounds_.emplace_back(rear_bound_l);
    rear_bounds_.emplace_back(rear_bound_r);
    center_bounds_.emplace_back(center_bound_l);
    center_bounds_.emplace_back(center_bound_r);
    front_bounds_.emplace_back(front_bound_l);
    front_bounds_.emplace_back(front_bound_r);
    return result;

}

const std::vector<hmpl::State> &PathOptimizer::getLeftBound() const {
    return this->left_bound_;
}

const std::vector<hmpl::State> &PathOptimizer::getRightBound() const {
    return this->right_bound_;
}

const std::vector<hmpl::State> &PathOptimizer::getSecondThirdPoint() const {
    return this->second_third_point_;
}

const std::vector<hmpl::State> &PathOptimizer::getRearBounds() const {
    return this->rear_bounds_;
}

const std::vector<hmpl::State> &PathOptimizer::getCenterBounds() const {
    return this->center_bounds_;
}

const std::vector<hmpl::State> &PathOptimizer::getFrontBounds() const {
    return this->front_bounds_;
}

const std::vector<hmpl::State> &PathOptimizer::getSmoothedPath() const {
    return this->smoothed_path_;
}
}
