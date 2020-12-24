//
// Created by ljn on 19-8-16.
//
#include <iostream>
#include <cmath>
#include <ctime>
#include "path_optimizer/path_optimizer.hpp"
#include "path_optimizer/reference_path_smoother/reference_path_smoother.hpp"
#include "path_optimizer/tools/tools.hpp"
#include "path_optimizer/data_struct/reference_path.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"
#include "path_optimizer/data_struct/vehicle_state_frenet.hpp"
#include "path_optimizer/tools/collosion_checker.hpp"
#include "path_optimizer/tools/Map.hpp"
#include "path_optimizer/tools/spline.h"
#include "path_optimizer/solver/solver.hpp"
#include "tinyspline_ros/tinysplinecpp.h"
#include "path_optimizer/reference_path_smoother/angle_diff_smoother.hpp"
#include "path_optimizer/reference_path_smoother/tension_smoother.hpp"

namespace PathOptimizationNS {

PathOptimizer::PathOptimizer(const State &start_state,
                             const State &end_state,
                             const grid_map::GridMap &map) :
    grid_map_(new Map{map}),
    collision_checker_(new CollisionChecker{map}),
    reference_path_(new ReferencePath),
    vehicle_state_(new VehicleState{start_state, end_state, 0, 0}) {
    updateConfig();
}

PathOptimizer::~PathOptimizer() {
    delete grid_map_;
    delete collision_checker_;
    delete reference_path_;
    delete vehicle_state_;
}

bool PathOptimizer::solve(const std::vector<State> &reference_points, std::vector<State> *final_path) {
    if (FLAGS_enable_computation_time_output) std::cout << "------" << std::endl;
    CHECK_NOTNULL(final_path);

    auto t1 = std::clock();
    if (reference_points.empty()) {
        LOG(ERROR) << "Empty input, quit path optimization";
        return false;
    }
    reference_path_->clear();

    // Smooth reference path.
    auto reference_path_smoother = ReferencePathSmoother::create(FLAGS_smoothing_method,
                                                                 reference_points,
                                                                 vehicle_state_->getStartState(),
                                                                 *grid_map_);
    bool smoothing_ok = reference_path_smoother->solve(reference_path_);
    if (!smoothing_ok) {
        LOG(ERROR) << "Path optimization FAILED!";
        return false;
    }

    auto t2 = std::clock();
    // Divide reference path into segments;
    if (!segmentSmoothedPath()) {
        LOG(ERROR) << "Path optimization FAILED!";
        return false;
    }

    auto t3 = std::clock();
    // Optimize.
    if (optimizePath(final_path)) {
        auto t4 = std::clock();
        if (FLAGS_enable_computation_time_output) {
            time_ms_out(t1, t2, "Reference smoothing");
            time_ms_out(t2, t3, "Reference segmentation");
            time_ms_out(t3, t4, "Optimization phase");
            time_ms_out(t1, t4, "All");
        }
        LOG(INFO) << "Path optimization SUCCEEDED! Total time cost: " << time_s(t1, t4) << " s";
        return true;
    } else {
        LOG(ERROR) << "Path optimization FAILED!";
        return false;
    }
}

bool PathOptimizer::solveWithoutSmoothing(const std::vector<PathOptimizationNS::State> &reference_points,
                                          std::vector<PathOptimizationNS::State> *final_path) {
    // This function is used to calculate once more based on the previous result.
    if (FLAGS_enable_computation_time_output) std::cout << "------" << std::endl;
    CHECK_NOTNULL(final_path);
    auto t1 = std::clock();
    if (reference_points.empty()) {
        LOG(ERROR) << "Empty input, quit path optimization!";
        return false;
    }
    vehicle_state_->setInitError(0, 0);
    // Set reference path.
    reference_path_->clear();
    reference_path_->setReference(reference_points);
    reference_path_->updateBounds(*grid_map_);
    reference_path_->updateLimits();
    size_ = reference_path_->getSize();

    if (optimizePath(final_path)) {
        auto t2 = std::clock();
        if (FLAGS_enable_computation_time_output) {
            time_ms_out(t1, t2, "Solve without smoothing");
        }
        LOG(INFO) << "Path optimization without smoothing SUCCEEDED! Total time cost: "
                  << time_s(t1, t2) << " s";
        return true;
    } else {
        LOG(ERROR) << "Path optimization without smoothing FAILED!";
        return false;
    }
}

bool PathOptimizer::segmentSmoothedPath() {
    if (reference_path_->getLength() == 0) {
        LOG(ERROR) << "Smoothed path is empty!";
        return false;
    }

    // Calculate the initial deviation and the angle difference.
    State first_point;
    first_point.x = reference_path_->getXS(0);
    first_point.y = reference_path_->getYS(0);
    first_point.z = getHeading(reference_path_->getXS(), reference_path_->getYS(), 0);
    auto first_point_local = global2Local(vehicle_state_->getStartState(), first_point);
    // In reference smoothing, the closest point to the vehicle is found and set as the
    // first point. So the distance here is simply the initial offset.
    double min_distance = distance(vehicle_state_->getStartState(), first_point);
    double initial_offset = first_point_local.y < 0 ? min_distance : -min_distance;
    double initial_heading_error = constraintAngle(vehicle_state_->getStartState().z - first_point.z);
    // If the start heading differs a lot with the ref path, quit.
    if (fabs(initial_heading_error) > 75 * M_PI / 180) {
        LOG(ERROR) << "Initial psi error is larger than 75°, quit path optimization!";
        return false;
    }
    vehicle_state_->setInitError(initial_offset, initial_heading_error);

    double end_distance =
        sqrt(pow(vehicle_state_->getEndState().x - reference_path_->getXS(reference_path_->getLength()), 2) +
            pow(vehicle_state_->getEndState().y - reference_path_->getYS(reference_path_->getLength()), 2));
    if (!isEqual(end_distance, 0)) {
        // If the goal position is not the same as the end position of the reference line,
        // then find the closest point to the goal and change max_s of the reference line.
        double search_delta_s = FLAGS_enable_exact_position ? 0.1 : 0.5;
        double tmp_s = reference_path_->getLength() - search_delta_s;
        auto min_dis_to_goal = end_distance;
        double min_dis_s = reference_path_->getLength();
        while (tmp_s > 0) {
            double x = reference_path_->getXS(tmp_s);
            double y = reference_path_->getYS(tmp_s);
            double tmp_dis =
                sqrt(pow(x - vehicle_state_->getEndState().x, 2) + pow(y - vehicle_state_->getEndState().y, 2));
            if (tmp_dis < min_dis_to_goal) {
                min_dis_to_goal = tmp_dis;
                min_dis_s = tmp_s;
            }
            if (tmp_dis > 8 && min_dis_to_goal < 8) break;
            tmp_s -= search_delta_s;
        }
        reference_path_->setLength(min_dis_s);
    }

    // If we want to make the result path dense by interpolation later, the interval here is 1.0m. This makes computation faster, but
    // may fail the collision check due to the large interval.
    // If we want to output the result directly, the interval is controlled by FLAGS_output_spacing.
    const double delta_s_smaller = FLAGS_enable_raw_output ? 0.15 : 0.5;
    const double delta_s_larger = FLAGS_enable_raw_output ? FLAGS_output_spacing : 1.0;
    reference_path_->buildReferenceFromSpline(delta_s_smaller, delta_s_larger);
    reference_path_->updateBounds(*grid_map_);
    reference_path_->updateLimits();
    size_ = reference_path_->getSize();
    return true;
}

bool PathOptimizer::optimizePath(std::vector<State> *final_path) {
    // Solve problem.
    auto solver = OsqpSolver::create(FLAGS_optimization_method, *reference_path_, *vehicle_state_, size_);
    if (solver && !solver->solve(final_path)) {
        LOG(ERROR) << "QP failed.";
        return false;
    }

    // Output. Choose from:
    // 1. set the interval smaller and output the result directly.
    // 2. set the interval larger and use interpolation to make the result dense.
    if (FLAGS_enable_raw_output) {
        double s{0};
        for (auto iter = final_path->begin(); iter != final_path->end(); ++iter) {
            if (iter != final_path->begin()) s += distance(*(iter - 1), *iter);
            iter->s = s;
            if (FLAGS_enable_collision_check && !collision_checker_->isSingleStateCollisionFreeImproved(*iter)) {
                final_path->erase(iter, final_path->end());
                LOG(ERROR) << "collision check failed at " << final_path->back().s << "m.";
                return final_path->back().s >= 20;
            }
        }
        return true;
    } else {
        std::vector<double> result_x, result_y, result_s;
        for (const auto &p : *final_path) {
            result_x.emplace_back(p.x);
            result_y.emplace_back(p.y);
            result_s.emplace_back(p.s);
        }
        tk::spline x_s, y_s;
        x_s.set_points(result_s, result_x);
        y_s.set_points(result_s, result_y);
        final_path->clear();
        double delta_s = FLAGS_output_spacing;
        for (int i = 0; i * delta_s <= result_s.back(); ++i) {
            double tmp_s = i * delta_s;
            State tmp_state{x_s(tmp_s),
                            y_s(tmp_s),
                            getHeading(x_s, y_s, tmp_s),
                            getCurvature(x_s, y_s, tmp_s),
                            tmp_s};
            if (FLAGS_enable_collision_check && !collision_checker_->isSingleStateCollisionFreeImproved(tmp_state)) {
                LOG(ERROR) << "[PathOptimizer] collision check failed at " << final_path->back().s << "m.";
                return final_path->back().s >= 20;
            }
            final_path->emplace_back(tmp_state);
        }
        LOG(INFO) << "Output densified result.";
        return true;
    }
}

std::vector<std::tuple<State, double, double>> PathOptimizer::display_abnormal_bounds() const {
    return this->reference_path_->display_abnormal_bounds();
}

const ReferencePath& PathOptimizer::getReferencePath() const {
    return *reference_path_;
}
}
