//
// Created by ljn on 19-8-16.
//

#ifndef PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
#define PATH_OPTIMIZER__PATHOPTIMIZER_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <memory>
#include <tinyspline_ros/tinysplinecpp.h>
#include "data_struct/data_struct.hpp"
#include "reference_path_smoother/reference_path_smoother.hpp"
#include "reference_path_smoother/frenet_reference_path_smoother.hpp"
#include "reference_path_smoother/cartesian_reference_path_smoother.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"
#include "tools/collosion_checker.hpp"
#include "config/config.hpp"
#include "solver_interface.hpp"
#include "tools/Map.hpp"

namespace PathOptimizationNS {

class PathOptimizer {
public:
    PathOptimizer() = delete;
    PathOptimizer(const std::vector<State> &points_list,
                  const State &start_state,
                  const State &end_state,
                  const grid_map::GridMap &map);
    
    // Call this to get the optimized path.
    bool solve(std::vector<State> *final_path);

    // Incomplete:
    // Sample a set of candidate paths of various longitudinal distance and lateral offset.
    // Note that it might be very slow if "densify_path" is set to false.
    bool samplePaths(const std::vector<double> &lon_set,
                     const std::vector<double> &lat_set,
                     std::vector<std::vector<State>> *final_path_set);

    // Incomplete:
    // For dynamic obstacle avoidance. Please Ignore this.
    bool optimizeDynamic(const std::vector<double> &sr_list,
                         const std::vector<std::vector<double>> &clearance_list,
                         std::vector<double> *x_list,
                         std::vector<double> *y_list,
                         std::vector<double> *s_list);

    // Only for visualization purpose.
    // TODO: some of these functions are no longer used.
    const std::vector<State> &getLeftBound() const;
    const std::vector<State> &getRightBound() const;
    const std::vector<State> &getSecondThirdPoint() const;
    const std::vector<State> &getRearBounds() const;
    const std::vector<State> &getCenterBounds() const;
    const std::vector<State> &getFrontBounds() const;
    const std::vector<State> &getSmoothedPath() const;
    std::vector<std::vector<double>> a_star_display_;

private:
    // TODO: abandon this function, use the config class instead.
    void setConfig();
    
    // Core function.
    bool optimizePath(std::vector<State> *final_path);
    
    // Generate a set of paths with the same longitudinal length on reference line.
    bool sampleSingleLongitudinalPaths(double lon,
                                       const std::vector<double> &lat_set,
                                       std::vector<std::vector<State>> *final_path_set,
                                       bool max_lon_flag);
    
    // Get bounds for each circle at each sampling point.
    std::vector<double> getClearanceWithDirectionStrict(State state,
                                                        double radius,
                                                        bool safety_margin_flag) const;
    std::vector<double> getClearanceFor4Circles(const State &state, bool safety_margin_flag);
    
    // Divide smoothed path into segments.
    bool divideSmoothedPath(bool safety_margin_flag);

    const Map grid_map_;
    CollisionChecker collision_checker_;
    Config config_;
    ReferencePath reference_path_;
    VehicleState vehicle_state_;
    size_t N_;
    // Input path
    std::vector<State> points_list_;
    size_t point_num_;
    // For dynamic obstacle avoidace. Please Ignore this.
    std::shared_ptr<SolverInterface> dynamic_solver_ptr;
    bool solver_dynamic_initialized;
    tk::spline xsr_, ysr_;
    // For visualization purpose.
    std::vector<State> left_bound_;
    std::vector<State> right_bound_;
    std::vector<State> second_third_point_;
    std::vector<State> empty_;
    std::vector<State> rear_bounds_;
    std::vector<State> center_bounds_;
    std::vector<State> front_bounds_;
    std::vector<State> smoothed_path_;
};

}

#endif //PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
