//
// Created by ljn on 19-8-16.
//

#ifndef PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
#define PATH_OPTIMIZER__PATHOPTIMIZER_HPP_

#include <iostream>
#include <ios>
#include <iomanip>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <chrono>
#include <Clothoid.hh>
#include <opt_utils/utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <tinyspline_ros/tinysplinecpp.h>
#include <OsqpEigen/OsqpEigen.h>
#include "data_struct/data_struct.hpp"
#include "reference_path_smoother/reference_path_smoother.hpp"
#include "tools/spline.h"
#include "tools/tools.hpp"
#include "tools/collosion_checker.hpp"
#include "config/config.hpp"
#include "solver_interface.hpp"

namespace PathOptimizationNS {

class PathOptimizer {
public:
    PathOptimizer() = delete;
    PathOptimizer(const std::vector<hmpl::State> &points_list,
                  const hmpl::State &start_state,
                  const hmpl::State &end_state,
                  const hmpl::InternalGridMap &map,
                  bool densify_path = true);
    
    // Call this to get the optimized path.
    bool solve(std::vector<hmpl::State> *final_path);

    // Only for test:
    // Sample a set of candidate paths of various longitudinal distance and lateral offset.
    // Note that it might be very slow if "densify_path" is set to false.
    bool samplePaths(const std::vector<double> &lon_set,
                     const std::vector<double> &lat_set,
                     std::vector<std::vector<hmpl::State>> *final_path_set);

    // For dynamic obstacle avoidance.
    bool optimizeDynamic(const std::vector<double> &sr_list,
                         const std::vector<std::vector<double>> &clearance_list,
                         std::vector<double> *x_list,
                         std::vector<double> *y_list,
                         std::vector<double> *s_list);

    // Only for visualization purpose.
    // TODO: some of these functions are no longer used.
    const std::vector<hmpl::State> &getLeftBound() const;
    const std::vector<hmpl::State> &getRightBound() const;
    const std::vector<hmpl::State> &getSecondThirdPoint() const;
    const std::vector<hmpl::State> &getRearBounds() const;
    const std::vector<hmpl::State> &getCenterBounds() const;
    const std::vector<hmpl::State> &getFrontBounds() const;
    const std::vector<hmpl::State> &getSmoothedPath() const;

private:
    // TODO: abandon this function, use the config class instead.
    void setConfig();
    
    // Core function.
    bool optimizePath(std::vector<hmpl::State> *final_path);
    
    // Generate a set of paths with the same longitudinal length on reference line.
    bool sampleSingleLongitudinalPaths(double lon,
                                       const std::vector<double> &lat_set,
                                       std::vector<std::vector<hmpl::State>> *final_path_set,
                                       bool max_lon_flag);
    
    // Get bounds for each circle at each sampling point.
    std::vector<double> getClearanceWithDirectionStrict(hmpl::State state,
                                                        double radius,
                                                        bool safety_margin_flag) const;
    std::vector<double> getClearanceFor4Circles(const hmpl::State &state, bool safety_margin_flag);
    
    // Divide smoothed path into segments.
    bool divideSmoothedPath(bool safety_margin_flag);

    hmpl::InternalGridMap grid_map_;
    CollisionChecker collision_checker_;
    Config config_;
    ReferencePath reference_path_;
    VehicleState vehicle_state_;
    size_t N_;
    // Input path
    std::vector<hmpl::State> points_list_;
    size_t point_num_;
    // For dynamic obstacle avoidace.
    OsqpEigen::Solver solver_dynamic;
    bool solver_dynamic_initialized;
    tk::spline xsr_, ysr_;
    // For visualization purpose.
    std::vector<hmpl::State> left_bound_;
    std::vector<hmpl::State> right_bound_;
    std::vector<hmpl::State> second_third_point_;
    std::vector<hmpl::State> empty_;
    std::vector<hmpl::State> rear_bounds_;
    std::vector<hmpl::State> center_bounds_;
    std::vector<hmpl::State> front_bounds_;
    std::vector<hmpl::State> smoothed_path_;
};

}

#endif //PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
