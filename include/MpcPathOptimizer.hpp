//
// Created by ljn on 19-8-16.
//

#ifndef MPC_PATH_OPTIMIZER__MPCPATHOPTIMIZER_HPP_
#define MPC_PATH_OPTIMIZER__MPCPATHOPTIMIZER_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <chrono>
#include <Clothoid.hh>
#include <ClothoidList.hh>
#include <opt_utils/utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include "tinyspline_ros/tinysplinecpp.h"
#include "spline.h"
#include "FgEvalFrenet.hpp"
#include "collosion_checker.hpp"

#define MAX_CURVATURE 0.25

namespace MpcSmoother {

enum CarType {ACKERMANN_STEERING = 0, SKID_STEERING = 1,};

class MpcPathOptimizer {
public:
    MpcPathOptimizer() = delete;
    MpcPathOptimizer(const std::vector<hmpl::State> &points_list,
                     const hmpl::State &start_state,
                     const hmpl::State &end_state,
                     const hmpl::InternalGridMap &map);
    bool solve(std::vector<hmpl::State> *final_path);
    const std::vector<std::vector<hmpl::State> > &getControlSamplingPathSet();
    // Just for visualization purpose.
    const std::vector<std::vector<hmpl::State> > &getControlSamplingFailedPathSet();
    const std::vector<hmpl::State> &getBestSamplingPath();

private:
    void getCurvature(const std::vector<double> &local_x,
                      const std::vector<double> &local_y,
                      std::vector<double> *pt_curvature_out,
                      double *max_curvature_abs,
                      double *max_curvature_change_abs);
    double getPointCurvature(const double &x1, const double &y1,
                             const double &x2, const double &y2,
                             const double &x3, const double &y3);
    double getClearanceWithDirection(const hmpl::State &state,
                                     double angle,
                                     const std::vector<double> &car_geometry);
    double getClearanceWithDirection(const hmpl::State &state,
                                     double angle);
    std::vector<double> getClearance(hmpl::State state,
                                     double ref_angle,
                                     const std::vector<double> &car_geometry);
    // Set angle range to -pi ~ pi.
    inline double constraintAngle(double angle) {
        if (angle > M_PI) {
            angle -= 2 * M_PI;
            return constraintAngle(angle);
        } else if (angle < -M_PI) {
            angle += 2 * M_PI;
            return constraintAngle(angle);
        } else {
            return angle;
        }
    }

    hmpl::InternalGridMap grid_map_;
    CollisionChecker collision_checker_;
    CarType car_type;
    // rear_axle_to_center_dis is needed only when using ackermann steering.
    double rear_axle_to_center_dis;


    std::vector<hmpl::State> points_list_;
    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> k_list_;
    std::vector<double> s_list_;
    std::vector<double> seg_s_list_;
    std::vector<double> seg_k_list_;
    std::vector<double> seg_x_list_;
    std::vector<double> seg_y_list_;
    std::vector<double> seg_angle_list_;
    std::vector<double> seg_clearance_left_list_;
    std::vector<double> seg_clearance_right_list_;


    size_t point_num_;
    hmpl::State start_state_;
    hmpl::State end_state_;
    // x_spline and y_spline are in global frame
    tk::spline x_spline_;
    tk::spline y_spline_;
    tk::spline k_spline_;

    std::vector<std::vector<double> > predicted_path_in_frenet_;
    std::vector<std::vector<hmpl::State> > sampling_path_set_;
    std::vector<std::vector<hmpl::State> > failed_sampling_path_set_;
    size_t best_sampling_index_;
};

}

#endif //MPC_PATH_OPTIMIZER__MPCPATHOPTIMIZER_HPP_
