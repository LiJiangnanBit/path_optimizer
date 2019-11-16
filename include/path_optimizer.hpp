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
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <chrono>
#include <Clothoid.hh>
//#include <Cl n1othoidList.hh>
#include <opt_utils/utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include "tinyspline_ros/tinysplinecpp.h"
#include "spline.h"
#include "FgEvalFrenet.hpp"
#include "FgEvalFrenetSmooth.hpp"
#include "collosion_checker.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#define MAX_CURVATURE 0.25

namespace PathOptimizationNS {

enum CarType { ACKERMANN_STEERING = 0, SKID_STEERING = 1, };

class PathOptimizer {
public:
    PathOptimizer() = delete;
    PathOptimizer(const std::vector<hmpl::State> &points_list,
                     const hmpl::State &start_state,
                     const hmpl::State &end_state,
                     const hmpl::InternalGridMap &map);
    bool solve(std::vector<hmpl::State> *final_path);
    const std::vector<std::vector<hmpl::State> > &getControlSamplingPathSet();
    // Just for visualization purpose.
    const std::vector<std::vector<hmpl::State> > &getControlSamplingFailedPathSet();
    const std::vector<hmpl::State> &getBestSamplingPath();
    const std::vector<hmpl::State> &getLeftBound();
    const std::vector<hmpl::State> &getRightBound();
    const std::vector<hmpl::State> &getSecondThirdPoint();
    const std::vector<hmpl::State> &getRearBounds();
    const std::vector<hmpl::State> &getCenterBounds();
    const std::vector<hmpl::State> &getFrontBounds();
    const std::vector<hmpl::State> &getSmoothedPath();


private:
    void reset(const std::vector<hmpl::State> &points_list);
    bool smoothPath(std::vector<hmpl::State> *smoothed_path);
    bool optimizePath(std::vector<hmpl::State> *final_path);
    void getCurvature(const std::vector<double> &local_x,
                      const std::vector<double> &local_y,
                      std::vector<double> *pt_curvature_out,
                      double *max_curvature_abs,
                      double *max_curvature_change_abs);
    double getPointCurvature(const double &x1, const double &y1,
                             const double &x2, const double &y2,
                             const double &x3, const double &y3);
//    double getClearanceWithDirection(const hmpl::State &state,
//                                     double angle,
//                                     const std::vector<double> &car_geometry);
    double getClearanceWithDirection(const hmpl::State &state,
                                     double angle);
    double getClearanceWithDirectionStrict(hmpl::State state, double angle, double radius);
    std::vector<double> getClearanceWithDirectionStrict(hmpl::State state,
                                                        double radius,
                                                        bool safety_margin_flag);
    std::vector<double> getClearanceFor3Circles(const hmpl::State &state,
                                                const std::vector<double> &car_geometry,
                                                bool safety_margin_flag);
//    std::vector<double> getClearance(hmpl::State state,
//                                     double ref_angle,
//                                     const std::vector<double> &car_geometry);
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

    //OSQP solver related
    void setHessianMatrix(size_t horizon, Eigen::SparseMatrix<double> *matrix_h);
    void setDynamicMatrix(size_t n, Eigen::Matrix<double, 2, 2> *matrix_a, Eigen::Matrix<double, 2, 1> *matrix_b);
    void setConstraintMatrix(size_t horizon,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound,
                             const std::vector<double> &init_state);
    hmpl::InternalGridMap grid_map_;
    CollisionChecker collision_checker_;
    CarType car_type;
    std::vector<double> car_geo_;
    // rear_axle_to_center_dis is needed only when using ackermann steering.
    double rear_axle_to_center_dis;
    double wheel_base;

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
    std::vector<std::vector<double> > seg_clearance_list_;
    bool control_sampling_first_flag_;
    bool enable_control_sampling;
    size_t point_num_;
    hmpl::State start_state_;
    hmpl::State end_state_;
    tk::spline x_spline_;
    tk::spline y_spline_;
    tk::spline k_spline_;
    std::vector<std::vector<double> > predicted_path_in_frenet_;


    // For visualization purpose.
    std::vector<std::vector<hmpl::State> > sampling_path_set_;
    std::vector<std::vector<hmpl::State> > failed_sampling_path_set_;
    std::vector<hmpl::State> left_bound_;
    std::vector<hmpl::State> right_bound_;
    std::vector<hmpl::State> second_third_point_;
    size_t best_sampling_index_;
    std::vector<hmpl::State> empty_;
    std::vector<hmpl::State> rear_bounds_;
    std::vector<hmpl::State> center_bounds_;
    std::vector<hmpl::State> front_bounds_;
    std::vector<hmpl::State> smoothed_path_;
};

}

#endif //PATH_OPTIMIZER__PATHOPTIMIZER_HPP_
