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
#include "FgEvalReferenceSmoothing.hpp"
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
                  const hmpl::InternalGridMap &map,
                  bool densify_path = true);
    bool solve(std::vector<hmpl::State> *final_path);
    bool samplePaths(const std::vector<double> &lon_set,
                     const std::vector<double> &lat_set,
                     std::vector<std::vector<hmpl::State>> *final_path_set);
    // For dynamic obstacle avoidance.
    bool optimizeDynamic(const std::vector<double> &sr_list,
                         const std::vector<std::vector<double>> &clearance_list,
                         std::vector<double> *x_list,
                         std::vector<double> *y_list,
                         std::vector<double> *s_list);
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
    void reset();
    void setCarGeometry();
    bool smoothPath(tk::spline *x_s_out, tk::spline *y_s_out, double *max_s);
    bool optimizePath(std::vector<hmpl::State> *final_path);
    bool sampleSingleLongitudinalPaths(double lon,
                                       const std::vector<double> &lat_set,
                                       std::vector<std::vector<hmpl::State>> *final_path_set,
                                       bool max_lon_flag);
    static void getCurvature(const std::vector<double> &local_x,
                      const std::vector<double> &local_y,
                      std::vector<double> *pt_curvature_out,
                      double *max_curvature_abs,
                      double *max_curvature_change_abs);
    static double getPointCurvature(const double &x1, const double &y1,
                             const double &x2, const double &y2,
                             const double &x3, const double &y3);
    double getClearanceWithDirection(const hmpl::State &state,
                                     double angle);
    double getClearanceWithDirectionStrict(hmpl::State state, double angle, double radius);
    std::vector<double> getClearanceWithDirectionStrict(hmpl::State state,
                                                        double radius,
                                                        bool safety_margin_flag);
    std::vector<double> getClearanceFor4Circles(const hmpl::State &state,
                                                const std::vector<double> &car_geometry,
                                                bool safety_margin_flag);

    bool divideSmoothedPath(bool safety_margin_flag);

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
    void setDynamicMatrix(size_t n,
                          const std::vector<double> &seg_s_list,
                          const std::vector<double> &seg_k_list,
                          Eigen::Matrix<double, 2, 2> *matrix_a,
                          Eigen::Matrix<double, 2, 1> *matrix_b);
    void setConstraintMatrix(size_t horizon,
                             const std::vector<double> &seg_s_list,
                             const std::vector<double> &seg_angle_list,
                             const std::vector<double> &seg_k_list,
                             const std::vector<std::vector<double>> &seg_clearance_list,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound,
                             const std::vector<double> &init_state,
                             double end_heading,
                             bool constraint_end_psi);

    void setConstraintMatrix(size_t horizon,
                             const std::vector<double> &seg_s_list,
                             const std::vector<double> &seg_angle_list,
                             const std::vector<double> &seg_k_list,
                             const std::vector<std::vector<double>> &seg_clearance_list,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound,
                             const std::vector<double> &init_state,
                             double end_angle,
                             double offset,
                             double angle_error_allowed = 0,
                             double offset_error_allowed = 0);

    hmpl::InternalGridMap grid_map_;
    CollisionChecker collision_checker_;
    CarType car_type;
    std::vector<double> car_geo_;
    double rear_axle_to_center_dis;
    double wheel_base;

    std::vector<double> seg_s_list_;
    std::vector<double> seg_k_list_;
    std::vector<double> seg_x_list_;
    std::vector<double> seg_y_list_;
    std::vector<double> seg_angle_list_;
    std::vector<std::vector<double> > seg_clearance_list_;
    bool control_sampling_first_flag_;
    bool enable_control_sampling;
    hmpl::State start_state_;
    hmpl::State end_state_;
    double smoothed_max_s;
    size_t N_;
    bool use_end_psi;
    // Start position and angle error with smoothed path.
    double cte_;
    double epsi_;
    bool densify_result;

    // Only for smoothing phase
    std::vector<hmpl::State> points_list_;
    size_t point_num_;
    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> k_list_;
    std::vector<double> s_list_;
    tk::spline x_spline_;
    tk::spline y_spline_;
    tk::spline k_spline_;
    tk::spline smoothed_x_spline;
    tk::spline smoothed_y_spline;

    // For dynamic obstacle avoidace.
    OsqpEigen::Solver solver_dynamic;
    bool solver_dynamic_initialized;
    Eigen::VectorXd lower_bound_dynamic_;
    Eigen::VectorXd upper_bound_dynamic_;
    tk::spline xsr_, ysr_;

    // For visualization purpose.
    std::vector<std::vector<hmpl::State> > control_sampling_path_set_;
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
