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
#include "spline.h"
#include "FgEvalFrenet.hpp"
#include <tinyspline_ros/tinysplinecpp.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/QR"
#include "Clothoid.hh"

#define MAX_CURVATURE 0.25
#define MAX_CURVATURE_RATE 0.3

namespace MpcSmoother {

enum Type { START_ON_PATH = 0, START_OFF_PATH = 1 };

struct State {
    double x{};
    double y{};
    double z{};
    double k{};
    double s{};
    double v{};
    double dk{};
};

class MpcPathOptimizer {
public:
    MpcPathOptimizer() = delete;
    MpcPathOptimizer(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     const State &start_state,
                     const State &end_state);
    void reset(const std::vector<double> &x,
               const std::vector<double> &y,
               const State &init_state,
               const State &goal_state);
    bool solve();
    std::vector<double> &getXList();
    std::vector<double> &getYList();
private:
    void getCurvature(const std::vector<double> &local_x, const std::vector<double> &local_y,
                      std::vector<double> *pt_curvature_out);
    double getPointCurvature(const double &x1, const double &y1,
                             const double &x2, const double &y2,
                             const double &x3, const double &y3);

    // todo: use this flag.
    bool flag;

    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> x_local;
    std::vector<double> y_local;
    std::vector<double> k_list;
    std::vector<double> s_list;

    double cte;  // lateral error
    double epsi; // navigable error
    size_t point_num;
    State start_state;
    State end_state;
    // x_spline and y_spline are in global frame
    tk::spline x_spline;
    tk::spline y_spline;
    tk::spline k_spline;

    std::vector<std::vector<double> > predicted_path_in_frenet;
    std::vector<double> predicted_path_x;
    std::vector<double> predicted_path_y;

};

}

#endif //MPC_PATH_OPTIMIZER__MPCPATHOPTIMIZER_HPP_
