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
#include <opt_utils/utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include "spline.h"
#include "FgEvalFrenet.hpp"
#include "collosion_checker.hpp"

#define MAX_CURVATURE 0.25

namespace MpcSmoother {

enum Type { START_ON_PATH = 0, START_OFF_PATH = 1 };

struct State {
    State() = default;
    State(double x, double y, double z, double k) :
        x(x),
        y(y),
        z(z),
        k(k) {}
    double x{};
    double y{};
    double z{};
    double k{};
};

class MpcPathOptimizer {
public:
    MpcPathOptimizer() = delete;
    MpcPathOptimizer(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     const State &start_state,
                     const State &end_state,
                     const hmpl::InternalGridMap &map);
    bool solve();
    std::vector<double> &getXList();
    std::vector<double> &getYList();
private:
    void getCurvature(const std::vector<double> &local_x,
                      const std::vector<double> &local_y,
                      std::vector<double> *pt_curvature_out,
                      double *max_curvature_abs,
                      double *max_curvature_change_abs);
    double getPointCurvature(const double &x1, const double &y1,
                             const double &x2, const double &y2,
                             const double &x3, const double &y3);

    hmpl::InternalGridMap grid_map_;
    CollisionChecker collision_checker_;
    bool large_init_psi_flag_;

    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> k_list_;
    std::vector<double> s_list_;
    std::vector<double> seg_list_;

    double cte_;  // lateral error
    double epsi_; // navigable error
    size_t point_num_;
    State start_state_;
    State end_state_;
    // x_spline and y_spline are in global frame
    tk::spline x_spline_;
    tk::spline y_spline_;
    tk::spline k_spline_;

    std::vector<std::vector<double> > predicted_path_in_frenet_;
    std::vector<double> predicted_path_x_;
    std::vector<double> predicted_path_y_;

};

}

#endif //MPC_PATH_OPTIMIZER__MPCPATHOPTIMIZER_HPP_
