//
// Created by ljn on 20-2-20.
//

#ifndef TRAJECTORY_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
#define TRAJECTORY_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
#include <memory>
#include <path_optimizer/data_struct/data_struct.hpp>
#include <path_optimizer/tools/Map.hpp>
#include <path_optimizer/tools/spline.h>
#include <path_optimizer/config/config.hpp>

using PathOptimizationNS::State;
using PathOptimizationNS::VehicleState;
using PathOptimizationNS::Map;
using PathOptimizationNS::tk::spline;
using PathOptimizationNS::CarType;
using grid_map::GridMap;

namespace TrajOptNS {

struct FrenetState {
    double ey{};
    double ephi{};
    double s{};
    double k{};
    double v{};
    double ay{}; // Lateral acc.
    double kp{}; // k' 
    double vp{}; // v'
    FrenetState(double ey, double ephi, double s, double k, double v)
    : ey(ey), ephi(ephi), s(s), k(k), v(v) {}
};

struct FrenetTrajectory {
    spline x_s, y_s;
    double path_length_{};
    double spacing{};
    double max_time{};
    std::vector<FrenetState> state_list;
};

struct CoveringCircleBounds {
    struct SingleCircleBounds {
        SingleCircleBounds &operator=(std::vector<double> bounds) {
            ub = bounds[0];
            lb = bounds[1];
        }
        double ub{}; // left
        double lb{}; // right
    } c0, c1, c2, c3;
};

struct TrajOptConfig {
    // Car param:
    static CarType car_type_;
    static double car_width_;
    static double car_length_;
    static double circle_radius_;
    static double wheel_base_;
    static double rear_axle_to_center_distance_; // Distance from rear axle center to the center of the vehicle.
    static double d1_, d2_, d3_, d4_; // Distance from vehicle center to the covering circles, from rear to front.
    static double max_steer_angle_;
    static double max_lon_acc_;
    static double max_lon_dacc_;
    static double max_lat_acc_;
    static double max_v_;

    //
    static double spacing_;
    static double max_length_;
};

struct SolverInput {
    void init(const std::shared_ptr<FrenetTrajectory> &traj_ptr, State start, State end) {
        reference_trajectory = traj_ptr;
        horizon = traj_ptr->state_list.size();
        start_state = start;
        end_state = end;
    }
    void updateLateralBounds(const Map &map);
    std::vector<double> getClearanceWithDirectionStrict(const State &state, const Map &map) const;

    std::shared_ptr<FrenetTrajectory> reference_trajectory;
    size_t horizon{};
    std::vector<CoveringCircleBounds> bounds;
    State start_state, end_state;
};
}

#endif //TRAJECTORY_OPTIMIZER_INCLUDE_DATA_STRUCT_DATA_STRUCT_HPP_
