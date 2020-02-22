//
// Created by ljn on 20-2-20.
//

#ifndef TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_TRAJECTORY_OPTIMIZER_HPP_
#define TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_TRAJECTORY_OPTIMIZER_HPP_
#include <iostream>
#include <grid_map_core/grid_map_core.hpp>
#include <vector>
#include "data_struct/data_struct.hpp"
#include "reference_provider/reference_provider.hpp"

namespace TrajOptNS {

class TrajectoryOptimizer {
public:
    TrajectoryOptimizer() = delete;
    TrajectoryOptimizer(const std::vector<State> &points_list,
                        const State &start_state,
                        const State &end_state,
                        const grid_map::GridMap &map);
    bool solve();

private:
    const std::vector<State> &points_list_;
    const State &start_state_, &end_state_;
    const Map grid_map_;
    Trajectory reference_trajectory_;
};
}

#endif //TRAJECTORY_OPTIMIZER_INCLUDE_TRAJECTORY_OPTIMIZER_TRAJECTORY_OPTIMIZER_HPP_
