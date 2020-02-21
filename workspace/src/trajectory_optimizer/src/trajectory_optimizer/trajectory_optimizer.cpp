//
// Created by ljn on 20-2-20.
//

#include "trajectory_optimizer/trajectory_optimizer.hpp"

namespace TrajOptNS{
TrajectoryOptimizer::TrajectoryOptimizer(const std::vector<State> &points_list,
                                         const State &start_state,
                                         const State &end_state,
                                         const grid_map::GridMap &map)
                                         : points_list_(points_list),
                                         start_state_(start_state),
                                         end_state_(end_state),
                                         grid_map_(map) {}

bool TrajectoryOptimizer::solve() {
    ReferenceProvider reference_provider(points_list_, start_state_, end_state_, grid_map_.maps);
    reference_provider.getReferenceTrajectory(&reference_trajectory_);
    SolverInput solver_input;
    solver_input.reference_trajectory = reference_trajectory_;
    solver_input.updateBounds(grid_map_);


}

}