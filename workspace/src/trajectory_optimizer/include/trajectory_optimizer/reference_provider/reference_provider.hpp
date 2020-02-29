//
// Created by ljn on 20-2-20.
//

#ifndef TRAJECTORY_OPTIMIZER_INCLUDE_REFERENCE_PROVIDER_REFERENCE_PROVIDER_HPP_
#define TRAJECTORY_OPTIMIZER_INCLUDE_REFERENCE_PROVIDER_REFERENCE_PROVIDER_HPP_

#include <memory>
#include <path_optimizer/path_optimizer.hpp>
#include <path_optimizer/tools/spline.h>
#include "../data_struct/data_struct.hpp"

namespace TrajOptNS {

class ReferenceProvider {
public:
    ReferenceProvider() = delete;
    ReferenceProvider(const std::vector<State> &points_list,
                      const State &start_state,
                      const State &end_state,
                      const grid_map::GridMap &map);
    bool getReferenceTrajectory(const std::shared_ptr<FrenetTrajectory> &reference_trajectory);

private:
    const std::vector<State> &points_list_;
    const State &start_state_, &end_state_;
    const grid_map::GridMap &grid_map_;
};
}

#endif //TRAJECTORY_OPTIMIZER_INCLUDE_REFERENCE_PROVIDER_REFERENCE_PROVIDER_HPP_
