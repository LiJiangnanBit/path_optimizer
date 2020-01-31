//
// Created by ljn on 20-1-31.
//

#ifndef PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#include <iostream>
#include <vector>
#include <string>
#include <opt_utils/opt_utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include "config/config.hpp"
#include "data_struct/data_struct.hpp"

namespace PathOptimizationNS {

template<typename Smoother>
class ReferencePathSmoother {

public:
    ReferencePathSmoother() = delete;
    ReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                          const hmpl::State &start_state,
                          const hmpl::InternalGridMap &grid_map,
                          const Config &config);

    bool solve(ReferencePath *reference_path, std::vector<hmpl::State> *smoothed_path_display = nullptr) const;

private:

    Smoother smoother_;
};

template <typename Smoother>
ReferencePathSmoother<Smoother>::ReferencePathSmoother(const std::vector<hmpl::State> &input_points,
                                                       const hmpl::State &start_state,
                                                       const hmpl::InternalGridMap &grid_map,
                                                       const Config &config) :
    smoother_(input_points, start_state, grid_map, config)
{}

template <typename Smoother>
bool ReferencePathSmoother<Smoother>::solve(PathOptimizationNS::ReferencePath *reference_path,
                                            std::vector<hmpl::State> *smoothed_path_display) const {
    return smoother_.smooth(reference_path, smoothed_path_display);
}
}



#endif //PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
