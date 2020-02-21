//
// Created by ljn on 20-2-12.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_

#include <iostream>
#include <cassert>
#include <stdexcept>
#include <grid_map_core/grid_map_core.hpp>
#include <glog/logging.h>

namespace PathOptimizationNS {

class Map {
public:
    Map() = delete;
    Map(const grid_map::GridMap &grid_map);
    double getObstacleDistance(const grid_map::Position &pos) const;
    const grid_map::GridMap &maps;
};
}

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_
