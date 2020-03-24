//
// Created by ljn on 20-2-12.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_

#include <iostream>
#include <cassert>
#include <stdexcept>
#include "Eigen/Core"
#include <grid_map_core/grid_map_core.hpp>

namespace PathOptimizationNS {

class Map {
 public:
    Map() = delete;
    explicit Map(const grid_map::GridMap &grid_map);
    double getObstacleDistance(const Eigen::Vector2d &pos) const;
    bool isInside(const Eigen::Vector2d &pos) const;

 private:
    const grid_map::GridMap &maps;
};
}

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_MAP_HPP_
