//
// Created by ljn on 20-1-31.
//

#ifndef PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <queue>
#include <ctime>
#include <tinyspline_ros/tinysplinecpp.h>
#include <path_optimizer/tools/spline.h>
#include "../data_struct/data_struct.hpp"

namespace PathOptimizationNS {

class Map;
class ReferencePath;
// This class uses searching method to improve the quality of the input points (if needed), and
// then uses a smoother to obtain a smoothed reference path.
class ReferencePathSmoother {

public:
    ReferencePathSmoother() = delete;
    ReferencePathSmoother(const std::vector<State> &input_points,
                          const State &start_state,
                          const Map &grid_map);
    virtual ~ReferencePathSmoother() = default;

    static std::unique_ptr<ReferencePathSmoother> create(std::string type,
                                                  const std::vector<State> &input_points,
                                                  const State &start_state,
                                                  const Map &grid_map);

    bool solve(ReferencePath *reference_path, std::vector<State> *smoothed_path_display = nullptr);
    std::vector<std::vector<double>> display() const;

 protected:
    const State &start_state_;
    const Map &grid_map_;
    // Data to be passed into solvers.
    std::vector<double> x_list_, y_list_, s_list_;

 private:
    virtual bool smooth(PathOptimizationNS::ReferencePath *reference_path,
                        std::vector<State> *smoothed_path_display) = 0;
    void bSpline();
    // A* search.
    bool modifyInputPoints();
    bool checkExistenceInClosedSet(const APoint &point) const;
    double getG(const APoint &point, const APoint &parent) const;
    inline double getH(const APoint &p) const;
    const std::vector<State> &input_points_;
    // Sampled points in searching process.
    std::vector<std::vector<APoint>> sampled_points_;
    double target_s_{};
    std::priority_queue<APoint*, std::vector<APoint*>, PointComparator> open_set_;
    std::vector<APoint*> closed_set_;

};
}

#endif //PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
