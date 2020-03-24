//
// Created by ljn on 20-1-31.
//

#ifndef PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <ctime>
#include <tinyspline_ros/tinysplinecpp.h>
#include "../config/config.hpp"
#include "../data_struct/data_struct.hpp"

namespace PathOptimizationNS {
#define OBSTACLE_COST 0.4
#define OFFSET_COST 0.4
#define SMOOTHNESS_COST 10

class Map;
class ReferencePath;
// This class use A* search to improve the quality of the input points (if needed), and
// then uses a smoother to obtain a smoothed reference path.
class ReferencePathSmoother {

public:
    ReferencePathSmoother() = delete;
    ReferencePathSmoother(const std::vector<State> &input_points,
                          const State &start_state,
                          const Map &grid_map,
                          const Config &config);

    template<typename Smoother>
    bool solve(ReferencePath *reference_path, std::vector<State> *smoothed_path_display = nullptr);
    std::vector<std::vector<double>> display() const {
        return std::vector<std::vector<double>>{x_list_, y_list_, s_list_};
    }

private:
    void bSpline();
    // A* search.
    bool modifyInputPoints();
    bool checkExistenceInClosedSet(const APoint &point) const;
    double getG(const APoint &point, const APoint &parent) const;
    inline double getH(const APoint &p) const;

    const std::vector<State> &input_points_;
    const State &start_state_;
    const Map &grid_map_;
    const Config &config_;
    // Data to be passed into solvers.
    std::vector<double> x_list_, y_list_, s_list_;
    // Sampled points.
    std::vector<std::vector<APoint>> sampled_points_;
    double target_s_{};
    std::priority_queue<APoint*, std::vector<APoint*>, PointComparator> open_set_;
    std::vector<APoint*> closed_set_;

};

template<typename Smoother>
bool ReferencePathSmoother::solve(PathOptimizationNS::ReferencePath *reference_path,
                                  std::vector<State> *smoothed_path_display) {
    bSpline();
    if (config_.modify_input_points_) {
        modifyInputPoints();
    }
    Smoother smoother(x_list_, y_list_, s_list_, start_state_, grid_map_, config_);
    return smoother.smooth(reference_path, smoothed_path_display);
}
}

#endif //PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
