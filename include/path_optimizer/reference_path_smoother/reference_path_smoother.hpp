//
// Created by ljn on 20-1-31.
//

#ifndef PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_set>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <string>
#include <queue>
#include <ctime>
#include <tinyspline_ros/tinysplinecpp.h>
#include <path_optimizer/tools/spline.h>
#include <bits/unordered_set.h>
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

    static std::unique_ptr<ReferencePathSmoother> create(const std::string &type,
                                                         const std::vector<State> &input_points,
                                                         const State &start_state,
                                                         const Map &grid_map);

    bool solve(PathOptimizationNS::ReferencePath *reference_path);
    std::vector<std::vector<double>> display() const;

 protected:
    bool segmentRawReference(std::vector<double> *x_list,
                             std::vector<double> *y_list,
                             std::vector<double> *s_list,
                             std::vector<double> *angle_list,
                             std::vector<double> *k_list) const;
    const State &start_state_;
    const Map &grid_map_;
    // Data to be passed into solvers.
    std::vector<double> x_list_, y_list_, s_list_;

 private:
    virtual bool smooth(PathOptimizationNS::ReferencePath *reference_path) = 0;
    void bSpline();
    bool postSmooth(PathOptimizationNS::ReferencePath *reference_path);
    void setPostHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const;
    void setPostConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                                 Eigen::VectorXd *lower_bound,
                                 Eigen::VectorXd *upper_bound) const;
    // search.
    bool graphSearch(ReferencePath *reference);
    bool graphSearchDp(ReferencePath *reference);
    void calculateCostAt(std::vector<std::vector<DpPoint>> &samples, int layer_index, int lateral_index) const;
    inline bool checkExistenceInClosedSet(const APoint &point) const;
    inline double getG(const APoint &point, const APoint &parent) const;
    inline double getH(const APoint &p) const;
    const std::vector<State> &input_points_;
    // Sampled points in searching process.
    std::vector<std::vector<APoint>> sampled_points_;
    double target_s_{};
    std::priority_queue<APoint *, std::vector<APoint *>, PointComparator> open_set_;
    std::unordered_set<const APoint *> closed_set_;
    std::vector<double> layers_s_list_;
    std::vector<std::pair<double, double>> layers_bounds_;
    double vehicle_l_wrt_smoothed_ref_;

};
}

#endif //PATH_OPTIMIZER_INCLUDE_REFERENCE_PATH_SMOOTHER_REFERENCE_PATH_SMOOTHER_HPP_
