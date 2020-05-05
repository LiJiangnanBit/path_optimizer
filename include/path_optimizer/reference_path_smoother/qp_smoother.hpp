//
// Created by ljn on 20-5-4.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_QP_SMOOTHER_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_QP_SMOOTHER_HPP_
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cfloat>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "path_optimizer/reference_path_smoother/tension_smoother.hpp"

namespace PathOptimizationNS {

class FgEvalQPSmoothing : public FgEvalReferenceSmoothing {
 public:
    FgEvalQPSmoothing(const std::vector<double> &seg_x_list,
                      const std::vector<double> &seg_y_list,
                      const std::vector<double> &seg_s_list,
                      const std::vector<double> &seg_angle_list,
                      const std::vector<double> &seg_k_list);
    ~FgEvalQPSmoothing() override = default;
    void operator()(ADvector &fg, const ADvector &vars) override;
 private:
    const std::vector<double> &seg_k_list_;
};

class QPSmoother final : public TensionSmoother {
 public:
    QPSmoother() = delete;
    QPSmoother(const std::vector<State> &input_points,
               const State &start_state,
               const Map &grid_map);
    ~QPSmoother() override = default;

 private:
    bool ipoptSmooth(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     const std::vector<double> &angle_list,
                     const std::vector<double> &k_list,
                     const std::vector<double> &s_list,
                     std::vector<double> *result_x_list,
                     std::vector<double> *result_y_list,
                     std::vector<double> *result_s_list) override;
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_QP_SMOOTHER_HPP_
