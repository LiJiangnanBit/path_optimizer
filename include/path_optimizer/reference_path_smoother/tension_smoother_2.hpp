//
// Created by ljn on 20-5-4.
//

#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
#include <vector>
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

class TensionSmoother2 final : public TensionSmoother {
 public:
    TensionSmoother2() = delete;
    TensionSmoother2(const std::vector<State> &input_points,
               const State &start_state,
               const Map &grid_map);
    ~TensionSmoother2() override = default;

 private:
    bool ipoptSmooth(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     const std::vector<double> &angle_list,
                     const std::vector<double> &k_list,
                     const std::vector<double> &s_list,
                     std::vector<double> *result_x_list,
                     std::vector<double> *result_y_list,
                     std::vector<double> *result_s_list) override;
    bool osqpSmooth(const std::vector<double> &x_list,
                    const std::vector<double> &y_list,
                    const std::vector<double> &angle_list,
                    const std::vector<double> &k_list,
                    const std::vector<double> &s_list,
                    std::vector<double> *result_x_list,
                    std::vector<double> *result_y_list,
                    std::vector<double> *result_s_list) override;
    void setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const override;
    void setConstraintMatrix(const std::vector<double> &x_list,
                             const std::vector<double> &y_list,
                             const std::vector<double> &angle_list,
                             const std::vector<double> &k_list,
                             const std::vector<double> &s_list,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const override;
    void setGradient(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     Eigen::VectorXd *gradient);
};
}
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
