//
// Created by ljn on 19-11-15.
//
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <ctime>

int main() {
    size_t horizon = 20;
    const size_t state_size = 2 * horizon;
    const size_t control_size = horizon - 1;
    const size_t matrix_size = state_size + control_size;
// Populate hessian matrix
    Eigen::Matrix2d matrix_Q;

    matrix_Q << 0, 0,
        0, 1;
    Eigen::SparseMatrix<double> matrix_R;
    matrix_R.resize(control_size, control_size);
    for (size_t i = 0; i != control_size; ++i) {
        for (size_t j = 0; j != control_size; ++j) {
            if (i == j) {
                if (i == 0 || i == control_size - 1) {
                    matrix_R.insert(i, j) = 2;
                } else {
                    matrix_R.insert(i, j) = 3;
                }
            } else if (i == j - 1 || i == j + 1) {
                matrix_R.insert(i, j) = -1;
            }
        }
    }
    Eigen::MatrixXd matrix_h = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
//    matrix_h.resize(matrix_size, matrix_size);
    for (size_t i = 0; i != horizon; ++i) {
        matrix_h.block(2*i, 2*i, 2, 2) = matrix_Q;
    }
    matrix_h.block(2*horizon, 2*horizon, horizon-1, horizon-1) = matrix_R;
//    std::cout << matrix_h << std::endl;
    auto start = clock();
    Eigen::SparseMatrix<double> sparse;
    sparse = matrix_h.sparseView();
    auto end = clock();
    std::cout << "cost: " << double(end - start)/CLOCKS_PER_SEC << std::endl;
    std::cout << sparse << std::endl;
    return 0;
}

