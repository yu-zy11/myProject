

#ifndef PSEUDO_INVERSE_H
#define PSEUDO_INVERSE_H

#include <Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
namespace core {
namespace math {
/**
 * Compute the dumped pseudo inverse of a matrix. If the singular value is less than
 * threshold, then replace it with 1/threshold. This is useful for
 * stabilizing the computation of the pseudo inverse.
 * \param matrix input matrix
 * \param threshold threshold for the singular values
 * \param inverse_matrix output matrix, the pseudo inverse of matrix
 */

template <typename T>
void dumpedPseudoInverse(Eigen::Matrix<T, -1, -1> const& matrix, double threshold,
                         Eigen::Matrix<T, -1, -1>& inverse_matrix) {
  if ((1 == matrix.rows()) && (1 == matrix.cols())) {
    inverse_matrix.resize(1, 1);
    if (matrix.coeff(0, 0) > threshold) {
      inverse_matrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      inverse_matrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }
  Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  int const nrows(svd.singularValues().rows());
  Eigen::Matrix<T, -1, -1> invS = Eigen::Matrix<T, -1, -1>::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > threshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {  // add dumped to sigular value
      invS.coeffRef(ii, ii) = 1.0 / threshold;
    }
  }
  inverse_matrix = svd.matrixV() * invS * svd.matrixU().transpose();
}

template <typename T>
void pseudoInverse(Eigen::Matrix<T, -1, -1> const& matrix, double threshold, Eigen::Matrix<T, -1, -1>& inverse_matrix) {
  if ((1 == matrix.rows()) && (1 == matrix.cols())) {
    inverse_matrix.resize(1, 1);
    if (matrix.coeff(0, 0) > threshold) {
      inverse_matrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      inverse_matrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }

  Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  int const nrows(svd.singularValues().rows());
  Eigen::Matrix<T, -1, -1> invS;
  invS = Eigen::Matrix<T, -1, -1>::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > threshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {  // do nothing
    }
  }
  inverse_matrix = svd.matrixV() * invS * svd.matrixU().transpose();
}
}  // namespace math
}  // namespace core
#endif
