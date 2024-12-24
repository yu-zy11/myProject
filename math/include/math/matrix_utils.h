#pragma once

#include <glog/logging.h>
#include <Eigen/Dense>

namespace math {
template <typename T>
Eigen::Matrix<T, -1, -1> StackMatrix(const Eigen::Matrix<T, -1, -1>& m1, const Eigen::Matrix<T, -1, -1>& m2) {
  int cols = m1.cols();
  if (cols != m2.cols()) {
    throw std::runtime_error("cols of m1 and m2 not equal");
  }
  Eigen::Matrix<T, -1, -1> res(m1.rows() + m2.rows(), cols);
  res << m1, m2;
  return res;
}

template <typename T>
Eigen::Matrix<T, -1, 1> StackMatrix(const Eigen::Matrix<T, -1, 1>& v1, const Eigen::Matrix<T, -1, 1>& v2) {
  Eigen::Matrix<T, -1, 1> res(v1.size() + v2.size());
  res << v1, v2;
  return res;
}

// 判断矩阵是否正定
template <typename T>
bool IsPositiveDefinite(const Eigen::Matrix<T, -1, -1>& matrix) {
  if (matrix.rows() != matrix.cols()) {
    throw std::runtime_error("matrix is not square");
  }
  try {
    Eigen::LLT<Eigen::Matrix<T, -1, -1>> llt(matrix);
    return llt.info() == Eigen::Success;
  } catch (...) {
    return false;
  }
}

template <typename T>
bool IsSemiPositiveDefinite(const Eigen::Matrix<T, -1, -1>& matrix) {
  if (matrix.rows() != matrix.cols()) {
    throw std::runtime_error("matrix is not square");
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, -1, -1>> eigenSolver(matrix);
  Eigen::Matrix<T, -1, 1> eigenvalues = eigenSolver.eigenvalues();
  if (eigenvalues.array().any() < T(0)) {
    return false;
  } else {
    return true;
  }
}

template <typename T>
bool RegualizeMatrix(Eigen::Matrix<T, -1, -1>& matrix, T lambda = 0.1, int max_iter = 100) {
  if (IsPositiveDefinite(matrix)) {
    return true;
  }
  int iter = 0;
  int rows = matrix.rows();
  while (!IsPositiveDefinite(matrix) && iter < max_iter) {
    matrix.diagonal() += Eigen::Matrix<T, -1, 1>::Ones(rows) * lambda;
    ++iter;
  }
  if (iter == max_iter) {
    return false;
  }
  return true;
}

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
