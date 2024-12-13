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
bool RegualizeMatrix(Eigen::Matrix<T, -1, -1>& matrix, T lambda = 0.1, int max_iter = 10) {
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

Eigen::MatrixXd RegualizeAndInverseMatrixUsingSVD(const Eigen::MatrixXd& matrix, double min_singular_value) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  int const nrows(svd.singularValues().rows());
  Eigen::MatrixXd invS;
  invS = Eigen::MatrixXd::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > min_singular_value) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {  // do nothing
    }
  }
  return svd.matrixV() * invS * svd.matrixU().transpose();
}

}  // namespace math
