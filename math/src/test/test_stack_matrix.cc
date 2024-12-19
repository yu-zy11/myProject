#include <Eigen/Dense>
#include <ctime>
#include <iostream>

#include "../../include/math/matrix_utils.h"

bool test_stack_matrix() {
  Eigen::MatrixXd m1(2, 3);
  Eigen::MatrixXd m2(3, 3);
  m1 << 1, 2, 3, 4, 5, 6;
  m2.setIdentity();
  Eigen::MatrixXd stacked_matrix = Eigen::MatrixXd(5, 3);
  stacked_matrix << m1, m2;
  Eigen::MatrixXd res = math::StackMatrix(m1, m2);
  std::cout << "res\n" << res << std::endl;
  if (stacked_matrix == res) {
    return true;
  } else {
    return false;
  }

  m1.resize(3, 1);
  m1 << 1, 2, 3;
  m2.resize(3, 1);
  m2 << 4, 5, 6;
  stacked_matrix.resize(6, 1);
  stacked_matrix << m1, m2;
  res = math::StackMatrix(m1, m2);
  if (stacked_matrix == res) {
    return true;
  } else {
    return false;
  }
  std::cout << "res\n" << res << std::endl;
};

bool test_IsPositiveDefinite() {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(3, 3);
  matrix(2, 2) = 0;
  bool semi_definite = math::IsSemiPositiveDefinite(matrix);
  matrix(2, 2) = 1;
  if (!math::IsPositiveDefinite(matrix)) {
    std::cout << "test_IsPositiveDefinite failed" << std::endl;
    return false;
  }
  matrix = -Eigen::MatrixXd::Identity(3, 3);
  if (math::IsPositiveDefinite(matrix)) {
    std::cout << "test_IsPositiveDefinite failed" << std::endl;
    return false;
  }

  matrix = Eigen::MatrixXd::Random(3, 3);
  matrix = (matrix + matrix.transpose()) * 0.5;
  std::cout << "matrix\n" << matrix << std::endl;
  std::cout << "is positive definite matrix\n" << math::IsPositiveDefinite(matrix) << std::endl;
  time_t begin, end;
  begin = clock();
  for (int i = 0; i < 10000; ++i) {
    math::IsPositiveDefinite(matrix);
  }
  end = clock();
  std::cout << "time2=" << (end - begin) << std::endl;
  return true;
}

bool test_regualize_matrix() {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(3, 3);
  std::cout << "matrix\n" << matrix << std::endl;
  math::RegualizeMatrix(matrix, 0.1, 10);
  std::cout << "regualized matrix\n" << matrix << std::endl;
  std::cout << "regualized success\n" << math::RegualizeMatrix(matrix, 0.1, 10) << std::endl;
  return true;
}

int main() {
  if (!test_IsPositiveDefinite()) {
    std::cout << "test_IsPositiveDefinite failed" << std::endl;
  } else {
    std::cout << "test_IsPositiveDefinite success" << std::endl;
  };

  if (!test_stack_matrix()) {
    std::cout << "test_IsPositiveDefinite failed" << std::endl;
  } else {
    std::cout << "test_IsPositiveDefinite success" << std::endl;
  };

  if (!test_regualize_matrix()) {
    std::cout << "test_IsPositiveDefinite failed" << std::endl;
  } else {
    std::cout << "test_IsPositiveDefinite success" << std::endl;
  };
  return 0;
}
