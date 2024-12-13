#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <Eigen/Dense>

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

int main() {
  Eigen::MatrixXd m1(2, 3);
  Eigen::MatrixXd m2(3, 3);
  m1 << 1, 2, 3, 4, 5, 6;
  m2.setIdentity();
  Eigen::MatrixXd res = StackMatrix(m1, m2);
  std::cout << "res\n" << res << std::endl;
}
