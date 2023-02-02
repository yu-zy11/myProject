#ifndef ROBOTICS_MATH_H
#define ROBOTICS_MATH_H
#include "dynamics.h"
#include <Eigen/Dense>
#include <string>

namespace ROBOTICS {
namespace Math {
// rotx
template <typename T> inline Eigen::Matrix<T, 3, 3> rotx(const T &theta) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
  return rot;
}
// roty
template <typename T> inline Eigen::Matrix<T, 3, 3> roty(const T &theta) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
  return rot;
}
// rotz
template <typename T> inline Eigen::Matrix<T, 3, 3> rotz(const T &theta) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  return rot;
}
// vec2so3
template <typename T>
inline Eigen::Matrix<T, 3, 3> vec2so3(const Eigen::Matrix<T, 3, 1> &vec) {
  Eigen::Matrix<T, 3, 3> so3;
  so3 << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return so3;
}
// exp2rotm
/*brief:计算矩阵指数 by yuzhiyou 20230131
%S 螺旋轴6*1
%theta 旋转角1*1
*/
template <typename T>
inline Eigen::Matrix<T, 4, 4> exp2rotm(const Eigen::Matrix<T, 6, 1> &S,
                                       T &theta) {
  Eigen::Matrix<T, 3, 1> w = S.block(0, 0, 3, 1);
  Eigen::Matrix<T, 3, 3> so3_w = vec2so3(w);
  Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 4, 4> rotm;
  rotm.block(0, 0, 3, 3) =
      eye3 + sin(theta) * so3_w + (1 - cos(theta)) * so3_w * so3_w;
  rotm.block(0, 3, 3, 1) = (eye3 * theta + (1 - cos(theta)) * so3_w +
                            (theta - sin(theta)) * so3_w * so3_w) *
                           S.block(3, 0, 3, 1);
  rotm.block(3, 0, 1, 4) << 0, 0, 0, 1;
  return rotm;
}
// invTransM
template <typename T>
inline Eigen::Matrix<T, 4, 4> invTransM(const Eigen::Matrix<T, 4, 4> &mat) {
  Eigen::Matrix<T, 4, 4> InvMat;
  Eigen::Matrix<T, 3, 3> R;
  Eigen::Matrix<T, 3, 1> P;
  R = mat.block(0, 0, 3, 3);
  P = mat.block(0, 3, 3, 1);
  InvMat.block(0, 0, 3, 3) = R.transpose();
  InvMat.block(0, 3, 3, 1) = -R.transpose() * P;
  InvMat.block(3, 0, 1, 4) << 0, 0, 0, 1;
  return InvMat;
}
// AdT
template <typename T>
inline Eigen::Matrix<T, 6, 6> AdT(const Eigen::Matrix<T, 4, 4> &mat) {
  // Pscrew=Vec2so3(T(1:3,4));
  // R=T(1:3,1:3);
  // AdT=[R     zeros(3,3);
  //     Pscrew*R     R ];
  Eigen::Matrix<T, 3, 3> R;
  Eigen::Matrix<T, 3, 1> P;
  R = mat.block(0, 0, 3, 3);
  P = mat.block(0, 3, 3, 1);
  Eigen::Matrix<T, 6, 6> aT;
  aT.setZero();
  aT.block(0, 0, 3, 3) = R;
  aT.block(3, 3, 3, 3) = R;
  aT.block(3, 0, 3, 3) = vec2so3(P) * R;
  return aT;
}

} // namespace Math
} // namespace ROBOTICS

#endif
