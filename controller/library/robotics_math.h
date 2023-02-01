#ifndef ROBOTICS_MATH_H
#define ROBOTICS_MATH_H
#include "dynamics.h"
#include <Eigen/Dense>
#include <string>

namespace ROBOTICS {
namespace Math {

template <typename T> Eigen::Matrix<T, 3, 3> rotx(T theta) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
  return rot;
}
template <typename T> Eigen::Matrix<T, 3, 3> roty(T theta) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
  return rot;
}
template <typename T> Eigen::Matrix<T, 3, 3> rotz(T theta) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  return rot;
}
// vec2so3
template <typename T>
Eigen::Matrix<T, 3, 3> vec2so3(Eigen::Matrix<T, 3, 1> vec) {
  Eigen::Matrix<T, 3, 3> so3;
  so3 << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return so3;
}
// exp2rotm
/*brief:计算矩阵指数 by yuzhiyou 20210706
%S 螺旋轴6*1
%theta 旋转角1*1
*/
template <typename T>
Eigen::Matrix<T, 4, 4> exp2rotm(Eigen::Matrix<T, 6, 1> S, T theta) {
  Eigen::Matrix<T, 3, 1> w, v;
  w << S[0], S[1], S[2];
  v << S[3], S[4], S[5];
  Eigen::Matrix<T, 3, 3> so3_w = vec2so3(w);
  Eigen::Matrix<T, 3, 3> eye3;
  eye3 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Eigen::Matrix<T, 4, 4> rotm;
  rotm.block(0, 0, 3, 3) =
      eye3 + sin(theta) * so3_w + (1 - cos(theta)) * so3_w * so3_w;
  rotm.block(0, 3, 3, 1) = (eye3 * theta + (1 - cos(theta)) * so3_w +
                            (theta - sin(theta)) * so3_w * so3_w) *
                           v;
  rotm.block(3, 0, 1, 4) << 0, 0, 0, 1;
  return rotm;
}
template <typename T>
Eigen::Matrix<T, 4, 4> invTransM(Eigen::Matrix<T, 4, 4> mat) {
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
template <typename T> Eigen::Matrix<T, 6, 6> AdT(Eigen::Matrix<T, 4, 4> mat) {
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
// namespace Math
// template <typename T> class yTransMath : public Dynamics {

//   // constexpr double pi{3.1415926};
//   T Addyu(T x, T y) {
//     // std::cout<<"addyu~\n";
//     return x + y;
//   }
//   void print(int x, int y);
//   void print_reference_value(double const &num);
//   // int* ptr{nullptr}; //申明空指针，表示还没有指向任何对象的地址=int
//   // int* ptra{};  //隐式申明空指针，避免悬空指针：int* ptr
//   const std::string &printName();
//   struct Student {
//     std::string name{};
//     double grade{};
//   };
//   enum Color {
//     red,
//     black,
//     yellow,
//   };                           // unscoped
//   enum class Pet { dog, cat }; // scoped
// };
} // namespace ROBOTICS

#endif
