#include <eigen3/Eigen/Dense>
#ifndef ORIETATION_MATH_H
#define ORIETATION_MATH_H
// #define PI 3.1415926
// #define ZEROS 0.0000001
#include <iostream>
template <typename T>
Eigen::Matrix<T, 3, 1> quat2rpy(const Eigen::Matrix<T, 4, 1> &quat);
template <typename T>
Eigen::Matrix<T, 4, 1> rotm2quat(const Eigen::Matrix<T, 3, 3> &r);

template <typename T>
inline Eigen::Matrix<T, 3, 3> cross2mat(Eigen::Matrix<T, 3, 1> vec) {
  Eigen::Matrix<T, 3, 3> mat;
  mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
  return mat;
}

/**
 * drpy=M*omega:get tranformation matrix from angular velocity to euler derivative
 * @param [in] rpy:euler angle,sequece is Z-Y-X,but formed as [rx ry rz]
 * @note:drpy=[drx dry drz],omega=[wx wy wz]
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> omega2rpyrate(Eigen::Matrix<T, 3, 1> rpy) {
  T c3 = cos(rpy[2]);
  T s3 = sin(rpy[2]);
  T c2 = cos(rpy[1]);
  T s2 = sin(rpy[1]);
  assert(fabs(c2) > 0.00001 && "c2 must not be zero");
  Eigen::Matrix<T, 3, 3> mat_omega2rpy;
  mat_omega2rpy << c3 / c2, s3 / c2, 0, -s3, c3, 0, c3 * s2 / c2, s3 * s2 / c2, 1;
  return mat_omega2rpy;
}

/**
 * drpy=M*omega:get tranformation matrix from angular velocity to euler derivative
 * @param [in] yaw pitch roll:euler angle,rotationsequece is Z-Y-X
 * @note:drpy=[dyaw dpitch droll],omega=[wx wy wz]
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> omega2drpy(T yaw, T pitch, T roll) {
  T c3 = cos(yaw);
  T s3 = sin(yaw);
  T c2 = cos(pitch);
  T s2 = sin(pitch);
  // assert(fabs(c2) > 0.00001 && "c2 must not be zero");
  if (fabs(c2) < 0.00001) {
    std::cout << "omega2drpy: pitch is nearly to be 90,check" << std::endl;
  }
  c2 = fmax(c2, 0.00001);
  Eigen::Matrix<T, 3, 3> mat_omega2drpy;
  // clang-format off
  mat_omega2drpy << c3 * s2 / c2, s3 * s2 / c2, 1,
                    -s3,          c3,           0, 
                    c3 / c2,      s3 / c2,      0;
  return mat_omega2drpy;
  // clang-format on
}

/**
 * omega=M*d_rpy:get tranformation matrix M from euler derivative to angular velocity in world frame
 * @param [in] rpy:euler angle,sequece is Z-Y-X,but formed as [rx ry rz]
 * @note:drpy=[drx dry drz],omega=[wx wy wz]
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> rpyrate2omega(Eigen::Matrix<T, 3, 1> rpy) {
  T c3 = cos(rpy[2]);
  T s3 = sin(rpy[2]);
  T c2 = cos(rpy[1]);
  T s2 = sin(rpy[1]);
  Eigen::Matrix<T, 3, 3> M;
  // M << c2 * c3, -s3, 0, c2 * s3, c3, 0, -s2, 0, 1;
  M.block(0, 0, 1, 3) << c3 * c2, -s3, 0;
  M.block(1, 0, 1, 3) << c2 * s3, c3, 0;
  M.block(2, 0, 1, 3) << -s2, 1, 0;
  return M;
}
/**
 * domega=M*dd_rpy+dM*d_rpy:get tranformation matrix dM from euler acceleration to angular acceleration in world frame
 * @param [in] rpy:euler angle,sequece is Z-Y-X,but formed as [rx ry rz]
 * @note:drpy=[drx dry drz],omega=[wx wy wz]
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> dM_rpyrate2omega(const Eigen::Matrix<T, 3, 1> &rpy, const Eigen::Matrix<T, 3, 1> &drpy) {
  T c3 = cos(rpy[2]);
  T s3 = sin(rpy[2]);
  T c2 = cos(rpy[1]);
  T s2 = sin(rpy[1]);
  T c1 = cos(rpy[0]);
  T s1 = sin(rpy[0]);
  Eigen::Matrix<T, 3, 3> dM;
  dM.block(0, 0, 1, 3) << -drpy(2) * c2 * s3 - drpy(1) * c3 * s2, -drpy(2) * c3, 0;
  dM.block(1, 0, 1, 3) << drpy(2) * c3 * c2 - drpy(1) * s3 * s2, -drpy(2) * s3, 0;
  dM.block(2, 0, 1, 3) << -drpy(1) * c2, 0, 0;
  return dM;
}
// note: euler is in sequence of zyx,but rpy= [x,y,z]
template <typename T>
inline Eigen::Matrix<T, 3, 3> euler2rotm(Eigen::Matrix<T, 3, 1> rpy) {
  T c3 = cos(rpy[2]);
  T s3 = sin(rpy[2]);
  T c2 = cos(rpy[1]);
  T s2 = sin(rpy[1]);
  T c1 = cos(rpy[0]);
  T s1 = sin(rpy[0]);
  Eigen::Matrix<T, 3, 3> rotm;
  rotm << c2 * c3, c3 * s1 * s2 - c1 * s3, s1 * s3 + c1 * c3 * s2, c2 * s3, c1 * c3 + s1 * s2 * s3, c1 * s2 * s3 - c3 * s1, -s2, c2 * s1, c1 * c2;
  return rotm;
}

// rotm ro euler "ZYX",buy euler is[rx ry rz]
template <typename T>
Eigen::Matrix<T, 3, 1> rotm2euler(const Eigen::Matrix<T, 3, 3> R) {
  Eigen::Matrix<T, 3, 1> rpy;
  Eigen::Matrix<T, 4, 1> quat;
  quat = rotm2quat(R);
  rpy = quat2rpy(quat);
  return rpy;
}

template <typename T>
Eigen::Matrix<T, 3, 3> rotZ(T rz) {
  Eigen::Matrix<T, 3, 3> rot;
  rot << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
  return rot;
}
// rpy zyx but [rx ry rz],quat[w x y z]
template <typename T>
T pow2(T a) {
  return a * a;
}

template <typename T>
Eigen::Matrix<T, 3, 1> quat2rpy(const Eigen::Matrix<T, 4, 1> &quat) {
  Eigen::Matrix<T, 4, 1> q = quat;
  q = q / q.norm();
  Eigen::Matrix<T, 3, 1> rpy;
  T as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]), pow2(q[0]) + pow2(q[1]) - pow2(q[2]) - pow2(q[3]));
  rpy(1) = std::asin(as);
  rpy(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]), pow2(q[0]) - pow2(q[1]) - pow2(q[2]) + pow2(q[3]));
  return rpy;
}

template <typename T>
Eigen::Matrix<T, 4, 1> rotm2quat(const Eigen::Matrix<T, 3, 3> &r) {
  Eigen::Matrix<T, 4, 1> q;
  T tr = r.trace();
  if (tr > 0.0) {
    T S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (r(2, 1) - r(1, 2)) / S;
    q(2) = (r(0, 2) - r(2, 0)) / S;
    q(3) = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    T S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q(0) = (r(2, 1) - r(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (r(0, 1) + r(1, 0)) / S;
    q(3) = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    T S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q(0) = (r(0, 2) - r(2, 0)) / S;
    q(1) = (r(0, 1) + r(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (r(1, 2) + r(2, 1)) / S;
  } else {
    T S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q(0) = (r(1, 0) - r(0, 1)) / S;
    q(1) = (r(0, 2) + r(2, 0)) / S;
    q(2) = (r(1, 2) + r(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  q = q / q.norm();
  return q;
}

template <typename T>
Eigen::Matrix<T, 3, 3> quat2rotm(const Eigen::Matrix<T, 4, 1> &q) {
  T e0 = q(0);  // w
  T e1 = q(1);  // x
  T e2 = q(2);  // y
  T e3 = q(3);  // z
  Eigen::Matrix<T, 3, 3> R;
  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3),
      2 * (e2 * e3 - e0 * e1), 2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2);
  return R;
}

template <typename T>
Eigen::Matrix<T, 4, 1> quat2axisangle(const Eigen::Matrix<T, 4, 1> &quat)  //轴角axis*angle
{
  Eigen::Matrix<T, 3, 1> so3;
  Eigen::Matrix<T, 4, 1> axisangle;
  so3[0] = quat[1];
  so3[1] = quat[2];
  so3[2] = quat[3];
  T theta = 2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));
  if (fabs(theta) < 0.0000001) {
    so3.setZero();
  } else {
    so3 /= so3.norm();
  }
  axisangle << so3(0), so3(1), so3(2), theta;
  return axisangle;
}

template <typename T>
Eigen::Matrix<T, 4, 1> axisangle2quat(const Eigen::Matrix<T, 4, 1> &axisangle) {
  Eigen::Matrix<T, 4, 1> quat;
  Eigen::Matrix<T, 3, 1> axis;
  axis = axisangle.head(3);
  T theta = axisangle(3);
  if (axis.norm() < 1.e-6 || fabs(theta) < 1.e-6) {
    quat << 1, 0, 0, 0;
  } else {
    axis /= axis.norm();
    quat[0] = cos(theta / 2.);
    quat[1] = axis[0] * sin(theta / 2.);
    quat[2] = axis[1] * sin(theta / 2.);
    quat[3] = axis[2] * sin(theta / 2.);
  }
  return quat;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> axisangle2rotm(const Eigen::Matrix<T, 4, 1> &axisangle) {
  Eigen::Matrix<T, 4, 1> q = axisangle2quat(axisangle);
  Eigen::Matrix<T, 3, 3> R = quat2rotm(q);
  return R;
}
// transfer rotation matrix to axis angle ,result=[axis angle]4*1
template <typename T>
Eigen::Matrix<T, 4, 1> rotm2axisangle(const Eigen::Matrix<T, 3, 3> &R) {
  Eigen::Matrix<T, 4, 1> q = rotm2quat(R);
  Eigen::Matrix<T, 4, 1> result = quat2axisangle(q);
  return result;
}
#endif