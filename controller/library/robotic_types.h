#ifndef ROBOTIC_TYPES_H
#define ROBOTIC_TYPES_H
#include <Eigen/Dense>
namespace ROBOTICS {
template <typename T> using Vec3 = Eigen::Matrix<T, 3, 1>;
template <typename T> using Vec4 = Eigen::Matrix<T, 4, 1>;
template <typename T> using Vec6 = Eigen::Matrix<T, 6, 1>;
template <typename T> using Vec12 = Eigen::Matrix<T, 12, 1>;
template <typename T> using Vecx = Eigen::Matrix<T, -1, 1>;

template <typename T> using Mat3 = Eigen::Matrix<T, 3, 3>;
template <typename T> using Mat4 = Eigen::Matrix<T, 4, 4>;
template <typename T> using Mat6 = Eigen::Matrix<T, 6, 6>;
template <typename T> using Matxx = Eigen::Matrix<T, -1, -1>;
template <typename T> using Mat6x = Eigen::Matrix<T, 6, -1>;

typedef float ktype; // type of Class Kinematics
} // namespace ROBOTICS
#endif
