#include <eigen3/Eigen/Dense>
template <typename T>
inline Eigen::Matrix<T,3,3> cross2mat(Eigen::Matrix<T,3,1> vec);
template <typename T>
inline Eigen::Matrix<T,3,3> omega2rpyrate(Eigen::Matrix<T,3,1> rpy);
template <typename T>
inline Eigen::Matrix<T,3,3> rpyrate2omega(Eigen::Matrix<T,3,1> rpy);