#include <eigen3/Eigen/Dense>
#ifndef MATH_UTILS
#define MATH_UTILS
template <typename T>
inline Eigen::Matrix<T,3,3> cross2mat(Eigen::Matrix<T,3,1> vec)
 {
     Eigen::Matrix<T, 3, 3> mat;
     mat << 0, -vec[2], vec[1],
         vec[2], 0, -vec[0],
         -vec[1], vec[0], 0;
     return mat;
 }

template <typename T>
inline Eigen::Matrix<T, 3, 3> omega2rpyrate(Eigen::Matrix<T, 3, 1> rpy)
{
    T c3 = cos(rpy[2]);
    T s3 = sin(rpy[2]);
    T c2 = cos(rpy[1]);
    T s2 = sin(rpy[1]);
    Eigen::Matrix<T, 3, 3> mat_omega2rpy;
    mat_omega2rpy << c3 / c2, s3 / c2, 0,
        -s3, c3, 0,
        c3 * s2 / c2, s3 * s2 / c2, 1;
    return mat_omega2rpy;
}
template <typename T>
inline Eigen::Matrix<T,3,3> rpyrate2omega(Eigen::Matrix<T,3,1> rpy)
 {
     T c3 = cos(rpy[2]);
     T s3 = sin(rpy[2]);
     T c2 = cos(rpy[1]);
     T s2 = sin(rpy[1]);
     Eigen::Matrix<T, 3, 3> ryprate2omega;
     ryprate2omega << c2 * c3, -s3, 0,
         c2 * s3, c3, 0,
         -s2, 0, 1;
     return ryprate2omega;
 }

template <typename T>
inline Eigen::Matrix<T,3,3> euler2rotm(Eigen::Matrix<T,3,1> rpy) //note: euler is in sequence of zyx,but rpy= [x,y,z]
 {
     T c3 = cos(rpy[2]);
     T s3 = sin(rpy[2]);
     T c2 = cos(rpy[1]);
     T s2 = sin(rpy[1]);
     T c1 = cos(rpy[0]);
     T s1 = sin(rpy[0]);
     Eigen::Matrix<T, 3, 3> rotm;
     rotm <<c2*c3, c3*s1*s2 - c1*s3, s1*s3 + c1*c3*s2,
            c2*s3, c1*c3 + s1*s2*s3, c1*s2*s3 - c3*s1,
              -s2,            c2*s1,            c1*c2;
     return rotm;
 }



#endif