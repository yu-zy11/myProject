#include "math_utils.h"
template <typename T>
inline Eigen::Matrix<T,3,3> cross2mat(Eigen::Matrix<T,3,1> vec)
{
    Eigen::Matrix<T,3,3> mat;
    mat<<0, -vec[2],vec[1], 
         vec[2], 0 ,-vec[0],
         -vec[1], vec[0],0; 
    return mat; 
}

template <typename T>
inline Eigen::Matrix<T,3,3> omega2rpyrate(Eigen::Matrix<T,3,1> rpy)  //euler zyx
{
    T c3 = cos(rpy[2]);
    T s3 = sin(rpy[2]);
    T c2 = cos(rpy[1]);
    T s2 = sin(rpy[1]);
    Eigen::Matrix<T, 3, 3> mat_omega2rpy;
    mat_omega2rpy << c3 / c2, s3 / c2, 0,
        -s3, c3, 0,
        c3 * s2 / c2, s3 * s2 / c2, 1;
    return mat_omega2rpy
}

template <typename T>
inline Eigen::Matrix<T,3,3> rpyrate2omega(Eigen::Matrix<T,3,1> rpy)
{
    T c3 = cos(rpy[2]);
    T s3 = sin(rpy[2]);
    T c2 = cos(rpy[1]);
    T s2 = sin(rpy[1]);
    Eigen::Matrix<T, 3, 3> ryprate2omega;
    ryprate2omega << c2*c3, -s3, 0,
                    c2*s3, c3, 0,
                    -s2, 0, 1;
    return ryprate2omega
}