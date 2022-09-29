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