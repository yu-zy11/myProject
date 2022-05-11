#ifndef TRANSFORMATIONMATH_H
#define TRANSFORMATIONMATH_H

//template <typename T>
//T Addyu(T x, T y);

namespace yTransMath
{
    constexpr double pi{3.1415926};
    template <typename T>
    T Addyu(T x, T y)
    {
        // std::cout<<"addyu~\n";
        return x + y;
    }
    void print(int x,int y);
}

#endif