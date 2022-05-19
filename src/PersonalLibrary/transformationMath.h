#ifndef TRANSFORMATIONMATH_H
#define TRANSFORMATIONMATH_H
#include <string>

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
    void print_reference_value(double const & num);
    //int* ptr{nullptr}; //申明空指针，表示还没有指向任何对象的地址=int
    //int* ptra{};  //隐式申明空指针，避免悬空指针：int* ptr
    const std::string & printName();
    struct Student{
        std::string name{};
        double grade{};
    };
    enum Color{red,black,yellow,}; //unscoped 
    enum class Pet{dog,cat,}; //scoped
}

#endif