#include <iostream>
#include<cstdint> 
#include <string>
#include <cassert>

#include "PersonalLibrary/transformationMath.h"
#include "PersonalLibrary/constant.h"
#ifndef tm
#define tm yTransMath
#endif
//template <typename T>
int main()
{
    // int a=yTransMath::Addyu();
    tm::print(3,2);
     std::cout << "yTransMath" << tm::Addyu(4, 5) << "\n";
    // std::cout << "yTransMath" << tm::Addyu(4, 5) << "\n";
    std::cout<<"pi:"<<constant::pi<<"G:"<<g_G<< std::endl;
    std::cout<<"static_cast"<<static_cast<double>(1)<<"G:"<<g_G<< std::endl;
    assert(5<0&&"wrong g");
    int a=(5>3)?3:5;
    std::cout<<"()"<<a<<std::endl;
    return 0;
}
