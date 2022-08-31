#include <iostream>
#include <cstdint>
#include <string>
#include <cassert>
#include <chrono>

#include <thread>
#include "controller/ThirdPartyLibrary/linearDynamics/getRegressionMatrix/getRegressionMatrix.h"
void f1(int n)
{
    for (int i = 0; i < 50000; ++i)
    {
        std::cout << "Thread 1 executing\n";
        ++n;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
void writeA(int a)
{
    for (int i = 0; i < 50000; ++i)
    {
        std::cout << "Thread 2 executing\n";
        std::cout << "a1:" << a << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};
void writeB(int &a) { a = 2; };
int main()
{
    GetYmatrix Ymatrix;
    double a_q1{1}, a_q2{1}, a_q3{1};
    double g{9.81};
    double q1{1}, q2{1}, q3{1};
    double v_q1{2}, v_q2{2}, v_q3{2};
    double Y[180];
    Ymatrix.getYmatrix_leg1(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, Y);
    int leg{1};
    double  Y_data[];
    int Y_size[2];
    getRegressionMatrix(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, leg, Y_data[], Y_size);
    int a = 0;
    int n = 10;
    std::thread t2(f1, n + 1); // pass by value
    std::thread t1(writeA, a);
    while (true)
    {
        /* code */
    }

    for (int i = 0; i < 10; i++)
    {
        // std::thread t1(writeA, a);
        // std::thread t2(f1, n + 1); // pass by value
        // std::thread t2
        // writeA(a);
        // std::cout << "a1:" << a << std::endl;
        // std::thread thread2(writeB, std::ref(a));
        // std::cout << "a2:" << a << std::endl;
    }
    return 0;
}
