cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(test_thread)

add_executable(main main.cpp src/getYmatrix_leg1.cpp)
find_package(Threads)
target_link_libraries(main ${CMAKE_THREAD_LIBS_INIT})

link_libraries(linearDynamics)

add_subdirectory(controller)
