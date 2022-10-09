cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(main)

# link_libraries(linearDynamics)
link_libraries(qpOASES)

include_directories( "./controller/third_party/qpOASES/include")

add_executable(${PROJECT_NAME} main.cpp)
find_package(Threads)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# target_link_libraries(${PROJECT_NAME} linearDynamics)

add_subdirectory(controller)
