cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(main)

# link_libraries(linearDynamics)
# link_libraries(qpOASES)

# include_directories( "./controller/third_party/qpOASES/include")
add_subdirectory("pinocchio_interface")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(pinocchio REQUIRED)
include_directories("/opt/openrobots/include") #包含头文件目录
link_directories("/opt/openrobots/lib")  #库文件目录
include_directories("./pinocchio_interfacce")

add_executable(${PROJECT_NAME} pinocchio_test.cpp)
find_package(Threads)
target_link_libraries(${PROJECT_NAME}
PUBLIC
${CMAKE_THREAD_LIBS_INIT}
# mpc_controller
# gait
# robotics
pinocchio
pinocchio_interface)


# target_link_libraries(${PROJECT_NAME} linearDynamics)

# add_subdirectory(controller)

# target_link_libraries(mpc_controller)
