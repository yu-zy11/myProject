cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(main)

# link_libraries(linearDynamics)
# link_libraries(qpOASES)

# include_directories( "./pinocchio_interface")
add_subdirectory("pinocchio_interface")
add_subdirectory("modelDynamics")
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    "./pinocchio_interface"
    "./modelDynamics"
)

add_executable(${PROJECT_NAME} robot_pinocchio_test.cpp)
find_package(Threads)
target_link_libraries(${PROJECT_NAME}
PUBLIC
${CMAKE_THREAD_LIBS_INIT}
# mpc_controller
# gait
# robotics
pinocchio
pinocchio_interface
robot_dynamics)


# target_link_libraries(${PROJECT_NAME} linearDynamics)

# add_subdirectory(controller)

# target_link_libraries(mpc_controller)
