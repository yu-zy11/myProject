cmake_minimum_required(VERSION 3.0.2)
project(quadruped_controller)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/mpc_controller)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/gait)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/library)
