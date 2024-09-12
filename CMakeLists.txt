cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(main)

# link_libraries(linearDynamics)
# link_libraries(qpOASES)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include(GetNamespace)
include(GetAliasNamespace)
include(AddGoogleTest)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(GTest REQUIRED)
find_package(glog REQUIRED)


# add_subdirectory("modelDynamics")

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
)
# set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/_install)
# add_executable(${PROJECT_NAME} robot_pinocchio_test.cpp)
# find_package(Threads)
# target_link_libraries(${PROJECT_NAME}
# PUBLIC
# ${CMAKE_THREAD_LIBS_INIT}
# # mpc_controller
# # gait
# # robotics
# pinocchio
# src::pinocchio_interface
# # glog::glog
# )

add_subdirectory(src)
# target_link_libraries(${PROJECT_NAME} linearDynamics)

# add_subdirectory(controller)

# target_link_libraries(mpc_controller)
