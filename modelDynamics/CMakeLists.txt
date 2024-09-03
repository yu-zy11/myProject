cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
# message(${CMAKE_CURRENT_SOURCE_DIR})
# Set target name
project(robot_dynamics)
message("-- [${PROJECT_NAME}] building")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

option(BUILD_TESTS "Build tests" ON)
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    "../pinocchio_interface"
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE test_files  ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
list(REMOVE_ITEM souce_files ${test_files})

# message("source files: ${souce_files}")
# Add target
add_library(${PROJECT_NAME} ${souce_files})
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    pinocchio
    pinocchio_interface
    ${pinocchio_LIBRARIES}
)

if (BUILD_TESTS)
add_executable(test_kinematics ${test_files})
target_link_libraries(test_kinematics
    PUBLIC
    pinocchio
    pinocchio_interface
    ${pinocchio_LIBRARIES}
    ${PROJECT_NAME}
)
endif()
