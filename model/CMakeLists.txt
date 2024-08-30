cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")


# Set target name
project(model)
message("-- [${PROJECT_NAME}] building")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)


include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    # ${pinocchio_interface_INCLUDE_DIRS}
    "../pinocchio_interface"
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
message("source files: ${souce_files}")
add_library(${PROJECT_NAME} ${souce_files})
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    pinocchio
    pinocchio_interface
    ${pinocchio_LIBRARIES}
)
