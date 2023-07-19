cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(pinocchio_interface)
message("-- [pinocchio_interface] building")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)
# include_directories("/opt/openrobots/include") #包含头文件目录
# link_directories("/opt/openrobots/lib")  #库文件目录


add_library(${PROJECT_NAME} pinocchio_interface.cpp)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    pinocchio
    ${pinocchio_LIBRARIES}
)
