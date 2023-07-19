cmake_minimum_required(VERSION 3.0.2)
project(main)
add_compile_options(-std=c++11)

add_subdirectory(control_msgs)
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  rospy
  std_msgs
)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# message(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp ros_interface.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
