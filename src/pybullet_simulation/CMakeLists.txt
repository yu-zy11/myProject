cmake_minimum_required(VERSION 3.0.2)
project(pybullet_simulation)
# set(CMAKE_CXX_STANDARD 17 )
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

# add_message_files(
#   FILES
#   # pybullet.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS  message_runtime std_msgs geometry_msgs)

###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

