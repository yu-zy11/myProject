cmake_minimum_required(VERSION 3.0.2)
project(quadruped_controller)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # geometry_msgs
  # message_generation
)

# add_message_files(
#   FILES
#   #pybullet.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   # geometry_msgs
# )

# catkin_package(CATKIN_DEPENDS  message_runtime std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS   std_msgs )
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mainController src/mainController.cpp)
target_link_libraries(mainController ${catkin_LIBRARIES})
# add_dependencies(mainController beginner_tutorials_generate_messages_cpp)
add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
# add_dependencies(subscriber beginner_tutorials_generate_messages_cpp)

