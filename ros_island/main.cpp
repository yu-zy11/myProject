// #include "msgs_test/msgs_test.h"
#include "control_msgs/foot_cmd.h"
#include "ros_interface.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "msgs_test_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<control_msgs::foot_cmd>("msgs_test", 10);

  ros::Rate rate(10);

  control_msgs::foot_cmd foot_cmd_;
  RosInterface ros_tmp;
  ros_tmp.initRos();
  ros_tmp.rosPublish();

  while (ros::ok()) {
    pub.publish(foot_cmd_);

    ros::spinOnce();

    rate.sleep();
    // std::cout << "run well" << std::endl;
  }

  return 0;
}