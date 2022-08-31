#include"ros/ros.h"
#include"std_msgs/String.h"
#include <iostream>
#include "ros_pybullet_msg/pybullet_msg.h"

void callback(const ros_pybullet_msg::pybullet_msg& msg)
{
    // ROS_INFO(msg->data.c_str());
    float kp_joint1=msg.kp_joint[0];
    std::cout<<"calback()"<<"kp_joint1:"<<kp_joint1<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("pybullet_control_data", 1000, callback);
    ros::spin();
    return 0;
}