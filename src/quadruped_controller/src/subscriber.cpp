#include"ros/ros.h"
#include"std_msgs/String.h"
#include <iostream>

void callback(const std_msgs::String::ConstPtr & msg)
{
    // ROS_INFO(msg->data.c_str());
    std::cout<<"calback()\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("main_control", 1000, callback);
    ros::spin();
    return 0;
}