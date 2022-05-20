#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

 #define ROS
int main(int argc, char **argv)
{
    #ifdef ROS
    ros::init(argc, argv, "mainController");
    ros::NodeHandle nh;    
    ros::Publisher control_data_pub = nh.advertise<std_msgs::String>("main_control", 1000);
    ros::Rate loopRate(10);

    int count{0};
    while (ros::ok())
    {
        std_msgs::String msg; 
        std::stringstream ss;
        ss<<"yuzhiyou"<< count;
        msg.data=ss.str();
        ROS_INFO("%s",msg.data.c_str());
        control_data_pub.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
        ++count;

    }
    // ros::spinOnce();
    ROS_INFO("init ROS in mainController,finished");

    #endif
    return 0;
}