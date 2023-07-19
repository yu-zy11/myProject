#include "ros_interface.h"
void RosInterface::initRos() {
  int a{0};
  ros::init(a, NULL, "ros_island_node");
  ros::NodeHandle nh;
  _pub_foot_cmd = nh.advertise<control_msgs::foot_cmd>("foot_cmd", 10);
  _pub_foot_state = nh.advertise<control_msgs::foot_state>("foot_state", 10);
  _pub_imu = nh.advertise<control_msgs::imu_data>("imu", 10);
  _pub_joint_cmd = nh.advertise<control_msgs::joint_cmd>("joint_cmd", 10);
  _pub_joint_state = nh.advertise<control_msgs::joint_state>("joint_state", 10);
  _pub_trunk_cmd = nh.advertise<control_msgs::trunk_cmd>("trunk_cmd", 10);
  _pub_trunk_state = nh.advertise<control_msgs::trunk_state>("trunk_state", 10);

  std::cout << "rosinterface init finished" << std::endl;
}

void RosInterface::updateRosData() {
  //     _foot_cmd;
  //   control_msgs::foot_state _foot_state;
  //   control_msgs::imu_data _imu_data;
  //   control_msgs::joint_cmd _joint_cmd;
  //   control_msgs::joint_state _joint_state;
  //   control_msgs::trunk_cmd _trunk_cmd;
  //   control_msgs::trunk_state _trunk_state;
}
void RosInterface::rosPublish() {
  ros::Rate rate(1000);
  _foot_cmd.foot_position_cmd[0] = 1;
  while (ros::ok()) {
    std::cout << "foot cmmd " << _foot_cmd.foot_position_cmd[0] << std::endl;
    _pub_foot_cmd.publish(_foot_cmd);
    _pub_foot_state.publish(_foot_state);
    _pub_imu.publish(_imu_data);
    _pub_joint_cmd.publish(_joint_cmd);
    _pub_joint_state.publish(_joint_state);
    _pub_trunk_cmd.publish(_trunk_cmd);
    _pub_trunk_state.publish(_trunk_state);
    ros::spinOnce();
    rate.sleep();
  }
}