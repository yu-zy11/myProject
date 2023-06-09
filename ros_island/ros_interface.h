#include "control_msgs/foot_cmd.h"
#include "control_msgs/foot_state.h"
#include "control_msgs/imu_data.h"
#include "control_msgs/joint_cmd.h"
#include "control_msgs/joint_state.h"
#include "control_msgs/trunk_cmd.h"
#include "control_msgs/trunk_state.h"
#include <iostream>
#include <ros/ros.h>
#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H
class RosInterface {
public:
  void initRos();
  void updateRosData();
  void rosPublish();

private:
  control_msgs::foot_cmd _foot_cmd;
  control_msgs::foot_state _foot_state;
  control_msgs::imu_data _imu_data;
  control_msgs::joint_cmd _joint_cmd;
  control_msgs::joint_state _joint_state;
  control_msgs::trunk_cmd _trunk_cmd;
  control_msgs::trunk_state _trunk_state;
  ros::Publisher _pub_foot_cmd, _pub_foot_state, _pub_imu, _pub_joint_cmd,
      _pub_joint_state, _pub_trunk_cmd, _pub_trunk_state;
};
#endif