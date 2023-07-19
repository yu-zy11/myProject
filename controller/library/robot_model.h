#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H
#include "robotic_types.h"

namespace ROBOTICS {
class RobotModel {
private:
  /* data */
public:
  RobotModel() {
    joint_seq_ << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
    joint_parent_ << -1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10; // -1:base
    foot_seq_ << 0, 1, 2, 3;
    foot_parent_ << 2, 5, 8, 11;
    joint_axis_[0] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; // joint 0
    joint_axis_[1] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[2] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[3] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[4] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[5] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[6] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[7] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[8] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[9] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[10] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    joint_axis_[11] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; // joint 11

    joint_position_[0] << 0.1875, -0.0875, 0.18, 0, 0, 0;
    joint_position_[1] << 0.0, -0.0775, 0.0, 0, 0, 0;
    joint_position_[2] << 0.0, 0.0, -0.3095, 0, 0, 0;
    joint_position_[3] << 0.1875, 0.0875, 0.18, 0, 0, 0;
    joint_position_[4] << 0.0, 0.0775, 0.0, 0, 0, 0;
    joint_position_[5] << 0.0, 0.0, -0.3095, 0, 0, 0;
    joint_position_[6] << -0.1875, -0.0875, 0.0, 0, 0, 0;
    joint_position_[7] << 0.0, -0.0775, 0.0, 0, 0, 0;
    joint_position_[8] << 0.0, 0.0, -0.225, 0, 0, 0;
    joint_position_[9] << -0.1875, 0.0875, 0.0, 0, 0, 0;
    joint_position_[10] << 0.0, 0.0775, 0.0, 0, 0, 0;
    joint_position_[11] << 0.0, 0.0, -0.225, 0, 0, 0;

    foot_position_[0] << 0, 0, -0.32, 0, 0, 0;
    foot_position_[1] << 0, 0, -0.32, 0, 0, 0;
    foot_position_[2] << -0.01, 0, -0.26, 0, 0, 0;
    foot_position_[3] << -0.01, 0, -0.26, 0, 0, 0;
  }
  Vec12<int> joint_seq_, joint_parent_;
  Vec4<int> foot_seq_, foot_parent_;
  Vec6<ktype> joint_axis_[12];
  Vec6<ktype> joint_position_[12]; // xyzrpy
  Vec6<ktype> foot_position_[4];
};

} // namespace ROBOTICS

#endif
