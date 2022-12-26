#ifndef SWING_LEG_PLANNER_H
#define SWING_LEG_PLANNER_H
#include "../common_data/common_data.h"
#include <Eigen/Dense>
class SwingLegPlanner {
public:
  void init();
  void update(const FusionData &state, const commandData &cmd);
  void getFootTarget();
  void getJointTarget();
  int a = 1;

private:
  int b = 1;
  Eigen::Vector3f kp_joint_;
  Eigen::Vector3f kd_joint_;
  Eigen::Vector3f foot_position_des_in_hip_;
  Eigen::Vector3f p_hip_abs_;
};
#endif
