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

  void calculateSwingTrajectory(Eigen::Vector3f psrc, Eigen::Vector3f pdes,
                                float Height, float phase, float swing_time,
                                int leg);
  // void setTrajStartPosition(Eigen::Vector3f p) { traj_p_src_ = p; };
  // void setTrajEndPosition(Eigen::Vector3f p) { traj_p_des_ = p; };
  void setTrajHeight(float p) { traj_height_ = p; };
  void getTraj(Eigen::Vector3f &pos, Eigen::Vector3f &vel,
               Eigen::Vector3f &acc);

private:
  Eigen::Vector3f kp_joint_;
  Eigen::Vector3f kd_joint_;
  Eigen::Vector3f foot_position_des_in_hip_;
  Eigen::Vector3f p_hip_abs_;
  Eigen::Vector4f swing_time_remaining_;
  Eigen::Matrix<float, 3, 4> foot_pos_world_target_;
  Eigen::Matrix<float, 3, 4> foot_pos_world_start_;
  // Eigen::Vector3f traj_p_src_;
  // Eigen::Vector3f traj_p_des_;
  Eigen::Matrix<float, 3, 4> traj_p_target_;
  Eigen::Matrix<float, 3, 4> traj_v_target_;
  Eigen::Matrix<float, 3, 4> traj_a_target_;
  float traj_height_;
  float traj_phase_;
  float traj_swing_time_;
  void CubicSpline(float &p, float &v, float &a, float q0, float q1, float t);
};
#endif
