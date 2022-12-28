#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_
#include <eigen3/Eigen/Dense>
struct BodyData {
  Eigen::Vector3f position;
  Eigen::Vector3f rpy;
  Eigen::Vector4f quaternion;
  Eigen::Vector3f vel_in_body;
  Eigen::Vector3f vel_in_world;
  Eigen::Vector3f omega_in_body;
  Eigen::Vector3f omega_in_world;
  Eigen::Vector3f acc_in_body;
  Eigen::Vector3f acc_in_world;
  Eigen::Matrix<float, 3, 3> rotm_body2world;
};

struct LegData {
  Eigen::Matrix<float, 12, 1> joint_position;
  Eigen::Matrix<float, 12, 1> joint_vel;
  Eigen::Matrix<float, 12, 1> joint_acc;
  Eigen::Matrix<float, 12, 1> joint_torque;
  Eigen::Matrix<float, 12, 1> foot_contact_force;
  Eigen::Matrix<float, 12, 1> foot_position_in_body;
  Eigen::Matrix<float, 12, 1> foot_position_in_world;
  Eigen::Matrix<float, 12, 1> foot_vel;
  Eigen::Matrix<float, 3, 12> foot_jacobian_in_body;
  Eigen::Matrix<float, 3, 12> foot_dot_jacobian;
};
struct GaitInfo {
  float period;
  float swing_duration;
  float stance_duration;
  // Eigen::Vector4f phase_offset;
  // Eigen::Vector4f stance_ratio;
  Eigen::Vector4f swing_phase;
  Eigen::Vector4f stance_phase;
  Eigen::Vector4i contact_state;
};

// topic: /locomotion
struct FusionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BodyData body;
  LegData leg;
  GaitInfo gait;
};

struct commandData {
  BodyData body;
};
#endif
