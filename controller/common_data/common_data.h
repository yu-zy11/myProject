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
  Eigen::Matrix<float, 3, 12> foot_jacobian;
  Eigen::Matrix<float, 3, 12> foot_dot_jacobian;
};

// topic: /locomotion
struct FusionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BodyData torso;
  LegData leg;
};

struct commandData {
  BodyData torso_cmd;
}

// struct GaitInfo {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   GaitInfo() { zero(); }

//   void zero() {
//     gait_flag = 0;
//     contact_target.setOnes();
//     swing_phase.setZero();
//     stance_phase.setOnes();
//   }

//   int gait_flag; // 0: no cyclic gait, 1: cyclic gait, such as trot, walk and
//   so
//                  // on
//   Vec4<float> contact_target;
//   Vec4<float> swing_phase;
//   Vec4<float> stance_phase;
// };
#endif
