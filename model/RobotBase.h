/*! @file RobotBase.h
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the RobotBase class.  This stores all the parameters for
 * a quadruped robot.
 */
#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H
// clang-format off
#include "PinocchioInterface.h"
#include <eigen3/Eigen/StdVector>
#include <vector>

#define A01_URDF_RELATIVE_PATH "/home/work/workspace/biped_robot/config/model/SA01p.urdf"
// clang-format on

enum class RobotType { SA01a, SA01b };
template <typename T>
struct endEffectorData {
  Eigen::Matrix<T, 3, 1> pos;
  Eigen::Matrix<T, 3, 3> rotm;
  Eigen::Matrix<T, 3, 1> vel;
  Eigen::Matrix<T, 3, 1> omega;
  Eigen::Matrix<T, 6, Eigen::Dynamic> jacobian;
  Eigen::Matrix<T, 6, Eigen::Dynamic> jacobian_derivative;
  void reshape(int size) {
    jacobian.resize(6, size);
    jacobian_derivative.resize(6, size);
    jacobian.setZero();
    jacobian_derivative.setZero();
  }
};

template <typename T>
class RobotBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // std::string base_link_name_;
  int base_link_id_;
  // std::vector<std::string> end_effector_name_;
  std::vector<int> end_effector_id_;
  std::vector<endEffectorData<T>> data_;

  PinocchioInterface pino_;
  RobotType robot_type_;
  // T body_mass_;
  // T root_mass_;
  // T _abadLinkLength, _hipLinkLength, _kneeLinkLength, _maxLegLength;
  // Eigen::Matrix<T, 3, 3> root_inertia_;

  // vectorAligned<Vec6<T>> damping_, frictionloss_;
  // vectorAligned<Vec3<T>> hip_pos_;
  // Vec3<T> root_com_;

  /*maximum limits of joint positions for each leg*/
  Eigen::VectorXd qpos_upper_limit_;
  /*minimum limits of joint positions for each leg*/
  Eigen::VectorXd qpos_lower_limit_;
  /*maximum limits of joint velocities for each leg*/
  Eigen::VectorXd qvel_limit_;
  /*minimum limits of joint torque for each leg*/
  Eigen::VectorXd qtor_limit_;
  /**
   * @brief compute forward kinematics and jacobian of robot, base link is fixed
   *
   * @param qpos joint positions of the robot,the dimension is same with actuator number,not include base freejoint
   * @param qvel joint velocities of the robot,dimension is same with qpos
   * @param data output data of all end-effectors set by the user
   */
  virtual void localForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data) = 0;

  /**
   * @brief compute forward kinematics and jacobian of robot, base link is free
   *
   * @param qpos joint positions of the robot,the dimension  actuator + base freejoint
   * @param qvel joint velocities of the robot,dimension is same with qpos
   * @param data output data of all end-effectors set by the user
   */
  virtual void fullForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data) = 0;
  /**
   * @brief compute the joint position and velocity of robot given the desired position and velocity of end effectors
   * @param qpos_ref [in] joint position reference
   * @param user_selected_end_effector_id [in] the id of end effectors that are selected for inverse kinematics,user_selected_end_effector_id=0 means
   * endefect id is end_effector_id_[0]
   * @param data [in] the current data of end effectors
   * @param qpos_des [out] joint position result
   */
  virtual void localInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<int> user_selected_end_effector_id, const std::vector<endEffectorData<T>>& data_des,
                                      Eigen::Matrix<T, -1, 1>& qpos_des) = 0;

  /**
   * @brief compute the joint position and velocity of robot given the desired position and velocity of end effectors
   * @param qpos_ref [in] joint position reference
   * @param user_selected_end_effector_id [in] the id of end effectors that are selected for inverse kinematics,user_selected_end_effector_id=0 means
   * endefect id is end_effector_id_[0]
   * @param data [in] the current data of end effectors
   * @param qpos_des [out] joint position result
   */
  virtual void fullInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<int> user_selected_end_effector_id, const std::vector<endEffectorData<T>>& data_des,
                                     Eigen::Matrix<T, -1, 1>& qpos_des) = 0;
};

#endif  // LIBBIOMIMETICS_QUADRUPED_H
