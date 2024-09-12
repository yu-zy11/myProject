
#ifndef ROBOT_BASH_H
#define ROBOT_BASH_H
// clang-format off
#include "PinocchioInterface.h"
#include <eigen3/Eigen/StdVector>
#include <vector>

#define A01_URDF_RELATIVE_PATH "/home/ubuntu/workspace/myProject/modelDynamics/SA01p.urdf"
// clang-format on

enum class RobotType { SA01a, SA01b };
template <typename T>
struct EndEffectorData {
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
  // must be initialized in child class
  std::string base_link_name_ = "base_link";
  // must be initialized in child class
  std::vector<std::string> end_effector_name_ = {"leg_l_foot_center", "leg_r_foot_center"};
  // must be initialized in child class
  RobotType robot_type_;

  // read from pinocchio
  // int base_link_id_;
  std::vector<EndEffectorData<T>> data_;
  // pinocchio interface
  PinocchioInterface pino_;

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
  virtual void localForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel,
                                      std::vector<EndEffectorData<T>>& data);

  /**
   * @brief compute forward kinematics and jacobian of robot, base link is free
   *
   * @param qpos joint positions of the robot,the dimension  actuator + base freejoint
   * @param qvel joint velocities of the robot,dimension is same with qpos
   * @param data output data of all end-effectors set by the user
   */
  virtual void fullForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel,
                                     std::vector<EndEffectorData<T>>& data);
  /**
   * @brief compute the joint position and velocity of robot given the desired position and velocity of end effectors
   * @param qpos_ref [in] joint position reference
   * @param user_selected_end_effector_name [in] he name of endeffectors that are selected for inverse kinematics
   * @param data [in] the current data of end effectors
   * @param qpos_des [out] joint position result
   */
  virtual void localInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref,
                                      std::vector<std::string> user_selected_end_effector_name,
                                      const std::vector<EndEffectorData<T>>& data_des,
                                      Eigen::Matrix<T, -1, 1>& qpos_des);

  /**
   * @brief compute the joint position and velocity of robot given the desired position and velocity of end effectors
   * @param qpos_ref [in] joint position reference
   * @param user_selected_end_effector_name [in] the name of endeffectors that are selected for inverse kinematics
   * @param data [in] the current data of end effectors
   * @param qpos_des [out] joint position result
   */
  virtual void fullInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref,
                                     std::vector<std::string> user_selected_end_effector_name,
                                     const std::vector<EndEffectorData<T>>& data_des,
                                     Eigen::Matrix<T, -1, 1>& qpos_des);

 private:
  void limitJointPosition(Eigen::Matrix<T, -1, 1>& qpos);
  int getIndexInEndEffectorDatas(const std::string& name);
};

#endif  // LIBBIOMIMETICS_QUADRUPED_H
