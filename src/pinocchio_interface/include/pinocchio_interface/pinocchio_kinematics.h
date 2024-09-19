
#ifndef PINOCCHIO_INTERFACE_PINOCCHIO_KINEMATICS_H_
#define PINOCCHIO_INTERFACE_PINOCCHIO_KINEMATICS_H_

#include "pinocchio_interface/pinocchio_interface.h"
namespace core {
namespace pinocchio_interface {

class PinocchioKinematics {
 public:
  static constexpr double kMaxError = 1e-4;
  static constexpr int kMaxIterationCounter = 20;
  static constexpr double kMaxPositionError = 0.15;
  static constexpr double kMaxOrietationError = 0.4;
  PinocchioKinematics(std::shared_ptr<PinocchioInterface> pino_ptr) : pino_ptr_(pino_ptr){};
  /**
   * @brief compute forward kinematics of floating base robot.
   * @param qpos generalized joint positions of the robot.
   * @param qvel generalized joint joint velocities.
   * @param data  all end-effectors datas
   */
  void FullModelForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                  std::vector<EndEffectorData>& data);

  /**
   * @brief compute forward kinematics of robot,set joint positions for base are all 0.
   * @param qpos generalized joint positions of the robot.
   * @param qvel generalized joint joint velocities.
   * @param data  all end-effectors datas
   */
  void FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                  std::vector<EndEffectorData>& data);
  std::vector<EndEffectorData> FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);
  /**
   * @brief compute forward kinematics of robot,set joint positions for base are all 0.
   * @param qpos generalized joint positions of the robot.
   * @param data  all end-effectors datas
   */
  void FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, std::vector<EndEffectorData>& data);

  /** @brief compute inverse kinematics of selected endeffectors including position and orientation, set all joints for
   * base to be  0
   * @param qpos_ref generalized joint positions of the robot.
   * @param user_selected_end_effector_name name of the end effector
   * @param data_des position and orientation of the end effector
   * @param qpos_des result of joint positions
   */
  void FixedBaseInverseKinematics(const Eigen::VectorXd& qpos_ref,
                                  std::vector<std::string> user_selected_end_effector_name,
                                  const std::vector<EndEffectorData>& data_des, Eigen::VectorXd& qpos_des);
  /** @brief compute inverse kinematics of one endeffector including position and orientation, set all joints for base
   * to be  0
   * @param qpos_ref generalized joint positions of the robot.
   * @param user_selected_end_effector_name name of the end effector
   * @param pos_des target position  of the end effector
   * @param rotm_des target orientation of the end effector
   * @param qpos_des result of joint positions
   */
  void FixedBaseInverseKinematics(const Eigen::VectorXd& qpos_ref, const std::string& end_effector_name,
                                  const Eigen::Vector3d& pos_des, const Eigen::Matrix3d& rotm_des,
                                  Eigen::VectorXd& qpos_des);

  /**  compute inverse kinematics of  position,recomand to use for point foot,base link is fixed
   * @param qpos_ref joint positions of the robot, including base freejoint
   * @param end_effector_name name of the end effector
   * @param pos_des position of the end effector
   * @param qpos_des joint positions of the robot
   */
  void FixedBaseInverseKinematics3Dof(const Eigen::VectorXd& qpos_ref, const std::string& end_effector_name,
                                      const Eigen::Vector3d& pos_des, Eigen::VectorXd& qpos_des);

  std::shared_ptr<PinocchioInterface> GetInterface() { return pino_ptr_; }

  std::shared_ptr<PinocchioModelInfo> GetModelInfo() { return pino_ptr_->GetModelInfo(); }

  /** transfer joint id to jacobian columns
   * @param [in] joint_id: joint id
   * @return  columns id in jacobian matrix.
   */
  int JointIdToJacobianColumns(const int& joint_id);

 private:
  std::shared_ptr<PinocchioInterface> pino_ptr_;
};
}  // namespace pinocchio_interface
}  // namespace core
#endif
