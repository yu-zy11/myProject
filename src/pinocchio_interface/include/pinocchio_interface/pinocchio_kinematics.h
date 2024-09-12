
#ifndef PINOCCHIO_INTERFACE_PINOCCHIO_KINEMATICS_H_
#define PINOCCHIO_INTERFACE_PINOCCHIO_KINEMATICS_H_

// clang-format off
#include "pinocchio_interface/pinocchio_interface.h"
// clang-format on
namespace pino {

class PinocchioKinematics {
 public:
  PinocchioKinematics(std::shared_ptr<PinocchioInterface> pino_ptr);
  /**
   * @brief compute forward kinematics and jacobian of robot, base link is fixed
   *
   * @param qpos joint positions of the robot, including base freejoint
   * @param qvel joint velocities of the robot,dimension is same with qpos
   * @param data output data of all end-effectors set by the user
   */
  void FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                  std::vector<EndEffectorData>& data);
  /** @brief compute inverse kinematics of endeffector including position and orientation,base link is fixed */
  void FixedBaseInverseKinematics(const Eigen::VectorXd& qpos_ref,
                                  std::vector<std::string> user_selected_end_effector_name,
                                  const std::vector<EndEffectorData>& data_des, Eigen::VectorXd& qpos_des);

  /** @brief compute inverse kinematics of endeffector position, used for point foot,base link is fixed */
  void FixedBaseInverseKinematics3Dof(const Eigen::VectorXd& qpos_ref, std::string end_effector_name,
                                      const Eigen::Vector3d& pos_des, Eigen::VectorXd& qpos_des);

  std::shared_ptr<PinocchioInterface> get_pinocchio_interface() { return pino_ptr_; }

 private:
  std::shared_ptr<PinocchioInterface> pino_ptr_;
};
}  // namespace pino
#endif
