

#ifndef PINOCCHIO_INTERFACE_PINOCCHIO_FULL_DYNAMICS_H_
#define PINOCCHIO_INTERFACE_PINOCCHIO_FULL_DYNAMICS_H_
#include "pinocchio_interface/pinocchio_interface.h"

namespace pino {
class PinocchioFullDynamics {
 public:
  PinocchioFullDynamics(std::shared_ptr<PinocchioInterface> pino_ptr) : pino_ptr_(pino_ptr){};
  // void UpdateFullDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  /** get joint space mass matrix
   * @param [in] qpos: joint position
   * @return joint space mass matrix
   */
  Eigen::MatrixXd GetJointSpaceMassMatrix(const Eigen::VectorXd& qpos);

  /** get inverse of joint space mass matrix
   * @param [in] qpos: joint position
   * @return inverse of joint space mass matrix
   */
  Eigen::MatrixXd GetJointSpaceMassMatrixInverse(const Eigen::VectorXd& qpos);

  /** get coriolis and gravity vector
   * @param [in] qpos: joint position
   * @param [in] qvel: joint velocity
   * @return coriolis and gravity vector
   */
  Eigen::VectorXd GetCoriolisAndGravity(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

 private:
  std::shared_ptr<PinocchioInterface> pino_ptr_;
};
}  // namespace pino
#endif
