#include "pinocchio_interface/pinocchio_kinematics.h"
#include "pinocchio_interface/pseudo_inverse.h"
// PinocchioKinematics::PinocchioKinematics() {
namespace pino {
void PinocchioDynamics::computeDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
  // computeKinematics(qpos, qvel);
  pinocchio::crba(*model_ptr_, *data_ptr_, qpos);
  pinocchio::nonLinearEffects(*model_ptr_, *data_ptr_, qpos, qvel);
  pinocchio::computeMinverse(*model_ptr_, *data_ptr_, qpos);
}

}  // namespace pino
