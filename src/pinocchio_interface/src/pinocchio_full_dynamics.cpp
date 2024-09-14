
#include "pinocchio_interface/pinocchio_full_dynamics.h"
namespace pino {

// void PinocchioFullDynamics::UpdateFullDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
//   // assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
//   // assert(qvel.rows() == pino_ptr_->GetModel()->nv && "qvel size not match");
//   // pinocchio::forwardKinematics(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
//   // pinocchio::updateFramePlacements(*pino_ptr_->GetModel(), *pino_ptr_->GetData());
//   // pinocchio::computeJointJacobians(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
//   // pinocchio::crba(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
// }

Eigen::MatrixXd PinocchioFullDynamics::GetJointSpaceMassMatrix(const Eigen::VectorXd& qpos) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  pinocchio::crba(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(pino_ptr_->GetModel()->nv, pino_ptr_->GetModel()->nv);
  M.triangularView<Eigen::Upper>() = pino_ptr_->GetData()->M.triangularView<Eigen::Upper>();
  M.triangularView<Eigen::Lower>() = pino_ptr_->GetData()->M.transpose().triangularView<Eigen::Lower>();
  return M;
}

Eigen::MatrixXd PinocchioFullDynamics::GetJointSpaceMassMatrixInverse(const Eigen::VectorXd& qpos) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  pinocchio::computeMinverse(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
  Eigen::MatrixXd Minv = Eigen::MatrixXd::Zero(pino_ptr_->GetModel()->nv, pino_ptr_->GetModel()->nv);
  Minv.triangularView<Eigen::Upper>() = pino_ptr_->GetData()->Minv.triangularView<Eigen::Upper>();
  Minv.triangularView<Eigen::Lower>() = pino_ptr_->GetData()->Minv.transpose().triangularView<Eigen::Lower>();
  return Minv;
}

Eigen::VectorXd PinocchioFullDynamics::GetCoriolisAndGravity(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
  pinocchio::nonLinearEffects(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
  return pino_ptr_->GetData()->nle;
};

}  // namespace pino
