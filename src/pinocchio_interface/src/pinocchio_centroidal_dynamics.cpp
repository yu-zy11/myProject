

#include "pinocchio_interface/pinocchio_centroidal_dynamics.h"
namespace pino {
Eigen::MatrixXd PinocchioCentroidalDynamics::GetCentroidalMomentumMap(const Eigen::VectorXd& qpos) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  pinocchio::computeCentroidalMap(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
  std::cout << "pino_ptr_->GetData()->Ag:\n" << pino_ptr_->GetData()->Ag << std::endl;
  return pino_ptr_->GetData()->Ag;
};

Eigen::MatrixXd PinocchioCentroidalDynamics::GetCentroidalMomentumMapTimeVariation(const Eigen::VectorXd& qpos,
                                                                                   const Eigen::VectorXd& qvel) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  assert(qvel.rows() == pino_ptr_->GetModel()->nv && "qvel size not match");
  pinocchio::computeCentroidalMapTimeVariation(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
  std::cout << "pino_ptr_->GetData()->dAg:\n" << pino_ptr_->GetData()->dAg << std::endl;
  return pino_ptr_->GetData()->dAg;
};

Eigen::MatrixXd PinocchioCentroidalDynamics::GetCentroidalMomentum(const Eigen::VectorXd& qpos,
                                                                   const Eigen::VectorXd& qvel) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  assert(qvel.rows() == pino_ptr_->GetModel()->nv && "qvel size not match");
  pinocchio::computeCentroidalMomentum(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
  Eigen::MatrixXd hg = Eigen::MatrixXd::Zero(6, 6);
  hg.block(0, 0, 3, 3) = pino_ptr_->GetData()->hg.angular();
  hg.block(3, 3, 3, 3) = pino_ptr_->GetData()->hg.linear();
  std::cout << "pino_ptr_->GetData()->H:\n" << hg << std::endl;
  return hg;
};

void PinocchioCentroidalDynamics::GetCentroidalMomentum(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                                        Eigen::MatrixXd& hg, Eigen::Vector3d& com,
                                                        Eigen::Vector3d& vel_com) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  assert(qvel.rows() == pino_ptr_->GetModel()->nv && "qvel size not match");
  pinocchio::computeCentroidalMomentum(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
  hg = Eigen::MatrixXd::Zero(6, 6);
  hg.block(0, 0, 3, 3) = pino_ptr_->GetData()->hg.angular();
  hg.block(3, 3, 3, 3) = pino_ptr_->GetData()->hg.linear();
  com = pino_ptr_->GetData()->com[0];
  vel_com = pino_ptr_->GetData()->vcom[0];
}

Eigen::MatrixXd PinocchioCentroidalDynamics::GetCentroidalMomentumTimeVariation(const Eigen::VectorXd& qpos,
                                                                                const Eigen::VectorXd& qvel,
                                                                                const Eigen::VectorXd& qacc) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  assert(qvel.rows() == pino_ptr_->GetModel()->nv && "qvel size not match");
  assert(qacc.rows() == pino_ptr_->GetModel()->nv && "qacc size not match");
  pinocchio::computeCentroidalMomentumTimeVariation(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel, qacc);
  Eigen::MatrixXd dhg = Eigen::MatrixXd::Zero(6, 6);
  dhg.block(0, 0, 3, 3) = pino_ptr_->GetData()->dhg.angular();
  dhg.block(3, 3, 3, 3) = pino_ptr_->GetData()->dhg.linear();
  std::cout << "pino_ptr_->GetData()->dHg:\n" << dhg << std::endl;
  return dhg;
}

void PinocchioCentroidalDynamics::GetCentroidalMomentumTimeVariation(const Eigen::VectorXd& qpos,
                                                                     const Eigen::VectorXd& qvel,
                                                                     const Eigen::VectorXd& qacc, Eigen::MatrixXd& hg,
                                                                     Eigen::MatrixXd& dhg, Eigen::Vector3d& com,
                                                                     Eigen::Vector3d& vel_com) {
  assert(qpos.rows() == pino_ptr_->GetModel()->nv && "qpos size not match");
  assert(qvel.rows() == pino_ptr_->GetModel()->nv && "qvel size not match");
  assert(qacc.rows() == pino_ptr_->GetModel()->nv && "qacc size not match");
  pinocchio::computeCentroidalMomentumTimeVariation(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel, qacc);
  hg = Eigen::MatrixXd::Zero(6, 6);
  hg.block(0, 0, 3, 3) = pino_ptr_->GetData()->hg.angular();
  hg.block(3, 3, 3, 3) = pino_ptr_->GetData()->hg.linear();
  dhg = Eigen::MatrixXd::Zero(6, 6);
  dhg.block(0, 0, 3, 3) = pino_ptr_->GetData()->dhg.angular();
  dhg.block(3, 3, 3, 3) = pino_ptr_->GetData()->dhg.linear();
  com = pino_ptr_->GetData()->com[0];
  vel_com = pino_ptr_->GetData()->vcom[0];
  std::cout << "pino_ptr_->GetData()->dHg:\n" << dhg << std::endl;
}

Eigen::Vector3d PinocchioCentroidalDynamics::GetEndEffectorPositionToComInWorldFrame(size_t endeffector_id) {
  return (pino_ptr_->GetData()->oMf[endeffector_id].translation() - pino_ptr_->GetData()->com[0]);
}

Eigen::Vector3d PinocchioCentroidalDynamics::GetEndEffectorPositionToComInWorldFrame(std::string endeffector_name) {
  int endeffector_id = pino_ptr_->GetLinkID(endeffector_name);
  return GetEndEffectorPositionToComInWorldFrame(endeffector_id);
}

}  // namespace pino
