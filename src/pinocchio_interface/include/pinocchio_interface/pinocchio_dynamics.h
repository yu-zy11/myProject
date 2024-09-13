

#ifndef PINOCCHIO_INTERFACE_PINOCCHIO_DYNAMICS_H_
#define PINOCCHIO_INTERFACE_PINOCCHIO_DYNAMICS_H_

#include "pinocchio_interface/pinocchio_interface.h"

namespace pino {
class PinocchioDynamics {
 public:
  PinocchioDynamics(std::shared_ptr<PinocchioInterface> pino_ptr) : pino_ptr_(pino_ptr)(){};

  void UpdateCentroidalDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  void UpdateFullDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  std::shared_ptr<PinocchioInterface> pino_ptr_;
};
}  // namespace pino
#endif
