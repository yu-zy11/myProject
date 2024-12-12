#ifndef CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_FK_NET_H_
#define CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_FK_NET_H_
#include <Eigen/Dense>

namespace estimator {

// TODO(qsy): Format the code
void FK(Eigen::VectorXd X, Eigen::VectorXd& Y);

}  // namespace estimator
#endif  // CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_FK_NET_H_
