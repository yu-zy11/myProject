#ifndef CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_FK_NET_H_
#define CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_FK_NET_H_
#include <Eigen/Dense>

namespace core {
namespace estimator {

// TODO(qsy): Format the code
void FK(Eigen::VectorXd X, Eigen::VectorXd& Y);

}  // namespace estimator
}  // namespace core
#endif  // CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_FK_NET_H_
