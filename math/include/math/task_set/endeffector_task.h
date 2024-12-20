#include <math.h>
#include <iostream>

#include "../task.h"

namespace math {

class EndeffectorTask : public Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EndeffectorTask(int weight = 10) : Task(weight) {}
  void CalculateJacobian() override {
    jacobian_ = Eigen::Matrix3d::Zero();
    jacobian_(0, 0) = 4 * std::pow(state_[0], 3) + 6 * std::pow(state_[0], 2) + 3;
    jacobian_(1, 1) = 8 * std::pow(state_[1], 3) + 12 * std::pow(state_[1], 2) + 8;
    jacobian_(2, 2) = -8 * std::pow(state_[2], 3) + 18 * std::pow(state_[2], 2) + 10;
  }
  void CalculateValue() override {
    value_.resize(3);
    value_[0] = std::pow(state_[0], 4) + 2 * std::pow(state_[0], 3) + 3 * std::pow(state_[0], 1);
    value_[1] = 2 * std::pow(state_[1], 4) + 4 * std::pow(state_[1], 3) + 8 * std::pow(state_[1], 1);
    value_[2] = -2 * std::pow(state_[2], 4) + 6 * std::pow(state_[2], 3) + 10 * std::pow(state_[2], 1);
  }
  virtual void CalculateConstraint() {
    A_.resize(0, 3);
    ub_.resize(0);
  };
};
}  // namespace math
