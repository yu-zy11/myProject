#pragma once

#include <Eigen/Dense>
namespace math {

// A class for task definition: weight*f(x) =weight* target_, Ax< = ub
class Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Task(int weight = 1) {
    if (weight < 0.0) {
      throw std::runtime_error("weight must be positive");
    }
    weight_ = weight;
  }
  virtual void CalculateJacobian() = 0;
  virtual void CalculateValue() = 0;
  virtual void CalculateConstraint() = 0;
  void SetTarget(const Eigen::VectorXd& target) { target_ = target; }
  void SetWeight(double weight) { weight_ = weight; }
  virtual void UpdateCommon(const Eigen::VectorXd& state) = 0;
  void Update(const Eigen::VectorXd& state) {
    state_ = state;
    CalculateJacobian();
    CalculateValue();
    CalculateConstraint();
    Check();
  };

  void Check() {
    if (jacobian_.rows() != value_.size()) {
      throw std::runtime_error("rows of jacobian and value not equal");
    }
    if (jacobian_.cols() != state_.size()) {
      throw std::runtime_error("cols of jacobian and state not equal");
    }
    if (jacobian_.rows() != target_.size()) {
      throw std::runtime_error("rows of jacobian and target not equal");
    }
  }

  Eigen::MatrixXd GetWeightedJacobian() { return weight_ * jacobian_; }
  Eigen::VectorXd GetWeightedValue() { return weight_ * value_; }
  Eigen::VectorXd GetWeightedTarget() { return weight_ * target_; }
  Eigen::MatrixXd GetJacobian() { return jacobian_; }
  Eigen::VectorXd GetValue() { return value_; }
  Eigen::VectorXd GetTarget() { return target_; }
  Eigen::MatrixXd& GetA() { return A_; }
  Eigen::VectorXd& GetUb() { return ub_; }

 protected:
  double weight_ = 1.0;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd ub_;
  Eigen::VectorXd target_;
  Eigen::VectorXd state_;
  Eigen::VectorXd value_;
};

}  // namespace math
