#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace math {

// Task definition: weight*f(x) =weight* p_d, Ax<= ub, Using Newton-Euler method to find x
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
  void SetTarget(const Eigen::VectorXd& target) { target_ = target; }
  void Update(const Eigen::VectorXd& state) {
    state_ = state;
    CalculateJacobian();
    CalculateValue();
    Check();
  };
  void Check() {
    if (jacobian_.rows() != value_.rows()) {
      throw std::runtime_error("rows of jacobian and value not equal");
    }
    if (jacobian_.cols() != state_.rows()) {
      throw std::runtime_error("cols of jacobian and state not equal");
    }
    if (jacobian_.rows() != target_.rows()) {
      throw std::runtime_error("rows of jacobian and target not equal");
    }
    if (jacobian_.rows() != value_.rows()) {
      throw std::runtime_error("rows of jacobian and value not equal");
    }
  }
  Eigen::MatrixXd GetJacobian() { return weight_ * jacobian_; }
  Eigen::VectorXd GetValue() { return weight_ * value_; }
  Eigen::VectorXd GetTarget() { return weight_ * target_; }
  Eigen::MatrixXd GetA() { return A_; }
  Eigen::VectorXd GetUb() { return ub_; }

 private:
  double weight_ = 1.0;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd ub_;
  Eigen::VectorXd target_;
  Eigen::VectorXd state_;
  Eigen::VectorXd value_;
};

class RootFinding {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RootFinding(const Eigen::VectorXd& init_state) {
    state_ = init_state;
    task_list_.clear();
  }
  void Reset() { task_list_.clear(); }
  void Run();
  void AddTask(std::shared_ptr<Task> task_added) { task_list_.push_back(task_added); }

 private:
  void Update();
  void Solve();
  std::vector<std::shared_ptr<Task>> task_list_;
  Eigen::VectorXd state_;
  Eigen::MatrixXd composite_jacobian_;
  Eigen::VectorXd composite_value_;
  Eigen::VectorXd composite_target_;
  Eigen::MatrixXd composite_A_;
  Eigen::VectorXd composite_ub_;
};

}  // namespace math
