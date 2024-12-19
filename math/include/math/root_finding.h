#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "./pos_task_set/pos_task.h"

namespace math {

class RootFinding {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RootFinding(const Eigen::VectorXd& init_state, double thresh_hold = 1e-6);
  void Reset() { task_list_.clear(); }
  void Solve();
  void AddTask(std::shared_ptr<PosTask> task_added) { task_list_.push_back(task_added); }
  void GetResult(Eigen::VectorXd& state) { state = state_; }

 private:
  static constexpr int kLineSearchStep = 10;
  void UpdateData();
  void UpdateValueOnly(const Eigen::VectorXd& state, Eigen::VectorXd& value);
  void SolveStep();
  std::vector<std::shared_ptr<PosTask>> task_list_;
  Eigen::VectorXd state_, state_old_;
  Eigen::MatrixXd composite_jacobian_;
  Eigen::VectorXd composite_value_;
  Eigen::VectorXd composite_target_;
  Eigen::MatrixXd composite_A_;
  Eigen::VectorXd composite_ub_;
  double thresh_hold_;
};

RootFinding::RootFinding(const Eigen::VectorXd& init_state, double thresh_hold) {
  thresh_hold_ = thresh_hold;
  state_ = init_state;
  task_list_.clear();
}

void RootFinding::UpdateData() {
  composite_jacobian_.resize(0, state_.size());
  composite_value_.resize(0);
  composite_target_.resize(0);
  composite_A_.resize(0, state_.size());
  composite_ub_.resize(0);
  for (auto& task : task_list_) {
    task->Update(state_);
    Eigen::MatrixXd task_jacobian = task->GetWeightedJacobian();
    composite_jacobian_ = StackMatrix(composite_jacobian_, task_jacobian);
    Eigen::VectorXd value = task->GetWeightedValue();
    composite_value_ = StackMatrix(composite_value_, value);
    Eigen::VectorXd target = task->GetWeightedTarget();
    composite_target_ = StackMatrix(composite_target_, target);
    composite_A_ = StackMatrix(composite_A_, task->GetA());
    composite_ub_ = StackMatrix(composite_ub_, task->GetUb());
  }
}

void RootFinding::Solve() {
  state_old_ = state_;
  SolveStep();
  while ((state_ - state_old_).norm() > thresh_hold_) {
    state_old_ = state_;
    SolveStep();
  }
};

void RootFinding::UpdateValueOnly(const Eigen::VectorXd& state, Eigen::VectorXd& value) {
  value.resize(0);
  for (auto& task : task_list_) {
    task->Update(state);
    value = StackMatrix(value, task->GetWeightedValue());
  }
};

void RootFinding::SolveStep() {
  UpdateData();
  // regualize hession matrix
  Eigen::MatrixXd hessian = composite_jacobian_.transpose() * composite_jacobian_;
  math::RegualizeMatrix(hessian);
  Eigen::VectorXd error = composite_value_ - composite_target_;
  Eigen::VectorXd gradient = composite_jacobian_.transpose() * error;
  Eigen::VectorXd delta = hessian.ldlt().solve(-gradient);
  // line search to avoid overshooting
  Eigen::VectorXd state_new = state_ + delta;
  Eigen::VectorXd value_new;
  UpdateValueOnly(state_new, value_new);
  Eigen::VectorXd error_new = value_new - composite_target_;
  double alpha = 1.0;
  int itr = 0;
  while (error_new.norm() > error.norm() && itr < kLineSearchStep) {
    alpha *= 0.5;
    state_new = state_ + alpha * delta;
    UpdateValueOnly(state_new, value_new);
    error_new = value_new - composite_target_;
    ++itr;
  }
  if (itr == kLineSearchStep) {
    std::cout << "warnnig, line search reach maximun steps:" << std::endl;
  }
  state_ = state_new;
}

}  // namespace math
