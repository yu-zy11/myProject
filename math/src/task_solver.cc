#pragma once

#include <glog/logging.h>

#include "../include/math/task_solver.h"

namespace math {

TaskSolver::TaskSolver(double thresh_hold, int max_iteration) {
  thresh_hold_ = thresh_hold;
  max_iteration_ = max_iteration;
  task_list_.clear();
}

void TaskSolver::UpdateTaskData(const Eigen::VectorXd& state) {
  composite_jacobian_.resize(0, state.size());
  composite_value_.resize(0);
  composite_target_.resize(0);
  composite_A_.resize(0, state.size());
  composite_ub_.resize(0);
  task_list_[0]->UpdateCommon(state);
  for (auto& task : task_list_) {
    task->Update(state);
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

void TaskSolver::SolveWithNoConstraint(const Eigen::VectorXd& init_state) {
  state_ = init_state;
  state_old_ = state_;
  SolveStepWithNoConstraint();
  int itr = 0;
  while ((state_ - state_old_).norm() > thresh_hold_ && itr < max_iteration_) {
    state_old_ = state_;
    SolveStepWithNoConstraint();
    ++itr;
  }
  if (itr == max_iteration_) {
    std::cout << "solver reach max iteration";
  }
};

void TaskSolver::UpdateTaskValue(const Eigen::VectorXd& state, Eigen::VectorXd& value) {
  value.resize(0);
  task_list_[0]->UpdateCommon(state);
  for (auto& task : task_list_) {
    task->Update(state);
    value = StackMatrix(value, task->GetWeightedValue());
  }
};

void TaskSolver::SolveStepWithNoConstraint() {
  UpdateTaskData(state_);
  // regualize hession matrix
  Eigen::MatrixXd hessian = composite_jacobian_.transpose() * composite_jacobian_;
  math::RegualizeMatrix(hessian);
  Eigen::VectorXd error = composite_value_ - composite_target_;
  Eigen::VectorXd gradient = composite_jacobian_.transpose() * error;
  Eigen::VectorXd delta = hessian.ldlt().solve(-gradient);
  // line search to avoid overshooting
  Eigen::VectorXd state_new = state_ + delta;
  Eigen::VectorXd value_new;
  UpdateTaskValue(state_new, value_new);
  Eigen::VectorXd error_new = value_new - composite_target_;
  double alpha = 1.0;
  int itr = 0;
  while (error_new.norm() > error.norm() && itr < kMaxLineSearchStep) {
    alpha *= 0.5;
    state_new = state_ + alpha * delta;
    UpdateTaskValue(state_new, value_new);
    error_new = value_new - composite_target_;
    ++itr;
  }
  if (itr == kMaxLineSearchStep) {
    std::cout << "warnnig, line search reach maximun steps:" << std::endl;
  }
  state_ = state_new;
}

void TaskSolver::SolveUsingAugmentedLagrange(const Eigen::VectorXd& init_state) {
  state_ = init_state;
  UpdateTaskData(state_);
  int num_constraint = composite_ub_.size();
  Eigen::VectorXd lagrange_multiplier = Eigen::VectorXd::Zero(num_constraint);
  double p_coef = kMultiplier;
  state_old_ = state_;
  AugmentedLagrangeNewtonStep(lagrange_multiplier, p_coef);
  int itr = 0;
  while ((state_ - state_old_).norm() > thresh_hold_ && itr < max_iteration_) {
    state_old_ = state_;
    lagrange_multiplier += p_coef * (composite_A_ * state_ - composite_ub_);
    p_coef *= kMultiplier;
    AugmentedLagrangeNewtonStep(lagrange_multiplier, p_coef);
    ++itr;
    // std::cout << "state_old_:\n" << state_old_.transpose() << std::endl;
    // std::cout << "state_:\n" << state_.transpose() << std::endl;
  }
  if (itr == max_iteration_) {
    std::cout << "solver reach max iteration in SolveUsingAugmentedLagrange";
  }
};

void TaskSolver::AugmentedLagrangeNewtonStep(const Eigen::VectorXd& lagrange_multiplier, double p_coef) {
  Eigen::VectorXd gradient;
  Eigen::MatrixXd hessian;
  CalculateAugmentedLagrangeGradientAndHessian(state_, lagrange_multiplier, p_coef, gradient, hessian);
  math::RegualizeMatrix(hessian);
  Eigen::VectorXd delta = hessian.ldlt().solve(-gradient);
  Eigen::VectorXd state_new = state_ + delta;
  Eigen::VectorXd gradient_new;
  Eigen::MatrixXd hessian_new;
  // Add line search to avoid overshooting
  CalculateAugmentedLagrangeGradient(state_new, lagrange_multiplier, p_coef, gradient_new);
  int itr_line_search = 0;
  while (gradient_new.norm() > gradient.norm() && itr_line_search < kMaxLineSearchStep) {
    delta = 0.5 * delta;
    state_new = state_ + delta;
    CalculateAugmentedLagrangeGradient(state_new, lagrange_multiplier, p_coef, gradient_new);
    ++itr_line_search;
  }
  CalculateAugmentedLagrangeHessian(p_coef, hessian_new);
  gradient = gradient_new;
  hessian = hessian_new;
  state_new = state_ + delta;
  int itr = 0;
  while (delta.norm() > thresh_hold_ && itr < max_iteration_) {
    // while (gradient.norm() > thresh_hold_ && itr < max_iteration_) {
    math::RegualizeMatrix(hessian);
    delta = hessian.ldlt().solve(-gradient);
    // Add line search to avoid overshooting
    Eigen::VectorXd state_new_tmp = state_new + delta;
    CalculateAugmentedLagrangeGradient(state_new_tmp, lagrange_multiplier, p_coef, gradient_new);
    int itr_line_search_tmp = 0;
    while (gradient_new.norm() > gradient.norm() && itr_line_search_tmp < kMaxLineSearchStep) {
      delta = 0.5 * delta;
      state_new_tmp = state_new + delta;
      CalculateAugmentedLagrangeGradient(state_new_tmp, lagrange_multiplier, p_coef, gradient_new);
      ++itr_line_search_tmp;
    }

    CalculateAugmentedLagrangeHessian(p_coef, hessian_new);
    // std::cout << "gradient:\n" << gradient.transpose() << std::endl;
    // std::cout << "delta:\n" << delta.transpose() << std::endl;
    state_new = state_new + delta;
    gradient = gradient_new;
    hessian = hessian_new;
    ++itr;
  }
  if (itr == max_iteration_) {
    std::cout << "reach max iteration in AugmentedLagrangeNewtonStep, itr=" << itr << std::endl;
  }
  state_ = state_new;
};
void TaskSolver::CalculateAugmentedLagrangeGradientAndHessian(const Eigen::VectorXd& state,
                                                              const Eigen::VectorXd& lagrange_multiplier, double p_coef,
                                                              Eigen::VectorXd& gradient, Eigen::MatrixXd& hessian) {
  CalculateAugmentedLagrangeGradient(state, lagrange_multiplier, p_coef, gradient);
  CalculateAugmentedLagrangeHessian(p_coef, hessian);
};

void TaskSolver::CalculateAugmentedLagrangeGradient(const Eigen::VectorXd& state,
                                                    const Eigen::VectorXd& lagrange_multiplier, double p_coef,
                                                    Eigen::VectorXd& gradient) {
  int num_constraint = composite_ub_.size();
  if (lagrange_multiplier.size() != num_constraint) {
    throw std::runtime_error("lagrange_multiplier size must be equal to size of inequality constriants");
  }
  UpdateTaskData(state);
  Eigen::VectorXd constraint = (composite_A_ * state - composite_ub_);
  for (int i = 0; i < num_constraint; ++i) {
    if (constraint[i] <= 0) {
      composite_A_.row(i).setZero();
      composite_ub_[i] = 0;
      constraint[i] = 0;
    }
  }
  Eigen::VectorXd error = composite_value_ - composite_target_;
  gradient = composite_jacobian_.transpose() * error + composite_A_.transpose() * lagrange_multiplier +
             p_coef * composite_A_.transpose() * constraint;
};

void TaskSolver::CalculateAugmentedLagrangeHessian(double p_coef, Eigen::MatrixXd& hessian) {
  hessian = composite_jacobian_.transpose() * composite_jacobian_ + p_coef * composite_A_.transpose() * composite_A_;
};

}  // namespace math
