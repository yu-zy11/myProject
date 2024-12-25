#pragma once

#include "../include/math/augmented_lagrange_solver.h"

#include <glog/logging.h>

namespace math {

AugmentedLagrangeSolver::AugmentedLagrangeSolver(double tolerance, int max_iteration) {
  tolerance_ = tolerance;
  max_iteration_ = max_iteration;
  task_list_.clear();
}

void AugmentedLagrangeSolver::UpdateTaskData(const Eigen::VectorXd& state) {
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

void AugmentedLagrangeSolver::SolveWithNoConstraint(const Eigen::VectorXd& init_state) {
  state_ = init_state;
  state_old_ = state_;
  SolveStepWithNoConstraint();
  int itr = 0;
  while ((state_ - state_old_).norm() > tolerance_ && itr < max_iteration_) {
    state_old_ = state_;
    SolveStepWithNoConstraint();
    ++itr;
  }
  if (itr == max_iteration_) {
    std::cout << "solver reach max iteration";
  }
};

void AugmentedLagrangeSolver::UpdateTaskValue(const Eigen::VectorXd& state, Eigen::VectorXd& value) {
  value.resize(0);
  task_list_[0]->UpdateCommon(state);
  for (auto& task : task_list_) {
    task->Update(state);
    value = StackMatrix(value, task->GetWeightedValue());
  }
};

void AugmentedLagrangeSolver::SolveStepWithNoConstraint() {
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
  while (error_new.norm() > error.norm() && itr < kMaxIterationLineSearch) {
    alpha *= 0.5;
    state_new = state_ + alpha * delta;
    UpdateTaskValue(state_new, value_new);
    error_new = value_new - composite_target_;
    ++itr;
  }
  if (itr == kMaxIterationLineSearch) {
    std::cout << "warnnig, line search reach maximun steps:" << std::endl;
  }
  state_ = state_new;
}

void AugmentedLagrangeSolver::SolveUsingAugmentedLagrange(const Eigen::VectorXd& init_state) {
  state_ = init_state;
  UpdateTaskData(state_);
  int num_constraint = composite_ub_.size();
  Eigen::VectorXd lagrange_multiplier = Eigen::VectorXd::Zero(num_constraint);
  double p_coef = kPenaltyMultiplier;
  state_old_ = state_;
  AugmentedLagrangeNewtonStep(lagrange_multiplier, p_coef);
  int itr = 0;
  while ((state_ - state_old_).norm() > tolerance_ && itr < max_iteration_) {
    state_old_ = state_;
    lagrange_multiplier += p_coef * (composite_A_ * state_ - composite_ub_);
    lagrange_multiplier = lagrange_multiplier.cwiseMax(0.0);
    p_coef *= kPenaltyMultiplier;
    AugmentedLagrangeNewtonStep(lagrange_multiplier, p_coef);
    ++itr;
  }
  if (itr == max_iteration_) {
    std::cout << "solver reach max iteration in SolveUsingAugmentedLagrange";
  }
  Eigen::VectorXd constraint = composite_A_ * state_ - composite_ub_;
  if (constraint.maxCoeff() > tolerance_) {
    std::cout << "Failed, constraint not satisfied in SolveUsingAugmentedLagrange" << std::endl;
  }
};

void AugmentedLagrangeSolver::AugmentedLagrangeNewtonStep(const Eigen::VectorXd& lagrange_multiplier, double p_coef) {
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
  while (gradient_new.norm() > gradient.norm() && itr_line_search < kMaxIterationLineSearch) {
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
  // while (gradient.norm() > tolerance_ && itr < kMaxIterationPerStep)
  while (delta.norm() > tolerance_ && itr < kMaxIterationPerStep) {
    math::RegualizeMatrix(hessian);
    delta = hessian.ldlt().solve(-gradient);
    // Add line search to avoid overshooting
    Eigen::VectorXd state_new_tmp = state_new + delta;
    CalculateAugmentedLagrangeGradient(state_new_tmp, lagrange_multiplier, p_coef, gradient_new);
    int itr_line_search_tmp = 0;
    while (gradient_new.norm() > gradient.norm() && itr_line_search_tmp < kMaxIterationLineSearch) {
      delta = 0.5 * delta;
      state_new_tmp = state_new + delta;
      CalculateAugmentedLagrangeGradient(state_new_tmp, lagrange_multiplier, p_coef, gradient_new);
      ++itr_line_search_tmp;
    }

    CalculateAugmentedLagrangeHessian(p_coef, hessian_new);
    gradient = gradient_new;
    hessian = hessian_new;
    state_new = state_new + delta;
    ++itr;
  }
  if (itr == kMaxIterationPerStep) {
    std::cout << "reach max iteration per step in AugmentedLagrangeNewtonStep, itr=" << itr << std::endl;
  }
  state_ = state_new;
};
void AugmentedLagrangeSolver::CalculateAugmentedLagrangeGradientAndHessian(const Eigen::VectorXd& state,
                                                                           const Eigen::VectorXd& lagrange_multiplier,
                                                                           double p_coef, Eigen::VectorXd& gradient,
                                                                           Eigen::MatrixXd& hessian) {
  CalculateAugmentedLagrangeGradient(state, lagrange_multiplier, p_coef, gradient);
  CalculateAugmentedLagrangeHessian(p_coef, hessian);
};

void AugmentedLagrangeSolver::CalculateAugmentedLagrangeGradient(const Eigen::VectorXd& state,
                                                                 const Eigen::VectorXd& lagrange_multiplier,
                                                                 double p_coef, Eigen::VectorXd& gradient) {
  int num_constraint = composite_ub_.size();
  if (lagrange_multiplier.size() != num_constraint) {
    throw std::runtime_error("lagrange_multiplier size must be equal to size of inequality constriants");
  }
  UpdateTaskData(state);
  Eigen::VectorXd constraint = (composite_A_ * state - composite_ub_);
  modified_A_ = composite_A_;
  modified_ub_ = composite_ub_;
  for (int i = 0; i < num_constraint; ++i) {
    if (constraint[i] <= 0) {
      modified_A_.row(i).setZero();
      modified_ub_[i] = 0;
      constraint[i] = 0;
    }
  }
  Eigen::VectorXd error = composite_value_ - composite_target_;
  gradient = composite_jacobian_.transpose() * error + modified_A_.transpose() * lagrange_multiplier +
             p_coef * modified_A_.transpose() * constraint;
};

void AugmentedLagrangeSolver::CalculateAugmentedLagrangeHessian(double p_coef, Eigen::MatrixXd& hessian) {
  hessian = composite_jacobian_.transpose() * composite_jacobian_ + p_coef * modified_A_.transpose() * modified_A_;
};

}  // namespace math
