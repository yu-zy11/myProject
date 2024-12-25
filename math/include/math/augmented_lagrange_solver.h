#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "matrix_utils.h"
#include "task.h"

namespace math {

// A class for augmented lagrange method to solve task: f(x) = target, s.t. Ax <= ub;
// Objective: min (f(x)- target|)^T*(f(x)- target|)+ lambda^T*(Ax-ub)+p_coef/2*[max(0,Ax-ub)]^2
// method:  Augmented Newton-Euler
class AugmentedLagrangeSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AugmentedLagrangeSolver(double tolerance = 1e-6, int max_iteration = 20);
  void Reset() { task_list_.clear(); }
  void AddTask(std::shared_ptr<Task> task_added) { task_list_.push_back(task_added); }
  void SolveWithNoConstraint(const Eigen::VectorXd& init_state);
  void SolveUsingAugmentedLagrange(const Eigen::VectorXd& init_state);
  void GetResult(Eigen::VectorXd& state) { state = state_; }
  void GetCompositeTarget(Eigen::VectorXd& target) { target = composite_target_; }
  void GetCompositeValue(Eigen::VectorXd& value) { value = composite_value_; }
  void GetCompositeJacobian(Eigen::MatrixXd& jacobian) { jacobian = composite_jacobian_; }

 private:
  static constexpr double kPenaltyMultiplier = 10.0;
  static constexpr int kMaxIterationLineSearch = 5;
  static constexpr int kMaxIterationPerStep = 20;

  void UpdateTaskData(const Eigen::VectorXd& state);
  void UpdateTaskValue(const Eigen::VectorXd& state, Eigen::VectorXd& value);
  void SolveStepWithNoConstraint();
  void AugmentedLagrangeNewtonStep(const Eigen::VectorXd& lagrange_multiplier, double punish_coef);
  void CalculateAugmentedLagrangeGradientAndHessian(const Eigen::VectorXd& state,
                                                    const Eigen::VectorXd& lagrange_multiplier, double p_coef,
                                                    Eigen::VectorXd& gradient, Eigen::MatrixXd& hessian);
  void CalculateAugmentedLagrangeGradient(const Eigen::VectorXd& state, const Eigen::VectorXd& lagrange_multiplier,
                                          double p_coef, Eigen::VectorXd& gradient);
  // CalculateAugmentedLagrangeGradient must be called first
  void CalculateAugmentedLagrangeHessian(double p_coef, Eigen::MatrixXd& hessian);

  std::vector<std::shared_ptr<Task>> task_list_;
  Eigen::VectorXd state_, state_old_;
  Eigen::MatrixXd composite_jacobian_;
  Eigen::VectorXd composite_value_;
  Eigen::VectorXd composite_target_;
  Eigen::MatrixXd composite_A_;
  Eigen::MatrixXd modified_A_;
  Eigen::VectorXd composite_ub_;
  Eigen::VectorXd modified_ub_;
  double tolerance_;
  int max_iteration_;
};
}  // namespace math
