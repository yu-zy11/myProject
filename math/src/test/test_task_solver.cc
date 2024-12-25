
#include <glog/logging.h>
#include <Eigen/Dense>
#include <ctime>
#include <iostream>

#include "../../include/math/augmented_lagrange_solver.h"
#include "../../include/math/matrix_utils.h"
#include "../../include/math/task.h"

class TestedTask : public math::Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TestedTask(int weight = 10) : math::Task(weight) {}
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
    // A_.resize(0, 3);
    // ub_.resize(0);
    A_.resize(6, 3);
    A_ << Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
    ub_.resize(6);
    ub_ << 1.5, 2, 3, -1.5, 7, 8;
  };
  void UpdateCommon(const Eigen::VectorXd& state) override{};
};

bool test_root_finding() {
  double task1_weight = 5;
  double task2_weight = 1;
  std::shared_ptr<math::Task> task1 = std::make_shared<TestedTask>(task1_weight);
  std::shared_ptr<math::Task> task2 = std::make_shared<TestedTask>(task2_weight);
  Eigen::VectorXd target1 = Eigen::Vector3d{1, 2, 3};
  std::cout << target1 << std::endl;
  Eigen::VectorXd target2 = Eigen::Vector3d{3, 2, 3};
  std::cout << target2 << std::endl;
  task1->SetTarget(target1);
  task2->SetTarget(target2);

  Eigen::Vector3d init_state{0, 0, 0};
  math::AugmentedLagrangeSolver solver{1e-8, 50};
  solver.AddTask(task1);
  solver.AddTask(task2);
  solver.SolveWithNoConstraint(init_state);
  Eigen::VectorXd state;
  solver.GetResult(state);

  task1->Update(state);
  task2->Update(state);
  Eigen::VectorXd value1 = task1->GetValue();
  Eigen::VectorXd value2 = task2->GetValue();
  double error1 = (target1 - value1).norm() * task1_weight * task1_weight;
  double error2 = (target2 - value2).norm() * task2_weight * task2_weight;
  std::cout << "state\n:" << state.transpose() << std::endl;

  std::cout << "value1\n:" << value1 << "\n error1:" << error1 << std::endl;
  std::cout << "value2\n:" << value2 << "\n error2:" << error2 << std::endl;
  if (std::abs(error1 - error2) > 1e-4) {
    std::cout << "failed\n:";
    return false;
  }
  return true;
}

bool TESTAugmentedLagrange() {
  double task1_weight = 5;
  double task2_weight = 1;
  std::shared_ptr<math::Task> task1 = std::make_shared<TestedTask>(task1_weight);
  std::shared_ptr<math::Task> task2 = std::make_shared<TestedTask>(task2_weight);
  Eigen::VectorXd target1 = Eigen::Vector3d{1, 2, 3};
  std::cout << target1 << std::endl;
  Eigen::VectorXd target2 = Eigen::Vector3d{3, 2, 3};
  std::cout << target2 << std::endl;
  task1->SetTarget(target1);
  task2->SetTarget(target2);

  Eigen::Vector3d init_state{0, 0, 0};
  math::AugmentedLagrangeSolver solver{1e-8, 50};
  solver.AddTask(task1);
  solver.AddTask(task2);
  solver.SolveUsingAugmentedLagrange(init_state);
  Eigen::VectorXd state;
  solver.GetResult(state);

  task1->Update(state);
  task2->Update(state);
  Eigen::VectorXd value1 = task1->GetValue();
  Eigen::VectorXd value2 = task2->GetValue();
  double error1 = (target1 - value1).norm() * task1_weight * task1_weight;
  double error2 = (target2 - value2).norm() * task2_weight * task2_weight;
  std::cout << "state\n:" << state.transpose() << std::endl;
  std::cout << "value1\n:" << value1 << "\n error1:" << error1 << std::endl;
  std::cout << "value2\n:" << value2 << "\n error2:" << error2 << std::endl;
  if (std::abs(error1 - error2) > 1e-4) {
    std::cout << "failed\n:";
    return false;
  }
  return true;
}

int main() {
  test_root_finding();
  TESTAugmentedLagrange();
}
