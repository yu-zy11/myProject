
#include <Eigen/Dense>
#include <ctime>
#include <iostream>

#include "../../include/math/matrix_utils.h"
#include "../../include/math/pos_task_set/endeffector_task.h"
#include "../../include/math/root_finding.h"

bool test_root_finding() {
  double task1_weight = 5;
  double task2_weight = 1;
  std::shared_ptr<math::PosTask> task1 = std::make_shared<math::EndeffectorTask>(task1_weight);
  std::shared_ptr<math::PosTask> task2 = std::make_shared<math::EndeffectorTask>(task2_weight);
  Eigen::VectorXd target1 = Eigen::Vector3d{1, 2, 3};
  std::cout << target1 << std::endl;
  Eigen::VectorXd target2 = Eigen::Vector3d{3, 2, 3};
  std::cout << target2 << std::endl;
  task1->SetTarget(target1);
  task2->SetTarget(target2);

  Eigen::Vector3d init_state{0, 0, 0};
  math::RootFinding root_finding{init_state, 1e-8};
  root_finding.AddTask(task1);
  root_finding.AddTask(task2);
  root_finding.Solve();
  Eigen::VectorXd state;
  root_finding.GetResult(state);

  task1->Update(state);
  task2->Update(state);
  Eigen::VectorXd value1 = task1->GetValue();
  Eigen::VectorXd value2 = task2->GetValue();
  double error1 = (target1 - value1).norm() * task1_weight * task1_weight;
  double error2 = (target2 - value2).norm() * task2_weight * task2_weight;
  std::cout << "value1\n:" << value1 << "\n error1:" << error1 << std::endl;
  std::cout << "value2\n:" << value2 << "\n error2:" << error2 << std::endl;
  if (std::abs(error1 - error2) > 1e-4) {
    std::cout << "failed\n:";
    return false;
  }
  return true;
}

int main() { test_root_finding(); }
