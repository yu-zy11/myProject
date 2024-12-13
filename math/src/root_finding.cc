
#include "../include/math/root_finding.h"
#include "../include/math/matrix_utils.h"

namespace math {

void RootFinding::Update() {
  composite_jacobian_.resize(0, 0);
  composite_value_.resize(0);
  composite_target_.resize(0);
  composite_A_.resize(0, 0);
  composite_ub_.resize(0);
  for (auto& task : task_list_) {
    task->Update(state_);
    composite_jacobian_ = StackMatrix(composite_jacobian_, task->GetJacobian());
    composite_value_ = StackMatrix(composite_value_, task->GetValue());
    composite_target_ = StackMatrix(composite_target_, task->GetTarget());
    composite_A_ = StackMatrix(composite_A_, task->GetA());
    composite_ub_ = StackMatrix(composite_ub_, task->GetUb());
  }
}
void RootFinding::Solve() {
  // regularize hession matrix
  Eigen::MatrixXd hessian = composite_jacobian_.transpose() * composite_jacobian_;
  if () Eigen::VectorXd gradient = composite_jacobian_.transpose() * (composite_value_ - composite_target_);
  Eigen::VectorXd delta = hessian.ldlt().solve(-gradient);
  state_ = state_ + delta;
}

}  // namespace math
