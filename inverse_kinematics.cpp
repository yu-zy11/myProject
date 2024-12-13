
#include <Eigen/Dense>
#include <vector>

// Task definition: weight*f(x) =weight* p_d, Ax<= ub, Using Newton-Euler method.
class Task {
 public:
  Task(int weight = 1) { weight_ = weight; }
  virtual void CalculateJacobian() = 0;
  virtual void CalculateValue() = 0;
  void SetTarget(const Eigen::VectorXd& target) { target_ = target; }

  void Update(const Eigen::VectorXd& states) {
    states_ = states;
    CalculateJacobian();
    CalculateValue();
  };
  Eigen::VectorXd GetJacobian() { return jacobian_; }
  Eigen::VectorXd GetValue() { return value_; }

 private:
  double weight_ = 1.0;
  Eigen::MatrixXd jacobian_;
  //   Eigen::MatrixXd A_;
  //   Eigen::VectorXd ub_;
  Eigen::VectorXd target_;
  Eigen::VectorXd states_;
  Eigen::VectorXd value_;
}

class TaskSolver {
 public:
  TaskSolver() { task_list_.clear(); }
  void Solve() {}
  void AddTask(std::shared_ptr<Task> task_added) { task_list_.push_back(task_added); }

 private:
  std::vector<std::shared_ptr<Task>> task_list_;
}
