
#ifndef PROJECT_LSA01A_H
#define PROJECT_LSA01A_H

#include "RobotBase.h"

template <typename T>
class RobotSA01a : public RobotBase<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotSA01a();
  //

  virtual void localForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data);
  virtual void fullForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data);
  virtual void localInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<int> user_selected_end_effector_id, const std::vector<endEffectorData<T>>& data_des,
                                      Eigen::Matrix<T, -1, 1>& qpos_des);
  virtual void fullInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<int> user_selected_end_effector_id, const std::vector<endEffectorData<T>>& data_des,
                                     Eigen::Matrix<T, -1, 1>& qpos_des);

 private:
  /*limit joint positions between joint position lower limit and upper limit*/
  void limitJointPosition(Eigen::Matrix<T, -1, 1>& qpos);
  const int base_dof_{6};
  std::string base_link_name_ = "base_link";
  std::vector<std::string> end_effector_name_ = {"leg_l_foot_center", "leg_r_foot_center"};  // {"leg_l_foot_center", "leg_r_foot_center"};
};

#endif
