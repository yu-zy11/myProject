
#ifndef PROJECT_LSA01A_H
#define PROJECT_LSA01A_H

#include "RobotBase.h"

template <typename T>
class RobotSA01a : public RobotBase<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotSA01a() {
    char* cwd = get_current_dir_name();
    std::string S;
    S += cwd;
    free(cwd);
    // std::string urdf_file_path{S + A01_URDF_RELATIVE_PATH};
    std::string urdf_file_path{A01_URDF_RELATIVE_PATH};
    this->pino_.init(urdf_file_path);
    this->robot_type_ = RobotType::SA01a;
    this->base_link_name_ = "base_link";
    std::vector<std::string> end_effector_name = {"leg_l_foot_center", "leg_r_foot_center"};
    this->end_effector_name_ = end_effector_name;

    // get parameters from urdf and pinocchio
    this->data_.resize(this->end_effector_name_.size());
    const int dof = this->pino_.getDoF();
    this->qpos_upper_limit_ = this->pino_.getJointUpperPositionLimit().tail(dof - this->pino_.getBaseDoF());
    this->qpos_lower_limit_ = this->pino_.getJointLowerPositionLimit().tail(dof - this->pino_.getBaseDoF());
    this->qvel_limit_ = this->pino_.getJointVelocityLimit().tail(dof - this->pino_.getBaseDoF());
    this->qtor_limit_ = this->pino_.getJointTorqueLimit().tail(dof - this->pino_.getBaseDoF());
    printf("[SA01a]init total mass: %f\n", this->pino_.getTotalMass());
  };
};

#endif
