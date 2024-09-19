#include <gtest/gtest.h>

#include <iostream>
#include "pinocchio_interface/pinocchio_full_dynamics.h"

TEST(TestPinocchioInterface, testFullDynamics) {
  core::pinocchio_interface::PinocchioModelInfo model_info;
  {
    model_info.urdf_file_path =
        "/home/ubuntu/workspace/engineai_robotics/src/core/pinocchio_interface/src/test/SA01p.urdf";
    model_info.base_link_name = "base_link";
    model_info.end_effector_names = {"leg_l_foot_center", "leg_r_foot_center"};
    model_info.contact_type = {core::pinocchio_interface::ContactType::kSixDofContact,
                               core::pinocchio_interface::ContactType::kSixDofContact};
    model_info.use_floating_base = true;
    model_info.print_pinocchio_info = false;
  };
  std::shared_ptr<core::pinocchio_interface::PinocchioInterface> pino_ptr_ =
      std::make_shared<core::pinocchio_interface::PinocchioInterface>(model_info);
  core::pinocchio_interface::PinocchioFullDynamics pino_dyn(pino_ptr_);

  Eigen::VectorXd qpos = Eigen::VectorXd::Zero(pino_ptr_->GetRobotDof());
  Eigen::VectorXd qvel = Eigen::VectorXd::Zero(pino_ptr_->GetRobotDof());
  Eigen::VectorXd qpos_des = Eigen::VectorXd::Zero(pino_ptr_->GetRobotDof());
  qpos.setRandom();
  Eigen::MatrixXd mass = pino_dyn.GetJointSpaceMassMatrix(qpos);
  Eigen::MatrixXd mass_inv = pino_dyn.GetJointSpaceMassMatrixInverse(qpos);
  Eigen::MatrixXd m = mass * mass_inv;
  EXPECT_LT((m.diagonal() - Eigen::VectorXd::Ones(pino_ptr_->GetRobotDof())).norm(), 1e-6);
}

// TEST(TestPinocchioInterface, testInverseKinematics3dof) {}
