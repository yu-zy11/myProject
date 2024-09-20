#include <gtest/gtest.h>

#include <iostream>
#include "pinocchio_interface/pinocchio_kinematics.h"

TEST(TestPinocchioInterface, testForwardKinematics) {
  pino::PinocchioModelInfo model_info;
  {
    model_info.urdf_file_path = "/home/ubuntu/workspace/myProject/src/pinocchio_interface/src/test/SA01p.urdf";
    model_info.base_link_name = "base_link";
    model_info.end_effector_names = {"leg_l_foot_center", "leg_r_foot_center"};
    model_info.contact_type = {pino::ContactType::kSixDofContact, pino::ContactType::kSixDofContact};
    model_info.use_floating_base = true;
    model_info.print_pinocchio_info = false;
  };
  std::shared_ptr<pino::PinocchioInterface> pino_ptr_ = std::make_shared<pino::PinocchioInterface>(model_info);
  pino::PinocchioKinematics pino_kine(pino_ptr_);

  Eigen::VectorXd qpos, qpos_des, qvel;
  qpos.resize(pino_ptr_->GetDof());
  qpos.setZero();
  qvel.resize(pino_ptr_->GetDof());
  qvel.setZero();

  qpos_des.resize(pino_ptr_->GetDof());
  qpos_des.setZero();
  qpos_des[0 + pino_ptr_->GetBaseDof()] = 0.340188;
  qpos_des[1 + pino_ptr_->GetBaseDof()] = -0.105617;
  qpos_des[2 + pino_ptr_->GetBaseDof()] = 0.283099;
  qpos_des[3 + pino_ptr_->GetBaseDof()] = 0.29844;
  qpos_des[4 + pino_ptr_->GetBaseDof()] = 0.411647;
  qpos_des[5 + pino_ptr_->GetBaseDof()] = -0.302449;
  qpos_des[6 + pino_ptr_->GetBaseDof()] = -0.164777;
  qpos_des[7 + pino_ptr_->GetBaseDof()] = 0.26823;
  qpos_des[8 + pino_ptr_->GetBaseDof()] = -0.222225;
  qpos_des[9 + pino_ptr_->GetBaseDof()] = 0.05397;
  qpos_des[10 + pino_ptr_->GetBaseDof()] = -0.0226029;
  qpos_des[11 + pino_ptr_->GetBaseDof()] = 0.128871;
  /*test inverse kinematics using two endeffector*/
  std::vector<pino::EndEffectorData> data, data_des;
  pino_kine.FixedBaseForwardKinematics(qpos_des, qvel, data_des);
  std::vector<std::string> select_name{model_info.end_effector_names[0], model_info.end_effector_names[1]};
  Eigen::VectorXd q_result;
  pino_kine.FixedBaseInverseKinematics(qpos, select_name, data_des, q_result);
  EXPECT_LT((q_result.tail(12) - qpos_des.tail(12)).norm(), 1e-3);

  /*test inverse kinematics using endeffector 0*/
  std::vector<std::string> select_name1{model_info.end_effector_names[0]};
  pino_kine.FixedBaseInverseKinematics(qpos, select_name1, data_des, q_result);
  EXPECT_LT((q_result.segment(6, 6) - qpos_des.segment(6, 6)).norm(), 1e-3);

  /*test inverse kinematics using endeffector 1*/
  std::vector<std::string> select_name2{model_info.end_effector_names[1]};
  pino_kine.FixedBaseInverseKinematics(qpos, select_name2, data_des, q_result);
  EXPECT_LT((q_result.tail(6) - qpos_des.tail(6)).norm(), 1e-3);
}

TEST(TestPinocchioInterface, testInverseKinematics3dof) {
  pino::PinocchioModelInfo model_info;
  {
    model_info.urdf_file_path = "/home/ubuntu/workspace/myProject/src/pinocchio_interface/src/test/SA01p.urdf";
    model_info.base_link_name = "base_link";
    model_info.end_effector_names = {"leg_l_foot_center", "leg_r_foot_center"};
    model_info.contact_type = {pino::ContactType::kSixDofContact, pino::ContactType::kSixDofContact};
    model_info.use_floating_base = false;
    model_info.print_pinocchio_info = false;
  };
  std::shared_ptr<pino::PinocchioInterface> pino_ptr_ = std::make_shared<pino::PinocchioInterface>(model_info);
  pino::PinocchioKinematics pino_kine(pino_ptr_);

  Eigen::VectorXd qpos, qpos_des, qvel;
  qpos.resize(pino_ptr_->GetDof());
  qpos.setZero();
  qvel.resize(pino_ptr_->GetDof());
  qvel.setZero();

  qpos_des.resize(pino_ptr_->GetDof());
  qpos_des.setZero();
  qpos_des[0 + pino_ptr_->GetBaseDof()] = 0.340188;
  qpos_des[1 + pino_ptr_->GetBaseDof()] = -0.105617;
  qpos_des[2 + pino_ptr_->GetBaseDof()] = 0.283099;
  qpos_des[3 + pino_ptr_->GetBaseDof()] = 0.29844;
  qpos_des[4 + pino_ptr_->GetBaseDof()] = 0.411647;
  qpos_des[5 + pino_ptr_->GetBaseDof()] = -0.302449;
  qpos_des[6 + pino_ptr_->GetBaseDof()] = -0.164777;
  qpos_des[7 + pino_ptr_->GetBaseDof()] = 0.26823;
  qpos_des[8 + pino_ptr_->GetBaseDof()] = -0.222225;
  qpos_des[9 + pino_ptr_->GetBaseDof()] = 0.05397;
  qpos_des[10 + pino_ptr_->GetBaseDof()] = -0.0226029;
  qpos_des[11 + pino_ptr_->GetBaseDof()] = 0.128871;
  /*test inverse kinematics using two endeffector*/
  std::vector<pino::EndEffectorData> data, data_des;
  pino_kine.FixedBaseForwardKinematics(qpos_des, qvel, data_des);

  /*test inverse kinematics using endeffector 0*/
  Eigen::VectorXd q_result;
  Eigen::Vector3d pos_des = data_des[0].pos;
  pino_kine.FixedBaseInverseKinematics3Dof(qpos, model_info.end_effector_names[0], pos_des, q_result);
  pino_kine.FixedBaseForwardKinematics(q_result, qvel, data);
  EXPECT_LT((data[0].pos - pos_des).norm(), 1e-3);

  /*test inverse kinematics using endeffector 1*/
  pino_kine.FixedBaseForwardKinematics(qpos_des, qvel, data_des);
  pos_des = data_des[1].pos;
  pino_kine.FixedBaseInverseKinematics3Dof(qpos, model_info.end_effector_names[1], pos_des, q_result);
  pino_kine.FixedBaseForwardKinematics(q_result, qvel, data);
  EXPECT_LT((data[1].pos - pos_des).norm(), 1e-3);
}
