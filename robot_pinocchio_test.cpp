#include <iostream>

#include "PinocchioInterface.h"
#include "RobotBase.h"
#include "RobotSA01a.h"

#define PINOCCHIO_MODEL_DIR "/home/ubuntu/workspace/myProject/modelDynamics/SA01p.urdf"

RobotSA01a<double> robot;
void testForwardKinematics() {
  Eigen::VectorXd qpos, qvel, qpos_full, qvel_full;
  qpos.resize(12);
  qpos.setZero();
  qvel.resize(12);
  qvel.setZero();
  qpos_full.resize(18);
  qpos_full.setZero();
  qpos_full.tail(12) = qpos;
  qvel_full.resize(18);
  qvel_full.setZero();
  RobotSA01a<double> robot;
  std::vector<endEffectorData<double>> data, data_full;
  robot.fullForwardKinematics(qpos_full, qvel_full, data_full);
  robot.localForwardKinematics(qpos, qvel, data);
  bool success = true;
  for (int i = 0; i < robot.end_effector_name_.size(); ++i) {
    if (data[i].pos[0] != data_full[i].pos[0] || data[i].pos[1] != data_full[i].pos[1] || data[i].pos[2] != data_full[i].pos[2]) {
      success = false;
    }
    if (data[i].vel[0] != data_full[i].vel[0] || data[i].vel[1] != data_full[i].vel[1] || data[i].vel[2] != data_full[i].vel[2]) {
      success = false;
    }
  }
  if (success) {
    std::cout << "ForwardKinematics success" << std::endl;
  } else {
    std::cout << "ForwardKinematics fail" << std::endl;
  }
}
void testInverseKinematics() {
  bool success = true;
  RobotSA01a<double> robot;
  Eigen::VectorXd qpos, qpos_des, qvel;
  std::vector<endEffectorData<double>> data, data_des;
  qpos.resize(12);
  qpos.setZero();
  qvel.resize(12);
  qvel.setZero();

  qpos_des.resize(12);
  qpos_des[0] = 0.340188;
  qpos_des[1] = -0.105617;
  qpos_des[2] = 0.283099;
  qpos_des[3] = 0.29844;
  qpos_des[4] = 0.411647;
  qpos_des[5] = -0.302449;
  qpos_des[6] = -0.164777;
  qpos_des[7] = 0.26823;
  qpos_des[8] = -0.222225;
  qpos_des[9] = 0.05397;
  qpos_des[10] = -0.0226029;
  qpos_des[11] = 0.128871;
  /*test use two endeffector*/
  robot.localForwardKinematics(qpos_des, qvel, data_des);
  std::vector<std::string> select_name{robot.end_effector_name_[0], robot.end_effector_name_[1]};
  Eigen::VectorXd q_result;
  robot.localInverseKinematics(qpos, select_name, data_des, q_result);
  if ((q_result - qpos_des).norm() > 1e-4) {
    success = false;
  }
  /*test use 0 endeffector*/
  robot.localForwardKinematics(qpos_des, qvel, data_des);
  std::vector<std::string> select_name1{robot.end_effector_name_[0]};
  robot.localInverseKinematics(qpos, select_name1, data_des, q_result);
  if ((q_result.head(6) - qpos_des.head(6)).norm() > 1e-3) {
    success = false;
  }

  /*test use 1 endeffector*/
  robot.localForwardKinematics(qpos_des, qvel, data_des);
  std::vector<std::string> select_name2{robot.end_effector_name_[1]};
  robot.localInverseKinematics(qpos, select_name2, data_des, q_result);
  if ((q_result.tail(6) - qpos_des.tail(6)).norm() > 1e-3) {
    success = false;
    std::cout << "q_result.tail(6): " << q_result.tail(6).transpose() << std::endl;
    std::cout << "qpos_des.tail(6): " << qpos_des.tail(6).transpose() << std::endl;
  }
  if (success) {
    std::cout << "InverseKinematics success" << std::endl;
  } else {
    std::cout << "InverseKinematics fail" << std::endl;
  }
}

int main(int argc, char **argv) {
  testForwardKinematics();
  testInverseKinematics();
  return 0;
}
