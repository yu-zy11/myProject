#include <iostream>

#include "model/RobotSA01a.h"
#include "pinocchio_interface/PinocchioInterface.h"

#define PINOCCHIO_MODEL_DIR "/home/work/workspace/biped_robot/simulator/mujoco/model/zq_sa01/meshes/zqsa01p.urdf"

template <typename T>
Eigen::Matrix<T, 3, 3> omega2rpyrate(Eigen::Matrix<T, 3, 1> rpy) {
  T c3 = cos(rpy[2]);
  T s3 = sin(rpy[2]);
  T c2 = cos(rpy[1]);
  T s2 = sin(rpy[1]);
  Eigen::Matrix<T, 3, 3> mat_omega2rpy;
  mat_omega2rpy << c3 / c2, s3 / c2, 0, -s3, c3, 0, c3 * s2 / c2, s3 * s2 / c2, 1;
  return mat_omega2rpy;
}

int main(int argc, char **argv) {
  PinocchioInterface pino;
  pino.init(PINOCCHIO_MODEL_DIR, false);
  // test total mass
  RobotSA01a<double> robot;

  double total_mass = pino.getTotalMass();
  std::cout << "total mass:" << total_mass << std::endl;

  // robot.forwardKinematics();

  Eigen::VectorXd qpos, qpos_des, qvel;
  std::vector<endEffectorData<double>> data, data_des;
  qpos.resize(12);
  qpos.setZero();
  qvel.resize(12);
  qvel.setZero();
  qpos[0] = 0.2;
  qpos[3] = 0.2;
  robot.localForwardKinematics(qpos, qvel, data);
  std::cout << "data pos:" << data[0].pos.transpose() << std::endl;
  std::cout << "data rotm:" << data[0].rotm << std::endl;
  // data_des

  qpos_des = qpos;
  qpos_des[3] = 0.8;
  robot.localForwardKinematics(qpos_des, qvel, data_des);

  std::cout << "data_des pos:" << data_des[0].pos.transpose() << std::endl;
  std::cout << "data_des rotm:" << data_des[0].rotm << std::endl;
  std::cout << "pos_error:" << data_des[0].pos - data[0].pos << std::endl;
  std::vector<int> select_id{0};
  Eigen::VectorXd q_result;
  robot.localInverseKinematics(qpos, select_id, data_des, q_result);
  std::cout << "qresult:" << q_result.transpose() << std::endl;
  robot.localForwardKinematics(q_result, qvel, data);
  std::cout << "pos:" << data[0].pos.transpose() << std::endl;

  // test local inverse kinematics
  //
  return 0;
}
