#include <iostream>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/fwd.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio_interface/pinocchio_interface.h"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own
// directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/yu/workspace/ocs2_robotic_assets/resources/anymal_c/urdf/"
// #define PINOCCHIO_MODEL_DIR                                                    \
//   "/home/yu/workspace/lquadleg/Cheetah/sim/mujoco_simulator/model/ls_dog105/"  \
//   "meshes"
#endif

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
  using namespace pinocchio;
  PinocchioInterface<double> pino;
  double total_mass = pino.getTotalMass();
  std::cout << "total mass is:" << total_mass << std::endl;
  std::vector<std::string> foot_name = pino.getFootNames();
  for (const auto &name : foot_name) std::cout << "footname:" << name << std::endl;
  int nq = pino.get_model_nq_();
  int nv = pino.get_model_nv_();
  Eigen::Matrix<double, -1, 1> qc;
  Eigen::Matrix<double, -1, 1> vc;
  qc.resize(nq, 1);
  qc.setZero();
  vc.resize(nv, 1);
  vc.setZero();
  qc[3] = 3.1415926 / 2;
  qc[4] = 2.1415926 / 2;
  qc[5] = 1.1415926 / 2;
  vc[3] = 1;
  vc[4] = 2;
  vc[5] = 3;
  pino.updatePinocchioData(qc, vc);
  Eigen::Matrix<double, 3, 3> ori_body = pino.getFrameOrientation(0);
  Eigen::Matrix<double, -1, 1> vel_body = pino.getFrameVelocity(0);
  Eigen::Matrix<double, 3, 1> rpy;
  rpy << qc[5], qc[4], qc[3];
  Eigen::Matrix<double, 3, 3> mat_rate = omega2rpyrate(rpy);
  Eigen::Matrix<double, 3, 1> euler_rate = mat_rate * vel_body.segment(3, 3);
  std::cout << "ori_body:\n" << ori_body << std::endl;
  std::cout << "vel_body:\n" << vel_body << std::endl;
  std::cout << "euler_rate:\n" << euler_rate << std::endl;
  std::cout << "qc:" << qc.transpose() << std::endl;
  std::cout << "vc:" << vc.transpose() << std::endl;
  Eigen::Matrix<double, 6, -1> jacob_world = pino.getFrameJabobianRelativeToBodyWorldFrame(4);
  Eigen::Matrix<double, 6, -1> jacob_world_body = pino.getFrameJabobianRelativeToBodyWorldFrame(4);
  Eigen::Matrix<double, 6, -1> jacob_body = pino.getFrameJabobianRelativeToBodyFrame(4);
  Eigen::Matrix<double, -1, -1> dj = pino.getWholeJabobianDerivativeRelativeToBodyWorldFrame();
  Eigen::Matrix<double, -1, -1> dj_leg0 = pino.getFrameJabobianDerivativeRelativeToBodyFrame(1);
  std::cout << "jacob_world:\n" << jacob_world << std::endl;
  std::cout << "jacob_world_body:\n" << jacob_world_body << std::endl;
  std::cout << "jacob_body:\n" << jacob_body << std::endl;
  std::cout << "dj:\n" << dj << std::endl;
  std::cout << "dj_leg0:\n" << dj_leg0 << std::endl;

  Eigen::Matrix<double, 3, 1> pos = pino.getFramePosition(0);
  std::cout << "pos 0:" << pos.transpose() << std::endl;
  pos = pino.getFramePosition(1);
  std::cout << "pos 1:" << pos.transpose() << std::endl;
  pos = pino.getFramePosition(2);
  std::cout << "pos 2:" << pos.transpose() << std::endl;
  pos = pino.getFramePosition(3);
  std::cout << "pos 3:" << pos.transpose() << std::endl;

  Eigen::Matrix<double, 3, 1> com = pino.getCenterOfMass(qc, 1);
  std::cout << "com0:" << com << std::endl;

  Eigen::Matrix<double, -1, -1> M = pino.getJointSpaceInertiaMassMatrix();
  Eigen::Matrix<double, -1, -1> Minv = pino.getJointSpaceInertialMassMatrixInverse();
  Eigen::Matrix<double, -1, 1> b = pino.getCoriolisAndGravity();
  std::cout << "M:" << M.block(0, 0, 5, 5) << std::endl;
  std::cout << "Minv:" << Minv << std::endl;
  std::cout << "b:" << b << std::endl;
  // Eigen::Matrix<double, -1, 1> qos = pino.getjointPosition();
  // You should change here to set up your own URDF file or just pass it as an
  // argument of this example.
  const std::string urdf_filename = (argc <= 1) ? PINOCCHIO_MODEL_DIR + std::string("/anymal.urdf") : argv[1];
  // (argc <= 1) ? PINOCCHIO_MODEL_DIR + std::string("/ls_dog105.urdf") :
  // argv[1]; (argc <= 1) ? PINOCCHIO_MODEL_DIR + std::string("/anymal.urdf") :
  // argv[1];

  // Load the urdf model
  // pino.loadURDF(urdf_filename, true);
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model, false);
  std::cout << "model name: " << model.name << std::endl;

  // Create data required by the algorithms
  Data data(model);

  // Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  std::cout << "q: " << q.transpose() << std::endl;

  // Perform the forward kinematics over the kinematic tree
  forwardKinematics(model, data, q);

  // Print out the placement of each joint of the kinematic tree
  for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed << std::setprecision(2) << data.oMi[joint_id].translation().transpose() << std::endl;
  auto mass = pinocchio::computeTotalMass(model);
  std::cout << "mass:" << mass << std::endl;
  std::cout << "nq:" << model.nq << std::endl;
  std::cout << "base :" << model.nq << std::endl;
}
