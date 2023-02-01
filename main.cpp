#include "./controller/library/kinematics.h"
#include "./controller/mpc_controller/convexMPC.h"

#include "controller/common_data/common_data.h"
#include "controller/gait/gait.h"
#include <Eigen/Dense>
#include <ctime>
#include <iostream>
#include <qpOASES.hpp>
#include <unistd.h>
#include <vector>

#include "controller/library/robotics_math.h"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
using std::vector;
/** Example for qpOASES main function using the SQProblem class. */
using Vec3f = Eigen::Vector3f;

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR                                                    \
  "/home/yuzy/workspace/learnCpp/myProject/model/laikago"
#endif

int main() {

  vector<int> max_vec(10);
  max_vec[1] = 10;
  std::cout << "max_vec" << max_vec[0] << max_vec[1] << std::endl;
  ROBOTICS::Kinemetics kine(false);
  Eigen::Matrix<float, 12, 1> qq;
  qq = Eigen::Matrix<float, 12, 1>::Random();
  qq.block(0, 0, 6, 1).setZero();
  qq.setZero();
  qq.block(0, 0, 9, 1) << 0.9509, 0.7223, 0.4001, 0.8319, 0.1343, 0.0605,
      0.0842, 0.1639, 0.3242;
  kine.setJointPosition(qq);
  kine.fkine();
  kine.jacobian();
  Vec3f pp;
  kine.fkine(pp, qq.block(9, 0, 3, 1), 1);
  std::cout << "P" << pp << std::endl;
  Eigen::Matrix3f rot;
  float theta{ROBOTICS::pi / 3};
  rot = ROBOTICS::Math::rotx(theta);
  std::cout << "rot:\n" << rot << std::endl;
  int leg = 1;
  Vec3f q, p;
  q << 0.716, 0.002, -1.294;
  kine.fkine(p, q, leg);
  std::cout << "P:\n" << p << std::endl;
  p << 0.1875, -0.165, -0.42;
  kine.ikine(q, p, leg);
  std::cout << "q:\n" << q << std::endl;
  kine.fkine(p, q, leg);
  std::cout << "P:\n" << p << std::endl;
  Eigen::Matrix3f jacob;
  kine.jacobian(jacob, q, leg);
  std::cout << "jacob:\n" << jacob << std::endl;
  qpOASES::real_t *A_qpoases;
  Eigen::Matrix<double, 12, 1> mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  A_qpoases = mat.data();
  for (int i = 0; i < 12; i++) {
    std::cout << A_qpoases[i] << std::endl;
  }
  // test gait
  __int64_t sec = 0;
  __int64_t nsec = 1000000;
  timespec req = {sec, nsec};
  Gait gait;
  /*test clock sleep*/
  int counter{0};
  FusionData state;
  commandData cmd;
  while (false) {
    counter++;
    gait.setGaitType(GaitType::TROT);
    if (counter % 10000 == 0) {
      gait.update(state, cmd);
      // gait.printGaitInfo();
      clock_nanosleep(CLOCK_MONOTONIC, 0, &req, nullptr);
    }
  }
  /*test pinocchio*/
  const std::string urdf_filename = std::string("urdf/laikago_simple.urdf");
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  return 0;
}

/*
 *	end of file
 */
