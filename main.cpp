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
#include "controller/library/test/test_kinematics.cpp"
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
template <typename T> using Mat4 = Eigen::Matrix<T, 4, 4>;
int main() {
  bool success = testKinematics();
  ROBOTICS::Kinematics kine(true);
  Eigen::Matrix<float, 18, 1> qq;
  qq = Eigen::Matrix<float, 18, 1>::Random();
  qq.setZero();
  qq.block(6, 0, 9, 1) << 0.9509, 0.7223, -0.4001, 0.8319, 0.1343, -0.0605,
      0.0842, 0.1639, -0.3242;
  Eigen::Matrix<float, 18, 1> dq;
  dq.setZero();
  dq.block(6, 0, 9, 1) << 0.9509f, 0.7223, 0.4001, 0.8319, 0.1343, 0.0605,
      0.0842, 0.1639, 0.3242;
  dq.setZero();
  time_t begin, end;
  begin = clock();
  kine.update(qq, dq);
  end = clock();
  std::cout << "time3=" << (end - begin) << std::endl;
  begin = clock();
  Eigen::Matrix<float, 6, 1> pfoot;
  kine.getFootPosition(pfoot, 1);
  std::cout << "pfoot:" << pfoot << std::endl;
  end = clock();
  std::cout << "time3=" << (end - begin) << std::endl;
  begin = clock();
  Eigen::Matrix<float, -1, -1> j;
  kine.getFootJacobian(j);
  end = clock();
  std::cout << "time4=" << end - begin << std::endl;
  begin = clock();
  Eigen::Matrix<float, -1, -1> dot_jacobian;
  kine.getFootDotJacobian(dot_jacobian);
  end = clock();
  std::cout << "time5=" << end - begin << std::endl;
  std::cout << "dot_jacobian:\n" << dot_jacobian << std::endl;

  Vec3f pp;
  std::cout << "P" << pp << std::endl;
  int leg = 1;
  kine.getFootPosition(pfoot, leg);
  Vec3f q, p;
  p = pfoot.block(0, 0, 3, 1);
  kine.ikine(q, p, leg);
  std::cout << "q:\n" << q << std::endl;
  Eigen::Matrix3f jacob;
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
