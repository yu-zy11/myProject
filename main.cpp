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
template <typename T> using Mat4 = Eigen::Matrix<T, 4, 4>;
int main() {
  Eigen::Matrix3f eye3 = Eigen::Matrix3f::Identity();
  Eigen::Vector3f euler, euler1;
  eye3 << 0.5455, -0.7497, 0.3747, 0.6874, 0.6560, 0.3116, -0.4794, 0.0876,
      0.8732;
  euler1 = ROBOTICS::Math::rot2eul(eye3);
  eye3 = ROBOTICS::Math::eul2rot(euler1);
  std::cout << "eye3" << eye3 << std::endl;
  std::cout << "euler1" << euler1 << std::endl;
  ROBOTICS::Kinematics kine(true);
  Eigen::Matrix<float, 18, 1> qq;
  qq = Eigen::Matrix<float, 18, 1>::Random();
  qq.block(0, 0, 6, 1).setZero();
  qq.setZero();
  qq.block(6, 0, 9, 1) << 0.9509, 0.7223, 0.4001, 0.8319, 0.1343, 0.0605,
      0.0842, 0.1639, 0.3242;
  Eigen::Matrix<float, 18, 1> dq;
  time_t begin, end;
  begin = clock();
  Eigen::Matrix4f m1, m2, m3;
  m1.setIdentity();
  m2.setIdentity();
  m3 = m1;
  end = clock();
  std::cout << "time=" << end - begin << std::endl;

  begin = clock();
  dq.setZero();
  dq.block(6, 0, 9, 1) << 0.9509f, 0.7223, 0.4001, 0.8319, 0.1343, 0.0605,
      0.0842, 0.1639, 0.3242;
  end = clock();
  std::cout << "time0=" << (end - begin) << std::endl;
  begin = clock();
  kine.update(qq, dq);
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
