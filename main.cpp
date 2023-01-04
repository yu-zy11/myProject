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
using std::vector;
/** Example for qpOASES main function using the SQProblem class. */
using Vec3f = Eigen::Vector3f;
int main() {
  // convexMPC mpc(2,0.02);
  // std::cout<<mpc.test<<"\n";

  // mpc.init();
  // for(int i=0;i<10;i++)
  // {
  // 	// mpc.MPCInterface();
  // 	// mpc.run();
  // }
  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  vector<Eigen::Matrix4f> max_vec;
  Eigen::Matrix4f test;
  ROBOTICS::Kinemetics kine;
  int leg = 1;
  Vec3f q, p;
  q << 0.1, 0.1, 0.5;
  kine.fkine(p, q, leg);
  std::cout << "P:\n" << p << std::endl;
  kine.ikine(q, p, leg);
  std::cout << "q:\n" << q << std::endl;
  qpOASES::real_t *A_qpoases;
  A_qpoases = mat.data();
  for (int i = 0; i < 12; i++) {
    std::cout << A_qpoases[i] << std::endl;
  }
  // test gait
  __int64_t sec = 0;
  __int64_t nsec = 1000000;
  timespec req = {sec, nsec};
  Gait gait;
  int counter{0};
  FusionData state;
  commandData cmd;
  while (true) {
    counter++;
    gait.setGaitType(GaitType::TROT);
    if (counter % 10000 == 0) {
      gait.update(state, cmd);
      // gait.printGaitInfo();
      clock_nanosleep(CLOCK_MONOTONIC, 0, &req, nullptr);
    }
  }

  return 0;
}

/*
 *	end of file
 */
