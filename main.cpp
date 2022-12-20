#include "./controller/mpc_controller/convexMPC.h"
#include "controller/gait/gait.h"
#include <Eigen/Dense>
#include <ctime>
#include <iostream>
#include <qpOASES.hpp>
#include <unistd.h>
/** Example for qpOASES main function using the SQProblem class. */
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
  std::cout << "mat" << mat << std::endl;
  qpOASES::real_t *A_qpoases;
  A_qpoases = mat.data();
  for (int i = 0; i < 12; i++) {
    std::cout << A_qpoases[i] << std::endl;
  }
  // test gait
  __int64_t sec = 1;
  __int64_t nsec = 10000000;
  timespec req = {sec, nsec};
  Gait gait;
  int counter{0};
  while (true) {
    counter++;
    gait.setGaitType(GaitType::TROT);
    if (counter % 10000 == 0) {
      gait.update();
      gait.printGaitInfo();
      clock_nanosleep(CLOCK_MONOTONIC, 0, &req, nullptr);
    }
  }

  return 0;
}

/*
 *	end of file
 */
