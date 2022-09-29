#ifndef PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_CONTROLLER_SOLVER_MPC_H_
#define PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_CONTROLLER_SOLVER_MPC_H_

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>
#include "modules/locomotion/locomotion_core/controller/mpc_controller/mpc_interface.h"

using Eigen::Matrix;
using Eigen::Quaterniond;
using Eigen::Quaternionf;

template <class T>
void print_array(T* array, uint16_t rows, uint16_t cols) {
  for (uint16_t r = 0; r < rows; r++) {
    for (uint16_t c = 0; c < cols; c++) std::cout << (float)array[c + r * cols] << " ";
    printf("\n");
  }
}

template <class T>
void print_named_array(const char* name, T* array, uint16_t rows, uint16_t cols) {
  printf("%s:\n", name);
  print_array(array, rows, cols);
}

// print named variable
template <class T>
void pnv(const char* name, T v) {
  printf("%s: ", name);
  std::cout << v << std::endl;
}

template <class T>
T t_min(T a, T b) {
  if (a < b) return a;
  return b;
}

template <class T>
T sq(T a) {
  return a * a;
}

void solve_mpc(update_data_t* update, problem_setup* setup);

void quat_to_rpy(Quaternionf q, Matrix<float, 3, 1>& rpy);
void ct_ss_mats(
    Matrix<float, 3, 3> I_world,
    float m,
    Matrix<float, 3, 4> r_feet,
    Matrix<float, 3, 3> R_yaw,
    Matrix<float, 13, 13>& A,
    Matrix<float, 13, 12>& B);
void resize_qp_mats(int16_t horizon);
// void c2qp(Matrix<float, 13, 13> Ac, Matrix<float, 13, 12> Bc, float dt, float horizon);
double* get_q_soln();

#endif  // PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_CONTROLLER_SOLVER_MPC_H_
