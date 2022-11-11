#include "convexMPC.h"
#include <Eigen/Dense>
#include <ctime>
#include <iostream>
#include <qpOASES.hpp>
#include <thread>
#include <unsupported/Eigen/MatrixFunctions>
#include "math_utils.h"

void matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<float, -1, -1> src, int rows, int cols) {
  int a = 0;
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      dst[a] = src(r, c);
      a++;
    }
  }
}
// resize dimensions of matrix using horizon_mpc_,and set weights for QP problem
void convexMPC::init() {
  contact_table_.resize(4, horizon_mpc_);
  foot_pos_table_.resize(12, horizon_mpc_);
  phase_table_.resize(4, horizon_mpc_);
  Lweight_qp_.resize(13 * horizon_mpc_, 13 * horizon_mpc_);
  Kweight_qp_.resize(12 * horizon_mpc_, 12 * horizon_mpc_);
  traj_.resize(13 * horizon_mpc_, 1);
  A_mpc_.resize(13 * horizon_mpc_, 13);
  B_mpc_.resize(13 * horizon_mpc_, 12 * horizon_mpc_);
  Hessian_qp_.resize(12 * horizon_mpc_, 12 * horizon_mpc_);
  Gradient_qp_.resize(12 * horizon_mpc_, 1);
  Hessian_qp_tmp.resize(12 * horizon_mpc_, 13 * horizon_mpc_);
  constraint_ub_.resize(12 * horizon_mpc_, 1);
  constraint_lb_.resize(12 * horizon_mpc_, 1);
  contact_table_.setZero();
  phase_table_.setZero();
  Lweight_qp_.setZero();
  Kweight_qp_.setZero();
  traj_.setZero();
  B_mpc_.setZero();
  A_mpc_.setZero();
  Hessian_qp_.setZero();
  Gradient_qp_.setZero();
  Hessian_qp_tmp.setZero();
  constraint_ub_.setZero();
  constraint_lb_.setZero();
  Amat_discrete_.setZero();
  Bmat_discrete_.setZero();
  root_omega_target_.setZero();
  root_velocity_target_.setZero();
  // load param
  auto param = ConfigLoaderInstance.getRobotConfig();
  config.dt = (float)ConfigLoaderInstance.getCommonConfig()->default_cycle_time_;
  config.body_mass = (float)param->mass;
  config.mu = (float)param->mu;
  config.max_reaction_force = (float)param->max_reaction_force;
  config.body_height = (float)param->body_height;
  config.filter_omega = param->yaw_filter;
  config.filter_vel = param->x_filter;
  config.weight_rpy << (float)param->weight_rpy[0], (float)param->weight_rpy[1], (float)param->weight_rpy[2];
  config.weight_xyz << (float)param->weight_pos[0], (float)param->weight_pos[1], (float)param->weight_pos[2];
  config.weight_omega << (float)param->weight_omega[0], (float)param->weight_omega[1], (float)param->weight_omega[2];
  config.weight_vel << (float)param->weight_vel[0], (float)param->weight_vel[1], (float)param->weight_vel[2];
  config.com_offset << param->com_offset[0], param->com_offset[1], param->com_offset[2];
  config.p_hip << param->bodyLength, param->bodyWidth, 0;
  config.inertial << param->I_body[0], 0, 0, 0, param->I_body[1], 0, 0, 0, param->I_body[2];
  config.weight_input = param->grf_weight;

  lweight_qp_unit_ << config.weight_rpy[0], config.weight_rpy[1], config.weight_rpy[2], config.weight_xyz[0],
      config.weight_xyz[1], config.weight_xyz[2], config.weight_omega[0], config.weight_omega[1],
      config.weight_omega[2], config.weight_vel[0], config.weight_vel[1], config.weight_vel[2], 0.0;
  kweight_qp_unit_ << config.weight_input, config.weight_input, config.weight_input;
  Lweight_qp_.diagonal() = lweight_qp_unit_.replicate(horizon_mpc_, 1);      // state weight
  Kweight_qp_.diagonal() = kweight_qp_unit_.replicate(4 * horizon_mpc_, 1);  // unput weight
  counter_ = 0;
  dt_mpc_ = config.dt * iteration_step_;
  std::cout << "init MPC finished\n";
}

void convexMPC::update(
    const MCC::common::message::FusionData& fusion_data,
    const MCC::common::message::OperatorData& operator_data,
    int* mpcTable,
    float* phase_table,
    float stance_duration) {
  // update robot states for mpc
  stance_duration_ = stance_duration;
  root_euler_ = fusion_data.root_euler;
  root_omega_ = fusion_data.root_angular_velocity_in_world;
  root_position_ = fusion_data.root_position;
  root_velocity_ = fusion_data.root_linear_velocity_in_world;
  rotm_body2world_ = fusion_data.rotation_matrix_body_to_world;  // ok

  root_omega_target_ = root_omega_target_ * (1 - config.filter_omega) +
                       config.filter_omega * rotm_body2world_ * operator_data.root_angular_velocity_in_body;
  root_velocity_target_ = root_velocity_target_ * (1 - config.filter_vel) +
                          config.filter_vel * rotm_body2world_ * operator_data.root_linear_velocity_in_body;
  if (!optimism_) {
    root_omega_target_[0] = 0.0;
    root_omega_target_[1] = 0.0;
    root_velocity_target_[2] = 0.0;
  }

  for (int leg = 0; leg < 4; leg++) {
    foot_position_in_world_.block(3 * leg, 0, 3, 1) =
        rotm_body2world_ * (fusion_data.foot_position_in_body.row(leg).transpose() - config.com_offset);
    jacobian_leg_.block(0, 3 * leg, 3, 3) = fusion_data.foot_jacobian_in_body.block<3, 3>(3 * leg, 3 * leg);
  }
#ifdef DEBUG_MPC
  std::cout << "jacobian_leg_:\n" << jacobian_leg_ << std::endl;
#endif
  if (!optimism_) {
    Eigen::Vector3f rpy;
    Eigen::Matrix<float, 3, 3> R_yaw;
    rpy << 0.0, 0.0, root_euler_[2];
    R_yaw = euler2rotm(rpy);
    Inertial_in_world_ = R_yaw * config.inertial * R_yaw.transpose();
  } else {
    Inertial_in_world_ = rotm_body2world_ * config.inertial * rotm_body2world_.transpose();
  }
  inv_inertial_ = Inertial_in_world_.inverse();

  mat_omega2rpy_rate_ = omega2rpyrate(root_euler_);

  for (int i = 0; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      contact_table_(leg, i) = mpcTable[4 * i + leg];
      phase_table_(leg, i) = static_cast<float>(phase_table[4 * i + leg]);
    }
  }
  sum_contact_ = contact_table_.sum();
}

void convexMPC::run(
    const MCC::common::message::FusionData& fusion_data,
    const MCC::common::message::OperatorData& operator_data,
    int* mpcTable,
    float* phase_table,
    float stance_duration) {
  time_t begin, end;
  update(fusion_data, operator_data, mpcTable, phase_table, stance_duration);
  if (counter_ % iteration_step_ == 0) {
    counter_ = 0;
#ifdef DEBUG_MPC
    std::cout << "command velocity:" << root_velocity_target_ << std::endl;
    std::cout << "current velocity:" << root_velocity_ << std::endl;
#endif
    begin = clock();
    calculateAdiscrete();
    calculateBdiscrete(Bmat_discrete_, foot_position_in_world_);
    generateReferenceTrajectory();
    // generateContraintV2();
    generateContraint();
    FormulateQPform();
    end = clock();
    std::cout << "before solving mpc time0:" << (end - begin) << std::endl;

    begin = clock();
    // solveMPCV2();
    solveMPC();
    end = clock();
    std::cout << "solving mpc time:" << (end - begin) << std::endl;
  }
  counter_++;
}

// calculate A and B in time-continous dynamic equation of motion:\dot x=Ax+Bu
void convexMPC::calculateContinuousEquation() {
  Amat_continuous_.setZero();
  Amat_continuous_.block<3, 3>(0, 6) = mat_omega2rpy_rate_;
  Amat_continuous_.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();
  Amat_continuous_(11, 12) = 1;
  Bmat_continuous_.setZero();
  for (int i = 0; i < 4; i++) {
    Eigen::Matrix<float, 3, 1> vec;
    vec = foot_position_in_world_.block(3 * i, 0, 3, 1);
    Bmat_continuous_.block(6, i * 3, 3, 3) = inv_inertial_ * cross2mat(vec);
    Bmat_continuous_.block(9, i * 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity() / config.body_mass;
  }
}

void convexMPC::discretizeStateEquation() {
  Eigen::Matrix<float, 25, 25> mat25, expm;
  mat25.setZero();
  mat25.block(0, 0, 13, 13) = Amat_continuous_ * dt_mpc_;
  mat25.block(0, 13, 13, 12) = Bmat_continuous_ * dt_mpc_;
  expm = mat25.exp();
  Amat_discrete_ = expm.block(0, 0, 13, 13);
  Bmat_discrete_ = expm.block(0, 13, 13, 12);
}

// calculate body's  reference trajectory for MPC
void convexMPC::generateReferenceTrajectory() {
  constexpr float max_pos_error{0.2};
  constexpr float max_rpy_error{0.1};
  root_position_target_ = root_position_target_ + root_velocity_target_ * dt_mpc_;
  root_position_target_[2] = config.body_height;
  root_euler_target_[2] = root_euler_target_[2] + root_omega_target_[2] * config.dt;
  root_euler_target_[0] = 0;
  root_euler_target_[1] = 0;
// avoid big difference betwween position and the first target position
#ifdef DEBUG_MPC
  std::cout << "root_position_target_\n" << root_position_target_ - root_position_ << std::endl;
#endif
  float p_start;
  for (int i = 0; i < 2; i++) {
    p_start = root_position_target_[i];
    if (p_start - root_position_[i] > max_pos_error) { root_position_target_[i] = root_position_[i] + max_pos_error; }
    if (p_start - root_position_[i] < -max_pos_error) { root_position_target_[i] = root_position_[i] - max_pos_error; }
  }
  for (int i = 0; i < 3; i++) {
    p_start = root_euler_target_[i];
    if (p_start - root_euler_[i] > max_rpy_error) { root_euler_target_[i] = root_euler_[i] + max_rpy_error; }
    if (p_start - root_euler_[i] < -max_rpy_error) { root_euler_target_[i] = root_euler_[i] - max_rpy_error; }
  }
  // set  reference trakectory
  traj_.block(0, 0, 3, 1) = root_euler_target_;  // rpy xyz omega vel
  traj_.block(3, 0, 3, 1) = root_position_target_;
  traj_.block(6, 0, 3, 1) = root_omega_target_;
  traj_.block(9, 0, 3, 1) = root_velocity_target_;
  traj_[12] = -9.81;
  for (int i = 1; i < horizon_mpc_; i++) {
    traj_.block(13 * i, 0, 12, 1) = traj_.block(13 * (i - 1), 0, 12, 1);
    if (!optimism_) {
      traj_.block(13 * i, 0, 3, 1) =
          traj_.block(13 * (i - 1), 0, 3, 1) + dt_mpc_ * traj_.block(13 * (i - 1) + 6, 0, 3, 1);
    } else {
      traj_.block(13 * i, 0, 3, 1) =
          traj_.block(13 * (i - 1), 0, 3, 1) + dt_mpc_ * mat_omega2rpy_rate_ * traj_.block(13 * (i - 1) + 6, 0, 3, 1);
    }
    traj_.block(13 * i + 3, 0, 3, 1) =
        traj_.block(13 * (i - 1) + 3, 0, 3, 1) + dt_mpc_ * traj_.block(13 * (i - 1) + 9, 0, 3, 1);
    traj_[13 * i + 12] = -9.81;
  }
}

// void convexMPC::getJointTorque(
//      Eigen::Matrix<float, 12, 1>& joint_torque) {
//   Eigen::Matrix<float, 3, 1> foot_foorce_tmp;
//   for (int leg = 0; leg < 4; leg++) {
//     foot_foorce_tmp = rotm_body2world_.transpose() * foot_force_in_world_.segment<3>(3 * leg);
//     Eigen::Matrix3f foot_jacob = fusion_data.foot_jacobian_in_body.block<3, 3>(3 * leg, 3 * leg);
//     joint_torque.segment<3>(3 * leg) = foot_jacob.transpose() * foot_foorce_tmp;
//   }
//   std::cout << "joint_torque:\n" << joint_torque << std::endl;
// }

// calaulate Ampc and Bmpc
void convexMPC::FormulateQPform() {
  A_mpc_.block(0, 0, 13, 13) = Amat_discrete_;
  for (int r = 1; r < horizon_mpc_; r++) {
    A_mpc_.block(13 * r, 0, 13, 13) = Amat_discrete_ * A_mpc_.block(13 * (r - 1), 0, 13, 13);
  }
  // B_mpc
  B_mpc_.resize(13 * horizon_mpc_, 3 * sum_contact_);
  B_mpc_.setZero();
  int col_B_mpc{0};
  Eigen::Matrix<float, 13, -1> Bmat_tmp;
  Eigen::Matrix<float, 13, 12> Bmat_discrete_tmp;
  updateFootPositionTable();
  for (int col = 0; col < horizon_mpc_; col++) {
    calculateBdiscrete(Bmat_discrete_tmp, foot_pos_table_.col(col));
    int num_contacti = contact_table_.block(0, col, 4, 1).sum();
    if (num_contacti == 0) continue;
    Bmat_tmp.resize(13, 3 * num_contacti);
    Bmat_tmp.setZero();
    col_B_mpc += 3 * num_contacti;
    int a{0};
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, col) == 1) {
        if (!optimism_)
          Bmat_tmp.block(0, 3 * a, 13, 3) = Bmat_discrete_.block(0, 3 * leg, 13, 3);
        else
          Bmat_tmp.block(0, 3 * a, 13, 3) = Bmat_discrete_tmp.block(0, 3 * leg, 13, 3);  // optimism
        a++;
      }
    }

    for (int row = col; row < horizon_mpc_; row++) {
      if (col == row) {
        B_mpc_.block(13 * row, col_B_mpc - 3 * num_contacti, 13, 3 * num_contacti) = Bmat_tmp;
      } else {
        B_mpc_.block(13 * row, col_B_mpc - 3 * num_contacti, 13, 3 * num_contacti) =
            A_mpc_.block(13 * (row - col - 1), 0, 13, 13) * Bmat_tmp;
      }
    }
  }
  x0_qp_.block(0, 0, 3, 1) = root_euler_;  // rpy xyz omega vel
  x0_qp_.block(3, 0, 3, 1) = root_position_;
  x0_qp_.block(6, 0, 3, 1) = root_omega_;
  x0_qp_.block(9, 0, 3, 1) = root_velocity_;
  x0_qp_[12] = -9.81;

  Kweight_qp_.resize(3 * sum_contact_, 3 * sum_contact_);
  Kweight_qp_.setZero();
  Kweight_qp_.diagonal() = kweight_qp_unit_.replicate(sum_contact_, 1);
  updateLWeight();
  Hessian_qp_tmp = 2 * B_mpc_.transpose() * Lweight_qp_;
  Gradient_qp_ = Hessian_qp_tmp * (A_mpc_ * x0_qp_ - traj_);
  Hessian_qp_ = (Hessian_qp_tmp * B_mpc_ + 2 * Kweight_qp_);
}

void convexMPC::generateContraint() {
  assert(config.mu >= 0.0001 && "mu_ must not be 0!");
  float mu_inv = 1.f / config.mu;
  Eigen::Matrix<float, 6, 3> constraint_unit;
  Eigen::Matrix<float, 12, 1> lbA_unit;
  constraint_unit << mu_inv, 0, 1.f, -mu_inv, 0, 1.f, 0, mu_inv, 1.f, 0, -mu_inv, 1.f, 0, 0, 1.f, 0, 0, -1.f;
  Eigen::Vector3f tau_max;
  tau_max << 60.0, 60.0, 60.0;
  lbA_unit << 0, 0, 0, 0, 0, -config.max_reaction_force, -tau_max[0], -tau_max[1], -tau_max[2], -tau_max[0],
      -tau_max[1], -tau_max[2];
  int cols{3};
  int rows{12};
  constraint_mat_.resize(rows * sum_contact_, cols * sum_contact_);
  constraint_lb_A_.resize(rows * sum_contact_, 1);
  constraint_mat_.setZero();
  constraint_lb_A_.setZero();

  Eigen::Matrix<float, 12, 3> JacobR;
  JacobR = jacobian_leg_.transpose() * (rotm_body2world_.transpose());
  int a{0};
  for (int i = 0; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, i) == 1) {
        constraint_mat_.block(a * rows, a * cols, 6, 3) = constraint_unit;
        constraint_mat_.block(a * rows + 6, a * cols, 3, 3) = -JacobR.block(3 * leg, 0, 3, 3);
        constraint_mat_.block(a * rows + 6 + 3, a * cols, 3, 3) = JacobR.block(3 * leg, 0, 3, 3);
        constraint_lb_A_.block(a * rows, 0, rows, 1) = lbA_unit;
        a++;
      }
    }
  }
}

void convexMPC::generateContraintV2() {
  assert(config.mu >= 0.0001 && "mu_ must not be 0!");
  float mu_inv = 1.f / config.mu;
  Eigen::Matrix<float, 6, 3> constraint_unit;
  Eigen::Matrix<float, 6, 1> lbA_unit;
  constraint_unit << mu_inv, 0, 1.f, -mu_inv, 0, 1.f, 0, mu_inv, 1.f, 0, -mu_inv, 1.f, 0, 0, 1.f, 0, 0, -1.f;
  lbA_unit << 0, 0, 0, 0, 0, -config.max_reaction_force;
  int cols = constraint_unit.cols();
  int rows = constraint_unit.rows();
  constraint_mat_.resize(rows * sum_contact_, cols * sum_contact_);
  constraint_lb_A_.resize(rows * sum_contact_, 1);
  constraint_mat_.setZero();
  constraint_lb_A_.setZero();
  int a{0};
  for (int i = 0; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, i) == 1) {
        constraint_mat_.block(a * rows, a * cols, rows, cols) = constraint_unit;
        // lbA_unit[5] = -config.max_reaction_force * constrainFzmax(phase_table_(leg, i));
        constraint_lb_A_.block(a * rows, 0, rows, 1) = lbA_unit;
        a++;
      }
    }
  }
}

void convexMPC::solveMPC() {
  if (real_allocated) {
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_A_qpoases);
    // free(ub_A_qpoases);
    // free(ub_qpoases);
    // free(lb_qpoases);
    free(q_soln);
  }
  int col_A{3};
  int row_A{12};
  H_qpoases = (qpOASES::real_t*)malloc(3 * sum_contact_ * 3 * sum_contact_ * sizeof(qpOASES::real_t));
  g_qpoases = (qpOASES::real_t*)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  A_qpoases = (qpOASES::real_t*)malloc(row_A * sum_contact_ * col_A * sum_contact_ * sizeof(qpOASES::real_t));
  lb_A_qpoases = (qpOASES::real_t*)malloc(row_A * sum_contact_ * sizeof(qpOASES::real_t));
  // ub_A_qpoases = (qpOASES::real_t*)malloc(row_A * sum_contact_ * sizeof(qpOASES::real_t));
  q_soln = (qpOASES::real_t*)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  // lb_qpoases = (qpOASES::real_t*)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  // ub_qpoases = (qpOASES::real_t*)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  real_allocated = 1;

  matrix_to_real(H_qpoases, Hessian_qp_, 3 * sum_contact_, 3 * sum_contact_);
  matrix_to_real(g_qpoases, Gradient_qp_, 3 * sum_contact_, 1);
  matrix_to_real(A_qpoases, constraint_mat_, row_A * sum_contact_, col_A * sum_contact_);
  matrix_to_real(lb_A_qpoases, constraint_lb_A_, row_A * sum_contact_, 1);
  // matrix_to_real(ub_A_qpoases, constraint_ub_A_, row_A * sum_contact_, 1);
  // matrix_to_real(lb_qpoases, constraint_lb_, 3 * sum_contact_, 1);
  // matrix_to_real(ub_qpoases, constraint_ub_, 3 * sum_contact_, 1);
#ifdef DEBUG_MPC
  std::cout << "Hessian_qp_:\n";
  printMatrix(Hessian_qp_);
  std::cout << "Gradient_qp_:\n";
  printMatrix(Gradient_qp_);
  std::cout << "constraint_mat_:\n";
  printMatrix(constraint_mat_);
  std::cout << "constraint_lb_A_:\n";
  printMatrix(constraint_lb_A_);
#endif
  qpOASES::int_t nV, nC;
  nV = 3 * sum_contact_;
  nC = row_A * sum_contact_;
  qpOASES::QProblem problem(nV, nC);
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  problem.setOptions(op);
  qpOASES::int_t nWSR = 10000;
  qpOASES::real_t cpu_time{0.01};
  int rval = problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_A_qpoases, NULL, nWSR, &cpu_time);
  if (rval == qpOASES::RET_INIT_FAILED) printf("failed to init!\n");
  if (rval == qpOASES::RET_MAX_NWSR_REACHED) printf("failed to solve,reach max nWSR!\n");
  int rval2 = problem.getPrimalSolution(q_soln);
  if (rval2 != qpOASES::SUCCESSFUL_RETURN) printf("failed to get answer!\n");
  std::cout << "variance:" << problem.getObjVal() << std::endl;
  std::cout << "number:" << problem.getObjVal() << std::endl;

  int tmp{0};
  for (int i = 0; i < 4; i++) {
    if (contact_table_(i, 0) != 0) {
      foot_force_in_world_[3 * i] = q_soln[3 * tmp];
      foot_force_in_world_[3 * i + 1] = q_soln[3 * tmp + 1];
      foot_force_in_world_[3 * i + 2] = q_soln[3 * tmp + 2];
      Eigen::Matrix<float, 11, 1> torque;
      torque = constraint_mat_.block(11 * tmp, 3 * tmp, 11, 3) * foot_force_in_world_.block(3 * i, 0, 3, 1);
      // std::cout << "torque contraint\n" << torque << std::endl;
      tmp++;

    } else {
      foot_force_in_world_[3 * i] = 0.0;
      foot_force_in_world_[3 * i + 1] = 0.0;
      foot_force_in_world_[3 * i + 2] = 0.0;
    }
  }
  std::cout << "foot_force_in_world_:\n" << foot_force_in_world_ << std::endl;
  foot_force_in_world_ = -foot_force_in_world_;
  for (int leg = 0; leg < 4; leg++) {
    joint_torque_.segment<3>(3 * leg) = jacobian_leg_.block(0, 3 * leg, 3, 3).transpose() *
                                        rotm_body2world_.transpose() * foot_force_in_world_.segment<3>(3 * leg);
  }
  std::cout << "joint_torque:\n" << joint_torque_ << std::endl;
}

void convexMPC::solveMPCV2()  // six constraints in each foot
{
  if (real_allocated) {
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_A_qpoases);
    free(q_soln);
  }
  int col_A{3};
  int row_A{6};
  H_qpoases = (qpOASES::real_t*)malloc(3 * sum_contact_ * 3 * sum_contact_ * sizeof(qpOASES::real_t));
  g_qpoases = (qpOASES::real_t*)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  A_qpoases = (qpOASES::real_t*)malloc(row_A * sum_contact_ * col_A * sum_contact_ * sizeof(qpOASES::real_t));
  lb_A_qpoases = (qpOASES::real_t*)malloc(row_A * sum_contact_ * sizeof(qpOASES::real_t));
  q_soln = (qpOASES::real_t*)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  real_allocated = 1;

  matrix_to_real(H_qpoases, Hessian_qp_, 3 * sum_contact_, 3 * sum_contact_);
  matrix_to_real(g_qpoases, Gradient_qp_, 3 * sum_contact_, 1);
  matrix_to_real(A_qpoases, constraint_mat_, row_A * sum_contact_, col_A * sum_contact_);
  matrix_to_real(lb_A_qpoases, constraint_lb_A_, row_A * sum_contact_, 1);
  // qp
  time_t begin, end;
  begin = clock();
  qpOASES::int_t nV = 3 * sum_contact_;
  qpOASES::int_t nC = row_A * sum_contact_;
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  qpOASES::QProblem problem(nV, nC);
  problem.setOptions(op);
  qpOASES::int_t nWSR = 200;
  qpOASES::real_t cpu_time{0.1};
  int rval = problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_A_qpoases, NULL, nWSR, &cpu_time);
  if (rval != qpOASES::SUCCESSFUL_RETURN) printf("failed to init!\n");
  int rval2 = problem.getPrimalSolution(q_soln);
  if (rval2 != qpOASES::SUCCESSFUL_RETURN) printf("failed to solve!\n");
  end = clock();
  std::cout << "qp time:" << end - begin << std::endl;

  // get answer
  int tmp{0};
  for (int i = 0; i < 4; i++) {
    if (contact_table_(i, 0) != 0) {
      foot_force_in_world_[3 * i] = q_soln[3 * tmp];
      foot_force_in_world_[3 * i + 1] = q_soln[3 * tmp + 1];
      foot_force_in_world_[3 * i + 2] = q_soln[3 * tmp + 2];
      tmp++;
    } else {
      foot_force_in_world_[3 * i] = 0.0;
      foot_force_in_world_[3 * i + 1] = 0.0;
      foot_force_in_world_[3 * i + 2] = 0.0;
    }
  }
  foot_force_in_world_ = -foot_force_in_world_;
  for (int leg = 0; leg < 4; leg++) {
    joint_torque_.segment<3>(3 * leg) = jacobian_leg_.block(0, 3 * leg, 3, 3).transpose() *
                                        rotm_body2world_.transpose() * foot_force_in_world_.segment<3>(3 * leg);
  }
  std::cout << "joint_torque:\n" << joint_torque_ << std::endl;
}

// float convexMPC::constrainFzmax(float phase) {
//   if (phase <= 0.1) {
//     return 10.0 * phase;
//   } else if (phase <= 0.9) {
//     return 1.0;
//   } else {
//     return 10.0 - 10.0 * phase;
//   }
//   // return -4.0 * (phase - 0.5) * (phase - 0.5) + 1;  // inverted pendulum

//   // return 1.0;
// }

void convexMPC::updateFootPositionTable() {
  foot_pos_table_.block(0, 0, 12, 1) = foot_position_in_world_;
  Eigen::Vector3f p_hip, root_vel;
  root_vel = root_velocity_;
  root_vel[2] = 0.0;
  for (int i = 1; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, i) == 0) {
        getHipPositionInWorld(p_hip, leg);
        foot_pos_table_.block(3 * leg, i, 3, 1) = p_hip + root_vel * stance_duration_ / 2;
      } else {
        foot_pos_table_.block(3 * leg, i, 3, 1) = foot_pos_table_.block(3 * leg, i - 1, 3, 1) - root_vel * dt_mpc_;
      }
    }
  }
}

void convexMPC::getHipPositionInWorld(Eigen::Vector3f& p, int leg) {
  p = config.p_hip;
  if (leg == 2 || leg == 3) p[0] = -p[0];
  if (leg == 0 || leg == 2) p[1] = -p[1];
  p = rotm_body2world_ * (p - config.com_offset);
}

void convexMPC::calculateAdiscrete() {
  Amat_discrete_.setIdentity();
  Amat_discrete_.block(0, 6, 3, 3) = mat_omega2rpy_rate_ * dt_mpc_;
  Amat_discrete_.block(3, 9, 3, 3) = Eigen::Matrix3f::Identity() * dt_mpc_;
  Amat_discrete_(5, 13) = 0.5 * dt_mpc_ * dt_mpc_;
  Amat_discrete_(11, 12) = dt_mpc_;
}

void convexMPC::calculateBdiscrete(Eigen::Matrix<float, 13, 12>& Bmat_discrete, Eigen::Matrix<float, 12, 1> r_foot) {
  Eigen::Matrix<float, 3, 12> Bmat_tmp;
  Eigen::Matrix<float, 3, 1> r;
  Bmat_discrete.setZero();
  for (int leg = 0; leg < 4; leg++) {
    r = r_foot.block(3 * leg, 0, 3, 1);
    Bmat_tmp.block(0, 3 * leg, 3, 3) = inv_inertial_ * cross2mat(r) * dt_mpc_;
  }
  Bmat_discrete.block(0, 0, 3, 12) = mat_omega2rpy_rate_ * Bmat_tmp * dt_mpc_ * 0.5;
  Bmat_discrete.block(6, 0, 3, 12) = Bmat_tmp;
  Bmat_discrete.block(9, 0, 3, 3) = Eigen::Matrix3f::Identity() / config.body_mass * dt_mpc_;
  Bmat_discrete.block(3, 0, 3, 3) = Bmat_discrete.block(9, 0, 3, 3) * dt_mpc_ * 0.5;
  for (int leg = 1; leg < 4; leg++) {
    Bmat_discrete.block(3, 3 * leg, 3, 3) = Bmat_discrete.block(3, 0, 3, 3);
    Bmat_discrete.block(9, 3 * leg, 3, 3) = Bmat_discrete.block(9, 0, 3, 3);
  }
}

void convexMPC::updateLWeight() {
  Eigen::Matrix<float, 13, 13> lweight_qp_tmp;
  lweight_qp_tmp.setZero();
  lweight_qp_tmp.diagonal() = lweight_qp_unit_;
  if (optimism_) {
    for (int i = 1; i < 4; i++) {
      lweight_qp_tmp.block(3 * i, 3 * i, 3, 3) =
          rotm_body2world_ * lweight_qp_tmp.block(3 * i, 3 * i, 3, 3) * rotm_body2world_.transpose();
    }
  }
  for (int i = 0; i < horizon_mpc_; i++) { Lweight_qp_.block(13 * i, 13 * i, 13, 13) = lweight_qp_tmp; }
}

void convexMPC::printMatrix(Eigen::Matrix<float, -1, -1> mat) { std::cout << mat << std::endl; }
