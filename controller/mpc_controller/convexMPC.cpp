#include "convexMPC.h"
#include "math_utils.h"
#include <Eigen/Dense>
#include <ctime>
#include <iostream>
#include <qpOASES.hpp>
#include <thread>
#include <unsupported/Eigen/MatrixFunctions>

void matrix_to_real(qpOASES::real_t *dst, Eigen::Matrix<float, -1, -1> src,
                    int rows, int cols) {
  int a = 0;
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      dst[a] = src(r, c);
      a++;
    }
  }
}
// resize  matrix using horizon_mpc_,and set weights for QP problem
void convexMPC::init() {
  contact_table_.resize(4, horizon_mpc_);
  foot_pos_table_.resize(12, horizon_mpc_);
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

  config.dt = 0.002;
  config.body_mass = 50;
  config.mu = 0.6;
  config.max_reaction_force = 1000;
  config.body_height = 0.45;
  config.filter_omega = 0.1;
  config.filter_vel = 0.1;
  config.weight_rpy << 40, 40, 20;
  config.weight_xyz << 50, 50, 200;
  config.weight_omega << 0.1, 0.1, 1.0;
  config.weight_vel << 2, 2, 0.5;
  config.com_offset << 0, 0, 0;
  config.p_hip << 0.2375, 0.077, 0;
  config.inertial << 1.652939, 3.20795, 3.027734;
  config.weight_input = 0.001;

  lweight_qp_unit_ << config.weight_rpy[0], config.weight_rpy[1],
      config.weight_rpy[2], config.weight_xyz[0], config.weight_xyz[1],
      config.weight_xyz[2], config.weight_omega[0], config.weight_omega[1],
      config.weight_omega[2], config.weight_vel[0], config.weight_vel[1],
      config.weight_vel[2], 0.0;
  kweight_qp_unit_ << config.weight_input, config.weight_input,
      config.weight_input;
  Lweight_qp_.diagonal() =
      lweight_qp_unit_.replicate(horizon_mpc_, 1); // state weight
  Kweight_qp_.diagonal() =
      kweight_qp_unit_.replicate(4 * horizon_mpc_, 1); // unput weight
  counter_ = 0;
  dt_mpc_ = config.dt * iteration_step_;
  std::cout << "init MPC finished\n";
}

void convexMPC::update(const FusionData &f, const commandData &o, int *mpcTable,
                       float stance_duration) {
  stance_duration_ = stance_duration;
  root_euler_ = f.body.rpy;
  root_omega_ = f.body.omega_in_world;
  root_position_ = f.body.position;
  root_velocity_ = f.body.vel_in_world;
  rotm_body2world_ = f.body.rotm_body2world;
  joint_vel_ = f.leg.joint_vel;

  root_omega_target_ =
      root_omega_target_ * (1 - config.filter_omega) +
      config.filter_omega * rotm_body2world_ * o.body.omega_in_body;
  root_velocity_target_ =
      root_velocity_target_ * (1 - config.filter_vel) +
      config.filter_vel * rotm_body2world_ * o.body.vel_in_body;
  if (!optimism_) {
    root_omega_target_[0] = 0.0;
    root_omega_target_[1] = 0.0;
    root_velocity_target_[2] = 0.0;
  }

  for (int leg = 0; leg < 4; leg++) {
    foot_position_in_world_.segment<3>(3 * leg) =
        rotm_body2world_ *
        (f.leg.foot_position_in_body.segment<3>(3 * leg) - config.com_offset);
    jacobian_leg_.block(0, 3 * leg, 3, 3) =
        f.leg.foot_jacobian_in_body.block<3, 3>(0, 3 * leg);
  }
  if (!optimism_) {
    Eigen::Vector3f rpy;
    Eigen::Matrix<float, 3, 3> R_yaw;
    rpy << 0.0, 0.0, root_euler_[2];
    R_yaw = euler2rotm(rpy);
    Inertial_in_world_ = R_yaw * config.inertial * R_yaw.transpose();
  } else {
    Inertial_in_world_ =
        rotm_body2world_ * config.inertial * rotm_body2world_.transpose();
  }
  inv_inertial_ = Inertial_in_world_.inverse();
  mat_omega2rpy_rate_ = omega2rpyrate(root_euler_);

  for (int i = 0; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      contact_table_(leg, i) = mpcTable[4 * i + leg];
    }
  }
  sum_contact_ = contact_table_.sum();
}

void convexMPC::run(const FusionData &f, const commandData &o, int *mpcTable,
                    float stance_duration) {
  time_t begin, end;
  update(f, o, mpcTable, stance_duration);
  if (counter_ % iteration_step_ == 0) {
    counter_ = 0;
    std::cout << "command velocity:" << root_velocity_target_ << std::endl;
    std::cout << "current velocity:" << root_velocity_ << std::endl;
    begin = clock();
    // testMPC();
#ifdef DEBUG_MPC
    std::cout << "mpc_table:\n" << contact_table_ << std::endl;
#endif
    // calculateContinuousEquation();
    // discretizeStateEquation();
    calculateAdiscrete();
    calculateBdiscrete(Bmat_discrete_, foot_position_in_world_);
    generateReferenceTrajectory();
    FormulateQPform();
    // generateContraint();
    generateContraint_TN();
    end = clock();
    std::cout << "before solving mpc time0:" << (end - begin) << std::endl;
    begin = clock();
    // solveMPC();
    solveMPC_TN();
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
    Bmat_continuous_.block(9, i * 3, 3, 3) =
        Eigen::Matrix<float, 3, 3>::Identity() / config.body_mass;
  }
#ifdef DEBUG_MPC
  std::cout << "inv_inertial_:\n" << inv_inertial_ << std::endl;
  std::cout << "foot_position_in_world_:\n"
            << foot_position_in_world_ << std::endl;
  std::cout << "Amat_continuous_:\n" << Amat_continuous_ << std::endl;
  std::cout << "Bmat_continuous_:\n" << Bmat_continuous_ << std::endl;
#endif
}

void convexMPC::discretizeStateEquation() {
  Eigen::Matrix<float, 25, 25> mat25, expm;
  mat25.setZero();
  mat25.block(0, 0, 13, 13) = Amat_continuous_ * dt_mpc_;
  mat25.block(0, 13, 13, 12) = Bmat_continuous_ * dt_mpc_;
  expm = mat25.exp();
  Amat_discrete_ = expm.block(0, 0, 13, 13);
  Bmat_discrete_ = expm.block(0, 13, 13, 12);
#ifdef DEBUG_MPC
  std::cout << "Amat_discrete_:\n" << Amat_discrete_ << std::endl;
  std::cout << "Bmat_discrete_:\n" << Bmat_discrete_ << std::endl;
#endif
}

// calculate body's  reference trajectory for MPC
void convexMPC::generateReferenceTrajectory() {
  constexpr float max_pos_error{0.1};
  constexpr float max_rpy_error{0.1};
  root_position_target_ =
      root_position_target_ + root_velocity_target_ * dt_mpc_;
  root_position_target_[2] = config.body_height;
  root_euler_target_[2] =
      root_euler_target_[2] + root_omega_target_[2] * dt_mpc_;
  root_euler_target_[0] = 0;
  root_euler_target_[1] = 0;
// avoid big difference betwween position and the first target position
#ifdef DEBUG_MPC
  std::cout << "root_position_target_\n"
            << root_position_target_ - root_position_ << std::endl;
#endif
  float p_start;
  for (int i = 0; i < 2; i++) {
    p_start = root_position_target_[i];
    if (p_start - root_position_[i] > max_pos_error) {
      root_position_target_[i] = root_position_[i] + max_pos_error;
    }
    if (p_start - root_position_[i] < -max_pos_error) {
      root_position_target_[i] = root_position_[i] - max_pos_error;
    }
  }
  for (int i = 0; i < 3; i++) {
    p_start = root_euler_target_[i];
    if (p_start - root_euler_[i] > max_rpy_error) {
      root_euler_target_[i] = root_euler_[i] + max_rpy_error;
    }
    if (p_start - root_euler_[i] < -max_rpy_error) {
      root_euler_target_[i] = root_euler_[i] - max_rpy_error;
    }
  }
  // set  reference trakectory
  traj_.block(0, 0, 3, 1) = root_euler_target_; // rpy xyz omega vel
  traj_.block(3, 0, 3, 1) = root_position_target_;
  traj_.block(6, 0, 3, 1) = root_omega_target_;
  traj_.block(9, 0, 3, 1) = root_velocity_target_;
  traj_[12] = -9.81;
  for (int i = 1; i < horizon_mpc_; i++) {
    traj_.block(13 * i, 0, 12, 1) = traj_.block(13 * (i - 1), 0, 12, 1);
    if (!optimism_) {
      traj_.block(13 * i, 0, 3, 1) =
          traj_.block(13 * (i - 1), 0, 3, 1) +
          dt_mpc_ * traj_.block(13 * (i - 1) + 6, 0, 3, 1);
    } else {
      traj_.block(13 * i, 0, 3, 1) = traj_.block(13 * (i - 1), 0, 3, 1) +
                                     dt_mpc_ * mat_omega2rpy_rate_ *
                                         traj_.block(13 * (i - 1) + 6, 0, 3, 1);
    }
    traj_.block(13 * i + 3, 0, 3, 1) =
        traj_.block(13 * (i - 1) + 3, 0, 3, 1) +
        dt_mpc_ * traj_.block(13 * (i - 1) + 9, 0, 3, 1);
    traj_[13 * i + 12] = -9.81;
  }
#ifdef DEBUG_MPC
  std::cout << "traj_:\n";
  for (int i = 0; i < horizon_mpc_; i++) {
    for (int j = 0; j < 13; j++) {
      std::cout << traj_[13 * i + j] << " ";
    }
    std::cout << std::endl;
  }
#endif
}
// calaulate Ampc and Bmpc
void convexMPC::FormulateQPform() {
  A_mpc_.block(0, 0, 13, 13) = Amat_discrete_;
  for (int r = 1; r < horizon_mpc_; r++) {
    A_mpc_.block(13 * r, 0, 13, 13) =
        Amat_discrete_ * A_mpc_.block(13 * (r - 1), 0, 13, 13);
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
    if (num_contacti == 0)
      continue;
    Bmat_tmp.resize(13, 3 * num_contacti);
    Bmat_tmp.setZero();
    col_B_mpc += 3 * num_contacti;
    int a{0};
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, col) == 1) {
        if (!optimism_)
          Bmat_tmp.block(0, 3 * a, 13, 3) =
              Bmat_discrete_.block(0, 3 * leg, 13, 3);
        else
          Bmat_tmp.block(0, 3 * a, 13, 3) =
              Bmat_discrete_tmp.block(0, 3 * leg, 13, 3); // optimism
        a++;
      }
    }

    for (int row = col; row < horizon_mpc_; row++) {
      if (col == row) {
        B_mpc_.block(13 * row, col_B_mpc - 3 * num_contacti, 13,
                     3 * num_contacti) = Bmat_tmp;
      } else {
        B_mpc_.block(13 * row, col_B_mpc - 3 * num_contacti, 13,
                     3 * num_contacti) =
            A_mpc_.block(13 * (row - col - 1), 0, 13, 13) * Bmat_tmp;
      }
    }
  }
  x0_qp_.block(0, 0, 3, 1) = root_euler_; // rpy xyz omega vel
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
#ifdef DEBUG_MPC
  std::cout << "Initial state in QP:\n" << x0_qp_ << std::endl;
  std::cout << "kweight_qp_unit_:\n" << kweight_qp_unit_ << std::endl;
  std::cout << "lweight_qp_unit_:\n" << lweight_qp_unit_ << std::endl;
  // std::cout << "A_mpc_:\n" << A_mpc_ << std::endl;
  std::cout << "B_mpc_20 20:\n" << B_mpc_.rows(1) << std::endl;
  std::cout << "Hessian_qp_:\n" << Hessian_qp_ << std::endl;
  std::cout << "Gradient_qp_:\n" << Gradient_qp_ << std::endl;
#endif
}

void convexMPC::generateContraint_TN() {
  assert(config.mu >= 0.0001 && "mu_ must not be 0!");
  float mu_inv = 1.f / config.mu;
  Eigen::Matrix<float, 6, 3> constraint_unit;
  Eigen::Matrix<float, 12, 1> lbA_unit;
  constraint_unit << mu_inv, 0, 1.f, -mu_inv, 0, 1.f, 0, mu_inv, 1.f, 0,
      -mu_inv, 1.f, 0, 0, 1.f, 0, 0, -1.f;
  Eigen::Vector3f tau_max;
  int cols{3};
  int rows{12};
  constraint_mat_.resize(rows * sum_contact_, cols * sum_contact_);
  constraint_lb_A_.resize(rows * sum_contact_, 1);
  constraint_mat_.setZero();
  constraint_lb_A_.setZero();

  Eigen::Matrix<float, 12, 3> JacobR;
  JacobR = jacobian_leg_.transpose() * (rotm_body2world_.transpose());
  int a{0};
  // std::cout << "joint_vel:" << joint_vel_ << std::endl;
  for (int i = 0; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, i) == 1) {
        tau_max << getTorqueBorderFromTN(joint_vel_[3 * leg]),
            getTorqueBorderFromTN(joint_vel_[3 * leg + 1]),
            getTorqueBorderFromTN(joint_vel_[3 * leg + 2]);
        lbA_unit << 0, 0, 0, 0, 0, -config.max_reaction_force, -tau_max[0],
            -tau_max[1], -tau_max[2], -tau_max[0], -tau_max[1], -tau_max[2];
        // std::cout << "tau_max:" << tau_max << std::endl;
        constraint_mat_.block(a * rows, a * cols, 6, 3) = constraint_unit;
        constraint_mat_.block(a * rows + 6, a * cols, 3, 3) =
            -JacobR.block(3 * leg, 0, 3, 3);
        constraint_mat_.block(a * rows + 6 + 3, a * cols, 3, 3) =
            JacobR.block(3 * leg, 0, 3, 3);
        constraint_lb_A_.block(a * rows, 0, rows, 1) = lbA_unit;
        a++;
      }
    }
  }
}

void convexMPC::generateContraint() {
  assert(config.mu >= 0.0001 && "mu_ must not be 0!");
  float mu_inv = 1.f / config.mu;
  Eigen::Matrix<float, 6, 3> constraint_unit;
  Eigen::Matrix<float, 6, 1> lbA_unit;
  constraint_unit << mu_inv, 0, 1.f, -mu_inv, 0, 1.f, 0, mu_inv, 1.f, 0,
      -mu_inv, 1.f, 0, 0, 1.f, 0, 0, -1.f;
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
        constraint_lb_A_.block(a * rows, 0, rows, 1) = lbA_unit;
        a++;
      }
    }
  }
}

void convexMPC::solveMPC_TN() {
  if (real_allocated) {
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_A_qpoases);
    free(q_soln);
  }
  int col_A{3};
  int row_A{12};
  H_qpoases = (qpOASES::real_t *)malloc(3 * sum_contact_ * 3 * sum_contact_ *
                                        sizeof(qpOASES::real_t));
  g_qpoases =
      (qpOASES::real_t *)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  A_qpoases = (qpOASES::real_t *)malloc(row_A * sum_contact_ * col_A *
                                        sum_contact_ * sizeof(qpOASES::real_t));
  lb_A_qpoases =
      (qpOASES::real_t *)malloc(row_A * sum_contact_ * sizeof(qpOASES::real_t));
  q_soln =
      (qpOASES::real_t *)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  real_allocated = 1;

  matrix_to_real(H_qpoases, Hessian_qp_, 3 * sum_contact_, 3 * sum_contact_);
  matrix_to_real(g_qpoases, Gradient_qp_, 3 * sum_contact_, 1);
  matrix_to_real(A_qpoases, constraint_mat_, row_A * sum_contact_,
                 col_A * sum_contact_);
  matrix_to_real(lb_A_qpoases, constraint_lb_A_, row_A * sum_contact_, 1);
  qpOASES::int_t nV, nC;
  nV = 3 * sum_contact_;
  nC = row_A * sum_contact_;
  qpOASES::QProblem problem(nV, nC);
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  problem.setOptions(op);
  qpOASES::int_t nWSR = 1000;
  qpOASES::real_t cpu_time{0.01};
  int rval = problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL,
                          lb_A_qpoases, NULL, nWSR, &cpu_time);
  if (rval == qpOASES::RET_INIT_FAILED)
    printf("failed to init!\n");
  if (rval == qpOASES::RET_MAX_NWSR_REACHED)
    printf("failed to solve,reach max nWSR!\n");
  int rval2 = problem.getPrimalSolution(q_soln);
  if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to get answer!\n");
  std::cout << "variance:" << problem.getObjVal() << std::endl;
  int tmp{0};
  for (int i = 0; i < 4; i++) {
    if (contact_table_(i, 0) != 0) {
      foot_force_in_world_[3 * i] = q_soln[3 * tmp];
      foot_force_in_world_[3 * i + 1] = q_soln[3 * tmp + 1];
      foot_force_in_world_[3 * i + 2] = q_soln[3 * tmp + 2];
      Eigen::Matrix<float, 11, 1> torque;
      torque = constraint_mat_.block(11 * tmp, 3 * tmp, 11, 3) *
               foot_force_in_world_.block(3 * i, 0, 3, 1);
      // std::cout << "torque contraint\n" << torque << std::endl;
      tmp++;
    } else {
      foot_force_in_world_[3 * i] = 0.0;
      foot_force_in_world_[3 * i + 1] = 0.0;
      foot_force_in_world_[3 * i + 2] = 0.0;
    }
  }
  // std::cout << "foot_force_in_world_:\n" << foot_force_in_world_ <<
  // std::endl;
  foot_force_in_world_ = -foot_force_in_world_;
  for (int leg = 0; leg < 4; leg++) {
    joint_torque_.segment<3>(3 * leg) =
        jacobian_leg_.block(0, 3 * leg, 3, 3).transpose() *
        rotm_body2world_.transpose() * foot_force_in_world_.segment<3>(3 * leg);
  }
  // std::cout << "joint_torque_:\n" << joint_torque_.transpose() << std::endl;
}

void convexMPC::solveMPC() // six constraints in each foot
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
  H_qpoases = (qpOASES::real_t *)malloc(3 * sum_contact_ * 3 * sum_contact_ *
                                        sizeof(qpOASES::real_t));
  g_qpoases =
      (qpOASES::real_t *)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  A_qpoases = (qpOASES::real_t *)malloc(row_A * sum_contact_ * col_A *
                                        sum_contact_ * sizeof(qpOASES::real_t));
  lb_A_qpoases =
      (qpOASES::real_t *)malloc(row_A * sum_contact_ * sizeof(qpOASES::real_t));
  q_soln =
      (qpOASES::real_t *)malloc(3 * sum_contact_ * sizeof(qpOASES::real_t));
  real_allocated = 1;
  matrix_to_real(H_qpoases, Hessian_qp_, 3 * sum_contact_, 3 * sum_contact_);
  matrix_to_real(g_qpoases, Gradient_qp_, 3 * sum_contact_, 1);
  matrix_to_real(A_qpoases, constraint_mat_, row_A * sum_contact_,
                 col_A * sum_contact_);
  matrix_to_real(lb_A_qpoases, constraint_lb_A_, row_A * sum_contact_, 1);
  qpOASES::int_t nV = 3 * sum_contact_;
  qpOASES::int_t nC = row_A * sum_contact_;
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  qpOASES::QProblem problem(nV, nC);
  problem.setOptions(op);
  qpOASES::int_t nWSR = 100;
  qpOASES::real_t cpu_time{0.1};
  int rval = problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL,
                          lb_A_qpoases, NULL, nWSR, &cpu_time);
  if (rval != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to init!\n");
  int rval2 = problem.getPrimalSolution(q_soln);
  if (rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to solve!\n");
  for (int i = 0; i < 12; i++)
    std::cout << q_soln[i] << std::endl;
  // qpOASES::SQProblem problem_sqp(nV, nC);
  // problem_sqp.setOptions(op);
  // qpOASES::int_t nWSR = 200;
  // qpOASES::real_t cpu_time{0.1};
  // int rval = problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL,
  // lb_A_qpoases, NULL, nWSR, &cpu_time); if (rval !=
  // qpOASES::SUCCESSFUL_RETURN) printf("failed to init!\n"); int rval2 =
  // problem.getPrimalSolution(q_soln); if (rval2 != qpOASES::SUCCESSFUL_RETURN)
  // printf("failed to solve!\n"); get foot force and joint torque
  int tmp{0};
  for (int i = 0; i < 4; i++) {
    if (contact_table_(i, 0) != 0) {
      foot_force_in_world_[3 * i] = -q_soln[3 * tmp];
      foot_force_in_world_[3 * i + 1] = -q_soln[3 * tmp + 1];
      foot_force_in_world_[3 * i + 2] = -q_soln[3 * tmp + 2];
      tmp++;
    } else {
      foot_force_in_world_[3 * i] = 0.0;
      foot_force_in_world_[3 * i + 1] = 0.0;
      foot_force_in_world_[3 * i + 2] = 0.0;
    }
  }
  for (int leg = 0; leg < 4; leg++) {
    joint_torque_.segment<3>(3 * leg) =
        jacobian_leg_.block(0, 3 * leg, 3, 3).transpose() *
        rotm_body2world_.transpose() * foot_force_in_world_.segment<3>(3 * leg);
  }
}
float convexMPC::constrainFzmax(float phase) {
  if (optimism_) {
    if (phase <= 0.1) {
      return 10.0 * phase;
    } else if (phase <= 0.9) {
      return 1.0;
    } else {
      return 10.0 - 10.0 * phase;
    }
    // return -4.0 * (phase - 0.5) * (phase - 0.5) + 1;  // inverted pendulum
  } else
    return 1.0;
}

void convexMPC::updateFootPositionTable() {
  foot_pos_table_.block(0, 0, 12, 1) = foot_position_in_world_;
  Eigen::Vector3f p_hip, root_vel;
  root_vel = root_velocity_;
  root_vel[2] = 0.0;
  for (int i = 1; i < horizon_mpc_; i++) {
    for (int leg = 0; leg < 4; leg++) {
      if (contact_table_(leg, i) == 0) {
        getHipPositionInWorld(p_hip, leg);
        foot_pos_table_.block(3 * leg, i, 3, 1) =
            p_hip + root_vel * stance_duration_ / 2;
      } else {
        foot_pos_table_.block(3 * leg, i, 3, 1) =
            foot_pos_table_.block(3 * leg, i - 1, 3, 1) - root_vel * dt_mpc_;
      }
    }
  }
}

void convexMPC::getHipPositionInWorld(Eigen::Vector3f &p, int leg) {
  p = config.p_hip;
  if (leg == 2 || leg == 3)
    p[0] = -p[0];
  if (leg == 0 || leg == 2)
    p[1] = -p[1];
  p = rotm_body2world_ * (p - config.com_offset);
}

void convexMPC::calculateAdiscrete() {
  Amat_discrete_.setIdentity();
  Amat_discrete_.block(0, 6, 3, 3) = mat_omega2rpy_rate_ * dt_mpc_;
  Amat_discrete_.block(3, 9, 3, 3) = Eigen::Matrix3f::Identity() * dt_mpc_;
  Amat_discrete_(5, 13) = 0.5 * dt_mpc_ * dt_mpc_;
  Amat_discrete_(11, 12) = dt_mpc_;
}

void convexMPC::calculateBdiscrete(Eigen::Matrix<float, 13, 12> &Bmat_discrete,
                                   Eigen::Matrix<float, 12, 1> r_foot) {
  Eigen::Matrix<float, 3, 12> Bmat_tmp;
  Eigen::Matrix<float, 3, 1> r;
  Bmat_discrete.setZero();
  for (int leg = 0; leg < 4; leg++) {
    r = r_foot.block(3 * leg, 0, 3, 1);
    Bmat_tmp.block(0, 3 * leg, 3, 3) = inv_inertial_ * cross2mat(r) * dt_mpc_;
  }
  Bmat_discrete.block(0, 0, 3, 12) =
      mat_omega2rpy_rate_ * Bmat_tmp * dt_mpc_ * 0.5;
  Bmat_discrete.block(6, 0, 3, 12) = Bmat_tmp;
  Bmat_discrete.block(9, 0, 3, 3) =
      Eigen::Matrix3f::Identity() / config.body_mass * dt_mpc_;
  Bmat_discrete.block(3, 0, 3, 3) =
      Bmat_discrete.block(9, 0, 3, 3) * dt_mpc_ * 0.5;
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
          rotm_body2world_ * lweight_qp_tmp.block(3 * i, 3 * i, 3, 3) *
          rotm_body2world_.transpose();
    }
  }
  for (int i = 0; i < horizon_mpc_; i++) {
    Lweight_qp_.block(13 * i, 13 * i, 13, 13) = lweight_qp_tmp;
  }
}
inline float convexMPC::getTorqueBorderFromTN(float omega) {
  float abs_omega = fabs(omega);
  assert(abs_omega < 19.896 && "omega must <190 rpm in convex mpc");
  if (abs_omega < 11.5191) {
    return 109.0;
  } else {
    return (-11.113 * abs_omega + 237.0);
  } // 110rpm
}

void convexMPC::testMPC() {
  std::cout << "test MPC!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
  Eigen::Vector3f position, vWorld, omegaWorld;
  Eigen::Vector4f orientation;
  Eigen::Matrix<float, 12, 1> r_foot;
  position << 0.0, 0.0, 0.45;
  vWorld << 1.0, 0.0, 0.0;
  omegaWorld << 0.0, 0.0, 0.1;
  orientation << 1.0, 0.0, 0.0, 0.0;

  r_foot << 0.2, -0.1, -0.4, 0.2, 0.1, -0.4, -0.2, -0.1, -0.4, -0.2, 0.1, -0.4;
  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
  float *weights = Q;
  float *p_test = position.data();
  float *v_test = vWorld.data();
  float *w_test = omegaWorld.data();
  float *q_test = orientation.data();
  float *r_test = r_foot.data();
  float yaw = 0.0;
  float alpha = 1e-5;
  int horizon_mpc = 10;
  int *mpc_table;
  Eigen::Matrix<int, 4, 10, Eigen::ColMajor> contact_table;
  contact_table << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  float trajInitial[12] = {0.000462805, 0,    0.0331643, -0.0189391,
                           0.00102649,  0.45, 0,         0,
                           0,           0,    0,         0};
  float trajAll[12 * 36];
  for (int i = 0; i < horizon_mpc; i++)
    for (int j = 0; j < 12; j++)
      trajAll[12 * i + j] = trajInitial[j];

  stance_duration_ = 0.5;
  root_euler_ << 0.0, 0.0, 0.0;
  root_omega_ = omegaWorld;
  root_position_ = position;
  root_velocity_ = vWorld;
  rotm_body2world_ = Eigen::Matrix<float, 3, 3>::Identity(); // ok
  // joint_vel_ = f.joint_velocity;

  root_omega_target_ << 0.000462805, 0, 0.0331643;
  root_velocity_target_ << 0, 0, 0;
  root_position_target_ << -0.0189391, 0.00102649, 0.45;
  foot_position_in_world_ = r_foot;
  Inertial_in_world_ << 0.07, 0, 0, 0, 0.26, 0, 0, 0, 0.242;
  inv_inertial_ = Inertial_in_world_.inverse();

  mat_omega2rpy_rate_ = omega2rpyrate(root_euler_);
  contact_table_ << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  sum_contact_ = contact_table_.sum();
  config.body_mass = 9;
  config.dt = 0.002;
  config.mu = 0.4;
  config.max_reaction_force = 120;
  config.body_height = 0.45;
  config.filter_omega = 1;
  config.filter_vel = 1;
  config.weight_rpy << 0.25, 0.25, 10;
  config.weight_xyz << 2, 2, 50;
  config.weight_omega << 0, 0, 0.3;
  config.weight_vel << 0.2, 0.2, 0.1;
  config.com_offset << 0, 0, 0;
  config.p_hip << 0, 0, 0;
  config.inertial << 0.07, 0, 0, 0, 0.26, 0, 0, 0, 0.242;
  config.weight_input = 1e-5;

  lweight_qp_unit_ << config.weight_rpy[0], config.weight_rpy[1],
      config.weight_rpy[2], config.weight_xyz[0], config.weight_xyz[1],
      config.weight_xyz[2], config.weight_omega[0], config.weight_omega[1],
      config.weight_omega[2], config.weight_vel[0], config.weight_vel[1],
      config.weight_vel[2], 0.0;
  kweight_qp_unit_ << config.weight_input, config.weight_input,
      config.weight_input;
  Lweight_qp_.diagonal() =
      lweight_qp_unit_.replicate(horizon_mpc_, 1); // state weight
  Kweight_qp_.diagonal() =
      kweight_qp_unit_.replicate(4 * horizon_mpc_, 1); // unput weight
  counter_ = 0;
  dt_mpc_ = config.dt * iteration_step_;
}
