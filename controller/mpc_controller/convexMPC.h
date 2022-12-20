
#ifndef CONVEXMPC
#define CONVEXMPC
#include "../common_data/common_data.h"
#include "math_utils.h"
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include <string>

#define BIG_FORCE 1000000
// #define DEBUG_MPC
struct MPC_CONFIG {
  float dt;
  float body_mass;
  float mu;
  float max_reaction_force;
  float body_height;
  float filter_omega, filter_vel;
  float weight_input;
  Eigen::Vector3f weight_rpy, weight_xyz, weight_omega, weight_vel;
  Eigen::Vector3f com_offset, p_hip;
  Eigen::Matrix<float, 3, 3> inertial;
};

class convexMPC {
public:
  convexMPC(int horizon_mpc, int iteration_step)
      : iteration_step_(iteration_step), horizon_mpc_(horizon_mpc){};
  void init();
  void run(const FusionData &fusion_data, const commandData &operator_data,
           int *mpcTable, float stance_duration);
  Eigen::Matrix<float, 12, 1> getFootForceInWorld() {
    return foot_force_in_world_;
  };
  Eigen::Matrix<float, 12, 1> getJointTorque() { return joint_torque_; };
  MPC_CONFIG config;

private:
  void update(const FusionData &fusion_data, const commandData &operator_data,
              int *mpcTable,
              float stance_duration); // change MPCInterface input inaccordence
                                      // with your own data
  void
  calculateContinuousEquation();  // calculate A and B in equation:
                                  // dotX=AX+BU,euler sequence is zyx [x,y,z]
  void discretizeStateEquation(); //
  void calculateAdiscrete();
  void calculateBdiscrete(Eigen::Matrix<float, 13, 12> &Bmat,
                          Eigen::Matrix<float, 12, 1> r_foot);
  void generateReferenceTrajectory();
  void FormulateQPform();
  void generateContraint_TN(); // joint torque limit
  void generateContraint();    // six constraints on each foot
  void solveMPC_TN();          // 5 constraints on each foot
  void solveMPC();

  float constrainFzmax(float phase);
  void updateFootPositionTable();
  void getHipPositionInWorld(Eigen::Vector3f &p, int leg);
  void updateLWeight(); // change Lweight from body frame to world frame. min
                        // (X'*Lweight*X +U'*Kweight*U)
  inline float getTorqueBorderFromTN(float omega);
  void testMPC();
  bool optimism_{false};
  float dt_mpc_;
  int iteration_step_;
  int horizon_mpc_;
  int counter_;
  int sum_contact_;
  float stance_duration_;
  Eigen::Matrix<float, 12, 1> foot_force_in_world_, joint_torque_, joint_vel_;
  Eigen::Matrix<float, 13, 13> Amat_continuous_;
  Eigen::Matrix<float, 13, 12> Bmat_continuous_;
  Eigen::Matrix<float, 13, 13> Amat_discrete_;
  Eigen::Matrix<float, 13, 12> Bmat_discrete_;

  Eigen::Matrix<float, 3, 3> Inertial_in_world_, inv_inertial_;
  Eigen::Matrix<float, -1, 13> A_mpc_;
  Eigen::Matrix<float, -1, -1> B_mpc_;
  Eigen::Matrix<float, -1, -1> Hessian_qp_, Gradient_qp_, Hessian_qp_tmp;
  Eigen::Matrix<float, -1, -1> Lweight_qp_, Kweight_qp_;
  Eigen::Matrix<float, 13, 1> lweight_qp_unit_;
  Eigen::Matrix<float, 3, 1> kweight_qp_unit_;
  Eigen::Matrix<float, 13, 1> x0_qp_;

  Eigen::Matrix<float, 3, 1> root_euler_target_, root_omega_target_,
      root_position_target_, root_velocity_target_;
  Eigen::Matrix<float, 3, 1> root_euler_, root_omega_, root_position_,
      root_velocity_;
  Eigen::Matrix<float, 12, 1> foot_position_in_world_;
  Eigen::Matrix<float, 3, 12> jacobian_leg_;
  Eigen::Matrix<float, -1, 1> traj_;
  Eigen::Matrix<float, 3, 3> rotm_body2world_;
  Eigen::Matrix3f mat_omega2rpy_rate_;
  Eigen::Matrix3f mat_omega2rpy_rate_simplified_;

  Eigen::Matrix<float, 4, -1> contact_table_;
  // Eigen::Matrix<float, 6, 3> constraint_unit_;
  Eigen::Matrix<float, -1, -1> constraint_mat_;
  Eigen::Matrix<float, -1, 1> constraint_ub_, constraint_lb_;
  Eigen::Matrix<float, -1, 1> constraint_lb_A_, constraint_ub_A_;
  // store foot postion in world,used for calculation od Bi
  Eigen::Matrix<float, 12, -1> foot_pos_table_;

  qpOASES::real_t *H_qpoases;
  qpOASES::real_t *g_qpoases;
  qpOASES::real_t *A_qpoases;
  qpOASES::real_t *lb_qpoases;
  qpOASES::real_t *ub_qpoases;
  qpOASES::real_t *q_soln;
  qpOASES::real_t *lb_A_qpoases;
  qpOASES::real_t *ub_A_qpoases;
  int real_allocated{0};
};

#endif
