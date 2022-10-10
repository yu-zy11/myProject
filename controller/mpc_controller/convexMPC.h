
#ifndef CONVEXMPC
#define CONVEXMPC
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include "math_utils.h"
class convexMPC{
public:
    convexMPC(int horizon_mpc,float dt_mpc):dt_mpc_(dt_mpc),horizon_mpc_(horizon_mpc){};
    void MPCInterface(); //change MPCInterface input inaccordence with your own data
    void initConvexMPC();
    void calculateContinuousEquation();  //calculate A and B in continuous equation of motion: dotX=AX+BU,euler sequence is zyx [x,y,z]
    void discretizeStateEquation();      //
    void generateReferenceTrajectory();
    void generateContraint();
    void FormulateQPform();
    void solveMPC();
    void run();
    int test{1};
private:
    float body_mass_;
    float dt_mpc_;
    int horizon_mpc_;
    float mu_;

    Eigen::Matrix<float, 13, 13> Amat_continuous_;
    Eigen::Matrix<float, 13, 12> Bmat_continuous_;
    Eigen::Matrix<float, 13, 13> Amat_discrete_;
    Eigen::Matrix<float, 13, 12> Bmat_discrete_;

    Eigen::Matrix<float, 3, 3>Inertial_,Inertial_in_world_;
    Eigen::Matrix<float,-1,13> A_mpc_;
    Eigen::Matrix<float,-1,-1> B_mpc_;
    Eigen::Matrix<float,-1,-1>Hessian_qp_,Gradient_qp_;
    Eigen::Matrix<float,-1,-1>Lweight_qp_,Kweight_qp_;
    Eigen::Matrix<float,12,1>lweight_qp_unit_,kweight_qp_unit_;
    Eigen::Matrix<float,13,1>x0_qp_;

    Eigen::Matrix<float,3,1> root_euler_target_,root_omega_target_,root_position_target_,root_velocity_target_;
    Eigen::Matrix<float,3,1> root_euler_,root_omega_,root_position_,root_velocity_;
    Eigen::Matrix<float, 3, 4>foot_position_in_world_;
    Eigen::Matrix<float,-1,1> reference_trajectory_;
    Eigen::Matrix<float,3,3> rotm_body2world_;
    Eigen::Matrix3f mat_omega2rpy_rate_;
    Eigen::Matrix3f mat_omega2rpy_rate_simplified_;

    Eigen::Matrix<float, 4, -1>foot_contact_table_;
    Eigen::Matrix<float, 5, 3> constraint_unit_;
    Eigen::Matrix<float, -1, -1> constraint_mat_;
    Eigen::Matrix<float, -1, 1> constraint_ub_;

    qpOASES::real_t *H_qpoases;
    qpOASES::real_t *g_qpoases;
    qpOASES::real_t *A_qpoases;
    qpOASES::real_t *lb_qpoases;
    qpOASES::real_t *ub_qpoases;
    qpOASES::real_t *q_soln;

    // qpOASES::real_t *H_red;
    // qpOASES::real_t *g_red;
    // qpOASES::real_t *A_red;
    // qpOASES::real_t *lb_red;
    // qpOASES::real_t *ub_red;
    // qpOASES::real_t *q_red;

};

#endif