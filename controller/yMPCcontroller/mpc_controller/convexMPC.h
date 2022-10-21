
#ifndef CONVEXMPC
#define CONVEXMPC
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include "math_utils.h"
#include<string>
#include "modules/common/message/sim_msg/FusionData.hpp"
#include "modules/common/message/sim_msg/OperatorData.hpp"
#include "modules/common/config_loader.h"
#include "modules/common/message/common_data.h"
#include "modules/locomotion/locomotion_core/gait/time_gait_generator.h"

#define BIG_FORCE 320
// #define DEBUG_MPC
class convexMPC{
public:
    convexMPC(int horizon_mpc,int iteration_step):iteration_step_(iteration_step),horizon_mpc_(horizon_mpc){};
    void init();
    void run(const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data,int* mpcTable);
    Eigen::Matrix<float,12,1>  getFootForceInWorld(){return foot_force_in_world_;};
    Eigen::Matrix<float,12,1> getJointTorque(const MCC::common::message::FusionData& fusion_data);
    const float dt_{0.002};
private:
    void updateMPC(const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data,int* mpcTable); //change MPCInterface input inaccordence with your own data
    void calculateContinuousEquation();  //calculate A and B in continuous equation of motion: dotX=AX+BU,euler sequence is zyx [x,y,z]
    void discretizeStateEquation();      //
    void generateReferenceTrajectory();
    void generateContraint();
    void FormulateQPform();
    void FormulateQPformV2();
    void generateContraintV2();
    void solveMPC();
    void solveMPCV2();
    void printMatrix(Eigen::Matrix<float,-1,-1> mat);
    float body_mass_;
    float dt_mpc_;
    int iteration_step_;
    int horizon_mpc_;
    float mu_;
    int counter_;
    int num_contact_foot_;
    // const float dt_{0.002};
    Eigen::Matrix<float,12,1>  foot_force_in_world_;
    Eigen::Matrix<float, 13, 13> Amat_continuous_;
    Eigen::Matrix<float, 13, 12> Bmat_continuous_;
    Eigen::Matrix<float, 13, 13> Amat_discrete_;
    Eigen::Matrix<float, 13, 12> Bmat_discrete_;

    Eigen::Matrix<float, 3, 3>Inertial_,Inertial_in_world_;
    Eigen::Matrix<float,-1,13> A_mpc_;
    Eigen::Matrix<float,-1,-1> B_mpc_;
    Eigen::Matrix<float,-1,-1>Hessian_qp_,Gradient_qp_,Hessian_qp_tmp;
    Eigen::Matrix<float,-1,-1>Lweight_qp_,Kweight_qp_;
    Eigen::Matrix<float,13,1>lweight_qp_unit_;
    Eigen::Matrix<float,12,1>kweight_qp_unit_;
    Eigen::Matrix<float,13,1>x0_qp_;

    Eigen::Matrix<float,3,1> root_euler_target_,root_omega_target_,root_position_target_,root_velocity_target_;
    Eigen::Matrix<float,3,1> root_euler_,root_omega_,root_position_,root_velocity_;
    Eigen::Matrix<float, 3, 4>foot_position_in_world_;
    Eigen::Matrix<float,-1,1> reference_trajectory_;
    Eigen::Matrix<float,3,3> rotm_body2world_;
    Eigen::Matrix3f mat_omega2rpy_rate_;
    Eigen::Matrix3f mat_omega2rpy_rate_simplified_;

    Eigen::Matrix<float, 4, -1>foot_contact_table_;
    Eigen::Matrix<float, 6, 3> constraint_unit_;
    Eigen::Matrix<float, -1, -1> constraint_mat_;
    Eigen::Matrix<float, -1, 1> constraint_ub_,constraint_lb_;
    Eigen::Matrix<float, -1, 1>constraint_lb_A_,constraint_ub_A_;

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
