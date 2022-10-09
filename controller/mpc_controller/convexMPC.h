
#ifndef CONVEXMPC
#define CONVEXMPC
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
class convexMPC{
    convexMPC(int horizon_mpc,float dt_mpc):dt_mpc_(dt_mpc),horizon_mpc_(horizon_mpc){foot_contact_table_.resize(4,horizon_mpc_);};
    void MPCInterface(Eigen::Matrix<float, 3, 3>Inertial, Eigen::Matrix<float, 3, 4>foot_position_in_world,float body_mass,float dt_mpc);
    // dotX=AX+BU
    void calculateContinuousEquation(Eigen::Vector3d root_euler);  //euler zyx [x,y,z]
    void discretizeStateEquation();
    void FormulateQPform();
    void generateReferenceTrajectory();
    void generateForceContriant(float mu);
private:
    float body_mass_;
    float dt_mpc_;
    int horizon_mpc_;
    float mu_;

    Eigen::Matrix<float, 13, 13> Amat_continuous_;
    Eigen::Matrix<float, 13, 12> Bmat_continuous_;
    Eigen::Matrix<float, 13, 13> Amat_discrete_;
    Eigen::Matrix<float, 13, 13> Bmat_discrete_;

    Eigen::Matrix<float, 3, 3>Inertial_;
    Eigen::Matrix<float,-1,13> A_qp_;
    Eigen::Matrix<float,-1,-1> B_qp_;
    Eigen::Matrix<float,3,1> root_euler_target_,root_omega_target_,root_position_target_,root_velocity_target_;
    Eigen::Matrix<float,3,1> root_euler_,root_omega_,root_position_,root_velocity_;
    Eigen::Matrix<float, 3, 4>foot_position_in_world_;
    Eigen::Matrix<float,-1,1> reference_trajectory_;
    Eigen::Matrix<float,3,3> rotm_body2world_;
    Eigen::Matrix3d mat_omega2rpy_rate_;
    Eigen::Matrix3d mat_omega2rpy_rate_simplified_;
    Eigen::Matrix<float, 4, -1>foot_contact_table_;

};

#endif