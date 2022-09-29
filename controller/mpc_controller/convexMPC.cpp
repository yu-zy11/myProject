#include"convexMPC.h"
#include "math_utils.h"

convexMPC::convexMPC(int horizon,float dt_mpc)
{
    dt_mpc_=dt_mpc;
    horizon_mpc_=horizon;
}
void convexMPC::MPCInterface(Eigen::Matrix<float, 3, 3>Inertial, Eigen::Matrix<float, 3, 4>foot_position_in_world,float body_mass,float dt_mpc)
{
    Inertial_=Inertial;
    foot_position_in_world_=foot_position_in_world;
    body_mass_=body_mass;
    mu_=0.5;
    // rotm_body2world=
    // root_euler_target,root_omega_target,root_position_target,root_velocity_target
    // root_euler,root_omega,root_position,root_velocity;

    // Eigen::Matrix3d mat_omega2rpy_rate,mat_omega2rpy_rate_simplified;
    double c3 = cos(root_euler_[2]);
    double s3 = sin(root_euler_[2]);
    double c2 = cos(root_euler_[1]);
    double s2 = sin(root_euler_[1]);
    mat_omega2rpy_rate_ << c3/c2,  s3 / c2,    0, 
                        -s3,     c3,         0, 
                     c3 * s2/c2, s3 * s2/c2, 1;
    mat_omega2rpy_rate_simplified_ << c3, s3, 0,-s3, c3, 0, 0, 0, 1; //simplified c2=1
}

void convexMPC::calculateContinuousEquation(Eigen::Vector3d root_euler)
{ 
    //Amatrix
    Amat_continuous_.setZero();
    // double c3 = cos(root_euler[2]);
    // double s3 = sin(root_euler[2]);
    // double c2 = cos(root_euler[1]);
    // double s2 = sin(root_euler[1]);

    // // Eigen::Matrix3d mat_omega2rpy_rate,mat_omega2rpy_rate_simplified;
    // mat_omega2rpy_rate << c3/c2,  s3 / c2,    0, 
    //                     -s3,     c3,         0, 
    //                  c3 * s2/c2, s3 * s2/c2, 1;

    // mat_omega2rpy_rate_simplified << c3, s3, 0,-s3, c3, 0, 0, 0, 1; //simplified c2=1
    Amat_continuous_.block<3, 3>(0, 6) = mat_omega2rpy_rate_;
    Amat_continuous_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    Amat_continuous_(11, 12) = 1;

    //Bmatrix
     Bmat_continuous_.setZero();
    for(int i = 0; i < 4; i++)
    {
        Eigen::Matrix<float,3,1> vec;
        vec=foot_position_in_world_.col(i);
        Bmat_continuous_.block(6,i*3,3,3) = Inertial_.inverse()*cross2mat(vec);
        Bmat_continuous_.block(9,i*3,3,3) = Eigen::Matrix3d::Identity() / body_mass_;
    }
}

void convexMPC::discretizeStateEquation()
{
  Eigen::Matrix<float,25,25>mat25;
  mat25.setZero();
  mat25.block(0,0,13,13) = Amat_continuous_;
  mat25.block(0,13,13,12) = Bmat_continuous_;
  mat25 = dt_mpc_*mat25;
  mat25 = mat25.exp();
  Amat_discrete_ = mat25.block(0,0,13,13);
  Bmat_discrete_ = mat25.block(0,13,13,12);
}
void convexMPC::FormulateQPform()
{
    // standard QP formulation
    // minimize 1/2 * x' * P * x + q' * x
    // subject to lb <= Ac * x <= ub
    // P: hessian
    // q: gradient
    // Ac: linear constraints

    // A_qp = [A,
    //         A^2,
    //         A^3,
    //         ...
    //         A^k]'

    // B_qp = [A^0*B(0),
    //         A^1*B(0),     B(1),
    //         A^2*B(0),     A*B(1),       B(2),
    //         ...
    //         A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]

    // auto t1 = std::chrono::high_resolution_clock::now();//
    // calculate A_qp and B_qp
    A_qp_.block(0,0,13,13)=Amat_discrete_;
    for(int r = 1; r < horizon_mpc_; r++)
    {
        A_qp_.block(13*r,0,13,13)=Amat_discrete_* A_qp_.block(13*(r-1),0,13,13);
    }
    //B_qp
    for (int row=0;row<horizon_mpc_;row++)
    {
        for(int col = 0; col <= row; col++)
        {
            if(col == row)
            {
                B_qp_.block(13*row,12*col,13,12) =Bmat_discrete_;
            }
            else
            {
                B_qp_.block(13*row,12*col,13,12) =A_qp_.block(13*(row-col-1),0,13,13)*Bmat_discrete_;
            }
        }
  }
}

void convexMPC::generateReferenceTrajectory()
{
    constexpr float max_pos_error{0.05};
    constexpr float max_rpy_error{0.1};
    float p_start;
    //avoid big difference betwween position and target position
    for (int i=0;i<3;i++)
    {
        p_start = root_position_target_[i];
        if (fabs(p_start - root_position_[i]) > max_pos_error)
            {p_start = root_position_[i] + (p_start - root_position_[i]) / fabs(p_start - root_position_[i]) * max_pos_error;}
        root_position_target_[i]=p_start;
        
        p_start=root_euler_target_[i];
        if(fabs(p_start-root_euler_[i])>max_rpy_error)
        {
            p_start=root_euler_[i]+(p_start-root_euler_[i])/fabs(p_start-root_euler_[i])*max_rpy_error;
        }
        root_euler_target_[i]=p_start;
    }

    reference_trajectory_.block(0,0,3,1)=root_euler_target_; //rpy xyz omega vel
    reference_trajectory_.block(3,0,3,1)=root_position_target_;
    reference_trajectory_.block(6,0,3,1)=root_omega_target_;
    reference_trajectory_.block(9,0,3,1)=root_velocity_target_;
    reference_trajectory_[12]=0;
    for (int i = 1; i < horizon_mpc_; i++)
    {
        reference_trajectory_.block(13*i,0,12,1)=reference_trajectory_.block(13*(i-1),0,12,1);
        reference_trajectory_.block(13*i,0,3,1)=reference_trajectory_.block(13*(i-1),0,3,1)+dt_mpc_*mat_omega2rpy_rate_*reference_trajectory_.block(13*(i-1)+6,0,3,1);
        reference_trajectory_.block(13*i+3,0,3,1)=reference_trajectory_.block(13*(i-1)+3,0,3,1)+dt_mpc_*reference_trajectory_.block(13*(i-1)+9,0,3,1);
        reference_trajectory_[13*i+12]=0;
    }
}