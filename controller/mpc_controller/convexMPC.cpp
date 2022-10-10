#include"convexMPC.h"
#include "math_utils.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <iostream>

//resize dimensions of matrix in accordence with horizon_mpc_,and set weights for QP problem
void convexMPC::initConvexMPC(){
    foot_contact_table_.resize(4,horizon_mpc_);
    //weight for qp problem;
    Lweight_qp_.resize(12*horizon_mpc_,12*horizon_mpc_);
    Lweight_qp_.setZero();
    Kweight_qp_.resize(12*horizon_mpc_,12*horizon_mpc_);
    Kweight_qp_.setZero();
    lweight_qp_unit_<<0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
    kweight_qp_unit_<<0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Lweight_qp_.diagonal()=lweight_qp_unit_.replicate(horizon_mpc_,1);
    Kweight_qp_.diagonal()=kweight_qp_unit_.replicate(horizon_mpc_,1);
    //
    reference_trajectory_.resize(13*horizon_mpc_,1);



    std::cout<<"init MPC finished\n";
}

void convexMPC::MPCInterface()
{
    Inertial_.setIdentity();
    foot_position_in_world_.setOnes();
    body_mass_=10;
    mu_=0.5;
    root_euler_<<0,0,0;

    //
    rotm_body2world_=euler2rotm(root_euler_);
    Inertial_in_world_=rotm_body2world_*Inertial_*rotm_body2world_.transpose();
    std::cout<<"test euler2rotm:"<<rotm_body2world_<<std::endl;
    //root_euler_target=,
    //root_omega_target=,
    //root_position_target=,
    //root_velocity_target=
    // root_euler=,
    //root_omega=,
    //root_position=,
    //root_velocity=;

    // Eigen::Matrix3d mat_omega2rpy_rate,mat_omega2rpy_rate_simplified;
    // double c3 = cos(root_euler_[2]);
    // double s3 = sin(root_euler_[2]);
    // double c2 = cos(root_euler_[1]);
    // double s2 = sin(root_euler_[1]);
    // mat_omega2rpy_rate_ << c3/c2,  s3 / c2,    0, 
    //                     -s3,     c3,         0, 
    //                  c3 * s2/c2, s3 * s2/c2, 1;
    mat_omega2rpy_rate_= omega2rpyrate(root_euler_);  //euler zyx
    Eigen::Vector3f root_euler_simplified;
    root_euler_simplified<<0.0,0.0,root_euler_[2];
    // mat_omega2rpy_rate_simplified_ << c3, s3, 0,-s3, c3, 0, 0, 0, 1; //simplified c2=1
    mat_omega2rpy_rate_simplified_=omega2rpyrate(root_euler_simplified);
}

void convexMPC::calculateContinuousEquation()
{ 
    //Amatrix
    Amat_continuous_.setZero();
    Amat_continuous_.block<3, 3>(0, 6) = mat_omega2rpy_rate_;
    Amat_continuous_.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();
    Amat_continuous_(11, 12) = 1;

    //Bmatrix
     Bmat_continuous_.setZero();
    for(int i = 0; i < 4; i++)
    {
        Eigen::Matrix<float,3,1> vec;
        vec=foot_position_in_world_.col(i);
        Bmat_continuous_.block(6,i*3,3,3) = Inertial_in_world_.inverse()*cross2mat(vec);
        Bmat_continuous_.block(9,i*3,3,3) = Eigen::Matrix<float,3,3>::Identity() / body_mass_;
    }
}

void convexMPC::discretizeStateEquation()
{
  Eigen::Matrix<float,25,25>mat25,expm;
  mat25.setZero();
  mat25.block(0,0,13,13) = Amat_continuous_*dt_mpc_;
  mat25.block(0,13,13,12) = Bmat_continuous_*dt_mpc_;
  expm = mat25.exp();
  Amat_discrete_ = expm.block(0,0,13,13);
  Bmat_discrete_ = expm.block(0,13,13,12);
}
void convexMPC::FormulateQPform()
{    // Ac: linear constraints

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
    A_mpc_.block(0,0,13,13)=Amat_discrete_;
    for(int r = 1; r < horizon_mpc_; r++)
    {
        A_mpc_.block(13*r,0,13,13)=Amat_discrete_* A_mpc_.block(13*(r-1),0,13,13);
    }
    //B_qp
    for (int row=0;row<horizon_mpc_;row++)
    {
        for(int col = 0; col <= row; col++)
        {
            if(col == row)
            {
                B_mpc_.block(13*row,12*col,13,12) =Bmat_discrete_;
            }
            else
            {
                B_mpc_.block(13*row,12*col,13,12) =A_mpc_.block(13*(row-col-1),0,13,13)*Bmat_discrete_;
            }
        }
  }
  x0_qp_.block(0, 0, 3, 1) = root_euler_target_; // rpy xyz omega vel
  x0_qp_.block(3, 0, 3, 1) = root_position_target_;
  x0_qp_.block(6, 0, 3, 1) = root_omega_target_;
  x0_qp_.block(9, 0, 3, 1) = root_velocity_target_;
  x0_qp_[12] = -9.81;
  Hessian_qp_=B_mpc_.transpose()*Lweight_qp_;
  Gradient_qp_=2*Hessian_qp_*(A_mpc_*x0_qp_-reference_trajectory_);
  Hessian_qp_=2*(Hessian_qp_*B_mpc_+Kweight_qp_);
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
    reference_trajectory_[12]=-9.81;
    for (int i = 1; i < horizon_mpc_; i++)
    {
        reference_trajectory_.block(13*i,0,12,1)=reference_trajectory_.block(13*(i-1),0,12,1);
        reference_trajectory_.block(13*i,0,3,1)=reference_trajectory_.block(13*(i-1),0,3,1)+dt_mpc_*mat_omega2rpy_rate_*reference_trajectory_.block(13*(i-1)+6,0,3,1);
        reference_trajectory_.block(13*i+3,0,3,1)=reference_trajectory_.block(13*(i-1)+3,0,3,1)+dt_mpc_*reference_trajectory_.block(13*(i-1)+9,0,3,1);
        reference_trajectory_[13*i+12]=-9.81;
    }
}

void convexMPC::generateContraint()
{
    assert(mu_ >= 0.0001 && "mu_ must not be 0!");
    float mu_inv = 1.f / mu_;
    constraint_unit_ << mu_inv, 0, 1.f,
        -mu_inv, 0, 1.f,
        0, mu_inv, 1.f,
        0, -mu_inv, 1.f,
        0, 0, 1.f;
    constraint_mat_.resize(5*4*horizon_mpc_,3*4*horizon_mpc_);
    for (int i = 0; i < horizon_mpc_; i++)
    {
        for (int leg=0;leg<4;leg++)
        {
           constraint_mat_.block(i * 20+leg*5, i * 12+leg*3, 5, 3) = constraint_unit_;
        }
    }
}
void convexMPC::run()
{
    std::cout<<"run mpc\n";
    MPCInterface();
    calculateContinuousEquation();
    discretizeStateEquation();
    generateReferenceTrajectory();
    generateContraint();
    solveMPC();
}
void convexMPC::solveMPC()
{
    using real_t =qpOASES::real_t;
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t H_new[2*2] = { 1.0, 0.5, 0.5, 0.5 };
	real_t A_new[1*2] = { 1.0, 5.0 };
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up SQProblem object. */
	qpOASES::SQProblem example( 2,1 );

	/* Solve first QP. */
	qpOASES::int_t nWSR = 10;
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    example.setOptions(op);
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0 );

	/* Get and print solution of second QP. */
	real_t xOpt[2];
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );
	std::cout<<"run qpoases finished\n";
}