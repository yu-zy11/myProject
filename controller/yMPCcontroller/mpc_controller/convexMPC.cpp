#include "convexMPC.h"
#include "math_utils.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "modules/common/config_loader.h"
#include <qpOASES.hpp>
#include <iostream>
#include<ctime>
#include<thread>

void matrix_to_real(qpOASES::real_t *dst, Eigen::Matrix<float, -1, -1> src, int rows, int cols)
{
    int a = 0;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            dst[a] = src(r, c);
            a++;
        }
    }
}
// resize dimensions of matrix in accordence with horizon_mpc_,and set weights for QP problem
void convexMPC::init()
{
    contact_table_.resize(4, horizon_mpc_);
    Lweight_qp_.resize(13 * horizon_mpc_, 13 * horizon_mpc_);
    Kweight_qp_.resize(12 * horizon_mpc_, 12 * horizon_mpc_);
    reference_trajectory_.resize(13 * horizon_mpc_, 1);
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
    reference_trajectory_.setZero();
    B_mpc_.setZero();
    A_mpc_.setZero();
    Hessian_qp_.setZero();
    Gradient_qp_.setZero();
    Hessian_qp_tmp.setZero();
    constraint_ub_.setZero();
    constraint_lb_.setZero();

    auto parameters = ConfigLoaderInstance.getRobotConfig();
    lweight_qp_unit_<<(float)parameters->weight_rpy[0],
        (float)parameters->weight_rpy[1],
        (float)parameters->weight_rpy[2],
        (float)parameters->weight_pos[0],
        (float)parameters->weight_pos[1],
        (float)parameters->weight_pos[2],
        (float)parameters->weight_omega[0],
        (float)parameters->weight_omega[1],
        (float)parameters->weight_omega[2],
        (float)parameters->weight_vel[0],
        (float)parameters->weight_vel[1],
        (float)parameters->weight_vel[2];
    kweight_qp_unit_<<parameters->grf_weight,parameters->grf_weight,0.001*parameters->grf_weight;//weight of input U
    Lweight_qp_.diagonal() = lweight_qp_unit_.replicate(horizon_mpc_, 1);
    Kweight_qp_.diagonal() = kweight_qp_unit_.replicate(4*horizon_mpc_, 1) ;
    Inertial_<<parameters->I_body[0],0,0, 0,parameters->I_body[1],0,0,0,parameters->I_body[2];

    counter_=0;
    dt_=(float)ConfigLoaderInstance.getCommonConfig()->default_cycle_time_;
    dt_mpc_=dt_*iteration_step_;

    body_mass_=(float)parameters->mass;
    mu_=(float)parameters->mu;
    body_height_=(float)parameters->body_height;

    std::cout << "init MPC finished\n";
    #ifdef DEBUG_MPC
    std::cout<<"dt_mpc_:"<<dt_mpc_<<std::endl;
    #endif
}

void convexMPC::update(const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data,int* mpcTable)
{
    //update robot states for mpc
    rotm_body2world_=fusion_data.rotation_matrix_body_to_world;
    foot_position_in_world_=rotm_body2world_*fusion_data.foot_position_in_body.transpose();
    root_euler_ =fusion_data.root_euler;
    root_omega_=fusion_data.root_angular_velocity_in_world;
    root_position_=fusion_data.root_position;
    root_velocity_=fusion_data.root_linear_velocity_in_world;
    Inertial_in_world_ = rotm_body2world_ * Inertial_ * rotm_body2world_.transpose();
    mat_omega2rpy_rate_ = omega2rpyrate(root_euler_);
    for(int i=0;i<horizon_mpc_;i++)
    {
        for(int leg=0;leg<4;leg++)
        {
            contact_table_(leg,i)=mpcTable[4*i+leg];
        }
    }
    sum_contact_=contact_table_.sum();
    //update command for mpc
    root_euler_target_ =operator_data.root_euler;
    root_omega_target_ =rotm_body2world_*operator_data.root_angular_velocity_in_body;
    root_velocity_target_ =rotm_body2world_*operator_data.root_linear_velocity_in_body;
    root_position_target_=root_position_+0.5*(root_velocity_target_+root_velocity_)*dt_mpc_;
    root_position_target_[2]=body_height_;
}

// calculate A and B in time-continous dynamic equation of motion:\dot x=Ax+Bu
void convexMPC::calculateContinuousEquation()
{
    Amat_continuous_.setZero();
    Amat_continuous_.block<3, 3>(0, 6) = mat_omega2rpy_rate_;
    Amat_continuous_.block<3, 3>(3, 9) = Eigen::Matrix3f::Identity();
    Amat_continuous_(11, 12) = 1;

    Bmat_continuous_.setZero();
    for (int i = 0; i < 4; i++)
    {
        Eigen::Matrix<float, 3, 1> vec;
        vec = foot_position_in_world_.col(i);
        Bmat_continuous_.block(6, i * 3, 3, 3) = Inertial_in_world_.inverse() * cross2mat(vec);
        Bmat_continuous_.block(9, i * 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity() / body_mass_;
    }
#ifdef DEBUG_MPC
    // std::cout << "Amat_continuous_" << std::endl;
    // printMatrix(Amat_continuous_);
    // std::cout << "Bmat_continuous_" << std::endl;
    // printMatrix(Bmat_continuous_);
#endif
}

void convexMPC::discretizeStateEquation()
{
    Eigen::Matrix<float, 25, 25> mat25, expm;
    mat25.setZero();
    mat25.block(0, 0, 13, 13) = Amat_continuous_ * dt_mpc_;
    mat25.block(0, 13, 13, 12) = Bmat_continuous_ * dt_mpc_;
    expm = mat25.exp();
    Amat_discrete_ = expm.block(0, 0, 13, 13);
    Bmat_discrete_ = expm.block(0, 13, 13, 12);
#ifdef DEBUG_MPC
    std::cout << "Amat_discrete_" << std::endl;
    printMatrix(Amat_discrete_);
    std::cout << "Bmat_discrete_" << std::endl;
    printMatrix(Bmat_discrete_);
#endif
}


// calculate body's  reference trajectory for MPC
void convexMPC::generateReferenceTrajectory()
{
    constexpr float max_pos_error{0.2};
    constexpr float max_rpy_error{0.1};
    float p_start;
    // avoid big difference betwween position and target position
    for (int i = 0; i < 2; i++)
    {
        p_start = root_position_target_[i];
        if (fabs(p_start - root_position_[i]) > max_pos_error)
        {
            p_start = root_position_[i] + (p_start - root_position_[i]) / fabs(p_start - root_position_[i]) * max_pos_error;
        }
        root_position_target_[i] = p_start;
    }
    for (int i = 0; i < 3; i++)
    {
        p_start = root_euler_target_[i];
        if (fabs(p_start - root_euler_[i]) > max_rpy_error)
        {
            p_start = root_euler_[i] + (p_start - root_euler_[i]) / fabs(p_start - root_euler_[i]) * max_rpy_error;
        }
        root_euler_target_[i] = p_start;
    }
    std::cout<<"root_position_target_[i]"<<root_position_target_<<std::endl;

    reference_trajectory_.block(0, 0, 3, 1) = root_euler_target_; // rpy xyz omega vel
    reference_trajectory_.block(3, 0, 3, 1) = root_position_target_;
    reference_trajectory_.block(6, 0, 3, 1) = root_omega_target_;
    reference_trajectory_.block(9, 0, 3, 1) = root_velocity_target_;
    reference_trajectory_[12] = -9.81;
    for (int i = 1; i < horizon_mpc_; i++)
    {
        reference_trajectory_.block(13 * i, 0, 12, 1) = reference_trajectory_.block(13 * (i - 1), 0, 12, 1);
        reference_trajectory_.block(13 * i, 0, 3, 1) = reference_trajectory_.block(13 * (i - 1), 0, 3, 1) + dt_mpc_ * mat_omega2rpy_rate_ * reference_trajectory_.block(13 * (i - 1) + 6, 0, 3, 1);
        reference_trajectory_.block(13 * i + 3, 0, 3, 1) = reference_trajectory_.block(13 * (i - 1) + 3, 0, 3, 1) + dt_mpc_ * reference_trajectory_.block(13 * (i - 1) + 9, 0, 3, 1);
        reference_trajectory_[13 * i + 12] = -9.81;
    }
#ifdef DEBUG_MPC
    std::cout << "reference_trajectory_" << std::endl;
    printMatrix(reference_trajectory_);
#endif
}

Eigen::Matrix<float,12,1> convexMPC::getJointTorque(const MCC::common::message::FusionData& fusion_data)
{
    Eigen::Matrix<float, 3, 1>foot_foorce_tmp;
    Eigen::Matrix<float,12,1> joint_torque_target;
    for (int leg=0;leg<4;leg++)
    {
        foot_foorce_tmp = -fusion_data.rotation_matrix_body_to_world.transpose()* foot_force_in_world_.segment<3>(3 * leg);
        Eigen::Matrix3f foot_jacob = fusion_data.foot_jacobian_in_body.block<3, 3>(3 * leg, 3 * leg);
        joint_torque_target.segment<3>(3 * leg) = foot_jacob.transpose() * foot_foorce_tmp;
    }
    return joint_torque_target;
}


void convexMPC::FormulateQPform()
{
    A_mpc_.block(0, 0, 13, 13) = Amat_discrete_;
    for (int r = 1; r < horizon_mpc_; r++)
    {
        A_mpc_.block(13 * r, 0, 13, 13) = Amat_discrete_ * A_mpc_.block(13 * (r - 1), 0, 13, 13);
    }
    // B_mpc
    B_mpc_.resize(13 * horizon_mpc_, 3 * sum_contact_);
    B_mpc_.setZero();
    int col_B_mpc{0};
    Eigen::Matrix<float,13,-1>Bmat_tmp;
    for (int col = 0; col < horizon_mpc_; col++)
    {
        int num_contacti=contact_table_.block(0,col,4,1).sum();
        if(num_contacti==0) continue;
        Bmat_tmp.resize(13,3*num_contacti);
        Bmat_tmp.setZero();
        col_B_mpc+=3*num_contacti;
        int a{0};
        for (int leg=0;leg<4;leg++)
        {
            if(contact_table_(leg,col)==1)
            {
                Bmat_tmp.block(0,3*a,13,3)=Bmat_discrete_.block(0,3*leg,13,3);
                a++;
            }
        }

        for (int row = col; row <horizon_mpc_; row++)
        {
            if (col == row)
            {
                B_mpc_.block(13 * row, col_B_mpc-3*num_contacti, 13, 3*num_contacti) = Bmat_tmp;
            }
            else
            {
                B_mpc_.block(13 * row, col_B_mpc-3*num_contacti, 13, 3*num_contacti) = A_mpc_.block(13 * (row - col - 1), 0, 13, 13) * Bmat_tmp;
            }
        }
    }
    x0_qp_.block(0, 0, 3, 1) = root_euler_; // rpy xyz omega vel
    x0_qp_.block(3, 0, 3, 1) = root_position_;
    x0_qp_.block(6, 0, 3, 1) = root_omega_;
    x0_qp_.block(9, 0, 3, 1) = root_velocity_;
    x0_qp_[12] = -9.81;

    Kweight_qp_.resize(3*sum_contact_,3*sum_contact_);
    Kweight_qp_.setZero();
    Kweight_qp_.diagonal() = kweight_qp_unit_.replicate(sum_contact_, 1) ;
    Hessian_qp_tmp = 2*B_mpc_.transpose() * Lweight_qp_;
    Gradient_qp_ =  Hessian_qp_tmp * (A_mpc_ * x0_qp_ - reference_trajectory_);
    Hessian_qp_ = (Hessian_qp_tmp * B_mpc_ + 2*Kweight_qp_);
#ifdef DEBUG_MPC
    std::cout << "Lweight_qp_" << std::endl;
    printMatrix(Lweight_qp_);
    std::cout << "Kweight_qp_" << std::endl;
    printMatrix(Kweight_qp_);

    std::cout << "A_mpc_" << std::endl;
    printMatrix(A_mpc_);
    std::cout << "B_mpc_" << std::endl;
    printMatrix(B_mpc_);
    std::cout << "Gradient_qp_" << std::endl;
    printMatrix(Gradient_qp_);
    std::cout << "Hessian_qp_" << std::endl;
    printMatrix(Hessian_qp_);
#endif
}

void convexMPC::generateContraint()
{
    assert(mu_ >= 0.0001 && "mu_ must not be 0!");
    float mu_inv = 1.f / mu_;
    constraint_unit_ << mu_inv, 0, 1.f, //constraint_unit_*U>=0
            -mu_inv, 0, 1.f,
             0, mu_inv, 1.f,
            0, -mu_inv, 1.f,
            0, 0, 1.f;
    Eigen::Matrix<float,5,1> lbA_unit,ubA_unit;
    Eigen::Matrix<float,3,1> lb_unit,ub_unit;
    lbA_unit<<0,0,0,0,0;
    ubA_unit<<2*BIG_FORCE,2*BIG_FORCE,2*BIG_FORCE,2*BIG_FORCE,BIG_FORCE;
    lb_unit<<-BIG_FORCE,-BIG_FORCE,0;
    ub_unit<<BIG_FORCE,BIG_FORCE,BIG_FORCE;
    int cols=constraint_unit_.cols();
    int rows=constraint_unit_.rows();
    constraint_mat_.resize(rows * sum_contact_, cols * sum_contact_);
    constraint_lb_A_.resize(rows * sum_contact_, 1);
    constraint_ub_A_.resize(rows * sum_contact_, 1);
    constraint_ub_.resize(3*sum_contact_,1);
    constraint_lb_.resize(3*sum_contact_,1);

    constraint_mat_.setZero();
    constraint_lb_A_.setZero();
    constraint_ub_A_.setZero();
    constraint_ub_.setZero();
    constraint_lb_.setZero();

    for (int i = 0; i < sum_contact_; i++)
    {
        constraint_mat_.block(i  * rows, i * cols, rows, cols) = constraint_unit_;
        constraint_lb_A_.block(i*rows,0,rows,1)=lbA_unit;
        constraint_ub_A_.block(i*rows,0,rows,1)=ubA_unit;
        constraint_ub_.block(i*3,0,3,1)=ub_unit;
        constraint_lb_.block(i*3,0,3,1)=lb_unit;
    }
#ifdef DEBUG_MPC
    std::cout << "constraint_mat_" << std::endl;
    printMatrix(constraint_mat_);
    std::cout << "constraint_lb_A_" << std::endl;
    printMatrix(constraint_lb_A_);
    std::cout << "constraint_ub_" << std::endl;
#endif
}


void convexMPC::solveMPC()
{
    if (real_allocated)
    {
        free(H_qpoases);
        free(g_qpoases);
        free(A_qpoases);
        free(lb_A_qpoases);
        free(ub_A_qpoases);
        free(ub_qpoases);
        free(lb_qpoases);
        free(q_soln);
    }
    int col_A{3};
    int row_A{5};
    H_qpoases = (qpOASES::real_t *)malloc(3* sum_contact_ * 3* sum_contact_ * sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t *)malloc(3* sum_contact_ * sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t *)malloc(row_A* sum_contact_ *col_A* sum_contact_* sizeof(qpOASES::real_t));
    lb_A_qpoases = (qpOASES::real_t *)malloc(row_A*sum_contact_  * sizeof(qpOASES::real_t));
    ub_A_qpoases = (qpOASES::real_t *)malloc(row_A*sum_contact_  * sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t *)malloc(3*sum_contact_  * sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t *)malloc(3*sum_contact_  * sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t *)malloc(3*sum_contact_  * sizeof(qpOASES::real_t));
    real_allocated = 1;

    matrix_to_real(H_qpoases, Hessian_qp_, 3* sum_contact_, 3* sum_contact_);
    matrix_to_real(g_qpoases, Gradient_qp_, 3* sum_contact_, 1);
    matrix_to_real(A_qpoases, constraint_mat_, row_A* sum_contact_, col_A* sum_contact_);
    matrix_to_real(lb_A_qpoases, constraint_lb_A_, row_A* sum_contact_, 1);
    matrix_to_real(ub_A_qpoases, constraint_ub_A_, row_A* sum_contact_, 1);
    matrix_to_real(lb_qpoases, constraint_lb_, 3* sum_contact_, 1);
    matrix_to_real(ub_qpoases, constraint_ub_, 3* sum_contact_, 1);

    qpOASES::int_t nV, nC;
    nV = 3* sum_contact_;
    nC = row_A* sum_contact_;
    qpOASES::QProblem problem_sqp(nV, nC);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_sqp.setOptions(op);
    qpOASES::int_t nWSR = 100;
    int rval = problem_sqp.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_A_qpoases, NULL, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to init!\n");
    int rval2 = problem_sqp.getPrimalSolution(q_soln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

    int tmp{0};
    for (int i = 0; i < 4; i++)
    {
        if(contact_table_(i,0)!=0)
        {
            foot_force_in_world_[3*i] = q_soln[3*tmp];
            foot_force_in_world_[3*i+1] = q_soln[3*tmp+1];
            foot_force_in_world_[3*i+2] = q_soln[3*tmp+2];
            tmp++;
        }
        else
        {
            foot_force_in_world_[3*i] = 0.0;
            foot_force_in_world_[3*i+1] =  0.0;
            foot_force_in_world_[3*i+2] =  0.0;
        }

    }
}


void convexMPC::run(const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data,int* mpcTable)
{
    // std::cout << "run mpc\n";
    time_t begin,end;
    begin =clock();
    update(fusion_data,operator_data,mpcTable);
if (counter_%4==0)
{
    std::cout<<"command velocity:"<<root_velocity_target_[0]<<std::endl;
    std::cout<<"current velocity:"<<root_velocity_[0]<<std::endl;
    begin =clock();

    calculateContinuousEquation();
    discretizeStateEquation();
    generateReferenceTrajectory();
    generateContraint();
    FormulateQPform();

    end=clock();
    std::cout<<"FormulateQPform time0:"<<(end-begin) <<std::endl;

    begin =clock();
    solveMPC();
    end=clock();
    std::cout<<"solving mpc time:"<<(end-begin) <<std::endl;
}
counter_++;

}


void convexMPC::printMatrix(Eigen::Matrix<float, -1, -1> mat)
{
    std::cout << mat << std::endl;
}
