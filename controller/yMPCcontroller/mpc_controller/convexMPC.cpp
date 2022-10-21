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
    foot_contact_table_.resize(4, horizon_mpc_);
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

    foot_contact_table_.setZero();
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
    // lweight_qp_unit_ << 10, 50, 10, 10, 10, 100, 1, 1, 1, 1, 1, 1, 0;
    kweight_qp_unit_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Lweight_qp_.diagonal() = lweight_qp_unit_.replicate(horizon_mpc_, 1);
    Kweight_qp_.diagonal() = kweight_qp_unit_.replicate(horizon_mpc_, 1) * 0.1;
    Inertial_<<1.2,0,0, 0,1.7,0, 0,0,1;
    counter_=0;

    dt_mpc_=dt_*iteration_step_;
    std::cout << "init MPC finished\n";
    #ifdef DEBUG_MPC
    std::cout<<"dt_mpc_:"<<dt_mpc_<<std::endl;
    #endif
}

// void convexMPC::MPCInterface( const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data)
void convexMPC::updateMPC(const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data,int* mpcTable)
{
    foot_position_in_world_=fusion_data.rotation_matrix_body_to_world*fusion_data.foot_position_in_body.transpose();
    body_mass_ = 50.0;
    mu_ = 0.5;
    root_euler_ =fusion_data.root_euler;
    root_omega_=fusion_data.root_angular_velocity_in_world;
    root_position_=fusion_data.root_position;
    root_velocity_=fusion_data.root_linear_velocity_in_world;
    rotm_body2world_ = euler2rotm(root_euler_);
    Inertial_in_world_ = rotm_body2world_ * Inertial_ * rotm_body2world_.transpose();
    root_euler_target_ =operator_data.root_euler;
    root_omega_target_ =rotm_body2world_*operator_data.root_angular_velocity_in_body;
    // root_position_target_ << 0, 0, 0.4;
    root_velocity_target_ =rotm_body2world_*operator_data.root_linear_velocity_in_body;
    mat_omega2rpy_rate_ = omega2rpyrate(root_euler_);
    for(int i=0;i<horizon_mpc_;i++)
    {
        for(int leg=0;leg<4;leg++)
        {
            foot_contact_table_(leg,i)=mpcTable[4*i+leg];
        }
    }
    num_contact_foot_=foot_contact_table_.sum();
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



void convexMPC::FormulateQPform()
{
    // Ac: linear constraints

    // A_mpc = [A,
    //         A^2,
    //         A^3,
    //         ...
    //         A^k]'

    // B_mpc = [A^0*B(0),
    //         A^1*B(0),     B(1),
    //         A^2*B(0),     A*B(1),       B(2),
    //         ...
    //         A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]
    // calculate A_mpc and B_mpc
    A_mpc_.block(0, 0, 13, 13) = Amat_discrete_;
    for (int r = 1; r < horizon_mpc_; r++)
    {
        A_mpc_.block(13 * r, 0, 13, 13) = Amat_discrete_ * A_mpc_.block(13 * (r - 1), 0, 13, 13);
    }
    // B_mpc
    for (int row = 0; row < horizon_mpc_; row++)
    {
        for (int col = 0; col <= row; col++)
        {
            if (col == row)
            {
                B_mpc_.block(13 * row, 12 * col, 13, 12) = Bmat_discrete_;
            }
            else
            {
                B_mpc_.block(13 * row, 12 * col, 13, 12) = A_mpc_.block(13 * (row - col - 1), 0, 13, 13) * Bmat_discrete_;
            }
        }
    }
    x0_qp_.block(0, 0, 3, 1) = root_euler_; // rpy xyz omega vel
    x0_qp_.block(3, 0, 3, 1) = root_position_;
    x0_qp_.block(6, 0, 3, 1) = root_omega_;
    x0_qp_.block(9, 0, 3, 1) = root_velocity_;
    x0_qp_[12] = -9.81;

    Hessian_qp_tmp = B_mpc_.transpose() * Lweight_qp_;
    Gradient_qp_ = 2 * Hessian_qp_tmp * (A_mpc_ * x0_qp_ - reference_trajectory_);
    Hessian_qp_ = 2 * (Hessian_qp_tmp * B_mpc_ + Kweight_qp_);
#ifdef DEBUG_MPC
    std::cout << "foot_contact_table_" << std::endl;
    printMatrix(foot_contact_table_);
    std::cout<<"num_contact_foot_:"<<num_contact_foot_<<std::endl;
    std::cout<<"foot_position_in_world_:"<<std::endl;
    printMatrix(foot_position_in_world_);
    std::cout << "Lweight_qp_0" << std::endl;
    printMatrix(Lweight_qp_);
    std::cout << "Kweight_qp_0" << std::endl;
    printMatrix(Kweight_qp_);

    std::cout << "A_mpc_0" << std::endl;
    printMatrix(A_mpc_);
    std::cout << "B_mpc_0" << std::endl;
    printMatrix(B_mpc_);
    std::cout << "Gradient_qp_0" << std::endl;
    printMatrix(Gradient_qp_);
    std::cout << "Hessian_qp_0" << std::endl;
    printMatrix(Hessian_qp_);
#endif
}

// calculate body's  reference trajectory for MPC
void convexMPC::generateReferenceTrajectory()
{
    constexpr float max_pos_error{0.1};
    constexpr float max_rpy_error{0.1};
    float p_start;
    // avoid big difference betwween position and target position
    for (int i = 0; i < 3; i++)
    {
        p_start = root_position_target_[i];
        if (fabs(p_start - root_position_[i]) > max_pos_error)
        {
            p_start = root_position_[i] + (p_start - root_position_[i]) / fabs(p_start - root_position_[i]) * max_pos_error;
        }
        root_position_target_[i] = p_start;

        p_start = root_euler_target_[i];
        if (fabs(p_start - root_euler_[i]) > max_rpy_error)
        {
            p_start = root_euler_[i] + (p_start - root_euler_[i]) / fabs(p_start - root_euler_[i]) * max_rpy_error;
        }
        root_euler_target_[i] = p_start;
    }

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

void convexMPC::generateContraint()
{
    assert(mu_ >= 0.0001 && "mu_ must not be 0!");
    float mu_inv = 1.f / mu_;
    constraint_unit_ << mu_inv, 0, 1.f,
        -mu_inv, 0, 1.f,
        0, mu_inv, 1.f,
        0, -mu_inv, 1.f,
        0, 0, 1.f;
    int cols = constraint_unit_.cols();
    int rows = constraint_unit_.rows();
    constraint_mat_.resize(rows * 4 * horizon_mpc_, cols * 4 * horizon_mpc_);
    constraint_lb_A_.resize(rows * 4 * horizon_mpc_, 1);
    constraint_lb_A_.setZero();
    for (int i = 0; i < horizon_mpc_; i++)
    {
        for (int leg = 0; leg < 4; leg++)
        {
            constraint_mat_.block((i * 4 + leg) * rows, (i * 4 + leg) * cols, rows, cols) = constraint_unit_;
            Eigen::Vector3f ub_unit,lb_unit;
            if(foot_contact_table_(leg, i) == 0)
            {
                ub_unit << 0.0, 0.0, 0.0;
                lb_unit<< 0.0, 0.0, 0.0;
            }
            else
            {
                ub_unit << mu_*BIG_FORCE, mu_*BIG_FORCE, BIG_FORCE;
                lb_unit<< -mu_*BIG_FORCE, -mu_*BIG_FORCE, 0.0;
            }
            constraint_ub_.block(12 * i + 3 * leg, 0, 3, 1) = ub_unit;
            constraint_lb_.block(12 * i + 3 * leg, 0, 3, 1) = lb_unit;
        }
    }
#ifdef DEBUG_MPC
    std::cout << "constraint_mat_" << std::endl;
    printMatrix(constraint_mat_);
    std::cout << "constraint_lb_A_" << std::endl;
    printMatrix(constraint_lb_A_);
    std::cout << "constraint_ub_" << std::endl;
    printMatrix(constraint_ub_);
#endif
}
void convexMPC::run(const MCC::common::message::FusionData& fusion_data, const MCC::common::message::OperatorData& operator_data,int* mpcTable)
{
    // std::cout << "run mpc\n";
    time_t begin,end;
    begin =clock();
    updateMPC(fusion_data,operator_data,mpcTable);
if (counter_%iteration_step_==0)
{
    calculateContinuousEquation();
    begin =clock();
    discretizeStateEquation();
    // end=clock();
    // std::cout<<"discretizeStateEquation mpc time0:"<<(end-begin) <<std::endl;
    generateReferenceTrajectory();
    // generateContraint();
    // FormulateQPform();

    generateContraintV2();
    FormulateQPformV2();
    end=clock();
    std::cout<<"solving mpc time0:"<<(end-begin) <<std::endl;

    begin =clock();
    // solveMPC();
    solveMPCV2();
    end=clock();
    std::cout<<"solving mpc time:"<<(end-begin) <<std::endl;
}
counter_++;

}
void convexMPC::solveMPC()
{
    if (real_allocated)
    {
        free(H_qpoases);
        free(g_qpoases);
        free(A_qpoases);
        free(lb_A_qpoases);
        free(ub_qpoases);
        free(lb_qpoases);
        free(q_soln);
    }
    H_qpoases = (qpOASES::real_t *)malloc(12 * 12 * horizon_mpc_ * horizon_mpc_ * sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t *)malloc(12 * 1 * horizon_mpc_ * sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t *)malloc(12 * 20 * horizon_mpc_ * horizon_mpc_ * sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t *)malloc(20 * 1 * horizon_mpc_ * sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t *)malloc(12 * 1 * horizon_mpc_ * sizeof(qpOASES::real_t));
    lb_A_qpoases = (qpOASES::real_t *)malloc(20 * 1 * horizon_mpc_ * sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t *)malloc(12 * horizon_mpc_ * sizeof(qpOASES::real_t));

    matrix_to_real(H_qpoases, Hessian_qp_, horizon_mpc_ * 12, horizon_mpc_ * 12);
    matrix_to_real(g_qpoases, Gradient_qp_, horizon_mpc_ * 12, 1);
    matrix_to_real(A_qpoases, constraint_mat_, horizon_mpc_ * 20, horizon_mpc_ * 12);
    matrix_to_real(ub_qpoases, constraint_ub_, horizon_mpc_ * 12, 1);
    matrix_to_real(lb_qpoases, constraint_lb_, horizon_mpc_ * 12, 1);
    matrix_to_real(lb_A_qpoases, constraint_lb_A_, horizon_mpc_ * 20, 1);
#ifdef DEBUG_MPC
    std::cout << "print H_qpoases" << std::endl;
    for (int i = 0; i < 12 * horizon_mpc_; i++)
    {
        for (int j = 0; j < 12 * horizon_mpc_; j++)
        {
            std::cout << H_qpoases[12 * horizon_mpc_ * i + j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "print g_qpoases" << std::endl;
    for (int i = 0; i < 12 * horizon_mpc_; i++)
    {
        std::cout << g_qpoases[i] << " ";
        std::cout << std::endl;
    }
#endif
    real_allocated = 1;
    qpOASES::int_t nV, nC;
    nV = horizon_mpc_ * 12;
    nC = 20 * horizon_mpc_;
    qpOASES::QProblem problem_sqp(nV, nC);
    qpOASES::Options op;
      op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_sqp.setOptions(op);

    qpOASES::int_t nWSR = 100;
    int rval = problem_sqp.init(H_qpoases, g_qpoases, A_qpoases, lb_qpoases, ub_qpoases, lb_A_qpoases, NULL, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to init!\n");
    int rval2 = problem_sqp.getPrimalSolution(q_soln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

    for (int i = 0; i < 12; i++)
    {
        foot_force_in_world_[i] = q_soln[i];
    }
    std::cout <<"yMPC result:";
    for (int i = 0; i < 12; i++)
    {
        std::cout << q_soln[i] << std::endl;
    }
    std::cout << "run yMPC qpoases finished\n";
}

void convexMPC::printMatrix(Eigen::Matrix<float, -1, -1> mat)
{
    std::cout << mat << std::endl;
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


void convexMPC::FormulateQPformV2()
{
    A_mpc_.block(0, 0, 13, 13) = Amat_discrete_;
    for (int r = 1; r < horizon_mpc_; r++)
    {
        A_mpc_.block(13 * r, 0, 13, 13) = Amat_discrete_ * A_mpc_.block(13 * (r - 1), 0, 13, 13);
    }
    // B_mpc
    int col_B_mpc{0};
    B_mpc_.resize(13 * horizon_mpc_, 3 * num_contact_foot_);
    B_mpc_.setZero();
    for (int col = 0; col < horizon_mpc_; col++)
    {
        Eigen::Matrix<float,13,-1>Bmat_tmp;
        int num_contact=foot_contact_table_.block(0,col,4,1).sum();
        if(num_contact==0) continue;
        Bmat_tmp.resize(13,3*num_contact);
        col_B_mpc+=3*num_contact;
        int a{0};
        for (int leg=0;leg<4;leg++)
        {
            if(foot_contact_table_(leg,col)==1)
            {
                Bmat_tmp.block(0,3*a,13,3)=Bmat_discrete_.block(0,3*leg,13,3);
                a++;
            }
        }

        for (int row = col; row <horizon_mpc_; row++)
        {
            if (col == row)
            {
                B_mpc_.block(13 * row, col_B_mpc-3*num_contact, 13, 3*num_contact) = Bmat_tmp;
            }
            else
            {
                B_mpc_.block(13 * row, col_B_mpc-3*num_contact, 13, 3*num_contact) = A_mpc_.block(13 * (row - col - 1), 0, 13, 13) * Bmat_tmp;
            }
        }
    }
    x0_qp_.block(0, 0, 3, 1) = root_euler_; // rpy xyz omega vel
    x0_qp_.block(3, 0, 3, 1) = root_position_;
    x0_qp_.block(6, 0, 3, 1) = root_omega_;
    x0_qp_.block(9, 0, 3, 1) = root_velocity_;
    x0_qp_[12] = -9.81;

    Kweight_qp_.resize(3*num_contact_foot_,3*num_contact_foot_);
    Kweight_qp_=Kweight_qp_.setIdentity()*0.00000001;
    Hessian_qp_tmp = B_mpc_.transpose() * Lweight_qp_;
    Gradient_qp_ = 2 * Hessian_qp_tmp * (A_mpc_ * x0_qp_ - reference_trajectory_);
    Hessian_qp_ = 2 * (Hessian_qp_tmp * B_mpc_ + Kweight_qp_);
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

void convexMPC::generateContraintV2()
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
    ubA_unit<<1000000*BIG_FORCE,1000000*BIG_FORCE,1000000*BIG_FORCE,1000000*BIG_FORCE,1000000*BIG_FORCE;
    lb_unit<<-BIG_FORCE,-BIG_FORCE,0;
    ub_unit<<BIG_FORCE,BIG_FORCE,BIG_FORCE;
    int cols=constraint_unit_.cols();
    int rows=constraint_unit_.rows();
    constraint_mat_.resize(rows * num_contact_foot_, cols * num_contact_foot_);
    constraint_lb_A_.resize(rows * num_contact_foot_, 1);
    constraint_ub_A_.resize(rows * num_contact_foot_, 1);
    constraint_ub_.resize(3*num_contact_foot_);
    constraint_lb_.resize(3*num_contact_foot_);
    constraint_ub_.setZero();
    constraint_lb_.setZero();

    constraint_mat_.setZero();
    constraint_lb_A_.setZero();
    constraint_ub_A_.setZero();


    for (int i = 0; i < num_contact_foot_; i++)
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
void convexMPC::solveMPCV2()
{
    if (real_allocated)
    {
        free(H_qpoases);
        free(g_qpoases);
        free(A_qpoases);
        free(lb_A_qpoases);
        free(ub_A_qpoases);
        free(q_soln);
        free(ub_qpoases);
        free(lb_qpoases);
    }
    int col_A{3};
    int row_A{5};
    H_qpoases = (qpOASES::real_t *)malloc(3* num_contact_foot_ * 3* num_contact_foot_ * sizeof(qpOASES::real_t));
    g_qpoases = (qpOASES::real_t *)malloc(3* num_contact_foot_ * sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t *)malloc(row_A* num_contact_foot_ *col_A* num_contact_foot_* sizeof(qpOASES::real_t));
    lb_A_qpoases = (qpOASES::real_t *)malloc(row_A*num_contact_foot_  * sizeof(qpOASES::real_t));
    ub_A_qpoases = (qpOASES::real_t *)malloc(row_A*num_contact_foot_  * sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t *)malloc(3*num_contact_foot_  * sizeof(qpOASES::real_t));

    lb_qpoases = (qpOASES::real_t *)malloc(3*num_contact_foot_  * sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t *)malloc(3*num_contact_foot_  * sizeof(qpOASES::real_t));

    matrix_to_real(H_qpoases, Hessian_qp_, 3* num_contact_foot_, 3* num_contact_foot_);
    matrix_to_real(g_qpoases, Gradient_qp_, 3* num_contact_foot_, 1);
    matrix_to_real(A_qpoases, constraint_mat_, row_A* num_contact_foot_, col_A* num_contact_foot_);
    matrix_to_real(lb_A_qpoases, constraint_lb_A_, row_A* num_contact_foot_, 1);
    matrix_to_real(ub_A_qpoases, constraint_ub_A_, row_A* num_contact_foot_, 1);

    matrix_to_real(lb_qpoases, constraint_lb_, 3* num_contact_foot_, 1);
    matrix_to_real(ub_qpoases, constraint_ub_, 3* num_contact_foot_, 1);
    real_allocated = 1;
    qpOASES::int_t nV, nC;
    nV = 3* num_contact_foot_;
    nC = row_A* num_contact_foot_;
    qpOASES::QProblem problem_sqp(nV, nC);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_sqp.setOptions(op);
    qpOASES::int_t nWSR = 100;
    int rval = problem_sqp.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_A_qpoases, ub_A_qpoases, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to init!\n");
    int rval2 = problem_sqp.getPrimalSolution(q_soln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");
    int tmp{0};
    for (int i = 0; i < 4; i++)
    {
        if(foot_contact_table_(i,0)!=0)
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


    #ifdef DEBUG_MPC
    std::cout << "print H_qpoases" << std::endl;
    for (int i = 0; i < 3* num_contact_foot_; i++)
    {
        for (int j = 0; j < 3* num_contact_foot_; j++)
        {
            std::cout << H_qpoases[3* num_contact_foot_ * i + j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "print g_qpoases" << std::endl;
    for (int i = 0; i < 3* num_contact_foot_; i++)
    {
        std::cout << g_qpoases[i] << " ";
        std::cout << std::endl;
    }
    std::cout <<"yMPC result:";
     printMatrix(foot_force_in_world_);
    std::cout << "run yMPC qpoases finished\n";
#endif
}
