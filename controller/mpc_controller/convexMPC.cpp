#include "convexMPC.h"
#include "math_utils.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <iostream>

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
    // weight for qp problem;
    Lweight_qp_.resize(13 * horizon_mpc_, 13 * horizon_mpc_); 
    Kweight_qp_.resize(12 * horizon_mpc_, 12 * horizon_mpc_);
    reference_trajectory_.resize(13 * horizon_mpc_, 1);
    A_mpc_.resize(13 * horizon_mpc_, 13);
    B_mpc_.resize(13 * horizon_mpc_, 12 * horizon_mpc_);
    Hessian_qp_.resize(12 * horizon_mpc_, 12 * horizon_mpc_);
    Gradient_qp_.resize(12 * horizon_mpc_, 1);
    Hessian_qp_tmp.resize(12 * horizon_mpc_, 13 * horizon_mpc_);
    constraint_ub_.resize(12 * horizon_mpc_, 1);

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

    lweight_qp_unit_ << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0;
    kweight_qp_unit_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Lweight_qp_.diagonal() = lweight_qp_unit_.replicate(horizon_mpc_, 1);
    Kweight_qp_.diagonal() = kweight_qp_unit_.replicate(horizon_mpc_, 1) * 0.000001;
    std::cout << "init MPC finished\n";
}

void convexMPC::MPCInterface()
{
    Inertial_.setIdentity();
    foot_position_in_world_ << 0.2, 0.2, -0.2, -0.2,
        -0.1, 0.1, -0.1, 0.1,
        -0.4, -0.4, -0.4, -0.4;
    body_mass_ = 50.0;
    mu_ = 0.5;
    root_euler_ << 0, 0, 0;
    foot_contact_table_.setOnes();

    rotm_body2world_ = euler2rotm(root_euler_);
    Inertial_in_world_ = rotm_body2world_ * Inertial_ * rotm_body2world_.transpose();
    // std::cout<<"test euler2rotm:"<<rotm_body2world_<<std::endl;
    root_euler_ << 0, 0, 0;
    root_omega_ << 0.0, 0.0, 0.0;
    root_position_ << 0, 0, 0.4;
    root_velocity_ << 0, 0, 0;

    root_euler_target_ << 0, 0, 0;
    root_omega_target_ << 0.0, 0.0, 0.0;
    root_position_target_ << 0, 0, 0.4;
    root_velocity_target_ << 0, 0, 0;
    mat_omega2rpy_rate_ = omega2rpyrate(root_euler_);
#ifdef PRINT_TEST
    std::cout << "foot_contact_table_" << std::endl;
    printMatrix(foot_contact_table_);
#endif
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
#ifdef PRINT_TEST
    std::cout << "Amat_continuous_" << std::endl;
    printMatrix(Amat_continuous_);
    std::cout << "Bmat_continuous_" << std::endl;
    printMatrix(Bmat_continuous_);
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
#ifdef PRINT_TEST
    std::cout << "Amat_discrete_" << std::endl;
    printMatrix(Amat_discrete_);
    std::cout << "Bmat_discrete_" << std::endl;
    printMatrix(Bmat_discrete_);
#endif
}
void convexMPC::FormulateQPform()
{ // Ac: linear constraints

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
#ifdef PRINT_TEST
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

// calculate body's  reference trajectory for MPC
void convexMPC::generateReferenceTrajectory()
{
    constexpr float max_pos_error{0.05};
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
#ifdef PRINT_TEST
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
            Eigen::Vector3f ub_unit;
            (foot_contact_table_(leg, i) == 0) ? (ub_unit << BIG_FORCE, BIG_FORCE, 0.0f) : (ub_unit << BIG_FORCE, BIG_FORCE, BIG_FORCE);
            constraint_ub_.block(12 * i + 3 * leg, 0, 3, 1) = ub_unit;
        }
    }
#ifdef PRINT_TEST
    std::cout << "constraint_mat_" << std::endl;
    printMatrix(constraint_mat_);
    std::cout << "constraint_lb_A_" << std::endl;
    printMatrix(constraint_lb_A_);
    std::cout << "constraint_ub_" << std::endl;
    printMatrix(constraint_ub_);
#endif
}
void convexMPC::run()
{
    std::cout << "run mpc\n";
    // MPCInterface();
    calculateContinuousEquation();
    discretizeStateEquation();
    generateReferenceTrajectory();
    generateContraint();
    FormulateQPform();
    solveMPC();
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
    matrix_to_real(lb_A_qpoases, constraint_lb_A_, horizon_mpc_ * 20, 1);
#ifdef PRINT_TEST
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
    //   Timer solve_timer;
    qpOASES::int_t nV, nC;
    nV = horizon_mpc_ * 12;
    nC = 20 * horizon_mpc_;
    qpOASES::QProblem problem_sqp(nV, nC);
    qpOASES::Options op;
    //   op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_sqp.setOptions(op);
    qpOASES::int_t nWSR = 50000;
    int rval = problem_sqp.init(H_qpoases, g_qpoases, A_qpoases, NULL, ub_qpoases, lb_A_qpoases, NULL, nWSR);
    if (rval != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to init!\n");
    int rval2 = problem_sqp.getPrimalSolution(q_soln);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");
    for (int i = 0; i < 12*horizon_mpc_; i++)
    {
        std::cout << q_soln[i] << std::endl;
    }
    std::cout << "run qpoases finished\n";
}

void convexMPC::printMatrix(Eigen::Matrix<float, -1, -1> mat)
{
    std::cout << mat << std::endl;
}
Eigen::Matrix<float, 12, 1> convexMPC::getFootForce()
{
    Eigen::Matrix<float, 12, 1> force;
    for (int i = 0; i < 12; i++)
    {
        force[i] = q_soln[i];
    }
    return force;
}