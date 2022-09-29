#include "solver_mpc.h"
#include <stdio.h>
#include <sys/time.h>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "modules/common/config_loader.h"
#include "modules/locomotion/locomotion_core/controller/mpc_controller/mpc_interface.h"
#include "qpOASES.hpp"

//#define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
// big enough to act like infinity, small enough to avoid numerical weirdness.
#define HORIZON_LENGTH 14

// RobotState rs;
using Eigen::Dynamic;
using std::cout;
using std::endl;

// qpOASES::real_t a;

Matrix<double, 13 * HORIZON_LENGTH, 13> A_qp;
Matrix<double, 13 * HORIZON_LENGTH, 12 * HORIZON_LENGTH> B_qp;
Matrix<double, 13, 12> Bdt;
Matrix<double, 13, 13> Adt;
Matrix<double, 25, 25> ABc, expmm;
// atrix<double,13*HORIZON_LENGTH,13*HORIZON_LENGTH> S;
Eigen::DiagonalMatrix<double, 13 * HORIZON_LENGTH> S;

Matrix<double, 13 * HORIZON_LENGTH, 1> X_d;
Matrix<double, 20 * HORIZON_LENGTH, 1> U_b;
// Matrix<double,20*HORIZON_LENGTH,12*HORIZON_LENGTH> fmat;
// Matrix<double,12*HORIZON_LENGTH,12*HORIZON_LENGTH> qH;

Matrix<double, 12 * HORIZON_LENGTH, 13 * HORIZON_LENGTH> BS_temp;
Matrix<double, 20 * HORIZON_LENGTH, 12 * HORIZON_LENGTH, Eigen::RowMajor> fmat;
Matrix<double, 12 * HORIZON_LENGTH, 12 * HORIZON_LENGTH, Eigen::RowMajor> qH;

Matrix<double, 12 * HORIZON_LENGTH, 1> qg;

Matrix<double, 12 * HORIZON_LENGTH, 12 * HORIZON_LENGTH> eye_12h;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
uint8_t real_allocated = 0;

char var_elim[2000];
char con_elim[2000];

double* get_q_soln() { return q_soln; }

int8_t near_zero(double a) { return (a < 0.01 && a > -.01); }

int8_t near_one(double a) { return near_zero(a - 1); }
void matrix_to_real(qpOASES::real_t* dst, Matrix<double, Dynamic, Dynamic> src, int16_t rows, int16_t cols) {
  int32_t a = 0;
  for (int16_t r = 0; r < rows; r++) {
    for (int16_t c = 0; c < cols; c++) {
      dst[a] = src(r, c);
      a++;
    }
  }
}

void c2qp(Matrix<double, 13, 13> Ac, Matrix<double, 13, 12> Bc, double dt, int16_t horizon) {
  ABc.setZero();
  ABc.block(0, 0, 13, 13) = Ac;
  ABc.block(0, 13, 13, 12) = Bc;
  ABc = dt * ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0, 0, 13, 13);
  Bdt = expmm.block(0, 13, 13, 12);
#ifdef K_PRINT_EVERYTHING
  cout << "Adt: \n" << Adt << "\nBdt:\n" << Bdt << endl;
#endif
  if (horizon > 19) {
    // throw std::runtime_error("horizon is too long!");
  }

  Matrix<double, 13, 13> powerMats[100];
  powerMats[0].setIdentity();
  for (int i = 1; i < horizon + 1; i++) { powerMats[i] = Adt * powerMats[i - 1]; }

  for (int16_t r = 0; r < horizon; r++) {
    A_qp.block(13 * r, 0, 13, 13) = powerMats[r + 1];  // Adt.pow(r+1);
    for (int16_t c = 0; c < horizon; c++) {
      if (r >= c) {
        int16_t a_num = r - c;
        B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout << "AQP:\n" << A_qp << "\nBQP:\n" << B_qp << endl;
#endif
}

void resize_qp_mats(int16_t horizon) {
  int mcount = 0;
  int h2 = horizon * horizon;

  // A_qp.resize(13*horizon, Eigen::NoChange);
  // mcount += 13*horizon*1;

  // B_qp.resize(13*horizon, 12*horizon);
  // mcount += 13*h2*12;

  // // S.resize(13*horizon, 13*horizon);
  // // mcount += 13*13*h2;
  // S.resize(13*horizon);
  // mcount += 13*horizon;

  // X_d.resize(13*horizon, Eigen::NoChange);
  // mcount += 13*horizon;

  // U_b.resize(20*horizon, Eigen::NoChange);
  // mcount += 20*horizon;

  // fmat.resize(20*horizon, 12*horizon);
  // mcount += 20*12*h2;

  // qH.resize(12*horizon, 12*horizon);
  // mcount += 12*12*h2;

  // qg.resize(12*horizon, Eigen::NoChange);
  // mcount += 12*horizon;

  // eye_12h.resize(12*horizon, 12*horizon);
  // mcount += 12*12*horizon;

  // printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_12h.setIdentity();

  // TODO: use realloc instead of free/malloc on size changes

  if (real_allocated) {
    // free(H_qpoases);
    // free(g_qpoases);
    // free(A_qpoases);
    free(lb_qpoases);
    // free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  // H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12 * 12 * h2;
  // g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  // A_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12 * 20 * h2;
  lb_qpoases = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  // ub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  q_soln = (qpOASES::real_t*)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  H_red = (qpOASES::real_t*)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 12 * h2;
  g_red = (qpOASES::real_t*)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  A_red = (qpOASES::real_t*)malloc(12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 20 * h2;
  lb_red = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  ub_red = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  q_red = (qpOASES::real_t*)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  real_allocated = 1;

  // printf("malloc'd %d floating point numbers.\n",mcount);

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

inline Matrix<double, 3, 3> cross_mat(Matrix<double, 3, 3> I_inv, Matrix<double, 3, 1> r) {
  Matrix<double, 3, 3> cm;
  cm << 0.f, -r(2), r(1), r(2), 0.f, -r(0), -r(1), r(0), 0.f;
  return I_inv * cm;
}
// continuous time state space matrices.
void ct_ss_mats(
    Matrix<double, 3, 3> I_world,
    double m,
    Matrix<double, 3, 4> r_feet,
    Matrix<double, 3, 3> R_yaw,
    Matrix<double, 13, 13>& A,
    Matrix<double, 13, 12>& B,
    float x_drag) {
  A.setZero();
  A(3, 9) = 1.f;
  A(11, 9) = x_drag;
  A(4, 10) = 1.f;
  A(5, 11) = 1.f;

  A(11, 12) = 1.f;
  A.block(0, 6, 3, 3) = R_yaw.transpose();

  B.setZero();
  Matrix<double, 3, 3> I_inv = I_world.inverse();

  for (int16_t b = 0; b < 4; b++) {
    B.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feet.col(b));
    B.block(9, b * 3, 3, 3) = Matrix<double, 3, 3>::Identity() / m;
  }
}

void quat_to_rpy(Quaternionf q, Matrix<double, 3, 1>& rpy) {
  // from my MATLAB implementation

  // edge case!
  double as = t_min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  rpy(0) = atan2(2.f * (q.x() * q.y() + q.w() * q.z()), sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f * (q.y() * q.z() + q.w() * q.x()), sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
}
void print_problem_setup(problem_setup* setup) {
  printf("DT: %.3f\n", setup->dt);
  printf("Mu: %.3f\n", setup->mu);
  printf("F_Max: %.3f\n", setup->f_max);
  printf("Horizon: %d\n", setup->horizon);
}

void print_update_data(update_data_t* update, int16_t horizon) {
  print_named_array("p", update->p, 1, 3);
  print_named_array("v", update->v, 1, 3);
  print_named_array("q", update->q, 1, 4);
  print_named_array("w", update->r, 3, 4);
  pnv("Yaw", update->yaw);
  print_named_array("weights", update->weights, 1, 12);
  print_named_array("trajectory", update->traj, horizon, 12);
  pnv("Alpha", update->alpha);
  print_named_array("gait", update->gait, horizon, 4);
}

Matrix<double, 13, 1> x_0;
Matrix<double, 3, 3> I_world;
Matrix<double, 13, 13> A_ct;
Matrix<double, 13, 12> B_ct_r;

void solve_mpc(update_data_t* update, problem_setup* setup) {
  Matrix<double, 3, 1> p, v, w;  //定义了机器人的在世界坐标系下的，位置p，速度p˙，以及机身坐标系下的旋转角ω,均为3×1矩阵
  Matrix<double, 3, 4> r_feet;   //机身参考系下的足端位置，3×4矩阵
  Matrix<double, 3, 3> R;        //机身坐标系到世界坐标系的旋转矩阵
  Matrix<double, 3, 3> R_yaw;    //偏航角旋转矩阵
  Matrix<double, 3, 3> I_body;   //机身坐标系下的惯量矩阵
  Quaternionf q;                 //四元素表示的世界坐标系下的旋转

  for (int i = 0; i < 3; ++i) {
    p(i) = update->p[i];
    v(i) = update->v[i];
    w(i) = update->w[i];
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) { r_feet(i, j) = update->r[i * 4 + j]; }
  }

  q.w() = update->q[0];
  q.x() = update->q[1];
  q.y() = update->q[2];
  q.z() = update->q[3];

  R = q.toRotationMatrix().cast<double>();
  float yc = cos(update->yaw);
  float ys = sin(update->yaw);

  R_yaw << yc, -ys, 0, ys, yc, 0, 0, 0, 1;

#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
  printf("   PROBLEM DATA  \n");
  printf("-----------------\n");
  print_problem_setup(setup);

  printf("-----------------\n");
  printf("    ROBOT DATA   \n");
  printf("-----------------\n");
  // rs.print();
  print_update_data(update, setup->horizon);
#endif

  // roll pitch yaw
  Matrix<double, 3, 1> rpy;
  quat_to_rpy(q, rpy);

  // std::cout << "get rpy value is: " << rpy << std::endl;
  I_body(0) = ConfigLoaderInstance.getRobotConfig()->I_body[0];
  I_body(4) = ConfigLoaderInstance.getRobotConfig()->I_body[1];
  I_body(8) = ConfigLoaderInstance.getRobotConfig()->I_body[2];
  double m = ConfigLoaderInstance.getRobotConfig()->mass;

  // initial state (13 state representation)
  x_0 << rpy(2), rpy(1), rpy(0), p, w, v, -9.8f;
  I_world = R_yaw * I_body * R_yaw.transpose();  // original
  // I_world = rs.R_yaw.transpose() * rs.I_body * rs.R_yaw;
  // cout<<rs.R_yaw<<endl;
  ct_ss_mats(I_world, m, r_feet, R_yaw, A_ct, B_ct_r, update->x_drag);

  // std::cout << "get I_body value is: " << rs.I_body << std::endl;
  // std::cout << "get mass value is  : " << rs.m << std::endl;

#ifdef K_PRINT_EVERYTHING
  cout << "Initial state: \n" << x_0 << endl;
  cout << "World Inertia: \n" << I_world << endl;
  cout << "A CT: \n" << A_ct << endl;
  cout << "B CT (simplified): \n" << B_ct_r << endl;
#endif
  // QP matrices
  c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);

  // weights
  Matrix<double, 13, 1> full_weight;
  for (uint8_t i = 0; i < 12; i++) full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon, 1);
  // Matrix<double,Eigen::Dynamic,Eigen::Dynamic>  m = full_weight.replicate(setup->horizon,1);
  // // m.replicate(setup->horizon,1);
  // for(int i=0; i<setup->horizon;i++)
  // {
  //   m.block(i*13,0,13,1)=full_weight*pow(1.05,setup->horizon-i);
  //   // m.block(i*13,0,13,1)=full_weight*(setup->horizon-i);
  // }
  // // S.diagonal() = full_weight.replicate(setup->horizon,1);
  // S.diagonal() = m;
  // // std::cout<<"full_weight1:"<<full_weight<<std::endl;
  // // std::cout<<"full_weight2:"<<m<<std::endl;

  // trajectory
  for (int i = 0; i < setup->horizon; i++) {
    for (int j = 0; j < 12; j++) X_d(13 * i + j, 0) = update->traj[12 * i + j];
  }
  // cout<<"XD:\n"<<X_d<<endl;

  // note - I'm not doing the shifting here.
  int16_t k = 0;
  for (int i = 0; i < setup->horizon; i++) {
    for (int j = 0; j < 4; j++) {
      U_b(5 * k + 0) = BIG_NUMBER;
      U_b(5 * k + 1) = BIG_NUMBER;
      U_b(5 * k + 2) = BIG_NUMBER;
      U_b(5 * k + 3) = BIG_NUMBER;
      U_b(5 * k + 4) = update->gait[i * 4 + j] * setup->f_max;
      k++;
    }
  }

  double mu = 1.f / setup->mu;
  Matrix<double, 5, 3> f_block;

  f_block << mu, 0, 1.f, -mu, 0, 1.f, 0, mu, 1.f, 0, -mu, 1.f, 0, 0, 1.f;

  for (int16_t i = 0; i < setup->horizon * 4; i++) { fmat.block(i * 5, i * 3, 5, 3) = f_block; }

  // start = clock();

  // qH = B_qp.transpose()*S;
  // qH = qH*B_qp;

  // std::cout << "111111111111111111 : " << B_qp.rows() << "  2222222222222222 :" << B_qp.cols() << std::endl;
  // std::cout << "222222222222222222 : " << S << std::endl;
  // std::cout << "333333333333333333 : " << B_qp << std::endl;

  // finish = clock();
  // double duration;
  // duration = (double)(finish - start) / CLOCKS_PER_SEC;
  // printf( "%f ****************************seconds\n", duration );

  // qH = 2*(B_qp.transpose()*S*B_qp + update->alpha*eye_12h);
  // qH.triangularView<Eigen::Upper>() = B_qp.transpose()*S*B_qp;  //上三角，行优先
  // qH.diagonal() += update->alpha*eye_12h.diagonal();
  // qH = 2*qH;

  // qg = 2*B_qp.transpose()*S*(A_qp*x_0 - X_d);

  BS_temp = B_qp.transpose() * S;
  // qH.triangularView<Eigen::Upper>() =BS_temp*B_qp;
  // qH.diagonal() += update->alpha*eye_12h.diagonal();

  qg = BS_temp * (A_qp * x_0 - X_d);

  // matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);
  // matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);
  // matrix_to_real(A_qpoases,fmat,setup->horizon*20, setup->horizon*12);
  // matrix_to_real(ub_qpoases,U_b,setup->horizon*20, 1);

  g_qpoases = qg.data();

  A_qpoases = fmat.data();

  ub_qpoases = U_b.data();

  for (int16_t i = 0; i < 20 * setup->horizon; i++) lb_qpoases[i] = 0.0f;

  int16_t num_constraints = 20 * setup->horizon;
  int16_t num_variables = 12 * setup->horizon;

  qpOASES::int_t nWSR = 100;

  int new_vars = num_variables;
  int new_cons = num_constraints;

  for (int i = 0; i < num_constraints; i++) con_elim[i] = 0;

  for (int i = 0; i < num_variables; i++) var_elim[i] = 0;

  for (int i = 0; i < num_constraints; i++) {
    if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
    double* c_row = &A_qpoases[i * num_variables];
    for (int j = 0; j < num_variables; j++) {
      if (near_one(c_row[j])) {
        new_vars -= 3;
        new_cons -= 5;
        int cs = (j * 5) / 3 - 3;
        var_elim[j - 2] = 1;
        var_elim[j - 1] = 1;
        var_elim[j] = 1;
        con_elim[cs] = 1;
        con_elim[cs + 1] = 1;
        con_elim[cs + 2] = 1;
        con_elim[cs + 3] = 1;
        con_elim[cs + 4] = 1;
      }
    }
  }
  // if(new_vars != num_variables)
  if (1 == 1) {
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for (int i = 0; i < num_variables; i++) {
      if (!var_elim[i]) {
        if (!(vc < new_vars)) { printf("BAD ERROR 1\n"); }
        var_ind[vc] = i;
        vc++;
      }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; i++) {
      if (!con_elim[i]) {
        if (!(vc < new_cons)) { printf("BAD ERROR 1\n"); }
        con_ind[vc] = i;
        vc++;
      }
    }
    for (int i = 0; i < new_vars; i++) {
      int olda = var_ind[i];
      g_red[i] = g_qpoases[olda];
      // for(int j = 0; j < new_vars; j++)
      // {
      //   int oldb = var_ind[j];
      //   // H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
      //   if(olda <= oldb) {
      //     H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
      //   } else {
      //     H_red[i*new_vars + j] = H_qpoases[oldb*num_variables + olda];
      //   }
      // }
      for (int j = 0; j <= i; j++) {
        int oldb = var_ind[j];

        H_red[i * new_vars + j] = (((B_qp.transpose()).block(olda, 0, 1, 13 * HORIZON_LENGTH).array()) *
                                   (((S.diagonal()).transpose()).array()) *
                                   (((B_qp.block(0, oldb, 13 * HORIZON_LENGTH, 1)).transpose()).array()))
                                      .sum();
      }
      //对角线元素，加上alpha
      H_red[i * new_vars + i] += update->alpha * eye_12h(olda, olda);
    }
    for (int i = 0; i < new_vars; i++)
      for (int j = i + 1; j < new_vars; j++) { H_red[i * new_vars + j] = H_red[j * new_vars + i]; }
    for (int con = 0; con < new_cons; con++) {
      for (int st = 0; st < new_vars; st++) {
        float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
        A_red[con * new_vars + st] = cval;
      }
    }
    for (int i = 0; i < new_cons; i++) {
      int old = con_ind[i];
      ub_red[i] = ub_qpoases[old];
      lb_red[i] = lb_qpoases[old];
    }

    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    // int_t nWSR = 50000;

    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    (void)rval;
    int rval2 = problem_red.getPrimalSolution(q_red);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN) printf("failed to solve!\n");

    // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);
    // for(int j = 0; j < 12; j++){
    //  std::cout << "var_elim value is: " << var_elim[j] << " ";
    //}
    // std::cout << std::endl;

    vc = 0;
    for (int i = 0; i < num_variables; i++) {
      if (var_elim[i]) {
        q_soln[i] = 0.0f;
      } else {
        q_soln[i] = q_red[vc];
        vc++;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  // cout<<"fmat:\n"<<fmat<<endl;
#endif
}
