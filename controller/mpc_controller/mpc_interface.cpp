#include "modules/locomotion/locomotion_core/controller/mpc_controller/mpc_interface.h"

#include <stdio.h>
#include <string.h>
#include <Eigen/Dense>
#include "modules/locomotion/locomotion_core/controller/mpc_controller/solver_mpc.h"

#define K_NUM_LEGS 4

problem_setup problem_configuration;
uint8_t gait_data[K_MAX_GAIT_SEGMENTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data_t update;
pthread_t solve_thread;

bool first_run = true;

void initialize_mpc() {
  // printf("Initializing MPC!\n");
  if (pthread_mutex_init(&problem_cfg_mt, NULL) != 0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if (pthread_mutex_init(&update_mt, NULL) != 0) printf("[MPC ERROR] Failed to initialize update data mutex.\n");

#ifdef K_DEBUG
  printf("[MPC] Debugging enabled.\n");
  printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
  printf("      Size of problem update struct: %ld bytes.\n", sizeof(update_data_t));
  printf("      Size of MATLAB floating point type: %ld bytes.\n", sizeof(double));
  printf("      Size of float: %ld bytes.\n", sizeof(float));
#else
    // printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt, int horizon, double mu, double f_max) {
  if (first_run) {
    first_run = false;
    initialize_mpc();
  }

#ifdef K_DEBUG
  printf("[MPC] Got new problem configuration!\n");
  printf(
      "[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
      horizon,
      f_max,
      mu,
      dt);
#endif

  // pthread_mutex_lock(&problem_cfg_mt);

  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.mu = mu;
  problem_configuration.dt = dt;

  // pthread_mutex_unlock(&problem_cfg_mt);
  resize_qp_mats(horizon);
}

// inline to motivate gcc to unroll the loop in here.
inline void double_to_float(float* dst, double* src, int n_items) {
  for (int i = 0; i < n_items; i++) *dst++ = *src++;
}

inline void int_to_u8(uint8_t* dst, int* src, int n_items) {
  for (int i = 0; i < n_items; i++) *dst++ = *src++;
}

int has_solved = 0;

void update_problem_data_floats(
    float* p,
    float* v,
    float* q,
    float* w,
    float* r,
    float yaw,
    float* weights,
    float* state_trajectory,
    float alpha,
    int* gait) {
  update.alpha = alpha;
  update.yaw = yaw;
  int_to_u8(update.gait, gait, 4 * problem_configuration.horizon);
  memcpy((void*)update.p, (void*)p, sizeof(float) * 3);
  memcpy((void*)update.v, (void*)v, sizeof(float) * 3);
  memcpy((void*)update.q, (void*)q, sizeof(float) * 4);
  memcpy((void*)update.w, (void*)w, sizeof(float) * 3);
  memcpy((void*)update.r, (void*)r, sizeof(float) * 12);
  memcpy((void*)update.weights, (void*)weights, sizeof(float) * 12);
  memcpy((void*)update.traj, (void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);
  // std::cout << "---------------" << std::endl;
  // std::cout << "yaw = " << update.yaw << " alpha = " << update.alpha << std::endl;
  // std::cout << "p = " << update.p[0] << " " << update.p[1] << " " << update.p[2] << std::endl;
  // std::cout << "v = " << update.v[0] << " " << update.v[1] << " " << update.v[2] << std::endl;
  // std::cout << "q = " << update.q[0] << " " << update.q[1] << " " << update.q[2] << " " << update.q[3] << std::endl;
  // std::cout << "w = " << update.w[0] << " " << update.w[1] << " " << update.w[2] << std::endl;
  // std::cout << "r = " << std::endl;
  // for (int i = 0; i < 12; i++) { std::cout << update.r[i] << " "; }
  // std::cout << std::endl;
  // std::cout << "weights = " << std::endl;
  // for (int i = 0; i < 12; i++) { std::cout << update.weights[i] << " "; }
  // std::cout << std::endl;
  // std::cout << "trajAll = " << std::endl;
  // for (int i = 0; i < problem_configuration.horizon; i++) {
  //   for (int j = 0; j < 12; j++) { std::cout << update.traj[12 * i + j] << " "; }
  // }
  // std::cout << std::endl;
  // std::cout << "gait = " << std::endl;
  // for (int i = 0; i < problem_configuration.horizon; i++) {
  //   for (int j = 0; j < 4; j++) { std::cout << (int)update.gait[4 * i + j] << " "; }
  // }
  // std::cout << std::endl;
  // std::cout << "---------------" << std::endl;
  solve_mpc(&update, &problem_configuration);
  has_solved = 1;
}

void update_x_drag(float x_drag) { update.x_drag = x_drag; }

double get_solution(int index) {
  if (!has_solved) return 0.f;
  double* qs = get_q_soln();
  return qs[index];
}
