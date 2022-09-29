#ifndef PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_INTERFACE_H_
#define PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_INTERFACE_H_
#define K_MAX_GAIT_SEGMENTS 36

struct problem_setup {
  float dt;
  float mu;
  float f_max;
  int horizon;
};

struct update_data_t {
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[12];
  float yaw;
  float weights[12];
  float traj[12 * K_MAX_GAIT_SEGMENTS];
  float alpha;
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
  float x_drag;
};

void setup_problem(double dt, int horizon, double mu, double f_max);
double get_solution(int index);
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
    int* gait);

void update_x_drag(float x_drag);

#endif  // PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_MPC_INTERFACE_H_
