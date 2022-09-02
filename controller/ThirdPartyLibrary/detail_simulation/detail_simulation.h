
#include <eigen3/Eigen/Dense>
// #include "modules/common/cpp_type.h"
// #include "modules/common/message/common_data.h"

namespace detail_simulation {
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;
template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;

class DetailSimulation {
 private:
  Eigen::Matrix<float, 3, 60> m_y_matrix;
  Eigen::Matrix<float, 3, 30> m_y_matrix_link;
  Eigen::Matrix<float, 3, 30> m_y_matrix_rotor;
  Eigen::Matrix<float, 4, 30> m_link_inertial_ident;
  Eigen::Matrix<float, 4, 30> m_rotor_inertial_ident;
  Eigen::Matrix<float, 4, 30> m_link_inertial_CAD;
  Eigen::Matrix<float, 4, 30> m_rotor_inertial_CAD;
  Eigen::Matrix<float, 4, 6> m_muvc, m_muvc_ident, m_muvc_CAD;
  Vec12<float> m_tau_link, m_tau_rotor, m_friction;
  Vec12<float> m_q, m_dq, m_ddq, m_dq_last;
  constexpr static float step = 0.002;
  constexpr static float alpha_ddq = 0.4;

 public:
  float testvar = 0.011;
  void detail_trajectory(Vec3<float>& rpy_des, Vec3<float>& pos_des);
  void getTorque(Vec12<float>& torque_link, Vec12<float>& torque_rotor, Vec12<float>& torque_friction);
  void calculateFrinction(int mode);
  void inverseDynamics(Vec12<float> q, Vec12<float> dq, int mode);
  void calculateJointStates(Vec12<float> q, Vec12<float> dq);
  int sign(float n);
  DetailSimulation();
};

}  // namespace detail_simulation