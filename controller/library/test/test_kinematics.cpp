#include "../kinematics.h"
#include "../robot_model.h"
#include <Eigen/Dense>
bool testKinematics() {
  ROBOTICS::RobotModel model;
  model.joint_seq_ << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  model.joint_parent_ << -1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10; // -1:base
  model.foot_seq_ << 0, 1, 2, 3;
  model.foot_parent_ << 2, 5, 8, 11;
  model.joint_axis_[0] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; // joint 0
  model.joint_axis_[1] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[2] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[3] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[4] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[5] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[6] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[7] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[8] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[9] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[10] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  model.joint_axis_[11] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; // joint 11

  model.joint_position_[0] << 0.1875, -0.0875, 0.18, 0, 0, 0;
  model.joint_position_[1] << 0.0, -0.0775, 0.0, 0, 0, 0;
  model.joint_position_[2] << 0.0, 0.0, -0.3095, 0, 0, 0;
  model.joint_position_[3] << 0.1875, 0.0875, 0.18, 0, 0, 0;
  model.joint_position_[4] << 0.0, 0.0775, 0.0, 0, 0, 0;
  model.joint_position_[5] << 0.0, 0.0, -0.3095, 0, 0, 0;
  model.joint_position_[6] << -0.1875, -0.0875, 0.0, 0, 0, 0;
  model.joint_position_[7] << 0.0, -0.0775, 0.0, 0, 0, 0;
  model.joint_position_[8] << 0.0, 0.0, -0.225, 0, 0, 0;
  model.joint_position_[9] << -0.1875, 0.0875, 0.0, 0, 0, 0;
  model.joint_position_[10] << 0.0, 0.0775, 0.0, 0, 0, 0;
  model.joint_position_[11] << 0.0, 0.0, -0.225, 0, 0, 0;

  model.foot_position_[0] << 0, 0, -0.32, 0, 0, 0;
  model.foot_position_[1] << 0, 0, -0.32, 0, 0, 0;
  model.foot_position_[2] << -0.01, 0, -0.26, 0, 0, 0;
  model.foot_position_[3] << -0.01, 0, -0.26, 0, 0, 0;
  ROBOTICS::Kinematics kine(true, model);
  Eigen::Matrix<float, 18, 1> q, dq;
  q.setZero();
  dq.setZero();
  q.block(0, 0, 9, 1) << 0.9509, 0.7223, 0.4001, 0.8319, 0.1343, 0.0605, 0.0842,
      0.1639, 0.3242;
  dq.block(0, 0, 9, 1) << 0.9969, 0.5535, 0.5155, 0.3307, 0.4300, 0.4918,
      0.0710, 0.8877, 0.0646;
  kine.update(q, dq);
  // verify position
  Eigen::Matrix<float, 6, 1> p, p_verify;
  p_verify << 0.966166307771374, 0.679868419756724, -0.017916304457526,
      0.256083834653386, 0.533390320787564, 0.919852312384661;
  kine.getFootPosition(p, 0);
  p = p - p_verify;
  if (p.norm() > 1e-6) {
    return false;
  }
  // verify jacobian
  Eigen::Matrix<float, -1, -1> jacob, jacob_whole, jacob_verify, delta_j;
  jacob_verify.resize(6, 9);
  jacob_verify << 0, 0, 0, 0.000000000000000, -0.739212302316879,
      0.667408056687314, -0.732407561332591, 0.653714782425179,
      -0.691531987055819, 0, 0, 0, 0.000000000000000, 0.673472473159356,
      0.732555918513281, 0.678224829841137, 0.725076388973370,
      0.688630660814948, 0, 0, 0, 1.000000000000000, -0.000000000000000,
      -0.133896647157527, 0.059918647262837, -0.216613049911912,
      0.218108513965408, 1.000000000000000, 0.000000000000000,
      0.000000000000000, 0.042431580243277, -0.281522474383945,
      -0.311901764193581, -0.375842708572764, -0.423791463574313,
      -0.166965042479795, 0.000000000000000, 1.000000000000000,
      0.000000000000000, 0.015266307771374, -0.309002794824041,
      0.276943341996548, -0.426214244300862, 0.422013444166765,
      -0.219201955845113, 0.000000000000000, -0.000000000000000,
      1.000000000000000, -0.000000000000000, 0.021084508071777,
      -0.039502602624102, 0.230296281988421, 0.133663414103154,
      0.162705799353918;
  kine.getFootJacobian(jacob_whole);
  jacob = jacob_whole.block(0, 0, 6, 9);
  //   std::cout << "jacob-" << jacob - jacob_verify << std::endl;
  delta_j = jacob - jacob_verify;
  if (delta_j.norm() > 1e-6) {
    return false;
  }
  // verify dot_jacobian
  Eigen::Matrix<float, -1, -1> djacob, djacob_whole, djacob_verify, ddelta_j;
  djacob_verify.resize(6, 9);
  djacob_verify << 0, 0, 0, 0, -0.222717346873799, -0.281031795879180,
      -0.140688612890672, -0.346320685313035, 0.239731147481833, 0, 0, 0, 0,
      -0.244457508376192, 0.178151283354829, -0.194599226752643,
      0.166899360763107, -0.170609908229965, 0, 0, 0, 0, -0.000000000000000,
      -0.426127979706809, 0.482998613938268, -0.486490383047217,
      1.298752467169573, 0, 0, 0, -0.338577778493441, 0.193209589443276,
      0.051790722136614, 0.341450647178085, 0.151468201522961,
      0.245462120721196, 0, 0, 0, -0.674086284296826, 0.006807870760404,
      -0.123926116527166, -0.020140375157733, -0.109089535416264,
      -0.095554620903440, 0, 0, 0, 0, 0.216879910390053, 0.728980345208280,
      0.278547704099272, 0.794478055185463, 0.123153776330593;
  kine.getFootDotJacobian(djacob_whole);
  djacob = djacob_whole.block(0, 0, 6, 9);
  ddelta_j = djacob - djacob_verify;
  //   std::cout << "ddelta_j-" << djacob - djacob_verify << std::endl;
  if (ddelta_j.norm() > 1e-6) {
    return false;
  }

  return true;
}
