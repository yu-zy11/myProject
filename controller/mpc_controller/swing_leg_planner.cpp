#include "swing_leg_planner.h"
void SwingLegPlanner::init() {
  kp_joint_ << 10, 10, 10;
  kd_joint_ << 1, 1, 1;
  p_hip_abs_ << 0.1, 0.05, 0;
}

bool SwingLegPlanner::update(const FusionData &state, const commandData &cmd) {
  // update parameters from control parameter interface
  Eigen::Vector3f body_vel = state.body.vel_in_world;
  Eigen::Vector3f body_vel_des = cmd.body.vel_in_world;
  body_vel[2] = 0;
  body_vel_des[2] = 0;
  constexpr float k_Raibert{0.005};
  foot_position_des_in_hip_ = 0.5 * body_vel * state.gait.stance_duration +
                              k_Raibert * (body_vel - body_vel_des);

  // foot placement
  float side_sign[4] = {-1, 1, -1, 1};
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::Vector3f p_hip((leg == 0 || leg == 1) ? parameters->bodyLength
                                                 : -parameters->bodyLength,
                          (leg == 1 || leg == 3) ? parameters->bodyWidth
                                                 : -parameters->bodyWidth,
                          0);
    Eigen::Vector3f hip_offset(0, side_sign[leg] * parameters->side_sign_offset,
                               0);
    foot_pos_rel_target_.col(leg) = p_hip + hip_offset + com_offset_;
    // simple Raibert heuristic
    foot_pos_rel_target_.col(leg) += delta_foot;
    foot_pos_rel_target_.col(leg) += track_error;
  }
  return true;
}
