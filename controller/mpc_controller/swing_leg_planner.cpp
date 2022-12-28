#include "swing_leg_planner.h"
void SwingLegPlanner::init() {
  kp_joint_ << 10, 10, 10;
  kd_joint_ << 1, 1, 1;
  p_hip_abs_ << 0.1, 0.05, 0;
  swing_time_remaining_.setZero();
  foot_pos_world_target_.setZero();
  foot_pos_world_start_.setZero();
  traj_height_ = 0.08;
}

void SwingLegPlanner::update(const FusionData &state, const commandData &cmd) {
  // update parameters from control parameter interface
  Eigen::Vector3f body_vel = state.body.vel_in_world;
  Eigen::Vector3f body_vel_des = cmd.body.vel_in_world;
  body_vel[2] = 0;
  body_vel_des[2] = 0;
  constexpr float k_Raibert{0.005};
  foot_position_des_in_hip_ = 0.5 * body_vel * state.gait.stance_duration +
                              k_Raibert * (body_vel - body_vel_des);

  // body placement remaining for swing leg
  Eigen::Vector3f body_pos_remain;
  for (int leg = 0; leg < 4; leg++) {
    swing_time_remaining_[leg] =
        (state.gait.period - state.gait.stance_duration) *
        (1 - state.gait.swing_phase[leg]);
  }
  float swing_time_remain = swing_time_remaining_.minCoeff();
  body_pos_remain = body_vel * swing_time_remain;
  // foot pos target in world
  Eigen::Vector4f foot_pos_target;
  Eigen::Vector3f p_hip;
  p_hip = p_hip_abs_;
  for (int leg = 0; leg < 4; ++leg) {
    (leg == 0 || leg == 1) ? p_hip[0] = p_hip[0] : p_hip[0] = -p_hip[0];
    (leg == 1 || leg == 3) ? p_hip[1] = p_hip[1] : p_hip[1] = -p_hip[1];
    if (!state.gait.contact_state[leg]) {
      foot_pos_world_target_.col(leg) =
          body_pos_remain + p_hip + foot_position_des_in_hip_;
    } else {
      foot_pos_world_start_.col(leg) =
          state.leg.foot_position_in_world.segment<3>(3 * leg);
    }
    if (!state.gait.contact_state[leg]) {
      float phase = state.gait.swing_phase[leg];
      float swing_time = state.gait.period - state.gait.stance_duration;
      calculateSwingTrajectory(foot_pos_world_start_.col(leg),
                               foot_pos_world_target_.col(leg), traj_height_,
                               phase, swing_time, leg);
    }
  }
  //
}

void SwingLegPlanner::calculateSwingTrajectory(Eigen::Vector3f psrc,
                                               Eigen::Vector3f pdes,
                                               float Height, float phase,
                                               float swing_time, int leg) {
  Eigen::Vector3f p_target, v_target, a_target;
  float p, v, a;
  CubicSpline(p, v, a, psrc[0], pdes[0], phase);
  p_target[0] = p;
  v_target[0] = v;
  a_target[0] = a;
  CubicSpline(p, v, a, psrc[1], pdes[1], phase);
  p_target[1] = p;
  v_target[1] = v;
  a_target[1] = a;

  if (phase < 0.5) {
    CubicSpline(p, v, a, psrc[2], (psrc[2] + pdes[2]) / 2 + Height, 2 * phase);
  } else {
    CubicSpline(p, v, a, (psrc[2] + pdes[2]) / 2 + Height, pdes[2],
                2 * phase - 1);
  }
  p_target[2] = p;
  v_target[2] = v * 2;
  a_target[2] = a * 4;

  v_target = v_target / swing_time;
  a_target = a_target / pow(swing_time, 2);
  traj_p_target_.col(leg) = p_target;
  traj_v_target_.col(leg) = v_target;
  traj_a_target_.col(leg) = a_target;
}
void SwingLegPlanner::CubicSpline(float &p, float &v, float &a, float q0,
                                  float q1, float t) {
  p = (-2 * pow(t, 3) + 3 * pow(t, 2)) * (q1 - q0) + q0;
  v = (-6 * t + 6 * pow(t, 2)) * (q1 - q0);
  a = (-6 + 12 * t) * (q1 - q0);
}
