#include "quick_stop.h"
#include <iostream>
namespace MCC {
namespace locomotion_core {
void QuickStop::init(const float& dt, const float& period, const float& height, const float& max_step_length) {
  control_step_ = dt;
  gait_period_ = period;
  gait_period_ = 0.5;
  body_height_ = height;
  max_step_length_ = max_step_length;
  root_pos_cmd_.setZero();
  root_vel_cmd_.setZero();
  root_acc_cmd_.setZero();
  foot_hold_pos_.setZero();
  root_velxy_.setZero();
  stop_finished_ = false;
  quick_stop_ = false;
  stop_mode_ = StopPhase::TrackVel;
  first_run_StepStop_ = true;
  Tc_ = sqrt(body_height_ / 9.81);
  time_lip_ = 0;
  vel_limit_ = max_step_length_ / Tc_;
}
void QuickStop::updateData(
    const common::message::FusionData& fu,
    const common::message::OperatorData& op,
    const Eigen::Vector4i& contact_target,
    const float& period,
    const bool& stop) {
  contact_target_ = contact_target;
  root_vel_cmd_world_ = fu.rotation_matrix_body_to_world * op.root_linear_velocity_in_body;
  root_velxy_cmd_ << root_vel_cmd_world_[0], root_vel_cmd_world_[1];
  // root_omega_cmd_ << op.root_angular_velocity_in_body[1], op.root_angular_velocity_in_body[2];
  root_velxy_ << fu.root_linear_velocity_in_world[0], fu.root_linear_velocity_in_world[1];
  root_pos_ << fu.root_position[0], fu.root_position[1];
  foot_pos_world_ = fu.foot_position_in_world;
  stop_signal_ = stop;
}
void QuickStop::run() {
  // control mode
  (stop_signal_ == true) ? quick_stop_ = true : quick_stop_ = false;
  if (!quick_stop_) {
    stop_mode_ = StopPhase::TrackVel;
  } else {
    if ((contact_target_.sum() == 4 && root_velxy_.norm() < vel_limit_) || stop_mode_ == StopPhase::StepStop) {
      stop_mode_ = StopPhase::StepStop;
    } else {
      (root_velxy_.norm() > vel_limit_) ? stop_mode_ = StopPhase::SlowDown : stop_mode_ = StopPhase::SlowDownFinished;
    }
  }
  // calculte command pos vel acc for root
  if (stop_mode_ != StopPhase::StepStop) {
    first_run_StepStop_ = true;
    stop_finished_ = false;
  }
  if (first_run_StepStop_ && stop_mode_ == StopPhase::StepStop) {
    step_stop_duration_ = gait_period_ - 5 * control_step_;
    time_lip_ = 0.0;
    first_run_StepStop_ = false;
    root_velxy_cmd0_ = root_velxy_;
    // vel0_ = root_velxy_;
    // pos0_ = -vel0_ * Tc_;
    // root_pos_world_start_ = root_pos_;
    // root_pos_des_ = root_velxy_ * Tc_ + root_pos_;
    std::cout << "first_run_StepStop_:" << first_run_StepStop_ << " gait_period_" << gait_period_ << std::endl;
  }

  if (stop_mode_ == StopPhase::StepStop) {
    time_lip_ += control_step_;
    if (time_lip_ < step_stop_duration_ / 2) {
      // if (time_lip_ < 2 * control_step_) {
      vel0_ = root_velxy_;
      pos0_ = -vel0_ * Tc_;
      root_pos_des_ = root_velxy_ * Tc_ + root_pos_;
      //
      root_pos_cmd_ = root_pos_;
      root_vel_cmd_ = root_velxy_cmd0_;  // root_velxy_
      // root_vel_cmd_ = root_velxy_cmd0_ * (1 - 2 * time_lip_ / step_stop_duration_);
      if (root_vel_cmd_.norm() >= vel_limit_) { root_vel_cmd_ = root_vel_cmd_ / root_vel_cmd_.norm() * vel_limit_; }
      root_acc_cmd_ << 0, 0;
    } else if (time_lip_ < step_stop_duration_ * 3) {
      Eigen::Vector2f root_pos_abs;
      float t = time_lip_ - step_stop_duration_ / 2;
      root_pos_abs = pos0_ * cosh(t / Tc_) + Tc_ * vel0_ * sinh(t / Tc_);
      root_pos_cmd_ = root_pos_abs + root_pos_des_;
      root_vel_cmd_ = pos0_ / Tc_ * sinh(t / Tc_) + vel0_ * cosh(t / Tc_);
      root_acc_cmd_ = root_pos_abs / (Tc_ * Tc_);
    } else {
      root_pos_cmd_ = root_pos_des_;
      root_vel_cmd_ << 0, 0;
      root_acc_cmd_ << 0, 0;
    }
    (time_lip_ < step_stop_duration_) ? stop_finished_ = false : stop_finished_ = true;
    // calculate foothold target
    foot_hold_pos_ = root_velxy_ * Tc_;
  }
}
void QuickStop::getTargetPos(Eigen::Vector3f& root_pos_cmd) {
  (stop_mode_ == StopPhase::StepStop) ? root_pos_cmd.head(2) = root_pos_cmd_ : root_pos_cmd = root_pos_cmd;
}
void QuickStop::getTargetVel(Eigen::Vector3f& root_vel_cmd) {
  if (stop_mode_ == StopPhase::TrackVel) {
    root_vel_cmd = root_vel_cmd;
  } else if (stop_mode_ == StopPhase::SlowDown || stop_mode_ == StopPhase::SlowDownFinished) {
    if (root_velxy_.norm() < 1e-6) {
      root_vel_cmd.head(2) = root_velxy_ * 0;
    } else if (root_velxy_.norm() < vel_limit_) {
      root_vel_cmd.head(2) = root_velxy_;
    } else {
      root_vel_cmd.head(2) = root_velxy_ / root_velxy_.norm() * vel_limit_;
    }
  } else if (stop_mode_ == StopPhase::StepStop) {
    root_vel_cmd.head(2) = root_vel_cmd_;
  } else {
    std::cout << "wong mode!! in getTargetVel" << std::endl;
    abort();
  }
}
void QuickStop::getTargetAcc(Eigen::Vector3f& root_acc_cmd) {
  (stop_mode_ == StopPhase::StepStop) ? root_acc_cmd.head(2) = root_acc_cmd_ : root_acc_cmd = root_acc_cmd;
}

void QuickStop::getFootHold(float& px_rel, float& py_rel) {
  if (stop_mode_ == StopPhase::StepStop) {
    px_rel = foot_hold_pos_[0];
    py_rel = foot_hold_pos_[1];
  }
}
}  // namespace locomotion_core
}  // namespace MCC
