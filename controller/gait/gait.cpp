#include "gait.h"

void Gait::init() {
  gait_type_ = GaitType::STAND;
  current_gait_.period_ = 1.0;
  current_gait_.phase_offset_ << 0., 0., 0., 0.;
  current_gait_.stance_duration_ << 1., 1., 1., 1.;
  counter_ = 0;
  dt_ = 0.002;
  gait_phase_ = 0.0;
  foot_contact_ << 1, 1, 1, 1;
}

void Gait::setGaitType(GaitType type, float period) {
  // gait_type_ = type;
  next_gait_.gait_type_ = type;
  switch (type) {
  case GaitType::TROT:
    next_gait_.period_ = 0.66;
    next_gait_.phase_offset_ << 0, 0.5, 0.5, 0;
    next_gait_.stance_duration_ << 0.5, 0.5, 0.5, 0.5;
    break;
  case GaitType::TROT_WALK:
    next_gait_.period_ = 0.72;
    next_gait_.phase_offset_ << 0, 0.5, 0.5, 0;
    next_gait_.stance_duration_ << 0.6, 0.6, 0.6, 0.6;
    break;
  case GaitType::TROT_RUN:
    next_gait_.period_ = 0.4;
    next_gait_.phase_offset_ << 0, 0.5, 0.5, 0;
    next_gait_.stance_duration_ << 0.375, 0.375, 0.375, 0.375;
    break;
  case GaitType::WALK:
    next_gait_.period_ = 0.96;
    next_gait_.phase_offset_ << 0.75, 0.25, 0.5, 0.0;
    next_gait_.stance_duration_ << 0.75, 0.75, 0.75, 0.75;
    break;
  case GaitType::STAND:
    next_gait_.period_ = 1.0;
    next_gait_.phase_offset_ << 0, 0, 0, 0;
    next_gait_.stance_duration_ << 1, 1, 1, 1;
    break;
  case GaitType::BOUND:
    next_gait_.period_ = 0.4;
    next_gait_.phase_offset_ << 0.5, 0.5, 0, 0;
    next_gait_.stance_duration_ << 0.5, 0.5, 0.5, 0.5;
    break;
  case GaitType::PACE:
    next_gait_.period_ = 0.4;
    next_gait_.phase_offset_ << 0.6, 0, 0.6, 0;
    next_gait_.stance_duration_ << 0.6, 0.6, 0.6, 0.6;
    break;
  case GaitType::PRONK:
    next_gait_.period_ = 0.5;
    next_gait_.phase_offset_ << 0, 0, 0, 0;
    next_gait_.stance_duration_ << 0.4, 0.4, 0.4, 0.4;
    break;
  case GaitType::GALLOP:
    next_gait_.period_ = 0.4;
    next_gait_.phase_offset_ << 0, 0.3, 0.6, 0.9;
    next_gait_.stance_duration_ << 0.4, 0.4, 0.4, 0.4;
    break;
  default:
    std::cerr << "INVALID GAIT TYPE. " << std::endl;
    break;
  }
  next_gait_.period_ = period;
}

void Gait::update() {
  transition();
  gait_phase_ += dt_ / current_gait_.period_;
  (gait_phase_ >= 1.0) ? gait_phase_ -= 1.0 : gait_phase_ -= 0.0;

  for (int leg = 0; leg < 4; ++leg) {
    float leg_phase = gait_phase_ + current_gait_.phase_offset_[leg];
    if (leg_phase >= 1.0) {
      leg_phase -= 1.0;
    }
    if (leg_phase > current_gait_.stance_duration_[leg]) {
      foot_contact_[leg] = 0;
      swing_phase_[leg] = leg_phase - current_gait_.stance_duration_[leg];
      stance_phase_[leg] = 0.0;
    } else {
      foot_contact_[leg] = 1;
      swing_phase_[leg] = 0.0;
      stance_phase_[leg] = leg_phase;
    }
  }
  stance_duration_ =
      current_gait_.period_ * current_gait_.stance_duration_.sum() / 4.0;
}

void Gait::transition() {
  // period transition
  constexpr float period_delta{0.002};
  if (current_gait_.period_ - next_gait_.period_ > period_delta) {
    current_gait_.period_ -= period_delta;
  }
  if (current_gait_.period_ - next_gait_.period_ < -period_delta) {
    current_gait_.period_ += period_delta;
  }
  // stance_offset and stance_duration transition
  constexpr float offset_delta{0.001};
  constexpr float duration_delta{0.002};
  for (int i = 0; i < 4; i++) {
    if (current_gait_.phase_offset_[i] - next_gait_.phase_offset_[i] >
        offset_delta) {
      current_gait_.phase_offset_[i] -= offset_delta;
    }
    if (current_gait_.phase_offset_[i] - next_gait_.phase_offset_[i] <
        -offset_delta) {
      current_gait_.phase_offset_[i] += offset_delta;
    }

    if (current_gait_.stance_duration_[i] - next_gait_.stance_duration_[i] >
        duration_delta) {
      current_gait_.stance_duration_[i] -= duration_delta;
    }
    if (current_gait_.stance_duration_[i] - next_gait_.stance_duration_[i] <
        -duration_delta) {
      current_gait_.stance_duration_[i] += duration_delta;
    }
  }
  // transition to stand:if leg is touchdown,then keep leg still touchdown
  if (next_gait_.gait_type_ == GaitType::STAND) {
    for (int leg = 0; leg < 4; leg++) {
      if (swing_phase_[leg] <= 0.0001) {
        swing_phase_[leg] = 0;
        stance_phase_[leg] = 1;
      }
    }
  }
  //
}

void Gait::initMPC(int horizon, int iteration) {
  horizon_mpc_ = horizon;
  iteration_mpc_ = iteration;
  contact_sequences_.resize(4, horizon);
  contact_sequences_.setZero();
}

void Gait::getMPCtable(Eigen::Matrix<int, 4, -1> &contact_table) {
  float gait_phase_tmp = gait_phase_;
  contact_sequences_.block(0, 0, 4, 1) = foot_contact_;
  for (int i = 1; i < horizon_mpc_; i++) {
    gait_phase_tmp += dt_ * iteration_mpc_ / current_gait_.period_;
    (gait_phase_tmp >= 1) ? gait_phase_tmp -= 1 : gait_phase_tmp -= 0;

    for (int leg = 0; leg < 4; ++leg) {
      float leg_phase = gait_phase_tmp + current_gait_.phase_offset_[leg];
      (leg_phase >= 1.0) ? leg_phase -= 1.0 : leg_phase -= 0.0;
      contact_sequences_(leg, i) = 1;
      if (leg_phase > current_gait_.stance_duration_[leg]) {
        contact_sequences_(leg, i) = 0;
      }
    }
  }
  contact_table.resize(4, horizon_mpc_);
  contact_table = contact_sequences_;
}

void Gait::printGaitInfo() {
  std::cout << "================= Gait info ==================\n";
  std::cout << "Gait period: " << current_gait_.period_ << std::endl;
  std::cout << "Gait phase offset:\n "
            << current_gait_.phase_offset_.transpose() << std::endl;
  std::cout << "Gait stance phase:\n "
            << current_gait_.stance_duration_.transpose() << std::endl;
  std::cout << "Gait phase: " << gait_phase_ << std::endl;
  std::cout << "Gait stance duration counter: " << stance_duration_
            << std::endl;
  std::cout << "================= Gait info end ==================\n";
}
