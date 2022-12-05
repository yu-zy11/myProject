#include "mc_modules/locomotion/locomotion_core/gait/time_gait_generator.h"
#include "mc_modules/common/algo_config_loader.h"

namespace MCC {
namespace locomotion_core {
TimeGaitGenerator::TimeGaitGenerator(float dt) : GaitGenerator(dt) { leg_states_.resize(4); }

void TimeGaitGenerator::Init() {
  counter_ = 0;
  for (int leg = 0; leg < 4; ++leg) {
    if (current_gait_.counter_offset[leg] <= 0) {
      counter_swing_[leg] = 0;
      counter_stance_[leg] = 0;
    } else if (
        current_gait_.counter_offset[leg] <= current_gait_.counter_period - current_gait_.counter_duration[leg]) {
      counter_swing_[leg] =
          current_gait_.counter_period - current_gait_.counter_offset[leg] - current_gait_.counter_duration[leg];
      counter_stance_[leg] = 0;
    } else {
      counter_swing_[leg] = 0;
      counter_stance_[leg] = current_gait_.counter_period - current_gait_.counter_offset[leg];
    }
  }

  // initialized swing and stance phase
  counter_phase_ = Eigen::Vector4i::Zero();
  counter_mpc_phase_ = Eigen::Vector4i::Zero();
  phase_swing_ = Eigen::Vector4f::Zero();
  phase_stance_ = Eigen::Vector4f::Zero();

  contact_state_scheduled_ = Eigen::Vector4i::Ones();
  contact_state_previous_ = Eigen::Vector4i::Ones();
  touchdown_scheduled_ = Eigen::Vector4i::Zero();
  liftoff_scheduled_ = Eigen::Vector4i::Zero();
}

bool TimeGaitGenerator::Update(const common::message::FusionData& data) {
  counter_++;
  int wraped_counter = counter_ % current_gait_.counter_period;
  for (int leg = 0; leg < 4; ++leg) {
    contact_state_previous_[leg] = contact_state_scheduled_[leg];
    counter_phase_[leg] = wraped_counter - current_gait_.counter_offset[leg];
    if (counter_phase_[leg] < 0) { counter_phase_[leg] += current_gait_.counter_period; }
    // TODO(Xiong Zhilin): fix the bug of the swing phase and stance phase
    // all zero, check whether use < or <=
    if (counter_phase_[leg] < current_gait_.counter_duration[leg]) {
      contact_state_scheduled_[leg] = 1;
      counter_stance_[leg] = counter_phase_[leg];
      counter_swing_[leg] = 0;
      phase_stance_[leg] = counter_stance_[leg] / (float)current_gait_.counter_duration[leg];
      phase_swing_[leg] = 0;
      leg_states_[leg] = LegState::STANCE;
      if (contact_state_previous_[leg] == 0) {
        touchdown_scheduled_[leg] = 1;
      } else {
        touchdown_scheduled_[leg] = 0;
      }
    } else {
      contact_state_scheduled_[leg] = 0;
      counter_swing_[leg] = counter_phase_[leg] - current_gait_.counter_duration[leg];
      counter_stance_[leg] = 0;
      phase_swing_[leg] =
          counter_swing_[leg] / (float)(current_gait_.counter_period - current_gait_.counter_duration[leg]);
      phase_stance_[leg] = 0;
      leg_states_[leg] = LegState::SWING;
      if (contact_state_previous_[leg] == 1) {
        liftoff_scheduled_[leg] = 1;
      } else {
        liftoff_scheduled_[leg] = 0;
      }
    }
  }

  // update MPC contact sequence
  // if (AlgoConfigLoaderInstance.GetAlgoCommonConfig()->use_mpc) { UpdatePredictiveSequence(); }
  UpdatePredictiveSequence();
  // update counter
  // counter_++;

  return true;
}

void TimeGaitGenerator::UpdatePredictiveSequence() {
  int iteration_mpc_ = AlgoConfigLoaderInstance.GetControllerConfig()->iteration_mpc;
  for (int leg = 0; leg < 4; ++leg) {
    if (contact_state_scheduled_[leg] == 1) {
      contact_sequences_[0 + leg] = 1;
    } else {
      contact_sequences_[0 + leg] = 0;
    }
  }
  // std::cout<<"contact_phase_table_:\n";
  for (int i = 1; i < gait_horizon_; i++) {
    int counter_tmp;
    counter_tmp = counter_ + iteration_mpc_ * i;
    int wraped_counter = counter_tmp % current_gait_.counter_period;  // counter in period
    for (int leg = 0; leg < 4; leg++) {
      counter_phase_[leg] = wraped_counter - current_gait_.counter_offset[leg];  // counter for 4leg
      if (counter_phase_[leg] < 0) { counter_phase_[leg] += current_gait_.counter_period; }
      (counter_phase_[leg] <= current_gait_.counter_duration[leg]) ? contact_sequences_[4 * i + leg] = 1
                                                                   : contact_sequences_[4 * i + leg] = 0;
    }
  }
  // std::cout<<"contact_phase_table_:\n";
  // for (int i = 0; i < gait_horizon_; i++) {
  //   std::cout << contact_sequences_[4 * i + 0] << "," << contact_sequences_[4 * i + 1] << ","
  //             << contact_sequences_[4 * i + 2] << "," << contact_sequences_[4 * i + 3] << std::endl;
  // }
}

void TimeGaitGenerator::Reset() {}

void TimeGaitGenerator::PrintGaitInfo() {
  std::cout << "#################### Gait Info ####################" << std::endl;
  std::cout << "Counter: " << counter_ << std::endl;
  std::cout << "Contact States scheduled: " << contact_state_scheduled_.transpose() << std::endl;
  std::cout << "Liftoff State: " << liftoff_scheduled_.transpose() << std::endl;
  std::cout << "Touchdown State: " << touchdown_scheduled_.transpose() << std::endl;
  std::cout << "Stance Counter: " << counter_stance_.transpose() << std::endl;
  std::cout << "Swing Counter: " << counter_swing_.transpose() << std::endl;
  std::cout << "Stance Phase: " << phase_stance_.transpose() << std::endl;
  std::cout << "Swing Phase: " << phase_swing_.transpose() << std::endl;
  std::cout << "---------------------------------------------------" << std::endl;
}
}  // namespace locomotion_core
}  // namespace MCC
