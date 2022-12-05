#include "mc_modules/locomotion/locomotion_core/gait/gait_generator.h"

#include "middleware/logger/px_log.h"

namespace MCC {
namespace locomotion_core {
Gait::Gait() {
  type = GaitType::STAND;
  period = 1.0;
  stance_offset << 0., 0., 0., 0.;
  stance_duration << 1., 1., 1., 1.;
}

GaitGenerator::GaitGenerator(float dt) : dt_(dt) {
  CreateGait(GaitType::TROT);
  // current_gait_ = next_gait_;
  // PrintGaitInfo();
}

void GaitGenerator::CreateGait(GaitType type, float period) {
  next_gait_.type = type;
  switch (type) {
    case GaitType::TROT:
      next_gait_.name = "TROT";
      next_gait_.period = 0.66;
      next_gait_.stance_offset << 0, 0.5, 0.5, 0;
      next_gait_.stance_duration << 0.5, 0.5, 0.5, 0.5;
      break;

    case GaitType::TROT_WALK:
      next_gait_.name = "TROT_WALK";
      next_gait_.period = 0.72;
      next_gait_.stance_offset << 0, 0.5, 0.5, 0;
      next_gait_.stance_duration << 0.6, 0.6, 0.6, 0.6;
      break;

    case GaitType::TROT_RUN:
      next_gait_.name = "TROT_RUN";
      next_gait_.period = 0.4;
      next_gait_.stance_offset << 0, 0.5, 0.5, 0;
      next_gait_.stance_duration << 0.375, 0.375, 0.375, 0.375;
      break;

    case GaitType::WALK:
      next_gait_.name = "WALK";
      next_gait_.period = 0.96;
      next_gait_.stance_offset << 0.75, 0.25, 0.5, 0.0;
      next_gait_.stance_duration << 0.75, 0.75, 0.75, 0.75;
      break;

    case GaitType::STAND:
      next_gait_.name = "STAND";
      next_gait_.period = 1.0;
      next_gait_.stance_offset << 0, 0, 0, 0;
      next_gait_.stance_duration << 1, 1, 1, 1;
      break;

    case GaitType::BOUND:
      next_gait_.name = "BOUND";
      next_gait_.period = 0.4;
      next_gait_.stance_offset << 0.5, 0.5, 0, 0;
      next_gait_.stance_duration << 0.5, 0.5, 0.5, 0.5;
      break;

    case GaitType::PACE:
      next_gait_.name = "PACE";
      next_gait_.period = 0.4;
      next_gait_.stance_offset << 0.6, 0, 0.6, 0;
      next_gait_.stance_duration << 0.6, 0.6, 0.6, 0.6;
      break;

    case GaitType::PRONK:
      next_gait_.name = "PRONK";
      next_gait_.period = 0.5;
      next_gait_.stance_offset << 0, 0, 0, 0;
      next_gait_.stance_duration << 0.4, 0.4, 0.4, 0.4;
      break;

    case GaitType::GALLOP:
      next_gait_.name = "GALLOP";
      next_gait_.period = 0.4;
      next_gait_.stance_offset << 0, 0.3, 0.6, 0.9;
      next_gait_.stance_duration << 0.4, 0.4, 0.4, 0.4;
      break;

    default:
      std::cerr << "INVALID GAIT TYPE. " << std::endl;
      break;
  }
  if (period > 0.3 && period < 2.0 && fabs(next_gait_.period - period) > 0.002) {
    next_gait_.period = period;
    LOGF_DEBUG("Adjust gait period to %f", period);
  }
  next_gait_.counter_period = ceil(next_gait_.period / dt_);
  for (int leg = 0; leg < 4; ++leg) {
    next_gait_.counter_offset[leg] = next_gait_.counter_period * next_gait_.stance_offset[leg];
    next_gait_.counter_duration[leg] = next_gait_.counter_period * next_gait_.stance_duration[leg];
  }
}

void GaitGenerator::Transition() {
  // TODO(Xiong Zhilin): gait transition
  current_gait_ = next_gait_;
}

void GaitGenerator::SetMPCRate(int mpc_rate) {
  use_mpc_ = true;
  mpc_rate_ = mpc_rate;
  gait_horizon_ = current_gait_.counter_period / mpc_rate_;
  if (contact_sequences_ != nullptr) { delete[] contact_sequences_; }
  contact_sequences_ = new int[gait_horizon_ * 4];
}

int* GaitGenerator::GetMPCTable() { return contact_sequences_; }

void GaitGenerator::PrintGaitInfo() {
  std::cout << "#################### Gait Type ####################" << std::endl;
  std::cout << "Current gait name: " << current_gait_.name << std::endl;
  std::cout << "Gait period: " << current_gait_.period << std::endl;
  std::cout << "Gait stance offset: " << current_gait_.stance_offset.transpose() << std::endl;
  std::cout << "Gait stance duration: " << current_gait_.stance_duration.transpose() << std::endl;
  std::cout << "Gait period counter: " << current_gait_.counter_period << std::endl;
  std::cout << "Gait stance offset counter: " << current_gait_.counter_offset.transpose() << std::endl;
  std::cout << "Gait stance duration counter: " << current_gait_.counter_duration.transpose() << std::endl;
  std::cout << "---------------------------------------------------" << std::endl;
}
}  // namespace locomotion_core
}  // namespace MCC
