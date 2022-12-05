#ifndef TIME_GAIT_GENERATOR_H
#define TIME_GAIT_GENERATOR_H

#include "mc_modules/locomotion/locomotion_core/gait/gait_generator.h"

namespace MCC {
namespace locomotion_core {
class TimeGaitGenerator : public GaitGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TimeGaitGenerator(float dt);

  virtual ~TimeGaitGenerator() = default;

  virtual void Init() override;

  virtual bool Update(const common::message::FusionData& data = common::message::FusionData()) override;

  virtual void Reset() override;

  virtual void PrintGaitInfo() override;

 private:
  // swing and stance counter
  int counter_;
  Eigen::Vector4i counter_phase_;
  Eigen::Vector4i counter_mpc_phase_;
  Eigen::Vector4i counter_stance_;
  Eigen::Vector4i counter_swing_;

  void UpdatePredictiveSequence();
};
}  // namespace locomotion_core
}  // namespace MCC
#endif /* TIME_GAIT_GENERATOR_H */
