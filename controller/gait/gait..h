#ifndef GAIT_H
#define GAIT_H

#include <Eigen/Dense>
#include <iostream>
// #include <string>
// #include <vector>
enum class GaitType { TROT, TROT_WALK, TROT_RUN, WALK, STAND, BOUND, PACE, PRONK, GALLOP };
class GaitData
{
    Eigen::Matrix4f stance_offset_;
    Eigen::Matrix4f stance_duration;
    float period_;
};

enum class LegState {
  STANCE,
  STANCE_DONE,
  SWING,
  LOSE_CONTACT  // TODO
};

struct Gait {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  Gait();
  void init();
  void update();
  void CreateGait(GaitType type, float period = -1);
  Eigen::Vector4f GetStancePhase();
  Eigen::Vector4f GetSwingPhase() ;
private:
  GaitType type_;
  float gait_period_; 
  float gait_phase_;
  int counter_period_;  // period counter
  Eigen::Vector4f phase_offset_;  // normalized phase offset
  Eigen::Vector4f stance_duration_;  // normalized stance phase duration
  GaitData current_gait_, next_gait_;

  void transition();
};

class GaitGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GaitGenerator(float dt);

  virtual ~GaitGenerator() = default;

  virtual void Init() = 0;

  virtual bool Update(const common::message::FusionData& data = common::message::FusionData()) = 0;

  virtual void Reset() = 0;

  virtual void Transition();

  void CreateGait(GaitType type, float period = -1);

  Eigen::Vector4i IsLiftoff() { return liftoff_scheduled_; }

  Eigen::Vector4i IsTouchdown() { return touchdown_scheduled_; }

  Eigen::Vector4f GetStancePhase() { return phase_stance_; }

  Eigen::Vector4f GetSwingPhase() { return phase_swing_; }

  Eigen::Vector4f GetStanceTime() { return current_gait_.period * current_gait_.stance_duration; }

  Eigen::Vector4f GetSwingTime() {
    return current_gait_.period * (Eigen::Vector4f::Ones() - current_gait_.stance_duration);
  }

  Eigen::Vector4i GetContactTarget() { return contact_state_scheduled_; }

  std::vector<LegState> GetLegStates() { return leg_states_; }

  GaitType GetGaitType() { return current_gait_.type; }

  void SetMPCRate(int mpc_rate);

  int* GetMPCTable();

  virtual void PrintGaitInfo();

 protected:
  // control period
  float dt_;

  // current and next gait
  Gait current_gait_;
  Gait next_gait_;

  // current leg state
  std::vector<LegState> leg_states_;

  // gait variables
  Eigen::Vector4f phase_stance_;
  Eigen::Vector4f phase_swing_;
  Eigen::Vector4i contact_state_scheduled_;
  Eigen::Vector4i contact_state_previous_;
  Eigen::Vector4i liftoff_scheduled_;
  Eigen::Vector4i touchdown_scheduled_;

  // MPC contact sequence
  bool use_mpc_ = false;
  int mpc_rate_;
  int gait_horizon_;
  int* contact_sequences_ = nullptr;
};
#endif /* GAIT_H */
