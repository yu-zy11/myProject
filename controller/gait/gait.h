#ifndef GAIT_H
#define GAIT_H

#include <Eigen/Dense>
#include <iostream>
// #include <string>
// #include <vector>
enum class GaitType {
  TROT,
  TROT_WALK,
  TROT_RUN,
  WALK,
  STAND,
  BOUND,
  PACE,
  PRONK,
  GALLOP
};
class GaitData {
public:
  Eigen::Matrix4f phase_offset_;
  Eigen::Matrix4f stance_duration_;
  float period_;
  GaitType gait_type_;
};

class Gait {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  void init();
  void update();
  void setGaitType(GaitType type, float period = -1);
  Eigen::Vector4f GetStancePhase();
  Eigen::Vector4f GetSwingPhase();
  void printGaitInfo();
  // funtions for mpc
  void initMPC(int horizon, int iteration);
  void getMPCtable(Eigen::Matrix<int, 4, -1> &contact_table);
  float getStanceDuration() { return stance_duration_; };

private:
  // GaitType gait_type_;
  float gait_phase_;
  int counter_; // period counter
  float dt_;
  float stance_duration_;
  Eigen::Vector4f phase_offset_;
  Eigen::Vector4f swing_phase_;
  Eigen::Vector4f stance_phase_;
  Eigen::Vector4i foot_contact_;
  GaitData current_gait_;
  GaitData next_gait_;
  void transition();
  // parameters for mpc
  int horizon_mpc_;
  int iteration_mpc_;
  Eigen::Matrix<int, 4, -1> contact_sequences_;
};
#endif /* GAIT_H */
