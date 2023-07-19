#ifndef QUICK_STOP_H
#define QUICK_STOP_H
#include <eigen3/Eigen/Dense>
#include "modules/common/message/common_data.h"
namespace MCC {
namespace locomotion_core {
class QuickStop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void init(const float& dt, const float& period, const float& height, const float& step_length);
  void run();
  void updateData(
      const common::message::FusionData& fu,
      const common::message::OperatorData& op,
      const Eigen::Vector4i& contact_target,
      const float& period,
      const bool& stop);
  //   void getTargetCmd(Eigen::Vector3f& root_pos_cmd, Eigen::Vector3f& root_vel_cmd, Eigen::Vector3f& root_acc_cmd);
  void getTargetPos(Eigen::Vector3f& root_pos_cmd);
  void getTargetVel(Eigen::Vector3f& root_vel_cmd);
  void getTargetAcc(Eigen::Vector3f& root_acc_cmd);
  void getFootHold(float& px_rel, float& py_rel);
  bool stopFinished() { return stop_finished_; }
  void getTargetPosDes(Eigen::Vector3f& root_pos_des) { root_pos_des.head(2) = root_pos_des_; };
  int getStopMode() { return stop_mode_; };

 private:
  enum StopPhase { TrackVel, SlowDown, SlowDownFinished, StepStop };
  // config
  float control_step_;
  float gait_period_;
  float body_height_;
  float max_step_length_;
  float vel_limit_;
  bool stop_signal_;
  // data
  int stop_mode_;
  // int stop_mode_last_;
  Eigen::Vector3f root_vel_cmd_world_;
  Eigen::Vector2f root_velxy_, root_velxy_cmd_, root_velxy_cmd0_;
  Eigen::Vector2f root_omega_cmd_;
  Eigen::Vector2f root_pos_cmd_;
  Eigen::Vector2f root_pos_;
  Eigen::Vector2f root_pos_world_start_;
  Eigen::Vector2f root_vel_cmd_;
  Eigen::Vector2f root_acc_cmd_;
  Eigen::Vector2f foot_hold_pos_;
  Eigen::Vector4i contact_target_;
  Eigen::Matrix<float, 4, 3> foot_pos_world_;
  bool quick_stop_{0};
  bool stop_finished_{0};
  // data for lip
  bool first_run_StepStop_{0};
  float step_stop_duration_{0};
  Eigen::Vector2f pos0_, vel0_;
  Eigen::Vector2f root_pos_des_;
  float time_lip_, Tc_;
};

}  // namespace locomotion_core
}  // namespace MCC

#endif
