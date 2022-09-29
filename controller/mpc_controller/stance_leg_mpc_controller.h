#ifndef PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_STANCE_LEG_MPC_CONTROLLER_H_
#define PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_STANCE_LEG_MPC_CONTROLLER_H_

#include "modules/locomotion/locomotion_core/utils/instant_qp.h"
#include "modules/locomotion/locomotion_core/utils/leg_controller.h"

namespace MCC {
namespace locomotion_core {
class StanceLegMPCController : public LegController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StanceLegMPCController(GaitGenerator* gait);

  virtual ~StanceLegMPCController();

  virtual void Init(const common::message::FusionData& fusion_data) override;

  virtual bool Update(
      const common::message::FusionData& fusion_data, const common::message::OperatorData& operator_data) override;

  virtual void Reset() override;

 private:
  float grf_weight_;
  Eigen::Vector3f acc_weight_lin_;
  Eigen::Vector3f acc_weight_ang_;
  Eigen::Vector3f kp_root_lin_;
  Eigen::Vector3f kd_root_lin_;
  Eigen::Vector3f kp_root_ang_;
  Eigen::Vector3f kd_root_ang_;

  Eigen::Vector3f root_pos_target_;
  Eigen::Vector3f root_pos_;
  Eigen::Vector3f root_lin_vel_target_;
  Eigen::Vector3f root_lin_vel_rel_;
  Eigen::Vector3f root_euler_target_;
  Eigen::Vector3f root_euler_;
  Eigen::Vector3f root_ang_vel_target_;
  Eigen::Vector3f root_ang_vel_rel_;
  Eigen::Vector3f com_offset_;

  Eigen::Vector3f root_pos_target_init_;
  Eigen::Vector3f root_euler_target_init_;

  Eigen::Vector3f root_linear_velocity_offset_;
  Eigen::Vector3f root_angular_velocity_offset_;

  InstantQP instant_qp_;
  Eigen::Matrix<float, NUM_VARIABLES, 1> grf_;

  float traj_all_[12 * 36] = {0.0};
  float x_vel_des_ = 0.0;
  float y_vel_des_ = 0.0;
  float yaw_turn_rate_ = 0.0;
  float yaw_des_ = 0.0;
  bool first_run_ = true;
  int iterationCounter = 0;
  Eigen::Vector3f world_position_desired_;

  Eigen::Matrix3f Skew(const Eigen::Vector3f& v);
};
}  // namespace locomotion_core
}  // namespace MCC
#endif  // PX_QUADRUPED_MODULES_LOCOMOTION_LOCOMOTION_CORE_CONTROLLER_STANCE_LEG_MPC_CONTROLLER_H_
