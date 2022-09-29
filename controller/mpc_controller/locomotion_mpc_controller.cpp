#include "modules/locomotion/locomotion_core/controller/mpc_controller/locomotion_mpc_controller.h"

#include "modules/common/config_loader.h"
#include "modules/common/math/orientation_tools.h"

namespace MCC {
namespace locomotion_core {
LocomotionMPCController::LocomotionMPCController(MCC::common::robot::RobotStates state) : BasedController(state) {
  float dt = ConfigLoaderInstance.getCommonConfig()->default_cycle_time_;
  if (ConfigLoaderInstance.getRobotConfig()->event_based) {
    gait_generator_ = new EventGaitGenerator(dt);
  } else {
    gait_generator_ = new TimeGaitGenerator(dt);
  }
  footstep_planner_ = new FootstepPlanner();
  swing_trajectory_planner_ = new SwingTrajectoryPlanner(gait_generator_);
  swing_leg_controller_ = new SwingLegController(gait_generator_);
  stance_leg_mpc_controller_ = new StanceLegMPCController(gait_generator_);
  nosiereduction_ = new NosieReduction();
}

LocomotionMPCController::~LocomotionMPCController() {
  delete gait_generator_;
  delete footstep_planner_;
  delete swing_trajectory_planner_;
  delete swing_leg_controller_;
  delete stance_leg_mpc_controller_;
  delete nosiereduction_;
}

void LocomotionMPCController::Init(const common::message::FusionData& fusion_data) {
  SetGaitType(GaitType::TROT);
  gait_generator_->Init();
  gait_generator_->SetMPCRate(ConfigLoaderInstance.getRobotConfig()->iteration_mpc);// counter_period/iteration_mpc
  footstep_planner_->Init();
  Eigen::Matrix<float, 3, 4> foot_position = Eigen::Matrix<float, 3, 4>::Zero();
  Eigen::Matrix3f root_mat_z = ori::coordinateRotation(ori::CoordinateAxis::Z, fusion_data.root_euler[2]);
  Eigen::Matrix<float, 3, 4> foot_pos_abs =
      fusion_data.rotation_matrix_body_to_world * fusion_data.foot_position_in_body.transpose();
  Eigen::Matrix<float, 3, 4> foot_pos_cur = root_mat_z * foot_pos_abs;//????root_mat_z.transpose()
  swing_trajectory_planner_->Init(foot_pos_cur);
  swing_leg_controller_->Init(fusion_data);
  stance_leg_mpc_controller_->Init(fusion_data);
  nosiereduction_->Init();
}

common::message::LocomotionData LocomotionMPCController::Run(
    const common::message::FusionData& fusion_data, const common::message::OperatorData& operator_data) {
  gait_generator_->Update(fusion_data);
  // gait_generator_->PrintGaitInfo();
  nosiereduction_->Update(fusion_data, operator_data);
  footstep_planner_->GetTargetClearance(nosiereduction_->SetTargetClearance());
  footstep_planner_->Update(fusion_data, operator_data);
  swing_trajectory_planner_->Update(fusion_data, footstep_planner_->Footholds());
  swing_leg_controller_->GetTouchForce(nosiereduction_->SetTouchForce());
  swing_leg_controller_->SetTargetFootPosition(swing_trajectory_planner_->TargetFootPosition());
  swing_leg_controller_->SetTargetFootVelocity(swing_trajectory_planner_->TargetFootVelocity());
  swing_leg_controller_->Update(fusion_data, operator_data);
  stance_leg_mpc_controller_->Update(fusion_data, operator_data);
  Eigen::Matrix<float, 12, 1> joint_torque_target = Eigen::Matrix<float, 12, 1>::Zero();
  for (int leg = 0; leg < 4; ++leg) {
    if (gait_generator_->GetContactTarget()[leg]) {
      joint_torque_target.segment<3>(3 * leg) = stance_leg_mpc_controller_->TargetJointTorque().segment<3>(3 * leg);
    } else {
      joint_torque_target.segment<3>(3 * leg) = swing_leg_controller_->TargetJointTorque().segment<3>(3 * leg);
      if (leg == 0 || leg == 2) {
        joint_torque_target(3 * leg) -= 0.8;
      } else {
        joint_torque_target(3 * leg) += 0.8;
      }
    }
  }

  Vec3<float> kd_joint = Vec3<float>::Zero();
  kd_joint[0] = ConfigLoaderInstance.getRobotConfig()->Kd_joint[0];
  kd_joint[1] = ConfigLoaderInstance.getRobotConfig()->Kd_joint[1];
  kd_joint[2] = ConfigLoaderInstance.getRobotConfig()->Kd_joint[2];

  common::message::LocomotionData locomotion_command;
  locomotion_command.joint_torque = joint_torque_target;
  locomotion_command.joint_damping << kd_joint, kd_joint, kd_joint, kd_joint;

  return locomotion_command;
}

void LocomotionMPCController::SetGaitType(GaitType type) {
  gait_generator_->CreateGait(type, ConfigLoaderInstance.getRobotConfig()->gait_period);
  // TODO(Xiong Zhilin): add smooth gait transition
  // if((gait_generator_->GetContactTarget().array() == 1).all())
  gait_generator_->Transition();
}

MCC::common::message::GaitInfo LocomotionMPCController::UpdateGaitInfo() {
  MCC::common::message::GaitInfo gait_info;
  gait_info.contact_target = gait_generator_->GetStancePhase();
  gait_info.swing_phase = gait_generator_->GetSwingPhase();
  gait_info.stance_phase = gait_generator_->GetStancePhase();
  return gait_info;
}

bool LocomotionMPCController::Safe() {
  for (int leg = 0; leg < 4; ++leg) {
    if (gait_generator_->GetLegStates()[leg] == LegState::SWING) {
      if (ConfigLoaderInstance.getRobotConfig()->event_based) {
        return false;
      } else {
        if (gait_generator_->GetSwingPhase()[leg] < 0.99) { return false; }
      }
    }
  }
  return true;
}
}  // namespace locomotion_core
}  // namespace MCC
