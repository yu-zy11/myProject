#include "swing_leg_planner.h"
#include "./controller/common_data/common_data.h"
SwingLegPlanner::SwingLegPlanner() {
  kf_foot_x_ = 0.1;
  kf_foot_y_ = 0.1;

  kd_foot_x_ = 0.1;
  kd_foot_y_ = 0.1;
  com_offset_.setZero();
}

void SwingLegPlanner::Init() {}

bool SwingLegPlanner::Update(
    const common::message::FusionData &fusion_data,
    const common::message::OperatorData &operator_data) {
  // update parameters from control parameter interface
  auto parameters = AlgoConfigLoaderInstance.GetControllerConfig();
  kf_foot_x_ = parameters->kf_foot_x;
  kf_foot_y_ = parameters->kf_foot_y;

  kd_foot_x_ = parameters->kd_foot_x;
  kd_foot_y_ = parameters->kd_foot_y;

  com_offset_ << parameters->com_offset[0], parameters->com_offset[1],
      parameters->com_offset[2];

  // root velocity w.r.t the horizontal frame
  Eigen::Matrix3f root_mat_z = ori::coordinateRotation(
      ori::CoordinateAxis::Z, fusion_data.root_euler[2]);
  Eigen::Vector3f root_lin_vel_rel =
      root_mat_z * fusion_data.root_linear_velocity_in_world;
  Eigen::Vector3f delta_foot = Eigen::Vector3f::Zero();
  Eigen::Vector3f track_error = Eigen::Vector3f::Zero();
  delta_foot[0] = kf_foot_x_ * root_lin_vel_rel[0];
  delta_foot[1] = kf_foot_y_ * root_lin_vel_rel[1];

  // TODO(Xiong Zhilin): post-process the operator data in an unified way
  Eigen::Vector3f root_lin_vel_target = Eigen::Vector3f::Zero();
  root_lin_vel_target << operator_data.root_linear_velocity_in_body[0],
      operator_data.root_linear_velocity_in_body[1], 0;
  // clamp the velocity
  root_lin_vel_target[0] =
      coerce(root_lin_vel_target[0], -(float)parameters->velocity_limit[0],
             (float)parameters->velocity_limit[0]);
  root_lin_vel_target[1] =
      coerce(root_lin_vel_target[1], -(float)parameters->velocity_limit[1],
             (float)parameters->velocity_limit[1]);

  track_error[0] = kd_foot_x_ * (root_lin_vel_rel[0] - root_lin_vel_target[0]);
  track_error[1] = kd_foot_y_ * (root_lin_vel_rel[1] - root_lin_vel_target[1]);

  // foot placement
  float side_sign[4] = {-1, 1, -1, 1};
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::Vector3f p_hip((leg == 0 || leg == 1) ? parameters->bodyLength
                                                 : -parameters->bodyLength,
                          (leg == 1 || leg == 3) ? parameters->bodyWidth
                                                 : -parameters->bodyWidth,
                          0);
    // TODO(Xiong Zhilin): default foot position w.r.t the body frame,
    // suppose to be the horizontal frame
    Eigen::Vector3f hip_offset(0, side_sign[leg] * parameters->side_sign_offset,
                               0);
    foot_pos_rel_target_.col(leg) = p_hip + hip_offset + com_offset_;
    if (AlgoConfigLoaderInstance.GetControllerConfig()->noise_reduction) {
      foot_pos_rel_target_.col(leg) +=
          Eigen::Vector3f(0., 0., -parameters->body_height + clearance_);
    } else {
      foot_pos_rel_target_.col(leg) += Eigen::Vector3f(
          0., 0., -parameters->body_height + parameters->touchdown_clearance);
    }
    // simple Raibert heuristic
    foot_pos_rel_target_.col(leg) += delta_foot;
    foot_pos_rel_target_.col(leg) += track_error;
  }
  return true;
}
