#include "modules/locomotion/locomotion_core/controller/mpc_controller/stance_leg_mpc_controller.h"

#include "modules/common/config_loader.h"
#include "modules/locomotion/locomotion_core/controller/mpc_controller/mpc_interface.h"

#include "modules/common/math//common_util.h"
#include "modules/common/math/orientation_tools.h"

namespace MCC {
namespace locomotion_core {
StanceLegMPCController::StanceLegMPCController(GaitGenerator* gait) : LegController(gait) {
  kp_root_lin_ << 0.0, 0.0, 5000.0;
  kd_root_lin_ << 500.0, 500.0, 500.0;
  kp_root_ang_ << 400.0, 400.0, 0.0;
  kd_root_ang_ << 10.0, 10.0, 20.0;

  root_pos_target_.setZero();
  root_pos_.setZero();
  root_lin_vel_target_.setZero();
  root_lin_vel_rel_.setZero();
  root_euler_target_.setZero();
  root_euler_.setZero();
  root_ang_vel_target_.setZero();
  root_ang_vel_rel_.setZero();
  com_offset_.setZero();

  root_pos_target_init_.setZero();
  root_linear_velocity_offset_.setZero();
  root_angular_velocity_offset_.setZero();
  root_euler_target_init_.setZero();

  grf_.setZero();
}

StanceLegMPCController::~StanceLegMPCController() {}

void StanceLegMPCController::Init(const common::message::FusionData& fusion_data) {
  foot_pos_target_.setZero();
  joint_torque_target_.setZero();
  root_pos_target_init_ = fusion_data.root_position;
  root_euler_target_init_ = fusion_data.root_euler;
  root_pos_target_ = root_pos_target_init_;
  root_euler_target_ = root_euler_target_init_;
}

bool StanceLegMPCController::Update(
    const common::message::FusionData& fusion_data, const common::message::OperatorData& operator_data) {
  // root states and target
  auto parameters = ConfigLoaderInstance.getRobotConfig();
  kp_root_lin_ << parameters->kp_root_lin[0], parameters->kp_root_lin[1], parameters->kp_root_lin[2];
  kd_root_lin_ << parameters->kd_root_lin[0], parameters->kd_root_lin[1], parameters->kd_root_lin[2];
  kp_root_ang_ << parameters->kp_root_ang[0], parameters->kp_root_ang[1], parameters->kp_root_ang[2];
  kd_root_ang_ << parameters->kd_root_ang[0], parameters->kd_root_ang[1], parameters->kd_root_ang[2];

  com_offset_ << parameters->com_offset[0], parameters->com_offset[1], parameters->com_offset[2];
  acc_weight_lin_ << parameters->acc_weight_lin[0], parameters->acc_weight_lin[1], parameters->acc_weight_lin[2];
  acc_weight_ang_ << parameters->acc_weight_ang[0], parameters->acc_weight_ang[1], parameters->acc_weight_ang[2];
  grf_weight_ = parameters->grf_weight;

  instant_qp_.SetAccWeight(acc_weight_lin_, acc_weight_ang_);
  instant_qp_.SetGRFWeight(grf_weight_);

  Eigen::Matrix3f root_mat_z = ori::coordinateRotation(ori::CoordinateAxis::Z, fusion_data.root_euler[2]);
  root_pos_ = fusion_data.root_position;
  root_lin_vel_rel_ = root_mat_z * fusion_data.root_linear_velocity_in_world;
  root_euler_ = fusion_data.root_euler;
  root_ang_vel_rel_ = fusion_data.root_angular_velocity_in_body;

  // TODO(Xiong Zhilin): no planning for the body horizontal position, use D
  // gain only
  // root_pos_target_ = root_pos_;
  // root_pos_target_[2] = parameters->body_height;
  root_lin_vel_target_ << operator_data.root_linear_velocity_in_body[0], operator_data.root_linear_velocity_in_body[1],
      0;
  // TODO(Xiong Zhilin): add pose target
  root_euler_target_ << 0., 0., 0.;
  root_ang_vel_target_ << 0., 0., operator_data.root_angular_velocity_in_body[2];

  if (gait_->GetGaitType() == GaitType::STAND) {
    kp_root_lin_ << parameters->kp_root_lin_stand[0], parameters->kp_root_lin_stand[1],
        parameters->kp_root_lin_stand[2];
    kd_root_lin_ << parameters->kd_root_lin_stand[0], parameters->kd_root_lin_stand[1],
        parameters->kd_root_lin_stand[2];
    kp_root_ang_[2] = 10;
    root_pos_target_ = root_pos_target_init_;
    root_euler_target_ = root_euler_target_init_;
    root_lin_vel_target_.setZero();
    root_ang_vel_target_.setZero();
    root_linear_velocity_offset_.setZero();
    root_angular_velocity_offset_.setZero();
  } else if (gait_->GetGaitType() == GaitType::TROT) {
    root_linear_velocity_offset_ << parameters->root_linear_velocity_offset[0],
        parameters->root_linear_velocity_offset[1], parameters->root_linear_velocity_offset[2];
    root_angular_velocity_offset_ << parameters->root_angular_velocity_offset[0],
        parameters->root_angular_velocity_offset[1], parameters->root_angular_velocity_offset[2];
    if (root_lin_vel_target_.head(2).norm() > 0.05) {
      root_pos_target_ = root_pos_;
      root_pos_target_[2] = parameters->body_height;
      // root_euler_target_[2] = root_euler_[2];
    } else {
      kp_root_lin_ << parameters->kp_root_lin_stand[0], parameters->kp_root_lin_stand[1],
          parameters->kp_root_lin_stand[2];
      kd_root_lin_ << parameters->kd_root_lin_stand[0], parameters->kd_root_lin_stand[1],
          parameters->kd_root_lin_stand[2];
      // kp_root_ang_[2] = 10;
      root_pos_target_[2] = parameters->body_height;
    }
  }
  root_lin_vel_target_ += root_linear_velocity_offset_;
  root_ang_vel_target_ += root_angular_velocity_offset_;

  // get cmd vel
  float dt = ConfigLoaderInstance.getCommonConfig()->default_cycle_time_;
  Eigen::Vector3f v_des_robot;
  Eigen::Vector3f v_des_world;

  x_vel_des_ = x_vel_des_ * (1 - parameters->x_filter) + root_lin_vel_target_[0] * parameters->x_filter;
  y_vel_des_ = y_vel_des_ * (1 - parameters->y_filter) + root_lin_vel_target_[1] * parameters->y_filter;
  yaw_turn_rate_ = yaw_turn_rate_ * (1 - parameters->yaw_filter) + root_ang_vel_target_[2] * parameters->yaw_filter;

  yaw_des_ += dt * yaw_turn_rate_;

  v_des_robot << x_vel_des_, y_vel_des_, 0.0;
  // clamp the velocity
  v_des_robot[0] = coerce(v_des_robot[0], -(float)parameters->velocity_limit[0], (float)parameters->velocity_limit[0]);
  v_des_robot[1] = coerce(v_des_robot[1], -(float)parameters->velocity_limit[1], (float)parameters->velocity_limit[1]);
  v_des_robot[2] = coerce(v_des_robot[2], -(float)parameters->velocity_limit[2], (float)parameters->velocity_limit[2]);

  v_des_world = root_mat_z.transpose() * v_des_robot;
  v_des_world << v_des_world[0], v_des_world[1], 0.0;

  world_position_desired_ += dt * v_des_world;

  if (first_run_) {
    v_des_world << 0.0, 0.0, 0.0;
    world_position_desired_ << root_pos_(0), root_pos_(1), 0.0;
    yaw_des_ = root_euler_(2);
    first_run_ = false;
  }

  iterationCounter++;
  float dtMPC = dt * parameters->iteration_mpc;
  int* mpcTable = gait_->GetMPCTable();

  // get desired traj and joint torque
  if ((iterationCounter % parameters->iteration_mpc) == 0) {
    float traj_initial[12] = {
        0.0,
        0.0,
        yaw_des_,
        root_pos_target_[0],
        root_pos_target_[1],
        (float)parameters->body_height,
        0.0,
        0.0,
        yaw_turn_rate_,
        v_des_world[0],
        v_des_world[1],
        0.0};

    for (int i = 0; i < parameters->horizon_length; i++) {
      for (int j = 0; j < 12; j++) traj_all_[12 * i + j] = traj_initial[j];

      if (i == 0)  // start at current position  TODO consider not doing this
      {
        traj_all_[2] = root_euler_[2];
      } else {
        traj_all_[12 * i + 3] = traj_all_[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
        traj_all_[12 * i + 4] = traj_all_[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
        traj_all_[12 * i + 2] = traj_all_[12 * (i - 1) + 2] + dtMPC * yaw_turn_rate_;
      }
    }

    float Q[12] = {
        (float)parameters->weight_rpy[0],
        (float)parameters->weight_rpy[1],
        (float)parameters->weight_rpy[2],
        (float)parameters->weight_pos[0],
        (float)parameters->weight_pos[1],
        (float)parameters->weight_pos[2],
        (float)parameters->weight_omega[0],
        (float)parameters->weight_omega[1],
        (float)parameters->weight_omega[2],
        (float)parameters->weight_vel[0],
        (float)parameters->weight_vel[1],
        (float)parameters->weight_vel[2]};
    float body_pos_world[3] = {root_pos_[0], root_pos_[1], root_pos_[2]};
    float body_vel_world[3] = {
        fusion_data.root_linear_velocity_in_world[0],
        fusion_data.root_linear_velocity_in_world[1],
        fusion_data.root_linear_velocity_in_world[2]};
    float body_omega_world[3] = {
        fusion_data.root_angular_velocity_in_world[0],
        fusion_data.root_angular_velocity_in_world[1],
        fusion_data.root_angular_velocity_in_world[2]};
    float body_orientation[4] = {
        fusion_data.root_quaternion[0],
        fusion_data.root_quaternion[1],
        fusion_data.root_quaternion[2],
        fusion_data.root_quaternion[3]};
    float r[12] = {0.0};
    float yaw = root_euler_[2];
    float* weights = Q;
    float* p = body_pos_world;
    float* v = body_vel_world;
    float* w = body_omega_world;
    float* q = body_orientation;

    for (int leg = 0; leg < 4; ++leg) {
      Eigen::Vector3f foot_pos_abs =
          fusion_data.rotation_matrix_body_to_world * fusion_data.foot_position_in_body.transpose().col(leg);
      foot_pos_abs -= fusion_data.rotation_matrix_body_to_world * com_offset_;
      r[leg] = foot_pos_abs[0];
      r[leg + 4] = foot_pos_abs[1];
      r[leg + 8] = foot_pos_abs[2];
    }

    setup_problem(dtMPC, parameters->horizon_length, parameters->foot_friction, parameters->max_reaction_force);
    // MCC::util::Timer solveTimer;
    update_problem_data_floats(p, v, q, w, r, yaw, weights, traj_all_, (float)parameters->alpha, mpcTable);
    // std::cout << "MPC time value is: " << solveTimer.getMs() << std::endl;

    for (int i = 0; i < 12; ++i) { grf_(i, 1) = get_solution(i); }
    for (int leg = 0; leg < 4; ++leg) {
      // TODO(Xiong Zhilin): check the reference frame and the negative sign
      foot_force_target_.col(leg) = -fusion_data.rotation_matrix_body_to_world.transpose() * grf_.segment<3>(3 * leg);
      Eigen::Matrix3f foot_jacob = fusion_data.foot_jacobian_in_body.block<3, 3>(3 * leg, 3 * leg);
      joint_torque_target_.segment<3>(3 * leg) = foot_jacob.transpose() * foot_force_target_.col(leg);
    }

    // std::cout << "stance leg value is: " << foot_force_target_ << std::endl;
  }

  return true;
}

void StanceLegMPCController::Reset() {}

Eigen::Matrix3f StanceLegMPCController::Skew(const Eigen::Vector3f& v) {
  Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
  S << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;

  return S;
}

}  // namespace locomotion_core
}  // namespace MCC
