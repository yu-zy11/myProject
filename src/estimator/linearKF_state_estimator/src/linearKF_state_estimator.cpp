

#include "linearKF_state_estimator/linearKF_state_estimator.h"
#include "math/roll_pitch_yaw.h"

namespace core {
namespace estimator {

LinearKFStateEstimator::LinearKFStateEstimator(const std::shared_ptr<data::DataStore>& data_store) {
  data_ptr_ = data_store;
  pino_kine_ = std::make_shared<core::pinocchio_interface::PinocchioKinematics>(data_ptr_->pinocchio_interface);
  quaternion_offset_.setIdentity();
  first_run_counter_ = 0;
  UpdateSetting();
}

void LinearKFStateEstimator::UpdateSetting() {
  param_ = std::make_unique<data::EstimatorParam>();
  assert(param_->foot_number > 0 && "error,foot number should > 0");
  setting_.foot_num = param_->foot_number;
  setting_.step = param_->step;
  setting_.foot_sensor_noise_height = param_->foot_sensor_noise_height;
  setting_.foot_sensor_noise_position = param_->foot_sensor_noise_position;
  setting_.foot_sensor_noise_velocity = param_->foot_sensor_noise_velocity;
  setting_.foot_process_noise_position = param_->foot_process_noise_position;
  setting_.imu_process_noise_position = param_->imu_process_noise_position;
  setting_.imu_process_noise_velocity = param_->imu_process_noise_velocity;
};

void LinearKFStateEstimator::Setup() {
  // resize and set matrix for Kalman filter
  xhat_.resize(setting_.foot_num * 3 + 6);
  xhat_.setZero();

  A_.resize(setting_.foot_num * 3 + 6, setting_.foot_num * 3 + 6);
  A_.setIdentity();
  A_.block(0, 3, 3, 3) = setting_.step * Eigen::Matrix3d::Identity();

  B_.resize(setting_.foot_num * 3 + 6, 3);
  B_.setZero();
  B_.block(3, 0, 3, 3) = setting_.step * Eigen::Matrix3d::Identity();

  C_.resize(setting_.foot_num * 7, setting_.foot_num * 3 + 6);  // observation matrix
  C_.setZero();
  for (int i = 0; i < setting_.foot_num; i++) {
    C_.block(3 * i, 0, 3, 3) << Eigen::Matrix3d::Identity();  // for p-pi
    C_.block(3 * i, 3 * i + 6, 3, 3) << -1.0 * Eigen::Matrix3d::Identity();
    C_.block(3 * setting_.foot_num + 3 * i, 3, 3, 3) << Eigen::Matrix3d::Identity();  // for v-vi
    C_.block(6 * setting_.foot_num + i, 6 + 3 * i, 1, 3) << 0, 0, 1;                  // for zi
  }

  P_.resize(setting_.foot_num * 3 + 6, setting_.foot_num * 3 + 6);  // state covariance matrix()
  P_.setIdentity();
  P_ = 100.0 * P_;
}

/*!
 * Run state estimator
 */
void LinearKFStateEstimator::Run() {
  UpdateOrietationData();
  RunLinearKalmanFilterForPositionVelocity();
}

void LinearKFStateEstimator::UpdateOrietationData() {
  if (first_run_counter_ < 100) {
    data_ptr_->imu_info().quaternion.normalize();
    core::math::RollPitchYaw rpy_init(data_ptr_->imu_info().quaternion);
    rpy_init.set(0.0, 0.0, -rpy_init.yaw());
    quaternion_offset_ = rpy_init.ToQuaternion();
    first_run_counter_++;
  }

  base_link_.pose.quaternion = quaternion_offset_ * data_ptr_->imu_info().quaternion;
  Eigen::Matrix3d rotm = base_link_.pose.quaternion.toRotationMatrix();
  base_link_.twist.angular = rotm * data_ptr_->imu_info().angular_velocity;
  base_link_.acceleration.linear = rotm * data_ptr_->imu_info().linear_acceleration;
}

void LinearKFStateEstimator::RunLinearKalmanFilterForPositionVelocity() {
  /*update leg data in base frame*/
  Eigen::VectorXd joint_pos, joint_vel;
  data_ptr_->joint_info.GetState(data::JointInfoType::kPosition, joint_pos);
  data_ptr_->joint_info.GetState(data::JointInfoType::kVelocity, joint_vel);
  Eigen::VectorXd qpos = Eigen::VectorXd::Zero(pino_kine_->GetInterface()->GetRobotDof());
  Eigen::VectorXd qvel = Eigen::VectorXd::Zero(pino_kine_->GetInterface()->GetRobotDof());
  qpos.tail(pino_kine_->GetInterface()->GetRobotDof() - pino_kine_->GetInterface()->GetBaseDof()) = joint_pos;
  qvel.tail(pino_kine_->GetInterface()->GetRobotDof() - pino_kine_->GetInterface()->GetBaseDof()) = joint_vel;

  pino_kine_->FixedBaseForwardKinematics(qpos, qvel);

  /*set covariance matrix for process noise and observation noise*/
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6 + 3 * setting_.foot_num, 6 + 3 * setting_.foot_num);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(7 * setting_.foot_num, 7 * setting_.foot_num);
  Q.block(0, 0, 3, 3) = (setting_.step / 20.f) * Eigen::Matrix3d::Identity() * setting_.imu_process_noise_position;
  Q.block(3, 3, 3, 3) =
      (setting_.step * 9.81f / 20.f) * Eigen::Matrix3d::Identity() * setting_.imu_process_noise_velocity;
  for (int i = 0; i < setting_.foot_num; i++) {
    Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) =
        setting_.step * Eigen::Matrix3d::Identity() * setting_.foot_process_noise_position;
    R.block(3 * i, 3 * i, 3, 3) = Eigen::Matrix3d::Identity() * setting_.foot_sensor_noise_position;
    R.block(3 * setting_.foot_num + 3 * i, 3 * setting_.foot_num + 3 * i, 3, 3) =
        Eigen::Matrix3d::Identity() * setting_.foot_sensor_noise_velocity;
    R(6 * setting_.foot_num + i, 6 * setting_.foot_num + i) = setting_.foot_sensor_noise_height;
  }

  /*update noise covariance matrix for prediction and observation,using the information of gait contact phases,if
   * foot is swinging, improve the noise of the foot,because we can not observe base velocity using swing
   * leg*/
  Eigen::Matrix3d rotm = base_link_.pose.quaternion.toRotationMatrix();
  Eigen::VectorXd pzs = Eigen::VectorXd::Zero(setting_.foot_num);
  Eigen::VectorXd pos_in_observation = Eigen::VectorXd::Zero(3 * setting_.foot_num);
  Eigen::VectorXd vel_in_observation = Eigen::VectorXd::Zero(3 * setting_.foot_num);
  for (int i = 0; i < setting_.foot_num; i++) {
    Eigen::VectorXd foot_contact_phase = data_ptr_->planner_info().gait_stance_phase;
    double phase = fmin(foot_contact_phase[i], 1.0);
    double trust_window = double(0.2);  // double trust_window = double(0.25);
    double trust;
    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (1.0 - trust_window)) {
      trust = (1.0 - phase) / trust_window;
    } else {
      trust = 1.0;
    }
    double high_suspect_number(1000);  // original 100
    Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) =
        (1.0 + (1.0 - trust) * high_suspect_number) * Q.block(6 + 3 * i, 6 + 3 * i, 3, 3);  // posotion z
    R.block(3 * setting_.foot_num + 3 * i, 3 * setting_.foot_num + 3 * i, 3, 3) =
        (1.0 + (1.0 - trust) * high_suspect_number) *
        R.block(3 * setting_.foot_num + 3 * i, 3 * setting_.foot_num + 3 * i, 3, 3);  // velocity
    R(6 * setting_.foot_num + i, 6 * setting_.foot_num + i) =
        (1.0 + (1.0 - trust) * high_suspect_number) *
        R(6 * setting_.foot_num + i, 6 * setting_.foot_num + i);  // z height

    /*get foot position and velocity relative to base link expressed in world frame*/
    std::string foot_name = pino_kine_->GetModelInfo()->foot_names[i];
    Eigen::Vector3d foot_local_pos_in_world = rotm * pino_kine_->GetInterface()->GetLinkPosition(foot_name);
    Eigen::Vector3d foot_local_vel_in_world = base_link_.twist.angular.cross(foot_local_pos_in_world) +
                                              rotm * pino_kine_->GetInterface()->GetLinkVelocity(foot_name);
    pos_in_observation.segment(3 * i, 3) = -foot_local_pos_in_world;
    vel_in_observation.segment(3 * i, 3) = (1.0 - trust) * xhat_.segment(3, 3) + trust * (-foot_local_vel_in_world);
    pzs[i] = (1.0 - trust) * (xhat_[2] + foot_local_pos_in_world[2]);
  }

  /**linear Kalman filter,y:[p-p1,p-p2,v,v,z1,z2]***/
  Eigen::Vector3d g(0, 0, -9.81);  // later can be changed using gravity estimator
  Eigen::Vector3d acc = base_link_.acceleration.linear + g;
  Eigen::VectorXd y = Eigen::VectorXd::Zero(7 * setting_.foot_num);
  y << pos_in_observation, vel_in_observation, pzs;
  xhat_ = A_ * xhat_ + B_ * acc;
  Eigen::MatrixXd At = A_.transpose();
  Eigen::MatrixXd Pm = A_ * P_ * At + Q;
  Eigen::MatrixXd Ct = C_.transpose();
  Eigen::VectorXd yModel = C_ * xhat_;
  Eigen::VectorXd ey = y - yModel;
  Eigen::MatrixXd S = C_ * Pm * Ct + R;

  // todo compute LU only once
  Eigen::VectorXd S_ey = S.lu().solve(ey);
  xhat_ += Pm * Ct * S_ey;
  Eigen::MatrixXd S_C = S.lu().solve(C_);
  P_ = (Eigen::MatrixXd::Identity(6 + 3 * setting_.foot_num, 6 + 3 * setting_.foot_num) - Pm * Ct * S_C) * Pm;
  Eigen::MatrixXd Pt = P_.transpose();
  P_ = (P_ + Pt) / 2.0;
  // if (P_.block(0, 0, 2, 2).determinant() > double(0.000001)) {
  //   P_.block(0, 2, 2, 6 + 3 * setting_.foot_num - 2).setZero();
  //   P_.block(2, 0, 6 + 3 * setting_.foot_num - 2, 2).setZero();
  //   P_.block(0, 0, 2, 2) /= double(10);
  // }
  base_link_.pose.position = xhat_.block(0, 0, 3, 1);
  base_link_.twist.linear = xhat_.block(3, 0, 3, 1);
}

}  // namespace estimator
}  // namespace core
