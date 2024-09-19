
#ifndef ESTIMATOR_INCLUDE_ESTIMATOR_LINEAR_KF_STATE_ESTIMATOR_H_
#define ESTIMATOR_INCLUDE_ESTIMATOR_LINEAR_KF_STATE_ESTIMATOR_H_

#include <Eigen/Dense>
#include "data_store/data_store.h"
#include "estimator_param/estimator_param.h"
#include "linearKF_state_estimator/linearKF_setting.h"
#include "pinocchio_interface/pinocchio_kinematics.h"

/*implements a Linear Kalman Filter (KF) for state estimation:
 *pridiction equation: X_{k+1}=AX_k+BU_k+noise,stateX=[pos_base ,vel_base,foot_pos1,foot_pos2...foot_posn]. include base
 *position,base velocity,and foot positions observation equation: y_k=H*X_k+noise, observation y=[pos_base-foot_pos1,
 *pos_base-foot_pos2 ,vel_base,vel_base,foot_pos1_z,foot_pos2_z]
 */
namespace core {
namespace estimator {

class LinearKFStateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFStateEstimator(const std::shared_ptr<data::DataStore>& data_store);

  virtual void Setup();
  virtual void Run();
  void UpdateSetting(const LinearKFSetting& setting) { setting_ = setting; };
  void UpdateSetting();
  void WriteDataSator(){};
  data::SE3Frame getState() { return base_link_; }

 private:
  // void UpdateEndeffctorDataInBaseFrame();
  void RunLinearKalmanFilterForPositionVelocity();
  void UpdateOrietationData();
  // state in Kalman filter, xhat:[xyz_base,vel_base,foot_pos1,...foot_posn]
  Eigen::VectorXd xhat_;
  // state matrix in prediction equation:x_{k+1}=A*x_{k}+B_*u_{k}
  Eigen::MatrixXd A_;
  // state matrix in prediction equation:x_{k+1}=A*x_{k}+B_*u_{k}
  Eigen::MatrixXd B_;
  // state covariance matrix()
  Eigen::MatrixXd P_;
  // observation matrix
  Eigen::MatrixXd C_;

  std::shared_ptr<data::DataStore> data_ptr_;
  std::unique_ptr<data::EstimatorParam> param_;
  std::shared_ptr<core::pinocchio_interface::PinocchioKinematics> pino_kine_;

  int first_run_counter_ = 0;
  Eigen::Quaterniond quaternion_offset_;
  LinearKFSetting setting_;
  data::SE3Frame base_link_;
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
// class CheaterPositionVelocityEstimator {
//  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   CheaterPositionVelocityEstimator(const std::shared_ptr<data::DataStore>& data_store) : data_ptr_(data_store){};
//   virtual void run();
//   virtual void setup();

//  private:
//   std::shared_ptr<data::DataStore> data_ptr_;
// };
}  // namespace estimator
}  // namespace core
#endif
