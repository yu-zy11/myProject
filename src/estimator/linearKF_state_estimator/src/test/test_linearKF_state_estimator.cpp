#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "common/tool/include/tool/string_join.h"
#include "data/_param/model_param/model_param.h"
#include "data/imu_info/imu_info.h"
#include "linearKF_state_estimator/linearKF_state_estimator.h"
TEST(linearKF, testLinearKF) {
  core::estimator::LinearKFSetting setting;
  setting.foot_num = 1;
  setting.step = 0.002;
  setting.foot_sensor_noise_height = 0.001;
  setting.foot_sensor_noise_position = 0.001;
  setting.foot_sensor_noise_velocity = 0.1;
  setting.foot_process_noise_position = 0.002;
  setting.imu_process_noise_position = 0.02;
  setting.imu_process_noise_velocity = 0.02;

  const std::string config_dir = std::getenv("ENGINEAI_ROBOTICS_CONFIG");
  std::string robot_name = "sa01p";
  std::string robot_dir = common::PathJoin(config_dir, robot_name);
  common::SetGlobalConfigPath(robot_dir);
  data::ModelParam model_param;
  model_param.Log();
  std::shared_ptr<data::DataStore> data_store;
  data_store = std::make_shared<data::DataStore>(std::make_shared<data::ModelParam>(model_param));
  /*set imu info*/
  data::ImuInfo imu_info;
  imu_info.quaternion = Eigen::Quaterniond(1, 0, 0, 1);
  imu_info.quaternion.normalize();
  imu_info.linear_acceleration = Eigen::Vector3d(0, 0, 0);
  imu_info.angular_velocity = Eigen::Vector3d(0, 0, 0);
  data_store->imu_info.Set(imu_info);

  core::estimator::LinearKFStateEstimator estimator(data_store);

  estimator.UpdateSetting(setting);
  estimator.Setup();
  for (int i = 0; i < 1000; i++) {
    estimator.Run();
  }
  EXPECT_LT(estimator.getState().pose.quaternion.w() - 1.0, 0.0000001);
}