#ifndef ESTIMATOR_INCLUDE_ESTIMATOR_LINEAR_KF_SETTING_H_
#define ESTIMATOR_INCLUDE_ESTIMATOR_LINEAR_KF_SETTING_H_
namespace core {
namespace estimator {

struct LinearKFSetting {
  size_t foot_num;
  double step;
  double foot_sensor_noise_height;
  double foot_sensor_noise_position;
  double foot_sensor_noise_velocity;
  double foot_process_noise_position;
  double imu_process_noise_position;
  double imu_process_noise_velocity;
};
}  // namespace estimator
}  // namespace core
#endif
