#ifndef PINOCCHIO_INTERFACE_CONSTANTS_H_
#define PINOCCHIO_INTERFACE_CONSTANTS_H_

#include <Eigen/Dense>

namespace core {
namespace pinocchio_interface {
constexpr double kPi = 3.14159265358979323846;
struct EndEffectorData {
  Eigen::Vector3d pos;
  Eigen::Matrix3d rotm;
  Eigen::Vector3d vel;
  Eigen::Vector3d omega;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd jacobian_derivative;
  void Reshape(int size) {
    jacobian.resize(6, size);
    jacobian_derivative.resize(6, size);
    jacobian.setZero();
    jacobian_derivative.setZero();
  }
};

enum ContactType { kNoContact = 0, kSixDofContact = 1, kThreeDofContact = 2 };
struct PinocchioModelInfo {
  std::string urdf_file_path;
  std::string base_link_name = "base_link";
  std::vector<std::string> end_effector_names = {"leg_l_foot_center", "leg_r_foot_center"};
  std::vector<ContactType> contact_type = {ContactType::kSixDofContact, ContactType::kSixDofContact};
  bool use_floating_base = true;
  bool print_pinocchio_info = false;
};
}  // namespace pinocchio_interface
}  // namespace core
#endif
