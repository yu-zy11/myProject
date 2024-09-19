#ifndef PINOCCHIO_INTERFACE_H_
#define PINOCCHIO_INTERFACE_H_

// clang-format off
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <Eigen/Dense>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <string>
#include <vector>
#include "pinocchio_interface/basic_structure.h"

// clang-format on
/**
 * Pinocchio interface class loading urdf and contatining robot model and data.
 * The robot model is shared between interface instances.
 */
namespace core {
namespace pinocchio_interface {
using Model = pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
using Data = pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;

class PinocchioInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinocchioInterface() = default;
  /** load urdf and construct robot
   * @param [in] model_info: information of the robot for pinocchio
   */
  PinocchioInterface(const PinocchioModelInfo& model_info);

  /** get link index in pinocchio model by link name
   * @param [in] link_name: link name in urdf
   */
  int GetLinkID(const std::string& link_name);

  /** get the parent joint id of a link in pinocchio model
   * @param [in] link_name: link name in urdf
   */
  int GetLinkParentJointID(const std::string& link_name);

  /** Get link mass
   * @param [in] joint_index:index of link's index joint,for floating base is 1
   */
  double GetLinkMass(int joint_index) { return model_ptr_->inertias[joint_index].mass(); };

  /** Get link inertia
   * @param [in] joint_index:index of link's index joint,for floating base is 1
   */
  Eigen::Matrix3d GetLinkInertia(int joint_index) { return model_ptr_->inertias[joint_index].inertia(); };

  int GetEndEffectorNumber();

  /** get joint index in pinocchio model by joint name
   * @param [in] joint_name: joint name in urdf
   * @return joint index in pinocchio model
   * */
  int GetJointID(const std::string& joint_name);

  void updateKinematicsData(const Eigen::VectorXd& qpos);

  void updateKinematicsData(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  /** Get the joint position vectors.
   * @note requires:
   * pinocchio::forwardKinematics();
   * pinocchiio::updateFramePlacements();run first
   */
  Eigen::Vector3d GetJointPosition(int jointID) { return data_ptr_->oMi[jointID].translation(); };
  Eigen::VectorXd GetJointLowerPositionLimit() { return model_ptr_->lowerPositionLimit; }
  Eigen::VectorXd GetJointUpperPositionLimit() { return model_ptr_->upperPositionLimit; }
  Eigen::VectorXd GetJointVelocityLimit() { return model_ptr_->velocityLimit; }
  Eigen::VectorXd GetJointTorqueLimit() { return model_ptr_->effortLimit; }
  /**
   * @brief get all related parent joint ids except base,used for inverse kinematics to select jacobian columns
   */
  std::vector<int> GetJointAllRelatedParentJointIDsExceptBase(std::vector<int> joint_ids);

  /** Get the total mass of robot.*/
  double GetRobotMass() { return pinocchio::computeTotalMass(*model_ptr_); };

  Eigen::Vector3d GetCenterOfMass() { return data_ptr_->com[0]; };

  /**  return degree of freedom of the robot
   * @param [out] model_nv_:degree of freedom
   */
  int GetRobotDof() { return model_ptr_->nv; }

  /**
   * @brief get the degree of freedom of base link
   * @return [int] the degree of freedom of base link
   * @note: for fixed base robot, it is 0
   *        for floating base robot,it is 6
   */
  int GetBaseDof() { return base_dof_; }

  /**
   * reset link mass
   * @param [in] index: joint index
   * @param [in] new_mass: total mass  to link
   * @note:requires pinocchioInterface to be updated with:pinocchio::forwardKinematics
   */
  void resetLinkMass(int index, double new_mass);

  /** reset center of mass for link
   * @param [in] joint_id: joint id
   * @param [in] com: new center of mass set to link
   * @note:requires pinocchioInterface to be updated with:pinocchio::forwardKinematics
   */
  void resetLinkCoM(int joint_id, Eigen::Vector3d com);

  std::shared_ptr<Model> GetModel() { return model_ptr_; }
  std::shared_ptr<Data> GetData() { return data_ptr_; }
  std::shared_ptr<PinocchioModelInfo> GetModelInfo() { return info_ptr_; }
  int GetFloatingBaseJointNum() { return floating_base_joint_num_; }

 private:
  /**
   * load urdf and construct robot
   * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
   * @param [in] model：model in pinocchio
   * @param [in] print_info: print information of building urdf model
   */
  void createFloatingBaseModel(const std::string& urdfFilePath, Model& model, bool print_info);
  std::shared_ptr<Model> model_ptr_;
  std::shared_ptr<Data> data_ptr_;
  int floating_base_joint_num_;
  int base_dof_;
  std::shared_ptr<PinocchioModelInfo> info_ptr_;
};
}  // namespace pinocchio_interface
}  // namespace core
#endif
