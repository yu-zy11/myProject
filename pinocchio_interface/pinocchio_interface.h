#ifndef PINOCCHIO_INTERFACE_H
#define PINOCCHIO_INTERFACE_H

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
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/parsers/urdf.hpp>
#include <string>
#include <vector>
/**
 * Pinocchio interface class loading urdf and contatining robot model and data.
 * The robot model is shared between interface instances.
 */
template <typename T>
class PinocchioInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Model = pinocchio::ModelTpl<T, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = pinocchio::DataTpl<T, 0, pinocchio::JointCollectionDefaultTpl>;
  using JointModel = pinocchio::JointModelTpl<T, 0, pinocchio::JointCollectionDefaultTpl>;
  using vector3_t = Eigen::Matrix<T, 3, 1>;

  PinocchioInterface();
  // ~PinocchioInterface();

  /**
   * load urdf and construct robot
   * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
   * @param [in] print_info: print information of building urdf model
   */
  void loadURDF(const std::string& urdfFilePath, bool print_info);

  /**
   * load urdf and construct robot
   * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
   * @param [in] model：model in pinocchio
   * @param [in] print_info: print information of building urdf model
   */
  void createFloatingBaseModel(const std::string& urdfFilePath, Model& model, bool print_info);

  /** Get the total mass of robot.*/
  T getTotalMass() { return (T)pinocchio::computeTotalMass(*robotModelPtr_); };

  /**
   * update pinocchio model and data
   * @param [in] q:robot configuration,including trunk xyz,trunk quat,joint
   * @param [in] v:joint velocity
   * position
   */
  void updatePinocchioData(const Eigen::Matrix<T, -1, 1>& q, const Eigen::Matrix<T, -1, 1>& v);

  /**
   * comnpute foot jacobian in world frame or body frame
   * @param [in] index: frame index,0:trunk,1-4:leg
   * @note:requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  Eigen::Matrix<T, 6, -1> getFrameJabobianRelativeToBodyWorldFrame(int index);
  Eigen::Matrix<T, -1, -1> getWholeJabobianRelativeToBodyWorldFrame();
  Eigen::Matrix<T, 6, -1> getFrameJabobianRelativeToBodyFrame(int index);

  /**
   * comnpute foot jacobian derivative in world frame or body frame
   * @param [in] index: frame index,0:trunk,1-4:leg
   * @note:requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  Eigen::Matrix<T, 6, -1> getFrameJabobianDerivativeRelativeToBodyWorldFrame(int index);
  Eigen::Matrix<T, 6, -1> getFrameJabobianDerivativeRelativeToBodyFrame(int index);
  Eigen::Matrix<T, -1, -1> getWholeJabobianDerivativeRelativeToBodyWorldFrame();

  /**
   * comnpute joint space inertia matrix
   * @note:requires pinocchioInterface to be updated with:
   *       updatePinocchioData()
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  Eigen::Matrix<T, -1, -1> getJointSpaceInertiaMassMatrix();

  /**
   * comnpute inverse of joint space inertia matrix
   * @note:requires pinocchioInterface to be updated with updatePinocchioData()
   */
  Eigen::Matrix<T, -1, -1> getJointSpaceInertialMassMatrixInverse();

  /**
   * comnpute coriolis and gravity force o in joint space
   * @note:requires pinocchioInterface to be updated with updatePinocchioData()
   */
  Eigen::Matrix<T, -1, 1> getCoriolisAndGravity();

  /** Get end-effector (names) */
  std::vector<std::string>& getFootNames() { return foot_name_; };

  /** Get the link position vectors.including 0:trunk,1-4:legs
   * @note requires pinocchioInterface to be updated with: updatePinocchioData()
   */
  Eigen::Matrix<T, 3, 1> getFramePosition(int leg);

  /** Get the end effector velocity vectors.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q, v)
   */
  Eigen::Matrix<T, -1, 1> getFrameVelocity(int link);

  /** Get the end effector velocity vectors.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q, v)
   */
  std::vector<vector3_t> getVelocity(const vector3_t& state, const vector3_t& input);

  /** set the foot names and get foot index,foot names mush be same with foot
   * names in urdf .
   * @param [in] foot_name:foot name in urdf
   */
  void setLinkName(const std::vector<std::string>& foot_name);

  int get_model_nq_() { return model_nq_; }
  int get_model_nv_() { return model_nv_; }

  Eigen::Matrix<T, 3, 1> getCenterOfMass(const Eigen::Matrix<T, -1, 1>& q, int index);

 private:
  std::shared_ptr<const Model> robotModelPtr_;
  std::unique_ptr<Data> robotDataPtr_;
  std::vector<std::string> foot_name_;
  std::vector<int> link_index_;
  int model_nv_;
  int model_nq_;
  Eigen::Matrix<T, -1, -1> Jacbian_all_;
  Eigen::Matrix<T, -1, 1> qpos_;
  Eigen::Matrix<T, -1, 1> qvel_;
  Eigen::Matrix<T, -1, -1> mass_matrix_;
  Eigen::Matrix<T, -1, -1> inverse_mass_matrix_;
  Eigen::Matrix<T, -1, 1> coriolis_;
};

#endif
