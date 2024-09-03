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

// clang-format on
/**
 * Pinocchio interface class loading urdf and contatining robot model and data.
 * The robot model is shared between interface instances.
 */

class PinocchioInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Model = pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;

  PinocchioInterface() = default;

  /**
   * load urdf and construct robot
   * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
   */
  void init(const std::string& urdfFilePath, bool print_info = false);

  /**
   * get link index in pinocchio model by link name
   * @param [in] link_name: link name in urdf
   * */
  int getLinkID(const std::string& link_name);

  /**
   * get the parent joint id of a link in pinocchio model
   * @param [in] link_name: link name in urdf
   * */
  int getLinkParentJointID(const std::string& link_name);

  /**
   * get joint index in pinocchio model by joint name
   * @param [in] joint_name: joint name in urdf
   * @return joint index in pinocchio model
   * */
  int getJointID(const std::string& joint_name);

  /** Get the total mass of robot.*/
  double getTotalMass() { return pinocchio::computeTotalMass(*model_ptr_); };

  Eigen::VectorXd getJointLowerPositionLimit() { return model_ptr_->lowerPositionLimit; }  // getJointLowerPositionLimit

  Eigen::VectorXd getJointUpperPositionLimit() { return model_ptr_->upperPositionLimit; }

  Eigen::VectorXd getJointVelocityLimit() { return model_ptr_->velocityLimit; }

  Eigen::VectorXd getJointTorqueLimit() { return model_ptr_->effortLimit; }

  std::vector<int> getAllJointParentIDsExceptBase(std::vector<int> joint_ids);

  /**
   * @brief transfer joint id to jacobian columns
   * @param [in] joint_id: joint id
   * @return  columns in jacobian matrix.jacobian is full jacobian matrix in cluding floating base
   *
   * For a robot with a floating base, the size of jacobian is 6 + nq,
   * where nq is the number of joints. The first 6 columns of jacobian are the
   * derivatives of the floating base, and the last nq columns are the
   * derivatives of the other joints. So we need to add 6 to the joint id to
   * get the correct jacobian column.
   *
   * However, the two extra joints for cloating base are replaced by base dof n the jacobian, so we need
   * to subtract the number of extra joints from the joint id.
   */
  int JointIdToJacobianColumns(const int& joint_id) { return joint_id - floating_base_joint_num_ + base_dof_; };

  /**
   * calculate pinocchio  kinematics data,include cartesian position, velocity, jacobian and jacobian derivatives.
   * @param [in] qpos:robot configuration,including nq*1
   * @param [in] qvel:joint velocity,nq*1,for bieped robot is 18*1
   */
  void computeKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  /**
   * calculate pinocchio dynamics data,include mass matrix and coriolis matrix.
   * @param [in] qpos:robot configuration,including trunk xyz,trunk quat,joint
   * @param [in] qvel:joint velocity,for bieped robot is 18*1
   * @note:requires pinocchioInterface to be updated computeKinematics()first
   */
  void computeDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  /** Get the link data.
   * @param [in] index: frame index,0:
   * @param [in] pos:link position
   * @param [in] rotm:link rotation matrix
   * @param [in] vel:link velocity
   * @param [in] omega:link angular velocity
   * @param [in] jacobian:link jacobian
   * @param [in] jacobian_derivative:link jacobian derivative
   * @note requires pinocchioInterface to be updated with: computeKinematics()
   */
  void getLinkKinematicsData(const int& index, Eigen::Vector3d& pos, Eigen::Matrix3d& rotm, Eigen::Vector3d& vel, Eigen::Vector3d& omega, Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian,
                             Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian_derivative);

  /** Get the joint position vectors.
   * @note requires pinocchioInterface to be updated with: computeKinematics()
   */
  Eigen::Matrix<double, 3, 1> getJointPosition(int jointID);

  /**
   * comnpute coriolis and gravity force in joint space
   * @note:requires pinocchioInterface to be updated with computeDynamics()
   */
  Eigen::VectorXd getCoriolisAndGravity();

  /**
   * comnpute joint space inertia matrix
   * @note:requires pinocchioInterface to be updated with:
   *        computeKinematics()
   *       computeDynamics()
   */
  Eigen::MatrixXd getJointSpaceInertiaMassMatrix();
  /**
   * comnpute inverse of joint space inertia matrix
   * @note:requires pinocchioInterface to be updated with computeKinematics()
   */
  Eigen::MatrixXd getJointSpaceInertialMassMatrixInverse();

  void computeCentroidalDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel);

  Eigen::Vector3d getCenterOfMass();

  /** Get link inertia
   * @param [in] joint_index:index of link's index joint,for floating base is 1
   */
  double getLinkMass(int joint_index);
  /** Get link inertia
   * @param [in] joint_index:index of link's index joint,for floating base is 1
   */
  Eigen::Matrix3d getLinkInertia(int joint_index);

  /**  return degree of freedom of the robot
   * @param [out] model_nv_:degree of freedom
   */
  int getDoF() { return model_ptr_->nv; }

  /**
   * @brief get the degree of freedom of base link
   * @return [int] the degree of freedom of base link
   * @note: for fixed base robot, it is 0
   *        for floating base robot,it is 6
   */
  int getBaseDoF() { return base_dof_; }

  // void convertVectorDouble2TemplateType(const Eigen::Matrix<double, -1, 1>& m, Eigen::Matrix<double, -1, 1>& m_des);

  /**
   * reset link mass
   * @param [in] index: joint index
   * @param [in] new_mass: total mass  to link
   * @note:requires pinocchioInterface to be updated with:pinocchio::forwardKinematics
   */
  void resetLinkMass(int index, double new_mass);

  /**
   * reset link com
   * @param [in] index: joint index
   * @param [in] mass: new mass set to link
   * @note:requires pinocchioInterface to be updated with:pinocchio::forwardKinematics
   */
  void resetLinkCoM(int index, Eigen::Vector3d com);

  int getExtraJointNumber() { return floating_base_joint_num_; }

 private:
  /**
   * load urdf and construct robot
   * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
   * @param [in] model：model in pinocchio
   * @param [in] print_info: print information of building urdf model
   */
  void createFloatingBaseModel(const std::string& urdfFilePath, Model& model, bool print_info);
  std::shared_ptr<Model> model_ptr_;
  std::unique_ptr<Data> data_ptr_;
  int floating_base_joint_num_{2};
  int base_dof_{6};
};

#endif
