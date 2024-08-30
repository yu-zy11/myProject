// clang-format off
#include "PinocchioInterface.h"
#include "pinocchio/spatial/symmetric3.hpp"
#include <Eigen/Dense>
#include <unistd.h>
// clang-format on

// PinocchioInterface::PinocchioInterface() {}

void PinocchioInterface::init(const std::string& urdfFilePath, bool print_info) {
  Model model;
  createFloatingBaseModel(urdfFilePath, model, print_info);
  model_ptr_ = std::make_shared<Model>(model);
  data_ptr_ = std::make_unique<Data>(*model_ptr_);
  printf("[PinocchioInterface] init finished\n");
}

int PinocchioInterface::getLinkID(const std::string& link_name) {
  if (!model_ptr_->existBodyName(link_name)) {
    const char* cstr = link_name.c_str();
    printf("Error,link name: [%s] does not exits in URDF, please check\n", cstr);
    abort();
  }
  return model_ptr_->getBodyId(link_name);
}

int PinocchioInterface::getJointID(const std::string& joint_name) { return model_ptr_->getJointId(joint_name); }

void PinocchioInterface::computeKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
  assert(qpos.rows() == model_ptr_->nv && "qpos size not match");
  assert(qvel.rows() == model_ptr_->nv && "qvel size not match");
  pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, qpos, qvel);
  pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);
  pinocchio::computeJointJacobians(*model_ptr_, *data_ptr_, qpos);
  // pinocchio::crba(*model_ptr_, *data_ptr_, qpos_);
}

void PinocchioInterface::computeDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
  computeKinematics(qpos, qvel);
  pinocchio::crba(*model_ptr_, *data_ptr_, qpos);
  pinocchio::nonLinearEffects(*model_ptr_, *data_ptr_, qpos, qvel);
  pinocchio::computeMinverse(*model_ptr_, *data_ptr_, qpos);
}

void PinocchioInterface::getLinkKinematicsData(const int& index, Eigen::Vector3d& pos, Eigen::Matrix3d& rotm, Eigen::Vector3d& vel, Eigen::Vector3d& omega,
                                               Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian, Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian_derivative) {
  assert(index < model_ptr_->nframes && " index out of range in getLinkPosition");
  pos = data_ptr_->oMf[index].translation();
  rotm = data_ptr_->oMf[index].rotation();

  auto vel_tmp = pinocchio::getFrameVelocity(*model_ptr_, *data_ptr_, index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  vel = vel_tmp.linear();
  omega = vel_tmp.angular();
  jacobian.setZero();
  jacobian_derivative.setZero();
  pinocchio::getFrameJacobian(*model_ptr_, *data_ptr_, index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
  pinocchio::getFrameJacobianTimeVariation(*model_ptr_, *data_ptr_, index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_derivative);
}

Eigen::Vector3d PinocchioInterface::getJointPosition(int jointID) { return data_ptr_->oMi[jointID].translation(); }

Eigen::VectorXd PinocchioInterface::getCoriolisAndGravity() { return data_ptr_->nle; };

Eigen::MatrixXd PinocchioInterface::getJointSpaceInertiaMassMatrix() {
  // Pinocchio only gives upper triangle of the M, needs to restore here
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(model_ptr_->nv, model_ptr_->nv);
  M.triangularView<Eigen::Upper>() = data_ptr_->M.triangularView<Eigen::Upper>();
  M.triangularView<Eigen::Lower>() = data_ptr_->M.transpose().triangularView<Eigen::Lower>();
  return M;
};

Eigen::MatrixXd PinocchioInterface::getJointSpaceInertialMassMatrixInverse() {
  // pinocchio::computeMinverse(*model_ptr_, *data_ptr_, qpos);
  Eigen::MatrixXd Minv = Eigen::MatrixXd::Zero(model_ptr_->nv, model_ptr_->nv);
  Minv.triangularView<Eigen::Upper>() = data_ptr_->Minv.triangularView<Eigen::Upper>();
  Minv.triangularView<Eigen::Lower>() = data_ptr_->Minv.transpose().triangularView<Eigen::Lower>();
  return Minv;
}
void PinocchioInterface::computeCentroidalDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) { pinocchio::computeCentroidalMomentum(*model_ptr_, *data_ptr_, qpos, qvel); }

double PinocchioInterface::getLinkMass(int joint_index) { return model_ptr_->inertias[joint_index].mass(); }

Eigen::Matrix3d PinocchioInterface::getLinkInertia(int joint_index) { return model_ptr_->inertias[joint_index].inertia(); };
void PinocchioInterface::resetLinkMass(int index, double new_mass) {
  assert(index < model_ptr_->nv && "index out of range in resetLinkMass");
  auto mass = model_ptr_->inertias[index].mass();
  auto com = model_ptr_->inertias[index].lever();
  auto inertia = model_ptr_->inertias[index].inertia();
  auto inertia_data = inertia.data();
  inertia_data *= new_mass / mass;
  pinocchio::Symmetric3 inertia_tmp(inertia_data);
  inertia = inertia_tmp;
  mass = new_mass;
  pinocchio::Inertia inerita_tmp(mass, com, inertia);
  model_ptr_->inertias[index] = inerita_tmp;
  // pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, qpos_, qvel_);
}

void PinocchioInterface::resetLinkCoM(int index, Eigen::Vector3d com) {
  assert(index < model_ptr_->nv && "index out of range in resetLinkCOM");
  auto mass = model_ptr_->inertias[index].mass();
  auto inertia = model_ptr_->inertias[index].inertia();
  pinocchio::Inertia inerita_tmp(mass, com, inertia);
  model_ptr_->inertias[index] = inerita_tmp;
  // pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, qpos_, qvel_);
}

Eigen::Vector3d PinocchioInterface::getCenterOfMass() {
  // pinocchio::centerOfMass(*model_ptr_, *data_ptr_, qpos_, true);
  return data_ptr_->com[0];
}

// add 6 joints to the root link,construct floating base model
void PinocchioInterface::createFloatingBaseModel(const std::string& urdfFilePath, Model& model, bool print_info) {
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  pinocchio::urdf::buildModel(urdfFilePath, jointComposite, model, print_info);
};
