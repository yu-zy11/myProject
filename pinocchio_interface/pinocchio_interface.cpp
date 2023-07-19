#include "pinocchio_interface.h"

#include <Eigen/Dense>

// #include <pinocchio/algorithm/center-of-mass.hpp>
// #include <pinocchio/algorithm/centroidal-derivatives.hpp>
// #include <pinocchio/algorithm/centroidal.hpp>
// #include <pinocchio/algorithm/frames.hpp>

// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/fwd.hpp"

template <typename T>
PinocchioInterface<T>::PinocchioInterface() {
  std::string file_path{"/home/yu/workspace/lquadleg/Cheetah/sim/mujoco_simulator/model/ls_dog105/meshes/ls_dog105.urdf"};
  loadURDF(file_path, false);
  std::vector<std::string> foot_name = {"base_link", "fr_foot", "fl_foot", "hr_foot", "hl_foot"};
  setLinkName(foot_name);
  qpos_.resize(model_nq_, 1);
  qvel_.resize(model_nv_, 1);
  qpos_.setZero();
  qvel_.setZero();
  mass_matrix_.resize(model_nv_, model_nv_);
  inverse_mass_matrix_.resize(model_nv_, model_nv_);
  coriolis_.resize(model_nv_, 1);
  mass_matrix_.setZero();
  inverse_mass_matrix_.setZero();
  coriolis_.setZero();
  std::cout << "[PinocchioInterface] init finished" << std::endl;
}

template <typename T>
void PinocchioInterface<T>::loadURDF(const std::string& urdfFilePath, bool print_info) {
  Model model;
  createFloatingBaseModel(urdfFilePath, model, print_info);
  robotModelPtr_ = std::make_shared<const Model>(model);
  robotDataPtr_ = std::make_unique<Data>(*robotModelPtr_);
  model_nv_ = model.nv;
  model_nq_ = model.nq;
};

template <typename T>
void PinocchioInterface<T>::createFloatingBaseModel(const std::string& urdfFilePath, Model& model, bool print_info) {  // add 6 joints to the root link,construct floating base model
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  pinocchio::urdf::buildModel(urdfFilePath, jointComposite, model, print_info);
};

template <typename T>
void PinocchioInterface<T>::updatePinocchioData(const Eigen::Matrix<T, -1, 1>& q, const Eigen::Matrix<T, -1, 1>& v) {
  qpos_ = q;
  qvel_ = v;
  pinocchio::forwardKinematics(*robotModelPtr_, *robotDataPtr_, q);
  pinocchio::updateFramePlacements(*robotModelPtr_, *robotDataPtr_);
  // pinocchio::crba(*robotModelPtr_, *robotDataPtr_, q);
  // pinocchio::computeMinverse(*robotModelPtr_, *robotDataPtr_, q);
  // pinocchio::nonLinearEffects(*robotModelPtr_, *robotDataPtr_, q, v);
}

template <typename T>
Eigen::Matrix<T, 6, -1> PinocchioInterface<T>::getFrameJabobianRelativeToBodyWorldFrame(int leg) {
  if (leg >= link_index_.size()) {
    std::cout << "[pinocchio_interface]leg must less than foot number,change leg from:" << leg << " to " << link_index_.size() << std::endl;
    leg = link_index_.size();
  }
  pinocchio::computeJointJacobians(*robotModelPtr_, *robotDataPtr_, qpos_);
  Eigen::Matrix<T, 6, -1> J = Eigen::Matrix<T, 6, -1>::Zero(6, model_nv_);
  pinocchio::getFrameJacobian(*robotModelPtr_, *robotDataPtr_, link_index_[leg], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  return J;
};
template <typename T>
Eigen::Matrix<T, -1, -1> PinocchioInterface<T>::getWholeJabobianRelativeToBodyWorldFrame() {
  pinocchio::computeJointJacobians(*robotModelPtr_, *robotDataPtr_, qpos_);

  int foot_num = link_index_.size() - 1;
  Eigen::Matrix<T, -1, -1> J = Eigen::Matrix<T, -1, -1>::Zero(6 + 3 * foot_num, model_nv_);
  Eigen::Matrix<T, 6, -1> Jtmp = Eigen::Matrix<T, 6, -1>::Zero(6, model_nv_);
  pinocchio::getFrameJacobian(*robotModelPtr_, *robotDataPtr_, link_index_[0], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jtmp);
  J.block(0, 0, 6, model_nv_) = Jtmp;
  for (int i = 0; i < foot_num; i++) {
    pinocchio::getFrameJacobian(*robotModelPtr_, *robotDataPtr_, link_index_[i + 1], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jtmp);
    J.block(6 + 3 * i, 0, 3, model_nv_) = Jtmp.block(0, 0, 3, model_nv_);
  }
  return J;
};

template <typename T>
Eigen::Matrix<T, 6, -1> PinocchioInterface<T>::getFrameJabobianRelativeToBodyFrame(int leg) {
  pinocchio::computeJointJacobians(*robotModelPtr_, *robotDataPtr_, qpos_);
  if (leg >= link_index_.size()) {
    std::cout << "[pinocchio_interface]leg must less than foot number,change leg from:" << leg << " to " << link_index_.size() << std::endl;
    leg = link_index_.size();
  }
  Eigen::Matrix<T, 6, -1> J = Eigen::Matrix<T, 6, -1>::Zero(6, model_nv_);
  pinocchio::getFrameJacobian(*robotModelPtr_, *robotDataPtr_, link_index_[leg], pinocchio::ReferenceFrame::LOCAL, J);
  return J;
};

template <typename T>
Eigen::Matrix<T, 6, -1> PinocchioInterface<T>::getFrameJabobianDerivativeRelativeToBodyWorldFrame(int link) {
  pinocchio::computeJointJacobiansTimeVariation(*robotModelPtr_, *robotDataPtr_, qpos_, qvel_);
  if (link >= link_index_.size()) {
    std::cout << "[pinocchio_interface]leg must less than foot number,change leg from:" << link << " to " << link_index_.size() << std::endl;
    link = link_index_.size();
  }
  Eigen::Matrix<T, 6, -1> dJ = Eigen::Matrix<T, 6, -1>::Zero(6, model_nv_);
  pinocchio::getFrameJacobianTimeVariation(*robotModelPtr_, *robotDataPtr_, link_index_[link], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
  return dJ;
};

template <typename T>
Eigen::Matrix<T, 6, -1> PinocchioInterface<T>::getFrameJabobianDerivativeRelativeToBodyFrame(int link) {
  pinocchio::computeJointJacobiansTimeVariation(*robotModelPtr_, *robotDataPtr_, qpos_, qvel_);
  if (link >= link_index_.size()) {
    std::cout << "[pinocchio_interface]leg must less than foot number,change leg from:" << link << " to " << link_index_.size() << std::endl;
    link = link_index_.size();
  }
  Eigen::Matrix<T, 6, -1> dJ = Eigen::Matrix<T, 6, -1>::Zero(6, model_nv_);
  pinocchio::getFrameJacobianTimeVariation(*robotModelPtr_, *robotDataPtr_, link_index_[link], pinocchio::ReferenceFrame::LOCAL, dJ);
  return dJ;
};

template <typename T>
Eigen::Matrix<T, -1, -1> PinocchioInterface<T>::getWholeJabobianDerivativeRelativeToBodyWorldFrame() {
  pinocchio::computeJointJacobiansTimeVariation(*robotModelPtr_, *robotDataPtr_, qpos_, qvel_);
  int foot_num = link_index_.size() - 1;
  Eigen::Matrix<T, -1, -1> dJ = Eigen::Matrix<T, -1, -1>::Zero(6 + 3 * foot_num, model_nv_);
  Eigen::Matrix<T, 6, -1> dJtmp = Eigen::Matrix<T, 6, -1>::Zero(6, model_nv_);
  // pinocchio::computeJointJacobiansTimeVariation();
  pinocchio::getFrameJacobianTimeVariation(*robotModelPtr_, *robotDataPtr_, link_index_[0], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJtmp);
  dJ.block(0, 0, 6, model_nv_) = dJtmp;
  for (int i = 0; i < foot_num; i++) {
    pinocchio::getFrameJacobianTimeVariation(*robotModelPtr_, *robotDataPtr_, link_index_[i + 1], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJtmp);
    dJ.block(6 + 3 * i, 0, 3, model_nv_) = dJtmp.block(0, 0, 3, model_nv_);
  }
  return dJ;
};

// void getInertialMassMatrix();
// void getCroliosAndGravity();
template <typename T>
Eigen::Matrix<T, 3, 1> PinocchioInterface<T>::getFramePosition(int leg) {
  if (leg >= link_index_.size()) {
    std::cout << "[pinocchio_interface]leg must less than foot number,change leg from:" << leg << " to " << link_index_.size() << std::endl;
    leg = link_index_.size();
  }
  const auto id = link_index_[leg];
  Eigen::Matrix<T, 3, 1> pos = robotDataPtr_->oMf[id].translation();
  return pos;
};

template <typename T>
Eigen::Matrix<T, -1, 1> PinocchioInterface<T>::getFrameVelocity(int link){};

template <typename T>
void PinocchioInterface<T>::setLinkName(const std::vector<std::string>& foot_name) {
  foot_name_.clear();
  for (const auto& name : foot_name) {
    foot_name_.push_back(name);
    link_index_.push_back(robotModelPtr_->getBodyId(name));
  }
  for (const auto& itr : link_index_) std::cout << "link_index_:" << itr << std::endl;
};

template <typename T>
Eigen::Matrix<T, 3, 1> PinocchioInterface<T>::getCenterOfMass(const Eigen::Matrix<T, -1, 1>& q, int index) {
  pinocchio::centerOfMass(*robotModelPtr_, *robotDataPtr_, q, true);
  return robotDataPtr_->com[index];
}

template <typename T>
Eigen::Matrix<T, -1, -1> PinocchioInterface<T>::getJointSpaceInertiaMassMatrix() {
  pinocchio::crba(*robotModelPtr_, *robotDataPtr_, qpos_);
  // mass_matrix_.triangularView<Eigen::Upper>() = robotDataPtr_->M.triangularView<Eigen::Upper>();
  // mass_matrix_.triangularView<Eigen::StrictlyLower>() = robotDataPtr_->M.transpose().triangularView<Eigen::StrictlyLower>();
  for (int row = 0; row < mass_matrix_.cols(); ++row) {
    for (int col = row; col < mass_matrix_.cols(); ++col) {
      mass_matrix_(row, col) = robotDataPtr_->M(row, col);
      mass_matrix_(col, row) = robotDataPtr_->M(row, col);
    }
  }
  return mass_matrix_;
};

template <typename T>
Eigen::Matrix<T, -1, -1> PinocchioInterface<T>::getJointSpaceInertialMassMatrixInverse() {
  pinocchio::computeMinverse(*robotModelPtr_, *robotDataPtr_, qpos_);
  for (int row = 0; row < inverse_mass_matrix_.cols(); ++row) {
    for (int col = row; col < inverse_mass_matrix_.cols(); ++col) {
      inverse_mass_matrix_(row, col) = robotDataPtr_->Minv(row, col);
      inverse_mass_matrix_(col, row) = robotDataPtr_->Minv(row, col);
    }
  }
  return inverse_mass_matrix_;
}

template <typename T>
Eigen::Matrix<T, -1, 1> PinocchioInterface<T>::getCoriolisAndGravity() {
  pinocchio::nonLinearEffects(*robotModelPtr_, *robotDataPtr_, qpos_, qvel_);
  coriolis_ = robotDataPtr_->nle;
  return coriolis_;
};

template class PinocchioInterface<double>;
// template class PinocchioInterface<float>;
