// clang-format off
#include "pinocchio_interface/pinocchio_interface.h"
#include "pinocchio/spatial/symmetric3.hpp"
#include <Eigen/Dense>
#include <unistd.h>
#include <algorithm>
// clang-format on
namespace pino {

PinocchioInterface::PinocchioInterface(const PinocchioModelInfo& model_info) {
  Model model;
  if (model_info.use_floating_base) {
    createFloatingBaseModel(model_info.urdf_file_path, model, model_info.print_pinocchio_info);
    base_dof_ = 6;
    floating_base_joint_num_ = 2;  // createFloatingBaseModel add 1,pinocchio create 1 univese joint
  } else {
    pinocchio::urdf::buildModel(model_info.urdf_file_path, model, model_info.print_pinocchio_info);
    floating_base_joint_num_ = 1;  // pinocchio create 1 univese joint
    base_dof_ = 0;
  }
  model_ptr_ = std::make_shared<Model>(model);
  data_ptr_ = std::make_unique<Data>(*model_ptr_);
  info_ptr_ = std::make_shared<PinocchioModelInfo>(model_info);
}

int PinocchioInterface::GetLinkID(const std::string& link_name) {
  if (!model_ptr_->existBodyName(link_name)) {
    const char* cstr = link_name.c_str();
    printf("Error,link name: [%s] does not exits in URDF, please check\n", cstr);
    abort();
  }
  return model_ptr_->getBodyId(link_name);
}
int PinocchioInterface::GetEndEffectorNumber() {
  int counter{0};
  for (std::string name : info_ptr_->end_effector_names) {
    counter = model_ptr_->existBodyName(name) ? counter + 1 : counter;
  }
  return counter;
};

int PinocchioInterface::GetLinkParentJointID(const std::string& link_name) {
  return model_ptr_->frames[GetLinkID(link_name)].parent;
}

int PinocchioInterface::GetJointID(const std::string& joint_name) { return model_ptr_->getJointId(joint_name); }

std::vector<int> PinocchioInterface::GetAllRelatedJointParentIDsExceptBase(std::vector<int> joint_ids) {
  std::vector<int> parents;
  parents.clear();
  assert(joint_ids.size() > 0 && "joint_ids is empty");
  for (int i = 0; i < joint_ids.size(); ++i) {
    int joint_id = joint_ids[i];
    assert(joint_id >= 0 && joint_id < model_ptr_->parents.size() && "joint_id out of range");
    auto it = std::find(parents.begin(), parents.end(), joint_id);  // check if joint_id is already in parents
    if (it == parents.end()) {
      parents.insert(parents.begin(), joint_id);
    }
    /*find all parents joint ids*/
    while (true) {
      if (model_ptr_->parents[joint_id] == joint_id) {  // base joint id and its parent are both 0
        break;
      }  // check if joint_id is already in parents
      auto it = std::find(parents.begin(), parents.end(), model_ptr_->parents[joint_id]);
      if (it == parents.end()) {
        parents.insert(parents.begin(), model_ptr_->parents[joint_id]);
      }
      joint_id = model_ptr_->parents[joint_id];
    }
  }
  /*resort parents and remove joint ids of floating base*/
  std::sort(parents.begin(), parents.end());
  for (int i = 0; i < floating_base_joint_num_; ++i) {
    parents.erase(parents.begin());
  }

  return parents;
}

// void PinocchioInterface::computeKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
//   assert(qpos.rows() == model_ptr_->nv && "qpos size not match");
//   assert(qvel.rows() == model_ptr_->nv && "qvel size not match");
//   pinocchio::forwardKinematics(*model_ptr_, *data_ptr_, qpos, qvel);
//   pinocchio::updateFramePlacements(*model_ptr_, *data_ptr_);
//   pinocchio::computeJointJacobians(*model_ptr_, *data_ptr_, qpos);
// }

// void PinocchioInterface::computeDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
//   // computeKinematics(qpos, qvel);
//   pinocchio::crba(*model_ptr_, *data_ptr_, qpos);
//   pinocchio::nonLinearEffects(*model_ptr_, *data_ptr_, qpos, qvel);
//   pinocchio::computeMinverse(*model_ptr_, *data_ptr_, qpos);
// }

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
void PinocchioInterface::computeCentroidalDynamics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
  pinocchio::computeCentroidalMomentum(*model_ptr_, *data_ptr_, qpos, qvel);
}

double PinocchioInterface::getLinkMass(int joint_index) { return model_ptr_->inertias[joint_index].mass(); }

Eigen::Matrix3d PinocchioInterface::getLinkInertia(int joint_index) {
  return model_ptr_->inertias[joint_index].inertia();
};
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
}

Eigen::Vector3d PinocchioInterface::getCenterOfMass() { return data_ptr_->com[0]; }

// add 6 joints to the root link,construct floating base model
void PinocchioInterface::createFloatingBaseModel(const std::string& urdfFilePath, Model& model, bool print_info) {
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  pinocchio::urdf::buildModel(urdfFilePath, jointComposite, model, false);
};
}  // namespace pino
