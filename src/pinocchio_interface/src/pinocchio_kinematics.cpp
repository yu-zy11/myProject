#include "pinocchio_interface/pinocchio_kinematics.h"
#include "pinocchio_interface/pseudo_inverse.h"
// PinocchioKinematics::PinocchioKinematics() {
namespace pino {
PinocchioKinematics::PinocchioKinematics(std::shared_ptr<PinocchioInterface> pino_ptr) {
  pino_ptr_ = pino_ptr;
  pino_ptr_->GetModel() = pino_ptr_->GetModel();
  pino_ptr_->GetData() = pino_ptr_->GetData();
}

void PinocchioKinematics::FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                                     std::vector<EndEffectorData>& data) {
  assert(qpos.size() == qvel.size() && "size of qpos must same with size of qvel");
  assert(qpos.size() == pino_ptr_->GetModel()->nv && "size of qpos must same with model dof");
  Eigen::VectorXd qpos_tmp = qpos;
  Eigen::VectorXd qvel_tmp = qvel;
  if (pino_ptr_->getBaseDoF() > 0) {  // set joint angles of base to zero
    qpos_tmp.head(pino_ptr_->getBaseDoF()) = Eigen::VectorXd::Zero(pino_ptr_->getBaseDoF());
    qvel_tmp.head(pino_ptr_->getBaseDoF()) = Eigen::VectorXd::Zero(pino_ptr_->getBaseDoF());
  }
  pinocchio::forwardKinematics(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
  pinocchio::updateFramePlacements(*pino_ptr_->GetModel(), *pino_ptr_->GetData());
  pinocchio::computeJointJacobians(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
  data.resize(pino_ptr_->GetModelInfo()->end_effector_names.size());

  for (size_t i = 0; i < pino_ptr_->GetModelInfo()->end_effector_names.size(); ++i) {
    size_t link_id = pino_ptr_->GetLinkID(pino_ptr_->GetModelInfo()->end_effector_names[i]);

    auto vel_tmp = pinocchio::getFrameVelocity(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), link_id,
                                               pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian =
        Eigen::MatrixXd::Zero(6, pino_ptr_->GetModel()->nv);  // must setZero before getFrameJacobian
    Eigen::MatrixXd jacobian_derivative = Eigen::MatrixXd::Zero(6, pino_ptr_->GetModel()->nv);
    pinocchio::getFrameJacobian(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), link_id,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    pinocchio::getFrameJacobianTimeVariation(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), link_id,
                                             pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_derivative);
    data[i].pos = pino_ptr_->GetData()->oMf[link_id].translation();
    data[i].rotm = pino_ptr_->GetData()->oMf[link_id].rotation();
    data[i].vel = vel_tmp.linear();
    data[i].omega = vel_tmp.angular();
    data[i].jacobian = jacobian;
    data[i].jacobian_derivative = jacobian_derivative;
  }
}

void PinocchioKinematics::FixedBaseInverseKinematics(const Eigen::VectorXd& qpos_ref,
                                                     std::vector<std::string> user_selected_end_effector_name,
                                                     const std::vector<EndEffectorData>& data_des,
                                                     Eigen::VectorXd& qpos_des) {
  const double MAX_ERROR = 1e-4;
  const int MAX_COUNTER = 20;
  const double MAX_COMPOSITE_ERROR_TRANS = 0.15;
  const double MAX_COMPOSITE_ERROR_ORI = 0.4;
  // check dimensions of input
  assert(qpos_ref.size() == pino_ptr_->getDoF() && "the size of reference qpos must be equal to joint bumber");
  assert(data_des.size() == pino_ptr_->GetModelInfo()->end_effector_names.size() &&
         "the size of data must be equal to end_effector size()");
  assert(data_des[0].jacobian.cols() == qpos_ref.size() && "wrong size of data_des.jacobian");
  assert(user_selected_end_effector_name.size() > 0 && user_selected_end_effector_name.size() <= data_des.size() &&
         "wrong size of user_selected_end_effector_name");

  size_t selected_end_effector_num = user_selected_end_effector_name.size();
  Eigen::VectorXd qpos = qpos_ref;
  Eigen::MatrixXd composite_Jacobian;
  Eigen::VectorXd composite_error;
  composite_Jacobian.resize(selected_end_effector_num * 6, qpos_ref.size());
  composite_error.resize(selected_end_effector_num * 6, 1);
  Eigen::VectorXd qvel = qpos * 0.0;
  std::vector<EndEffectorData> data;
  std::vector<int> related_joint_ids;
  related_joint_ids.clear();
  int counter{0};
  while (true) {
    counter++;
    FixedBaseForwardKinematics(qpos, qvel, data);
    for (int i = 0; i < selected_end_effector_num; ++i) {
      /** get use selected end effector index in datas */
      auto it = std::find(pino_ptr_->GetModelInfo()->end_effector_names.begin(),
                          pino_ptr_->GetModelInfo()->end_effector_names.end(), user_selected_end_effector_name[i]);
      if (it == pino_ptr_->GetModelInfo()->end_effector_names.end()) {
        printf("Error,link name: [%s] does not exits in endeffctor names, please check\n",
               user_selected_end_effector_name[i].c_str());
        abort();
      }
      int index = std::distance(pino_ptr_->GetModelInfo()->end_effector_names.begin(), it);

      /*get position and orirtation error*/
      composite_Jacobian.block(i * 6, 0, 6, qpos_ref.size()) = data[index].jacobian;
      composite_error.block(i * 6, 0, 3, 1) = data_des[index].pos - data[index].pos;  // position erorr
      Eigen::Matrix3d rotm_error = data_des[index].rotm * data[index].rotm.transpose();
      Eigen::AngleAxis<double> theta_lambda(rotm_error);
      composite_error.block(i * 6 + 3, 0, 3, 1) = theta_lambda.angle() * theta_lambda.axis();

      /*add limit to position and orirtation errors*/
      if (composite_error.block(i * 6, 0, 3, 1).norm() > MAX_COMPOSITE_ERROR_TRANS) {
        composite_error.block(i * 6, 0, 3, 1) *=
            MAX_COMPOSITE_ERROR_TRANS / composite_error.block(i * 6, 0, 3, 1).norm();
      }
      if (composite_error.block(i * 6 + 3, 0, 3, 1).norm() > MAX_COMPOSITE_ERROR_ORI) {
        composite_error.block(i * 6 + 3, 0, 3, 1) *=
            MAX_COMPOSITE_ERROR_ORI / composite_error.block(i * 6 + 3, 0, 3, 1).norm();
      }
      /*get related joint ids for selected end effector*/
      related_joint_ids.push_back(pino_ptr_->GetLinkParentJointID(user_selected_end_effector_name[i]));
    }

    /*get all joint parent ids which relative to  selected end effectors, joint ids are related with cols of
     *jacobian,column=jointid-base_joint_num+base_dof. so we
     * can select related columns of jacobian to form  net jacobian,used in inverse kimenatics.un-relative joints
     * columns jacobians are all zero,zero colums in jacobian could cause bad results*/
    std::vector<int> selected_joint_ids = pino_ptr_->GetAllRelatedJointParentIDsExceptBase(related_joint_ids);
    Eigen::MatrixXd net_jacobian;
    net_jacobian.resize(composite_Jacobian.rows(), selected_joint_ids.size());
    net_jacobian.setZero();
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      net_jacobian.col(i) = composite_Jacobian.col(pino_ptr_->JointIdToJacobianColumns(selected_joint_ids[i]));
    }
    // std::cout << "net_jacobian:\n" << net_jacobian << std::endl;
    // std::cout << "composite_Jacobian:\n" << composite_Jacobian << std::endl;

    Eigen::MatrixXd jacobian_inverse_test;
    core::math::dumpedPseudoInverse(net_jacobian, 0.001, jacobian_inverse_test);
    Eigen::VectorXd delta_q1 = jacobian_inverse_test * composite_error;
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      qpos[pino_ptr_->JointIdToJacobianColumns(selected_joint_ids[i])] += delta_q1[i];
    }

    /*limit joint position to -180~180*/
    for (int i = 0; i < qpos.rows(); ++i) {
      while (qpos[i] > kPi) {
        qpos[i] -= 2 * kPi;
      }
      while (qpos[i] < -kPi) {
        qpos[i] += 2 * kPi;
      }
    }
    /*limit joint position inside joint limits*/
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      int index = pino_ptr_->JointIdToJacobianColumns(selected_joint_ids[i]);
      if (qpos[index] > pino_ptr_->GetModel()->upperPositionLimit[index]) {
        qpos[index] = pino_ptr_->GetModel()->upperPositionLimit[index];
      } else if (qpos[index] < pino_ptr_->GetModel()->lowerPositionLimit[index]) {
        qpos[index] = pino_ptr_->GetModel()->lowerPositionLimit[index];
      } else {
      }
    }
    if (composite_error.norm() < MAX_ERROR) {
      break;
    }
    if (counter > MAX_COUNTER) {
      break;
    }
  }
  qpos_des = qpos;
}

void PinocchioKinematics::FixedBaseInverseKinematics3Dof(const Eigen::VectorXd& qpos_ref, std::string end_effector_name,
                                                         const Eigen::Vector3d& pos_des, Eigen::VectorXd& qpos_des) {}
}  // namespace pino
