#include "pinocchio_interface/pinocchio_kinematics.h"
#include "pinocchio_interface/pseudo_inverse.h"
// PinocchioKinematics::PinocchioKinematics() {
namespace pino {

void PinocchioKinematics::FullModelForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                                     std::vector<EndEffectorData>& data) {
  assert(qvel.size() == pino_ptr_->GetModel()->nv && "size of qpos must same with size of qvel");
  assert(qpos.size() == pino_ptr_->GetModel()->nv && "size of qpos must same with model dof");
  pinocchio::forwardKinematics(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos, qvel);
  pinocchio::updateFramePlacements(*pino_ptr_->GetModel(), *pino_ptr_->GetData());
  pinocchio::computeJointJacobians(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), qpos);
  data.resize(pino_ptr_->GetModelInfo()->end_effector_names.size());

  for (size_t i = 0; i < pino_ptr_->GetModelInfo()->end_effector_names.size(); ++i) {
    size_t link_id = pino_ptr_->GetLinkID(pino_ptr_->GetModelInfo()->end_effector_names[i]);

    auto vel_tmp = pinocchio::getFrameVelocity(*pino_ptr_->GetModel(), *pino_ptr_->GetData(), link_id,
                                               pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    // must setZero before getFrameJacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, pino_ptr_->GetModel()->nv);
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

void PinocchioKinematics::FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel,
                                                     std::vector<EndEffectorData>& data) {
  Eigen::VectorXd qpos_tmp = qpos;
  Eigen::VectorXd qvel_tmp = qvel;
  if (pino_ptr_->getBaseDoF() > 0) {
    qpos_tmp.head(pino_ptr_->getBaseDoF()) = Eigen::VectorXd::Zero(pino_ptr_->getBaseDoF());
    qvel_tmp.head(pino_ptr_->getBaseDoF()) = Eigen::VectorXd::Zero(pino_ptr_->getBaseDoF());
  }
  FullModelForwardKinematics(qpos_tmp, qvel_tmp, data);
}

void PinocchioKinematics::FixedBaseForwardKinematics(const Eigen::VectorXd& qpos, std::vector<EndEffectorData>& data) {
  Eigen::VectorXd qvel = Eigen::VectorXd::Zero(qpos.size());
  FixedBaseForwardKinematics(qpos, qvel, data);
}

void PinocchioKinematics::FixedBaseInverseKinematics(const Eigen::VectorXd& qpos_ref,
                                                     std::vector<std::string> user_selected_end_effector_name,
                                                     const std::vector<EndEffectorData>& data_des,
                                                     Eigen::VectorXd& qpos_des) {
  // check dimensions of input
  assert(qpos_ref.size() == pino_ptr_->getDoF() && "the size of reference qpos must be equal to joint bumber");
  assert(data_des.size() == pino_ptr_->GetModelInfo()->end_effector_names.size() &&
         "the data size  must be equal to end_effector size()");
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
      if (composite_error.block(i * 6, 0, 3, 1).norm() > kMaxPositionError) {
        composite_error.block(i * 6, 0, 3, 1) *= kMaxPositionError / composite_error.block(i * 6, 0, 3, 1).norm();
      }
      if (composite_error.block(i * 6 + 3, 0, 3, 1).norm() > kMaxOrietationError) {
        composite_error.block(i * 6 + 3, 0, 3, 1) *=
            kMaxOrietationError / composite_error.block(i * 6 + 3, 0, 3, 1).norm();
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
    Eigen::MatrixXd jacobian_inverse;
    core::math::dumpedPseudoInverse(net_jacobian, 0.001, jacobian_inverse);
    Eigen::VectorXd delta_qos = jacobian_inverse * composite_error;
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      qpos[pino_ptr_->JointIdToJacobianColumns(selected_joint_ids[i])] += delta_qos[i];
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
    if (composite_error.norm() < kMaxError) {
      break;
    }
    if (counter > kMaxIterationCounter) {
      break;
    }
  }
  qpos_des = qpos;
}

void PinocchioKinematics::FixedBaseInverseKinematics(const Eigen::VectorXd& qpos_ref,
                                                     const std::string& end_effector_name,
                                                     const Eigen::Vector3d& pos_des, const Eigen::Matrix3d& rotm_des,
                                                     Eigen::VectorXd& qpos_des) {
  auto it = std::find(pino_ptr_->GetModelInfo()->end_effector_names.begin(),
                      pino_ptr_->GetModelInfo()->end_effector_names.end(), end_effector_name);
  if (it == pino_ptr_->GetModelInfo()->end_effector_names.end()) {
    printf("Error,link name: [%s] does not exits in endeffctor names, please check\n", end_effector_name.c_str());
    abort();
  }
  int index = std::distance(pino_ptr_->GetModelInfo()->end_effector_names.begin(), it);

  std::vector<EndEffectorData> data_des;
  FixedBaseForwardKinematics(qpos_ref, data_des);
  data_des[index].pos = pos_des;
  data_des[index].rotm = rotm_des;

  std::vector<std::string> ee_name;
  ee_name.clear();
  ee_name.push_back(end_effector_name);
  FixedBaseInverseKinematics(qpos_ref, ee_name, data_des, qpos_des);
}

void PinocchioKinematics::FixedBaseInverseKinematics3Dof(const Eigen::VectorXd& qpos_ref,
                                                         const std::string& end_effector_name,
                                                         const Eigen::Vector3d& pos_des, Eigen::VectorXd& qpos_des) {
  // check dimensions of input
  assert(qpos_ref.size() == pino_ptr_->getDoF() && "the size of reference qpos must be equal to joint bumber");
  Eigen::VectorXd qpos = qpos_ref;
  Eigen::MatrixXd composite_Jacobian;
  Eigen::VectorXd composite_error;
  composite_Jacobian.resize(3, qpos_ref.size());
  composite_error.resize(3, 1);
  Eigen::VectorXd qvel = qpos * 0.0;
  std::vector<EndEffectorData> data;
  std::vector<int> related_joint_ids;
  related_joint_ids.clear();
  related_joint_ids.push_back(pino_ptr_->GetLinkParentJointID(end_effector_name));

  std::vector<int> selected_joint_ids = pino_ptr_->GetAllRelatedJointParentIDsExceptBase(related_joint_ids);
  int counter{0};
  while (true) {
    counter++;
    FixedBaseForwardKinematics(qpos, qvel, data);
    /** get use selected end effector index in datas */
    auto it = std::find(pino_ptr_->GetModelInfo()->end_effector_names.begin(),
                        pino_ptr_->GetModelInfo()->end_effector_names.end(), end_effector_name);
    if (it == pino_ptr_->GetModelInfo()->end_effector_names.end()) {
      printf("Error,link name: [%s] does not exits in endeffctor names, please check\n", end_effector_name.c_str());
      abort();
    }
    int index = std::distance(pino_ptr_->GetModelInfo()->end_effector_names.begin(), it);

    /*get position  error,and add limit to position and orirtation error*/
    composite_error = pos_des - data[index].pos;
    if (composite_error.norm() > kMaxPositionError) {
      composite_error *= kMaxPositionError / composite_error.norm();
    }

    /*get net jacobian related to target pos.first get all joint parent ids which relative to  selected end effectors,
     *joint ids are related with cols of jacobian,column=jointid-base_joint_num+base_dof. so we can select related
     *columns of jacobian to form  net jacobian.un-relative joints columns jacobians are all zero,zero columns in
     *jacobian could cause bad results*/
    composite_Jacobian = data[index].jacobian.topRows(3);

    Eigen::MatrixXd net_jacobian;
    net_jacobian.resize(3, selected_joint_ids.size());
    net_jacobian.setZero();
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      net_jacobian.col(i) = composite_Jacobian.col(pino_ptr_->JointIdToJacobianColumns(selected_joint_ids[i]));
    }

    Eigen::MatrixXd jacobian_inverse;
    core::math::dumpedPseudoInverse(net_jacobian, 0.001, jacobian_inverse);
    Eigen::VectorXd delta_qos = jacobian_inverse * composite_error;
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      qpos[pino_ptr_->JointIdToJacobianColumns(selected_joint_ids[i])] += delta_qos[i];
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
    if (composite_error.norm() < kMaxError) {
      break;
    }
    if (counter > kMaxIterationCounter) {
      break;
    }
  }
  qpos_des = qpos;
}
}  // namespace pino
