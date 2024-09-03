
// clang-format off
#include "PinocchioInterface.h"
#include "RobotBase.h"
#include "PseudoInverse.h"
#include "orientation_tools.h"
// clang-format on

template <typename T>
void RobotBase<T>::localForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data) {
  assert(qpos.size() == qvel.size() && "size of qpos must same with size of qvel");
  assert(qpos.size() == pino_.getDoF() - pino_.getBaseDoF() && "size of qpos must same with actuator number");
  Eigen::VectorXd qpos_tmp, qvel_tmp;
  qpos_tmp.resize(pino_.getDoF());
  qvel_tmp.resize(pino_.getDoF());
  qpos_tmp.setZero();
  qvel_tmp.setZero();
  qpos_tmp.tail(pino_.getDoF() - pino_.getBaseDoF()) = qpos.template cast<double>();
  qvel_tmp.tail(pino_.getDoF() - pino_.getBaseDoF()) = qvel.template cast<double>();
  pino_.computeKinematics(qpos_tmp, qvel_tmp);
  data.resize(end_effector_name_.size());

  endEffectorData<double> data_tmp;
  data_tmp.reshape(pino_.getDoF());
  for (size_t i = 0; i < end_effector_name_.size(); ++i) {
    pino_.getLinkKinematicsData(pino_.getLinkID(end_effector_name_[i]), data_tmp.pos, data_tmp.rotm, data_tmp.vel, data_tmp.omega, data_tmp.jacobian, data_tmp.jacobian_derivative);
    data[i].pos = data_tmp.pos.template cast<T>();
    data[i].rotm = data_tmp.rotm.template cast<T>();
    data[i].vel = data_tmp.vel.template cast<T>();
    data[i].omega = data_tmp.omega.template cast<T>();
    data[i].jacobian = data_tmp.jacobian.block(0, pino_.getBaseDoF(), 6, pino_.getDoF() - pino_.getBaseDoF()).template cast<T>();
    data[i].jacobian_derivative = data_tmp.jacobian_derivative.block(0, pino_.getBaseDoF(), 6, pino_.getDoF() - pino_.getBaseDoF()).template cast<T>();
  }
}
template <typename T>
void RobotBase<T>::fullForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data) {
  assert(qpos.size() == qvel.size() && "size of qpos must same with size of qvel");
  assert(qpos.size() == pino_.getDoF() && "size of qpos must same with freedom of robot");
  Eigen::VectorXd qpos_tmp = qpos.template cast<double>();
  Eigen::VectorXd qvel_tmp = qvel.template cast<double>();

  pino_.computeKinematics(qpos_tmp, qvel_tmp);
  data.resize(end_effector_name_.size());

  endEffectorData<double> data_tmp;
  data_tmp.reshape(pino_.getDoF());
  for (size_t i = 0; i < end_effector_name_.size(); ++i) {
    pino_.getLinkKinematicsData(pino_.getLinkID(end_effector_name_[i]), data_tmp.pos, data_tmp.rotm, data_tmp.vel, data_tmp.omega, data_tmp.jacobian, data_tmp.jacobian_derivative);
    data[i].pos = data_tmp.pos.template cast<T>();
    data[i].rotm = data_tmp.rotm.template cast<T>();
    data[i].vel = data_tmp.vel.template cast<T>();
    data[i].omega = data_tmp.omega.template cast<T>();
    data[i].jacobian = data_tmp.jacobian.template cast<T>();
    data[i].jacobian_derivative = data_tmp.jacobian_derivative.template cast<T>();
  }
}
template <typename T>
void RobotBase<T>::localInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<std::string> user_selected_end_effector_name, const std::vector<endEffectorData<T>>& data_des,
                                          Eigen::Matrix<T, -1, 1>& qpos_des) {
  const double MAX_ERROR = 1e-4;
  const int MAX_COUNTER = 20;
  const double MAX_COMPOSITE_ERROR_TRANS = 0.15;
  const double MAX_COMPOSITE_ERROR_ORI = 0.4;
  const T PI{3.1415926};
  // check dimensions of input
  assert(qpos_ref.size() == (pino_.getDoF() - pino_.getBaseDoF()) && "the size of reference qpos must be equal to joint bumber");
  assert(data_des.size() == end_effector_name_.size() && "the size of data must be equal to end_effector size()");
  assert(data_des[0].jacobian.cols() == qpos_ref.size() && "wrong size of data_des.jacobian");
  assert(user_selected_end_effector_name.size() > 0 && user_selected_end_effector_name.size() <= data_des.size() && "wrong size of user_selected_end_effector_name");

  size_t selected_end_effector_num = user_selected_end_effector_name.size();
  Eigen::Matrix<T, -1, 1> qpos = qpos_ref;
  Eigen::Matrix<T, -1, -1> composite_Jacobian;
  Eigen::Matrix<T, -1, 1> composite_error;
  composite_Jacobian.resize(selected_end_effector_num * 6, qpos_ref.size());
  composite_error.resize(selected_end_effector_num * 6, 1);
  Eigen::Matrix<T, -1, 1> qvel = qpos * 0.0;
  std::vector<endEffectorData<T>> data;
  std::vector<int> joint_ids;
  joint_ids.clear();
  int counter{0};
  while (true) {
    counter++;
    localForwardKinematics(qpos, qvel, data);
    for (int i = 0; i < selected_end_effector_num; ++i) {
      int index = getIndexInEndEffectorDatas(user_selected_end_effector_name[i]);  // pino_.getLinkID(user_selected_end_effector_name[i]);
      composite_Jacobian.block(i * 6, 0, 6, qpos_ref.size()) = data[index].jacobian;
      composite_error.block(i * 6, 0, 3, 1) = data_des[index].pos - data[index].pos;  // position erorr
      Eigen::Matrix<T, 3, 3> rotm_error = data_des[index].rotm * data[index].rotm.transpose();
      Eigen::Matrix<T, 4, 1> axis_angle = rotm2axisangle(rotm_error);
      composite_error.block(i * 6 + 3, 0, 3, 1) = axis_angle.head(3) * axis_angle[3];  // orientation erorr

      /*add limit to position and orirtation errors*/
      if (composite_error.block(i * 6, 0, 3, 1).norm() > MAX_COMPOSITE_ERROR_TRANS) {
        composite_error.block(i * 6, 0, 3, 1) *= MAX_COMPOSITE_ERROR_TRANS / composite_error.block(i * 6, 0, 3, 1).norm();
      }
      if (composite_error.block(i * 6 + 3, 0, 3, 1).norm() > MAX_COMPOSITE_ERROR_ORI) {
        axis_angle[3] *= MAX_COMPOSITE_ERROR_ORI / composite_error.block(i * 6 + 3, 0, 3, 1).norm();
      }
      /*get joint ids for selected end effector*/
      joint_ids.push_back(pino_.getLinkParentJointID(user_selected_end_effector_name[i]));
    }
    /*get all joint parent ids which relative to jacobians of selected end effector*/
    std::vector<int> selected_joint_ids = pino_.getAllJointParentIDsExceptBase(joint_ids);
    /*get net jacobian relative to relative joints,remove columns of un-relative joints,whose jacobians are all zero,zero colums in jacobian could cause bad results*/
    Eigen::Matrix<T, -1, -1> net_jacobian;
    net_jacobian.resize(composite_Jacobian.rows(), selected_joint_ids.size());
    net_jacobian.setZero();
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      net_jacobian.col(i) = composite_Jacobian.col(pino_.JointIdToJacobianColumns(selected_joint_ids[i]) - pino_.getBaseDoF());
    }
    // std::cout << "net_jacobian:\n" << net_jacobian << std::endl;
    // std::cout << "composite_Jacobian:\n" << composite_Jacobian << std::endl;

    Eigen::Matrix<T, -1, -1> jacobian_inverse_test;
    dumpedPseudoInverse(net_jacobian, 0.001, jacobian_inverse_test);
    Eigen::Matrix<T, -1, 1> delta_q1 = jacobian_inverse_test * composite_error;
    for (int i = 0; i < selected_joint_ids.size(); ++i) {
      net_jacobian.col(i) = composite_Jacobian.col(pino_.JointIdToJacobianColumns(selected_joint_ids[i]) - pino_.getBaseDoF());
      qpos[pino_.JointIdToJacobianColumns(selected_joint_ids[i]) - pino_.getBaseDoF()] += delta_q1[i];
    }

    // Eigen::Matrix<T, -1, 1> dq1 = composite_Jacobian.colPivHouseholderQr().solve(composite_error);  // ldlt()
    // Eigen::Matrix<T, -1, -1> jacobian_inverse;
    // dumpedPseudoInverse(composite_Jacobian, 0.001, jacobian_inverse);
    // Eigen::Matrix<T, -1, 1> delta_q = jacobian_inverse * composite_error;

    /*limit joint position to -180~180*/
    for (int i = 0; i < qpos.rows(); ++i) {
      while (qpos[i] > PI) {
        qpos[i] -= 2 * PI;
      }
      while (qpos[i] < -PI) {
        qpos[i] += 2 * PI;
      }
    }
    limitJointPosition(qpos);
    if (composite_error.norm() < MAX_ERROR) {
      break;
    }
    if (counter > MAX_COUNTER) {
      break;
    }
  }
  qpos_des = qpos;
}

template <typename T>
void RobotBase<T>::fullInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<std::string> user_selected_end_effector_name, const std::vector<endEffectorData<T>>& data_des,
                                         Eigen::Matrix<T, -1, 1>& qpos_des) {
  const double MAX_ERROR = 1e-4;
  const int MAX_COUNTER = 20;
  const double MAX_COMPOSITE_ERROR_TRANS = 0.15;
  const double MAX_COMPOSITE_ERROR_ORI = 0.4;
  const T PI{3.1415926};
  // check dimensions of input
  assert(qpos_ref.size() == pino_.getDoF() && "the size of reference qpos must be equal to dof of robot");
  assert(data_des.size() == end_effector_name_.size() && "the size of data must be equal to end_effector size()");
  assert(data_des[0].jacobian.cols() == qpos_ref.size() && "wrong size of data_des.jacobian");
  assert(user_selected_end_effector_name.size() > 0 && user_selected_end_effector_name.size() <= data_des.size() && "wrong size of user_selected_end_effector_name");

  size_t selected_end_effector_num = user_selected_end_effector_name.size();
  Eigen::Matrix<T, -1, 1> qpos = qpos_ref;
  Eigen::Matrix<T, -1, -1> composite_Jacobian;
  Eigen::Matrix<T, -1, 1> composite_error;
  composite_Jacobian.resize(selected_end_effector_num * 6, qpos_ref.size());
  composite_error.resize(selected_end_effector_num * 6, 1);
  // composite_Jacobian composite_error;
  Eigen::Matrix<T, -1, 1> qvel = qpos * 0.0;
  std::vector<endEffectorData<T>> data;
  int counter{0};
  while (true) {
    counter++;
    fullForwardKinematics(qpos, qvel, data);
    for (int i = 0; i < selected_end_effector_num; ++i) {
      int index = getIndexInEndEffectorDatas(user_selected_end_effector_name[i]);
      composite_Jacobian.block(i * 6, 0, 6, qpos_ref.size()) = data[index].jacobian;
      composite_error.block(i * 6, 0, 3, 1) = data_des[index].pos - data[index].pos;  // position erorr
      Eigen::Matrix<T, 3, 3> rotm_error = data_des[index].rotm * data[index].rotm.transpose();
      Eigen::Matrix<T, 4, 1> axis_angle = rotm2axisangle(rotm_error);
      composite_error.block(i * 6 + 3, 0, 3, 1) = axis_angle.head(3) * axis_angle[3];  // orientation erorr
      /*add limit to position and orirtation error*/
      if (composite_error.block(i * 6, 0, 3, 1).norm() > MAX_COMPOSITE_ERROR_TRANS) {
        composite_error.block(i * 6, 0, 3, 1) *= MAX_COMPOSITE_ERROR_TRANS / composite_error.block(i * 6, 0, 3, 1).norm();
      }
      if (composite_error.block(i * 6 + 3, 0, 3, 1).norm() > MAX_COMPOSITE_ERROR_ORI) {
        axis_angle[3] *= MAX_COMPOSITE_ERROR_ORI / composite_error.block(i * 6 + 3, 0, 3, 1).norm();
      }
    }
    Eigen::Matrix<T, -1, -1> jacobian_inverse;
    dumpedPseudoInverse(composite_Jacobian, 0.001, jacobian_inverse);
    Eigen::Matrix<T, -1, 1> dq = jacobian_inverse * composite_error;
    qpos += dq;
    // std::cout << "qpos:" << qpos.transpose() << std::endl;

    for (int i = 0; i < qpos.rows(); ++i) { /* limit qpos to -180~180*/
      while (qpos[i] > PI) {
        qpos[i] -= 2 * PI;
      }
      while (qpos[i] < -PI) {
        qpos[i] += 2 * PI;
      }
    }
    limitJointPosition(qpos);
    if (dq.norm() < MAX_ERROR) {
      break;
    }
    if (counter > MAX_COUNTER) {
      break;
    }
  }
  qpos_des = qpos;
}

template <typename T>
void RobotBase<T>::limitJointPosition(Eigen::Matrix<T, -1, 1>& qpos) {
  const T PI{3.1415926};
  assert(qpos.size() == (pino_.getDoF() - pino_.getBaseDoF()) && "qpos size not match in limitJointPosition");
  for (int i = 0; i < qpos.size(); ++i) {
    /* limit qpos to qpos_upper_limit_~qpos_lower_limit_*/
    if (qpos[i] > qpos_upper_limit_[i]) {
      qpos[i] = qpos_upper_limit_[i];
    } else if (qpos[i] < qpos_lower_limit_[i]) {
      qpos[i] = qpos_lower_limit_[i];
    } else {
    }
  }
}
template <typename T>
int RobotBase<T>::getIndexInEndEffectorDatas(const std::string& name) {
  for (int i = 0; i < end_effector_name_.size(); ++i) {
    if (end_effector_name_[i] == name) {
      return i;
    }
  }
  printf("Error, end effector name: [%s] does not exits in end_effector_datas, please check\n", name.c_str());
  abort();
  return -1;
}

template class RobotBase<double>;
template class RobotBase<float>;
