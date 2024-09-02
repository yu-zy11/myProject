/*! @file quadrupedA01.h
 *  @brief Utility function to build a Mini Cheetah RobotBase object
 *
 *
 */

#include "RobotSA01a.h"

#include "PinocchioInterface.h"
#include "RobotBase.h"
#include "orientation_math.h"

template <typename T>
void dumpedPseudoInverse(Eigen::Matrix<T, -1, -1> const& matrix, double sigmaThreshold, Eigen::Matrix<T, -1, -1>& invMatrix) {
  if ((1 == matrix.rows()) && (1 == matrix.cols())) {
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }
  Eigen::JacobiSVD<Eigen::Matrix<T, -1, -1>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  Eigen::Matrix<T, -1, -1> invS = Eigen::Matrix<T, -1, -1>::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      invS.coeffRef(ii, ii) = 1.0 / sigmaThreshold;
      // printf("sigular value is too small: %f\n",svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
}

template <typename T>
RobotSA01a<T>::RobotSA01a() {
  char* cwd = get_current_dir_name();
  std::string S;
  S += cwd;
  free(cwd);
  // std::string urdf_file_path{S + A01_URDF_RELATIVE_PATH};
  std::string urdf_file_path{A01_URDF_RELATIVE_PATH};
  this->pino_.init(urdf_file_path);

  // get end effector link id,used for getting whole body jacobian
  this->base_link_id_ = this->pino_.getLinkID(this->base_link_name_);
  this->end_effector_id_.clear();
  for (int i = 0; i < end_effector_name_.size(); ++i) {
    this->end_effector_id_.push_back(this->pino_.getLinkID(end_effector_name_[i]));
  }
  this->data_.resize(end_effector_name_.size());
  this->robot_type_ = RobotType::SA01a;

  const int dof = this->pino_.getDoF();
  this->qpos_upper_limit_ = this->pino_.getJointUpperPositionLimit().tail(dof - base_dof_);
  this->qpos_lower_limit_ = this->pino_.getJointLowerPositionLimit().tail(dof - base_dof_);
  this->qvel_limit_ = this->pino_.getJointVelocityLimit().tail(dof - base_dof_);
  this->qtor_limit_ = this->pino_.getJointTorqueLimit().tail(dof - base_dof_);
  printf("[SA01a]init total mass: %f\n", this->pino_.getTotalMass());
}
template <typename T>
void RobotSA01a<T>::localForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data) {
  assert(qpos.size() == qvel.size() && "size of qpos must same with size of qvel");
  assert(qpos.size() == this->pino_.getDoF() - base_dof_ && "size of qpos must same with actuator number");
  Eigen::VectorXd qpos_tmp, qvel_tmp;
  qpos_tmp.resize(this->pino_.getDoF());
  qvel_tmp.resize(this->pino_.getDoF());
  qpos_tmp.setZero();
  qvel_tmp.setZero();
  qpos_tmp.tail(this->pino_.getDoF() - base_dof_) = qpos.template cast<double>();
  qvel_tmp.tail(this->pino_.getDoF() - base_dof_) = qvel.template cast<double>();
  // std::cout << "qpos_tmp:" << qpos_tmp.transpose() << std::endl;
  // std::cout << "qvel_tmp:" << qvel_tmp.transpose() << std::endl;
  this->pino_.computeKinematics(qpos_tmp, qvel_tmp);
  data.resize(end_effector_name_.size());

  endEffectorData<double> data_tmp;
  data_tmp.reshape(this->pino_.getDoF());
  for (size_t i = 0; i < end_effector_name_.size(); ++i) {
    this->pino_.getLinkKinematicsData(this->end_effector_id_[i], data_tmp.pos, data_tmp.rotm, data_tmp.vel, data_tmp.omega, data_tmp.jacobian, data_tmp.jacobian_derivative);
    data[i].pos = data_tmp.pos.template cast<T>();
    data[i].rotm = data_tmp.rotm.template cast<T>();
    data[i].vel = data_tmp.vel.template cast<T>();
    data[i].omega = data_tmp.omega.template cast<T>();
    data[i].jacobian = data_tmp.jacobian.block(0, base_dof_, 6, this->pino_.getDoF() - base_dof_).template cast<T>();
    data[i].jacobian_derivative = data_tmp.jacobian_derivative.block(0, base_dof_, 6, this->pino_.getDoF() - base_dof_).template cast<T>();
  }
}

template <typename T>
void RobotSA01a<T>::fullForwardKinematics(const Eigen::Matrix<T, -1, 1>& qpos, const Eigen::Matrix<T, -1, 1>& qvel, std::vector<endEffectorData<T>>& data) {
  assert(qpos.size() == qvel.size() && "size of qpos must same with size of qvel");
  assert(qpos.size() == this->pino_.getDoF() && "size of qpos must same with freedom of robot");
  Eigen::VectorXd qpos_tmp = qpos.template cast<double>();
  Eigen::VectorXd qvel_tmp = qvel.template cast<double>();

  this->pino_.computeKinematics(qpos_tmp, qvel_tmp);
  data.resize(end_effector_name_.size());

  endEffectorData<double> data_tmp;
  data_tmp.reshape(this->pino_.getDoF());
  for (size_t i = 0; i < end_effector_name_.size(); ++i) {
    this->pino_.getLinkKinematicsData(this->end_effector_id_[i], data_tmp.pos, data_tmp.rotm, data_tmp.vel, data_tmp.omega, data_tmp.jacobian, data_tmp.jacobian_derivative);
    data[i].pos = data_tmp.pos.template cast<T>();
    data[i].rotm = data_tmp.rotm.template cast<T>();
    data[i].vel = data_tmp.vel.template cast<T>();
    data[i].omega = data_tmp.omega.template cast<T>();
    data[i].jacobian = data_tmp.jacobian.template cast<T>();
    data[i].jacobian_derivative = data_tmp.jacobian_derivative.template cast<T>();
    std::cout << "data_tmp.jacobian:\n" << data_tmp.jacobian << std::endl;
    std::cout << "data[i].jacobian:\n" << data[i].jacobian << std::endl;
    std::cout << "data_tmp.jacobian_derivative:\n" << data_tmp.jacobian_derivative << std::endl;
    std::cout << "data[i].jacobian_derivative:\n" << data[i].jacobian_derivative << std::endl;
  }
}

template <typename T>
void RobotSA01a<T>::localInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<int> user_selected_end_effector_id, const std::vector<endEffectorData<T>>& data_des,
                                           Eigen::Matrix<T, -1, 1>& qpos_des) {
  const double MAX_ERROR = 1e-4;
  const int MAX_COUNTER = 20;
  const double MAX_COMPOSITE_ERROR_TRANS = 0.15;
  const double MAX_COMPOSITE_ERROR_ORI = 0.4;
  const T PI{3.1415926};
  // check dimensions of input
  assert(qpos_ref.size() == (this->pino_.getDoF() - base_dof_) && "the size of reference qpos must be equal to joint bumber");
  assert(data_des.size() == end_effector_name_.size() && "the size of data must be equal to end_effector size()");
  assert(data_des[0].jacobian.cols() == qpos_ref.size() && "wrong size of data_des.jacobian");
  assert(user_selected_end_effector_id.size() > 0 && "wrong size of user_selected_end_effector_id");
  for (int i = 0; i < user_selected_end_effector_id.size(); ++i) {
    assert(user_selected_end_effector_id[i] < end_effector_name_.size() && "wrong id of end effector");
  }

  size_t end_effector_num = user_selected_end_effector_id.size();
  Eigen::Matrix<T, -1, 1> qpos = qpos_ref;
  Eigen::Matrix<T, -1, -1> composite_Jacobian;
  Eigen::Matrix<T, -1, 1> composite_error;
  composite_Jacobian.resize(end_effector_num * 6, qpos_ref.size());
  composite_error.resize(end_effector_num * 6, 1);
  Eigen::Matrix<T, -1, 1> qvel = qpos * 0.0;
  std::vector<endEffectorData<T>> data;
  int counter{0};
  while (true) {
    counter++;
    localForwardKinematics(qpos, qvel, data);
    for (int i = 0; i < end_effector_num; ++i) {
      int endeffector_id = user_selected_end_effector_id[i];
      composite_Jacobian.block(i * 6, 0, 6, qpos_ref.size()) = data[endeffector_id].jacobian;
      composite_error.block(i * 6, 0, 3, 1) = data_des[endeffector_id].pos - data[endeffector_id].pos;  // position erorr
      Eigen::Matrix<T, 3, 3> rotm_error = data_des[endeffector_id].rotm * data[endeffector_id].rotm.transpose();
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
    // std::cout << "composite_error:\n" << composite_error << std::endl;
    // Eigen::Matrix<T, -1, 1> dq1 = composite_Jacobian.colPivHouseholderQr().solve(composite_error);  // ldlt()
    Eigen::Matrix<T, -1, -1> invMatrix;
    dumpedPseudoInverse(composite_Jacobian, 0.001, invMatrix);
    Eigen::Matrix<T, -1, 1> dq3 = invMatrix * composite_error;
    qpos += dq3;
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
void RobotSA01a<T>::fullInverseKinematics(const Eigen::Matrix<T, -1, 1>& qpos_ref, std::vector<int> user_selected_end_effector_id, const std::vector<endEffectorData<T>>& data_des,
                                          Eigen::Matrix<T, -1, 1>& qpos_des) {
  const double MAX_ERROR = 1e-4;
  const int MAX_COUNTER = 20;
  const double MAX_COMPOSITE_ERROR_TRANS = 0.15;
  const double MAX_COMPOSITE_ERROR_ORI = 0.4;
  const T PI{3.1415926};
  // check dimensions of input
  assert(qpos_ref.size() == this->pino_.getDoF() && "the size of reference qpos must be equal to dof of robot");
  assert(data_des.size() == end_effector_name_.size() && "the size of data must be equal to end_effector size()");
  assert(data_des[0].jacobian.cols() == qpos_ref.size() && "wrong size of data_des.jacobian");
  assert(user_selected_end_effector_id.size() > 0 && user_selected_end_effector_id.size() == data_des.size() && "wrong size of user_selected_end_effector_id");
  for (int i = 0; i < user_selected_end_effector_id.size(); ++i) {
    assert(user_selected_end_effector_id[i] < end_effector_name_.size() && "wrong id of end effector");
  }

  size_t end_effector_num = user_selected_end_effector_id.size();
  Eigen::Matrix<T, -1, 1> qpos = qpos_ref;
  Eigen::Matrix<T, -1, -1> composite_Jacobian;
  Eigen::Matrix<T, -1, 1> composite_error;
  composite_Jacobian.resize(end_effector_num * 6, qpos_ref.size());
  composite_error.resize(end_effector_num * 6, 1);
  // composite_Jacobian composite_error;
  Eigen::Matrix<T, -1, 1> qvel = qpos * 0.0;
  std::vector<endEffectorData<T>> data;
  int counter{0};
  while (true) {
    counter++;
    fullForwardKinematics(qpos, qvel, data);
    for (int i = 0; i < end_effector_num; ++i) {
      int endeffector_id = user_selected_end_effector_id[i];
      composite_Jacobian.block(i * 6, 0, 6, qpos_ref.size()) = data[endeffector_id].jacobian;
      composite_error.block(i * 6, 0, 3, 1) = data_des[endeffector_id].pos - data[endeffector_id].pos;  // position erorr
      Eigen::Matrix<T, 3, 3> rotm_error = data_des[endeffector_id].rotm * data[endeffector_id].rotm.transpose();
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
    Eigen::Matrix<T, -1, -1> invMatrix;
    dumpedPseudoInverse(composite_Jacobian, 0.001, invMatrix);
    Eigen::Matrix<T, -1, 1> dq = invMatrix * composite_error;
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
void RobotSA01a<T>::limitJointPosition(Eigen::Matrix<T, -1, 1>& qpos) {
  const T PI{3.1415926};
  assert(qpos.size() == (this->pino_.getDoF() - base_dof_) && "qpos size not match in limitJointPosition");
  for (int i = 0; i < qpos.size(); ++i) {
    /* limit qpos to qpos_upper_limit_~qpos_lower_limit_*/
    if (qpos[i] > this->qpos_upper_limit_[i]) {
      qpos[i] = this->qpos_upper_limit_[i];
    } else if (qpos[i] < this->qpos_lower_limit_[i]) {
      qpos[i] = this->qpos_lower_limit_[i];
    } else {
    }
  }
}

template class RobotSA01a<double>;
template class RobotSA01a<float>;
