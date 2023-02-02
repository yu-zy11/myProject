#include "kinematics.h"
#include "robotics_math.h"
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
namespace ROBOTICS {
using namespace Math;
float angleInPi(float theta);
float nearZero(float theta);
#define ZERO 1e-6
void Kinematics::init() {
  // std::cout << "==============init Kinematics==============\n";
  clearConfig();
  // set joint sequence and joint number,not including 6joint for floating base
  Eigen::Matrix<int, 12, 1> joint_seq, joint_parent;
  Eigen::Matrix<int, 4, 1> foot_seq, foot_parent;
  joint_seq << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  joint_parent << -1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10; // -1:base
  foot_seq << 0, 1, 2, 3;
  foot_parent << 2, 5, 8, 11;

  config_.joint_num = joint_seq.rows();
  for (int i = 0; i < config_.joint_num; ++i) {
    config_.joint_seq.push_back(joint_seq[i]);
    config_.joint_parent.push_back(joint_parent[i]);
  }
  config_.foot_num = foot_parent.rows();
  for (int i = 0; i < config_.foot_num; ++i) {
    config_.foot_seq.push_back(foot_seq[i]);
    config_.foot_parent.push_back(foot_parent[i]);
  }
  // set Alist and Tlist for each joint
  Vec6f joint_axis[12];
  joint_axis[0] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; // joint 0
  joint_axis[1] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[2] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[3] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[4] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[5] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[6] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[7] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[8] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[9] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[10] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  joint_axis[11] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0; // joint 11

  Vec3f joint_position[12];
  joint_position[0] << 0.1875, -0.0875, 0.18;
  joint_position[1] << 0.0, -0.0775, 0.0;
  joint_position[2] << 0.0, 0.0, -0.3095;
  joint_position[3] << 0.1875, 0.0875, 0.18;
  joint_position[4] << 0.0, 0.0775, 0.0;
  joint_position[5] << 0.0, 0.0, -0.3095;
  joint_position[6] << -0.1875, -0.0875, 0.0;
  joint_position[7] << 0.0, -0.0775, 0.0;
  joint_position[8] << 0.0, 0.0, -0.225;
  joint_position[9] << -0.1875, 0.0875, 0.0;
  joint_position[10] << 0.0, 0.0775, 0.0;
  joint_position[11] << 0.0, 0.0, -0.225;

  Vec3f foot_position[4];
  foot_position[0] << 0, 0, -0.32;
  foot_position[1] << 0, 0, -0.32;
  foot_position[2] << -0.01, 0, -0.26;
  foot_position[3] << -0.01, 0, -0.26;
  // set Alist and Tlist for joints
  Mat4f Tlist_tmp;
  for (int i = 0; i < config_.joint_num; ++i) {
    config_.Alist.push_back(joint_axis[i]);
    Tlist_tmp = Mat4f::Identity();
    Tlist_tmp.block(0, 3, 3, 1) = joint_position[i];
    config_.Tlist.push_back(Tlist_tmp);
  }
  // set Alist and Tlist for foots
  config_.Tlist_foot.clear();
  for (int i = 0; i < config_.foot_num; ++i) {
    Tlist_tmp = Mat4f::Identity();
    Tlist_tmp.block(0, 3, 3, 1) = foot_position[i];
    config_.Tlist_foot.push_back(Tlist_tmp);
  }
  //
  if (config_.use_floating_Base) { //
    addFloatingBase();
  }
#ifdef DEBUG_KINE
  std::cout << "use_floating_Base: " << config_.use_floating_Base << std::endl;
  std::cout << "joint number: " << config_.joint_num << std::endl;
  std::cout << "joint sequence: \n";
  for (int i = 0; i < config_.joint_num; ++i) {
    std::cout << config_.joint_seq[i] << " ";
  }
  std::cout << "\n joint parent: \n";
  for (int i = 0; i < config_.joint_num; ++i) {
    std::cout << config_.joint_parent[i] << " ";
  }
  std::cout << std::endl << "Alist: \n";
  for (int i = 0; i < config_.joint_num; ++i) {
    std::cout << "i=" << i << "\n" << config_.Alist[i].transpose() << std::endl;
  }
  std::cout << "Tlist: \n";
  for (int i = 0; i < config_.joint_num; ++i) {
    std::cout << "i=" << i << "\n" << config_.Tlist[i] << std::endl;
  }
  std::cout << "foot parent: \n";
  for (int i = 0; i < config_.foot_num; ++i) {
    std::cout << " " << config_.foot_parent[i] << std::endl;
  }
  std::cout << "foot Tlist: \n";
  for (int i = 0; i < config_.foot_num; ++i) {
    std::cout << "i=" << i << "\n" << config_.Tlist_foot[i] << std::endl;
  }
#endif

  config_.p_hip_.clear();
  config_.p_hip_.push_back(Vec3f(0.1875, -0.0875, 0.18)); // phip[0]
  config_.p_hip_.push_back(Vec3f(0.1875, 0.0875, 0.18));
  config_.p_hip_.push_back(Vec3f(-0.1875, -0.0875, 0.0));
  config_.p_hip_.push_back(Vec3f(-0.1875, 0.0875, 0.0));
  config_.len_.clear();
  Vec5f len;
  len << 0.0775, 0.3095, 0.32, 0, 0;
  config_.len_.push_back(len); // len[0]:fr leg
  config_.len_.push_back(len);
  len << 0.0775, 0.225, 0.26, -0.01, 0;
  config_.len_.push_back(len);
  config_.len_.push_back(len);
  // std::cout << "config_.len size:\n" << config_.len_.size() << std::endl;
  resize();
  // std::cout << "============init Kinematics finished==============\n";
}
void Kinematics::update() {
  updatePosition();
  // jacobian();
}
void Kinematics::resize() {
  data_.T_foot.clear();
  data_.T_foot.resize(config_.foot_num);
  data_.V_foot.resize(config_.foot_num);
  data_.T_tree.resize(config_.joint_num);
  data_.V_tree.resize(config_.joint_num);
  // data_.jacobian_foot.clear();
  data_.jacobian_foot.resize(config_.foot_num);
  data_.jacobian_whole.resize(6 * config_.foot_num, config_.joint_num);
  data_.dot_jacobian_whole.resize(6 * config_.foot_num, config_.joint_num);
  data_.dot_jacobian_foot.resize(config_.foot_num);
}
void Kinematics::getFootPosition(Eigen::Matrix<float, -1, 1> &p) {
  p.resize(config_.foot_num * 3, 1);
  for (int i = 0; i < config_.foot_num; ++i) {
    p.block(3 * i, 0, 3, 1) = data_.T_foot[i].block(0, 3, 3, 1);
  }
}
void Kinematics::addFloatingBase() {
  Vec6f Alist_float[6];
  Alist_float[0] << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  Alist_float[1] << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  Alist_float[2] << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  Alist_float[3] << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  Alist_float[4] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  Alist_float[5] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Mat4f Tlist_float[6];
  Tlist_float[0] = Mat4f::Identity();
  Tlist_float[1] = Mat4f::Identity();
  Tlist_float[2] = Mat4f::Identity();
  Tlist_float[3] = Mat4f::Identity();
  Tlist_float[4] = Mat4f::Identity();
  Tlist_float[5] = Mat4f::Identity();

  vector<int> joint_seq_tmp(config_.joint_seq);
  vector<int> joint_parent_tmp(config_.joint_parent);
  config_.joint_num += 6;
  config_.joint_seq.resize(config_.joint_num);
  config_.joint_parent.resize(config_.joint_num);
  for (int i = 0; i < config_.joint_num; ++i) {
    if (i < 6) {
      config_.joint_seq[i] = i;
      config_.joint_parent[i] = i - 1;
    } else {
      config_.joint_seq[i] = joint_seq_tmp[i - 6] + 6;
      config_.joint_parent[i] = joint_parent_tmp[i - 6] + 6;
    }
  }
  for (int i = 0; i < config_.foot_num; ++i) {
    config_.foot_parent[i] += 6;
  }
  // set Alist and Tlist
  for (int i = 0; i < 6; ++i) {
    config_.Alist.insert(config_.Alist.begin(), Alist_float[5 - i]);
    config_.Tlist.insert(config_.Tlist.begin(), Tlist_float[5 - i]);
  }
}
void Kinematics::fkine(Vec3f &p, Vec3f q, int leg) {
  auto len = config_.len_[leg];
  auto p_hip = config_.p_hip_[leg];
  float l1 = len[0];
  float l2 = len[1];
  float l3 = len[2];
  float lx = len[3];
  float ly = len[4];
  int s; // side sign
  ((leg == 0) || (leg == 2)) ? s = -1 : s = 1;
  float s1 = sin(q[0]);
  float c1 = cos(q[0]);
  float s2 = sin(q[1]);
  float c2 = cos(q[1]);
  float s3 = sin(q[2]);
  float c3 = cos(q[2]);
  float px = lx * (c1 * c3 - c2 * s1 * s3) - l3 * (c1 * s3 + c2 * c3 * s1) -
             l2 * c2 * s1 + ly * s1 * s2;
  float py = l1 * s + ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3;
  float pz = l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3) -
             l2 * c1 * c2 + ly * c1 * s2;
  p << px, py, pz;
  p += p_hip;
}

void Kinematics::ikine(Vec3f &q, Vec3f p, int leg) {
  auto len = config_.len_[leg];
  auto p_hip = config_.p_hip_[leg];
  float l1 = len[0];
  float l2 = len[1];
  float l3 = len[2];
  float lx = len[3];
  float ly = len[4];
  int s; // side sign
  ((leg == 0) || (leg == 2)) ? s = -1 : s = 1;
  float x = p[0] - p_hip[0];
  float y = p[1] - p_hip[1];
  float z = p[2] - p_hip[2];
  // q2 - pi / 2 ~pi / 2
  float a = sqrt(lx * lx + l3 * l3);
  float beta = atan2(l3, lx);
  float s4 = (x * x + z * z + pow(y - s * l1, 2) - a * a - ly * ly - l2 * l2) /
             (2 * a * l2);
  if ((s4 > 1 + ZERO) || (s4 < -1 - ZERO)) {
    std::cout << "warning!!,foot position is out of space limitation\n";
  }
  s4 = std::max(std::min(s4, 1.0f), -1.0f);
  float c4 = sqrt(1 - s4 * s4);
  Eigen::Vector2f theta;
  theta[0] = angleInPi(atan2(s4, c4) - beta);
  theta[1] = angleInPi(atan2(s4, -c4) - beta);
  theta[0] = nearZero(theta[0]);
  theta[1] = nearZero(theta[1]);
  // (fabs(theta[0]) < fabs(theta[1])) ? q[2] = theta[0] : q[2] = theta[1];
  if (theta[0] * theta[1] >= 0) {
    (fabs(theta[0]) > fabs(theta[1])) ? q[2] = theta[1] : q[2] = theta[0];
  } else {
    (theta[0] > theta[1]) ? q[2] = theta[1] : q[2] = theta[0];
  }
  // % get q0
  c4 = cos(q[2] + beta);
  float a_tmp = sqrt(x * x + z * z);
  beta = atan2(z, x);
  float s_tmp, c_tmp;
  c_tmp = a * c4 / a_tmp;
  if ((c_tmp > 1 + ZERO) || (c_tmp < -1 - ZERO)) {
    std::cout << "warning!!,foot position is out of space limitation\n";
  }
  c_tmp = std::max(std::min(c_tmp, 1.0f), -1.0f);
  s_tmp = sqrt(1 - c_tmp * c_tmp);
  float test = atan2(s_tmp, c_tmp);
  theta[0] = angleInPi(atan2(s_tmp, c_tmp) - beta);
  theta[1] = angleInPi(atan2(-s_tmp, c_tmp) - beta);
  theta[0] = nearZero(theta[0]);
  theta[1] = nearZero(theta[1]);
  (fabs(theta[0]) > fabs(theta[1])) ? q[0] = theta[1] : q[0] = theta[0];
  // get q1
  beta = atan2(ly, l2 + a * s4);
  a_tmp = sqrt(ly * ly + pow(l2 + a * s4, 2));
  s_tmp = (y - s * l1) / a_tmp;
  if ((s_tmp > 1 + ZERO) || (s_tmp < -1 - ZERO)) {
    std::cout << "warning!!,foot position is out of space limitation\n";
  }
  s_tmp = std::max(std::min(s_tmp, 1.0f), -1.0f);
  c_tmp = sqrt(1 - s_tmp * s_tmp);
  theta[0] = angleInPi(atan2(s_tmp, c_tmp) - beta);
  theta[1] = angleInPi(atan2(s_tmp, -c_tmp) - beta);
  theta[0] = nearZero(theta[0]);
  theta[1] = nearZero(theta[1]);
  (fabs(theta[0]) > fabs(theta[1])) ? q[1] = theta[1] : q[1] = theta[0];
  (std::isnan(q[0]) || std::isnan(q[1]) || std::isnan(q[2])) ? q = q_last_
                                                             : q_last_ = q;
}

void Kinematics::jacobian(Eigen::Matrix<float, 3, 3> &j, Vec3f q, int leg) {
  auto len = config_.len_[leg];
  auto p_hip = config_.p_hip_[leg];
  float l1 = len[0];
  float l2 = len[1];
  float l3 = len[2];
  float lx = len[3];
  float ly = len[4];
  float s1 = sin(q[0]);
  float c1 = cos(q[0]);
  float s2 = sin(q[1]);
  float c2 = cos(q[1]);
  float s3 = sin(q[2]);
  float c3 = cos(q[2]);
  int s; // side sign
  ((leg == 0) || (leg == 2)) ? s = -1 : s = 1;
  j(0, 0) = l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3) -
            l2 * c1 * c2 + ly * c1 * s2;
  j(0, 1) = s1 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3);
  j(0, 2) = -l3 * (c1 * c3 - c2 * s1 * s3) - lx * (c1 * s3 + c2 * c3 * s1);
  j(1, 0) = 0;
  j(1, 1) = l2 * c2 - ly * s2 + l3 * c2 * c3 + lx * c2 * s3;
  j(1, 2) = s2 * (lx * c3 - l3 * s3);
  j(2, 0) = l3 * (c1 * s3 + c2 * c3 * s1) - lx * (c1 * c3 - c2 * s1 * s3) +
            l2 * c2 * s1 - ly * s1 * s2;
  j(2, 1) = c1 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3);
  j(2, 2) = l3 * (c3 * s1 + c1 * c2 * s3) + lx * (s1 * s3 - c1 * c2 * c3);
}
void Kinematics::dotjacobian(Mat3f &dj, Vec3f q, Vec3f dq, int leg) {
  auto len = config_.len_[leg];
  auto p_hip = config_.p_hip_[leg];
  float l1 = len[0];
  float l2 = len[1];
  float l3 = len[2];
  float lx = len[3];
  float ly = len[4];
  float s1 = sin(q[0]);
  float c1 = cos(q[0]);
  float s2 = sin(q[1]);
  float c2 = cos(q[1]);
  float s3 = sin(q[2]);
  float c3 = cos(q[2]);
  float dq1 = dq[0];
  float dq2 = dq[1];
  float dq3 = dq[2];
  int s; // side sign
  ((leg == 0) || (leg == 2)) ? s = -1 : s = 1;
  dj(0, 0) =
      dq3 * (l3 * (c3 * s1 + c1 * c2 * s3) + lx * (s1 * s3 - c1 * c2 * c3)) +
      dq1 * (l3 * (c1 * s3 + c2 * c3 * s1) - lx * (c1 * c3 - c2 * s1 * s3) +
             l2 * c2 * s1 - ly * s1 * s2) +
      dq2 *
          (ly * c1 * c2 + l2 * c1 * s2 + l3 * c1 * c3 * s2 + lx * c1 * s2 * s3);
  dj(0, 1) = dq2 * s1 * (l2 * c2 - ly * s2 + l3 * c2 * c3 + lx * c2 * s3) +
             dq1 * c1 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3) +
             dq3 * s1 * s2 * (lx * c3 - l3 * s3);
  dj(0, 2) =
      dq2 * (lx * c3 * s1 * s2 - l3 * s1 * s2 * s3) +
      dq1 * (l3 * (c3 * s1 + c1 * c2 * s3) + lx * (s1 * s3 - c1 * c2 * c3)) +
      dq3 * (l3 * (c1 * s3 + c2 * c3 * s1) - lx * (c1 * c3 - c2 * s1 * s3));
  dj(1, 0) = 0;
  dj(1, 1) = dq3 * c2 * (lx * c3 - l3 * s3) -
             dq2 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3);
  dj(1, 2) = dq2 * c2 * (lx * c3 - l3 * s3) - dq3 * s2 * (l3 * c3 + lx * s3);
  dj(2, 0) =
      dq3 * (l3 * (c1 * c3 - c2 * s1 * s3) + lx * (c1 * s3 + c2 * c3 * s1)) -
      dq1 * (l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3) -
             l2 * c1 * c2 + ly * c1 * s2) -
      dq2 *
          (ly * c2 * s1 + l2 * s1 * s2 + l3 * c3 * s1 * s2 + lx * s1 * s2 * s3);
  dj(2, 1) = dq2 * c1 * (l2 * c2 - ly * s2 + l3 * c2 * c3 + lx * c2 * s3) -
             dq1 * s1 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3) +
             dq3 * c1 * s2 * (lx * c3 - l3 * s3);
  dj(2, 2) =
      dq1 * (l3 * (c1 * c3 - c2 * s1 * s3) + lx * (c1 * s3 + c2 * c3 * s1)) -
      dq3 * (l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3)) +
      dq2 * (lx * c1 * c3 * s2 - l3 * c1 * s2 * s3);
}
void Kinematics::clearConfig() {
  config_.Tlist.clear();
  config_.Alist.clear();
  config_.joint_seq.clear();
  config_.joint_parent.clear();
  config_.foot_parent.clear();
  data_.T_tree.clear();
  data_.T_foot.clear();
  data_.V_foot.clear();
  data_.V_tree.clear();
}
void Kinematics::setJointPosition(const Eigen::Matrix<float, -1, 1> &q) {
  assert(q.rows() == config_.joint_num &&
         "wrong dimension of q in setJointPosition");
  data_.q = q;
}
void Kinematics::setJointVelocity(const Eigen::Matrix<float, -1, 1> &dq) {
  assert(dq.rows() == config_.joint_num &&
         "wrong dimension of dq in setJointVelocity");
  data_.dq = dq;
}
void Kinematics::updatePosition() {
  assert(config_.joint_num == config_.Alist.size() &&
         "wrong dimension of Alist");
  // Ttree
  for (int i = 0; i < config_.joint_num; ++i) {
    data_.T_tree[i] = config_.Tlist[i] * exp2rotm(config_.Alist[i], data_.q[i]);
#ifdef DEBUG_KINE
    std::cout << "data_.T_tree[i]:\n" << data_.T_tree[i] << std::endl;
#endif
  }
  // foot position
  for (int i = 0; i < config_.foot_num; ++i) {
    int link = config_.foot_parent[i];
    data_.T_foot[i] = config_.Tlist_foot[i];
    for (int j = 0; j <= config_.joint_num; ++j) {
      data_.T_foot[i] = data_.T_tree[link] * data_.T_foot[i];
      link = config_.joint_parent[link];
      if (link == -1) {
        break;
      }
    }
#ifdef DEBUG_KINE
    std::cout << "i=" << i << "T_foot[i]:\n" << data_.T_foot[i] << std::endl;
#endif
  }
}

void Kinematics::updateVelocity() {
  Eigen::Matrix<float, 4, 4> Ttmp;
  int parent_id;
  assert(data_.dq.rows() == config_.joint_num &&
         "wrong dimension of dq in updateVelocity");
  data_.V_tree[0] = config_.Alist[0] * data_.dq[0];
  // get velocity for joint in joint frame
  for (int i = 1; i < config_.joint_num; ++i) {
    Ttmp = data_.T_tree[i];
    parent_id = config_.joint_parent[i];
    data_.V_tree[i] = AdT(invTransM(Ttmp)) * data_.V_tree[parent_id] +
                      config_.Alist[i] * data_.dq[i];
  }
  // get velocity for each foot in body frame
  for (int i = 0; i < config_.foot_num; ++i) {
    Ttmp = config_.Tlist_foot[i];
    parent_id = config_.foot_parent[i];
    data_.V_foot[i] = AdT(invTransM(Ttmp)) * data_.V_tree[parent_id];
  }
}

// calculate jacobian for each foot,and combine them to be on whole body
// jacobian.
void Kinematics::jacobian() {
  // data_.jacobian_foot.clear();
  // data_.jacobian_foot.resize(config_.foot_num);
  /* calculate jacobian for each single foot */
  for (int foot = 0; foot < config_.foot_num; ++foot) {
    data_.jacobian_foot[foot].resize(6, config_.joint_num);
    int link = config_.foot_parent[foot];
    Eigen::Matrix<float, 4, 4> T_foot_2_inertia;
    T_foot_2_inertia = data_.T_foot[foot];
    T_foot_2_inertia.block(0, 3, 3, 1) << 0, 0, 0;
    Eigen::Matrix<float, 4, 4> Ttmp, T_child;
    Ttmp = config_.Tlist_foot[foot]; // tcp for foot
    T_child.setIdentity();
    // if joint i is not this foot's parent joint,then column i of jacobian is
    // zero
    for (int i = config_.joint_num - 1; i >= 0; i--) {
      // link=-1 means there are not parent joints for foot anymore,so column i
      // of jacobian is zero
      if (i != link || link == -1) {
        data_.jacobian_foot[foot].block(0, i, 6, 1) = Vec6f::Zero();
        continue;
      } else {
        Ttmp = T_child * Ttmp;
        // get column i of jacobian in inertia frame
        data_.jacobian_foot[foot].block(0, i, 6, 1) =
            AdT(T_foot_2_inertia) * AdT(invTransM(Ttmp)) * config_.Alist[link];
        // update T_child and link
        T_child = data_.T_tree[link];
        link = config_.joint_parent[link];
      }
    }
  }
  /*combine jacobians to a bigger one*/
  // data_.jacobian_whole.resize(6 * config_.foot_num, config_.joint_num);
  for (int i = 0; i < config_.foot_num; ++i) {
    data_.jacobian_whole.block(6 * i, 0, 6, config_.joint_num) =
        data_.jacobian_foot[i];
  }
#ifdef DEBUG_KINE
  std::cout << "data_.jacobian_whole\n" << data_.jacobian_whole << std::endl;
#endif
}
// refer to designing document 20230202
void Kinematics::dotJacobian() {
  updateVelocity();
  // data_.dot_jacobian_foot.clear();
  // data_.dot_jacobian_foot.resize(config_.foot_num);
  /* calculate jacobian for each single foot */
  for (int foot = 0; foot < config_.foot_num; ++foot) {
    data_.dot_jacobian_foot[foot].resize(6, config_.joint_num);
    int link = config_.foot_parent[foot];
    // formulation(9)
    Vec3f pe, pi, ve, vi, wi, delta_p, delta_v;
    Eigen::Matrix3f Ri;
    Ri = data_.T_foot[foot].block(0, 0, 3, 3);
    pe = data_.T_foot[foot].block(0, 3, 3, 1);
    ve = Ri * data_.V_foot[foot].block(3, 0, 3, 1);
    Eigen::Matrix<float, 4, 4> Ttmp, T_i;
    T_i = data_.T_foot[foot] * invTransM(config_.Tlist_foot[foot]);
    Ttmp.setIdentity();
    // if joint i is not this foot's parent joint,then column i of jacobian is
    // zero
    for (int i = config_.joint_num - 1; i >= 0; i--) {
      // link=-1 means there are not parent joints for foot anymore,so column i
      // of dot_jacobian is zero
      if (i != link || link == -1) {
        data_.dot_jacobian_foot[foot].block(0, i, 6, 1) = Vec6f::Zero();
        continue;
      } else {
        T_i = T_i * invTransM(Ttmp);
        Ri = T_i.block(0, 0, 3, 3);
        pi = T_i.block(0, 3, 3, 1);
        vi = Ri * data_.V_tree[i].block(3, 0, 3, 1);
        wi = Ri * data_.V_tree[i].block(0, 0, 3, 1);
        delta_p = pi - pe;
        delta_v = vi - ve;
        Eigen::Matrix<float, 6, 6> j_tmp;
        j_tmp.setZero();
        j_tmp.block(0, 0, 3, 3) = vec2so3(wi) * Ri;
        j_tmp.block(3, 3, 3, 3) = j_tmp.block(0, 0, 3, 3);
        j_tmp.block(3, 0, 3, 3) =
            vec2so3(delta_v) * Ri + vec2so3(delta_p) * vec2so3(wi) * Ri;
        data_.dot_jacobian_foot[foot].block(0, i, 6, 1) =
            j_tmp * config_.Alist[i];
        // update T_child and link
        Ttmp = data_.T_tree[link];
        link = config_.joint_parent[link];
      }
    }
  }
  /*combine dot jacobians to a bigger one*/
  // data_.dot_jacobian_whole.resize(6 * config_.foot_num, config_.joint_num);
  for (int i = 0; i < config_.foot_num; ++i) {
#ifdef DEBUG_KINE
    std::cout << "data_.dot_jacobian_foot\n"
              << data_.dot_jacobian_foot[i] << std::endl;
#endif
    data_.dot_jacobian_whole.block(6 * i, 0, 6, config_.joint_num) =
        data_.dot_jacobian_foot[i];
  }
#ifdef DEBUG_KINE
  std::cout << "data_.dot_jacobian_whole\n"
            << data_.dot_jacobian_whole << std::endl;
#endif
}
//
float angleInPi(float theta) {
  float th = theta;
  while (th > pi)
    th -= 2 * pi;
  while (th < -pi)
    th += 2 * pi;
  return th;
}
float nearZero(float theta) {
  float th = theta;
  if (fabs(theta) < 1e-6) {
    th = 0;
  }
  return th;
}
} // namespace ROBOTICS
