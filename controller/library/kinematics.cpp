#include "kinematics.h"
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
namespace ROBOTICS {

float angleInPi(float theta);
float nearZero(float theta);
void Kinemetics::init() {
  std::cout << "==============init Kinemetics==============\n";
  Eigen::Matrix<int, 13, 1> link_seq, parent_seq;
  link_seq << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12; // 0:base link 1;
  parent_seq << 0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11;
  // init Alist according to direction of each joint and link frame
  // for (int i = 1; i < link_seq.rows(); i++) {
  //   if (i == 0 || i == 1 || i == 4 || i == 7 || i == 10) {
  //     config_.Alist_[0] << 1, 0, 0, 0, 0, 0;
  //   } else {
  //     config_.Alist_[i] << 0, 1, 0, 0, 0, 0;
  //   }
  // }
  //  tf matrix of base relative to base
  // config_.Tlist_init_[0] = Mat4f::Identity();
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
  std::cout << "config_.len size:\n" << config_.len_.size() << std::endl;

  std::cout << "============init Kinemetics finished==============\n";
}
void Kinemetics::fkine(Vec3f &p, Vec3f q, int leg) {
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

void Kinemetics::ikine(Vec3f &q, Vec3f p, int leg) {
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
  s4 = std::max(std::min(s4, 1.0f), -1.0f);
  float c4 = sqrt(1 - s4 * s4);
  Eigen::Vector2f theta;
  theta[0] = angleInPi(atan2(s4, c4) - beta);
  theta[1] = angleInPi(atan2(s4, -c4) - beta);
  theta[0] = nearZero(theta[0]);
  theta[1] = nearZero(theta[1]);
  // (fabs(theta[0]) < fabs(theta[1])) ? q[2] = theta[0] : q[2] = theta[1];
  if (leg == 0 || leg == 1) {
    if (theta[0] * theta[1] >= 0) {
      (fabs(theta[0]) > fabs(theta[1])) ? q[2] = theta[1] : q[2] = theta[0];
    } else {
      (theta[0] < theta[1]) ? q[2] = theta[1] : q[2] = theta[0];
    }
  } else {
    if (theta[0] * theta[1] >= 0) {
      (fabs(theta[0]) > fabs(theta[1])) ? q[2] = theta[1] : q[2] = theta[0];
    } else {
      (theta[0] > theta[1]) ? q[2] = theta[1] : q[2] = theta[0];
    }
  }
  // % get q0
  c4 = cos(q[2] + beta);
  float a_tmp = sqrt(x * x + z * z);
  beta = atan2(z, x);
  float s_tmp, c_tmp;
  c_tmp = a * c4 / a_tmp;
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
  s_tmp = std::max(std::min(s_tmp, 1.0f), -1.0f);
  c_tmp = sqrt(1 - s_tmp * s_tmp);
  theta[0] = angleInPi(atan2(s_tmp, c_tmp) - beta);
  theta[1] = angleInPi(atan2(s_tmp, -c_tmp) - beta);
  theta[0] = nearZero(theta[0]);
  theta[1] = nearZero(theta[1]);
  // (fabs(theta[0]) > fabs(theta[1])) ? q[1] = theta[1] : q[1] = theta[0];
  if (leg == 0 || leg == 1) {
    if (theta[0] * theta[1] >= 0) {
      (fabs(theta[0]) > fabs(theta[1])) ? q[1] = theta[1] : q[1] = theta[0];
    } else {
      (theta[0] < theta[1]) ? q[1] = theta[0] : q[1] = theta[1];
    }
  } else {
    if (theta[0] * theta[1] >= 0) {
      (fabs(theta[0]) > fabs(theta[1])) ? q[1] = theta[1] : q[1] = theta[0];
    } else {
      (theta[0] > theta[1]) ? q[1] = theta[0] : q[1] = theta[1];
    }
  }
}

void Kinemetics::jacobian(Eigen::Matrix<float, 3, 3> &j, Vec3f q, int leg) {
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
