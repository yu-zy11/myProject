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
  clear();
  // joint seq and joint parent
  config.joint_num = model.joint_seq_.rows();
  for (int i = 0; i < config.joint_num; ++i) {
    config.joint_seq.push_back(model.joint_seq_[i]);
    config.joint_parent.push_back(model.joint_parent_[i]);
  }
  config.foot_num = model.foot_parent_.rows();
  for (int i = 0; i < config.foot_num; ++i) {
    config.foot_seq.push_back(model.foot_seq_[i]);
    config.foot_parent.push_back(model.foot_parent_[i]);
  }
  //
  Mat4<ktype> Tlist_tmp;
  Vec3<ktype> rpy;
  Tlist_tmp.setIdentity();
  for (int i = 0; i < config.joint_num; ++i) {
    config.Alist.push_back(model.joint_axis_[i]);

    rpy = model.joint_position_[i].block(3, 0, 3, 1);
    Tlist_tmp.block(0, 0, 3, 3) = eul2rot(rpy);
    Tlist_tmp.block(0, 3, 3, 1) = model.joint_position_[i].block(0, 0, 3, 1);
    config.Tlist.push_back(Tlist_tmp);
  }
  // set Tlist for foots
  config.Tlist_foot.clear();
  for (int i = 0; i < config.foot_num; ++i) {
    rpy = model.foot_position_[i].block(3, 0, 3, 1);
    Tlist_tmp.block(0, 0, 3, 3) = eul2rot(rpy);
    Tlist_tmp.block(0, 3, 3, 1) = model.foot_position_[i].block(0, 0, 3, 1);
    config.Tlist_foot.push_back(Tlist_tmp);
  }
  // add floating base
  if (config.use_floating_base) { //
    addFloatingBase();
  }
#ifdef DEBUG_KINE
  std::cout << "use_floating_base: " << config.use_floating_base << std::endl;
  std::cout << "joint number: " << config.joint_num << std::endl;
  std::cout << "joint sequence: \n";
  for (int i = 0; i < config.joint_num; ++i) {
    std::cout << config.joint_seq[i] << " ";
  }
  std::cout << "\n joint parent: \n";
  for (int i = 0; i < config.joint_num; ++i) {
    std::cout << config.joint_parent[i] << " ";
  }
  std::cout << std::endl << "Alist: \n";
  for (int i = 0; i < config.joint_num; ++i) {
    std::cout << "i=" << i << "\n" << config.Alist[i].transpose() << std::endl;
  }
  std::cout << "Tlist: \n";
  for (int i = 0; i < config.joint_num; ++i) {
    std::cout << "i=" << i << "\n" << config.Tlist[i] << std::endl;
  }
  std::cout << "foot parent: \n";
  for (int i = 0; i < config.foot_num; ++i) {
    std::cout << " " << config.foot_parent[i] << std::endl;
  }
  std::cout << "foot Tlist: \n";
  for (int i = 0; i < config.foot_num; ++i) {
    std::cout << "i=" << i << "\n" << config.Tlist_foot[i] << std::endl;
  }
#endif

  config.p_hip_.clear();
  config.p_hip_.push_back(Vec3<ktype>(0.1875, -0.0875, 0.18)); // phip[0]
  config.p_hip_.push_back(Vec3<ktype>(0.1875, 0.0875, 0.18));
  config.p_hip_.push_back(Vec3<ktype>(-0.1875, -0.0875, 0.0));
  config.p_hip_.push_back(Vec3<ktype>(-0.1875, 0.0875, 0.0));
  config.len_.clear();
  Vec5f len;
  len << 0.0775, 0.3095, 0.32, 0, 0;
  config.len_.push_back(len); // len[0]:fr leg
  config.len_.push_back(len);
  len << 0.0775, 0.225, 0.26, -0.01, 0;
  config.len_.push_back(len);
  config.len_.push_back(len);
  // std::cout << "config.len size:\n" << config.len_.size() << std::endl;
  resize();
  // std::cout << "============init Kinematics finished==============\n";
}
void Kinematics::update(const Vecx<ktype> &q, const Vecx<ktype> &dq) {
  assert(q.rows() == config.joint_num && dq.rows() == config.joint_num &&
         "wrong dimension of q dq in update");
  data_.q = q;
  data_.dq = dq;
  for (int i = 0; i < config.joint_num; ++i) {
    if (std::isnan(q[i])) {
      data_.q[i] = data_.q_last[i];
      std::cout << "warning! q i is nan in kine" << std::endl;
    }
    if (std::isnan(dq[i])) {
      data_.dq[i] = data_.dq_last[i];
      std::cout << "warning, dq i is nan in kine" << std::endl;
    }
  }
  updateTtree();
  updateFootPosition();
  updateVtree();
  data_.q_last = data_.q;
  data_.dq_last = data_.dq;
}
void Kinematics::resize() {
  data_.T_foot.resize(config.foot_num);
  data_.V_foot.resize(config.foot_num);
  data_.T_tree.resize(config.joint_num);
  data_.V_tree.resize(config.joint_num);

  data_.jacobian_foot.resize(config.foot_num);
  data_.jacobian_whole.resize(6 * config.foot_num, config.joint_num);
  data_.dot_jacobian_whole.resize(6 * config.foot_num, config.joint_num);
  data_.dot_jacobian_foot.resize(config.foot_num);
}
void Kinematics::getFootPosition(Vec6<ktype> &p, const int &index) {
  assert(index <= config.foot_num && "wonrg index in getFootPosition");
  p.block(0, 0, 3, 1) = data_.T_foot[index].block(0, 3, 3, 1);
  Mat3<ktype> rot = data_.T_foot[index].block(0, 0, 3, 3);
  std::cout << "rot:" << rot << std::endl;
  p.block(3, 0, 3, 1) = rot2eul(rot); // rpy
}
void Kinematics::addFloatingBase() {
  Vec6<ktype> Alist_float[6];
  Alist_float[0] << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  Alist_float[1] << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  Alist_float[2] << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  Alist_float[3] << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  Alist_float[4] << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  Alist_float[5] << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Mat4<ktype> Tlist_float[6];
  Tlist_float[0] = Mat4<ktype>::Identity();
  Tlist_float[1] = Mat4<ktype>::Identity();
  Tlist_float[2] = Mat4<ktype>::Identity();
  Tlist_float[3] = Mat4<ktype>::Identity();
  Tlist_float[4] = Mat4<ktype>::Identity();
  Tlist_float[5] = Mat4<ktype>::Identity();

  vector<int> joint_seq_tmp(config.joint_seq);
  vector<int> joint_parent_tmp(config.joint_parent);
  config.joint_num += 6;
  config.joint_seq.resize(config.joint_num);
  config.joint_parent.resize(config.joint_num);
  for (int i = 0; i < config.joint_num; ++i) {
    if (i < 6) {
      config.joint_seq[i] = i;
      config.joint_parent[i] = i - 1;
    } else {
      config.joint_seq[i] = joint_seq_tmp[i - 6] + 6;
      config.joint_parent[i] = joint_parent_tmp[i - 6] + 6;
    }
  }
  for (int i = 0; i < config.foot_num; ++i) {
    config.foot_parent[i] += 6;
  }
  // set Alist and Tlist
  for (int i = 0; i < 6; ++i) {
    config.Alist.insert(config.Alist.begin(), Alist_float[5 - i]);
    config.Tlist.insert(config.Tlist.begin(), Tlist_float[5 - i]);
  }
}
// void Kinematics::fkine(Vec3<ktype> &p, Vec3<ktype> q, int leg) {
//   auto len = config.len_[leg];
//   auto p_hip = config.p_hip_[leg];
//   float l1 = len[0];
//   float l2 = len[1];
//   float l3 = len[2];
//   float lx = len[3];
//   float ly = len[4];
//   int s; // side sign
//   ((leg == 0) || (leg == 2)) ? s = -1 : s = 1;
//   float s1 = sin(q[0]);
//   float c1 = cos(q[0]);
//   float s2 = sin(q[1]);
//   float c2 = cos(q[1]);
//   float s3 = sin(q[2]);
//   float c3 = cos(q[2]);
//   float px = lx * (c1 * c3 - c2 * s1 * s3) - l3 * (c1 * s3 + c2 * c3 * s1) -
//              l2 * c2 * s1 + ly * s1 * s2;
//   float py = l1 * s + ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3;
//   float pz = l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3) -
//              l2 * c1 * c2 + ly * c1 * s2;
//   p << px, py, pz;
//   p += p_hip;
// }

void Kinematics::ikine(Vec3<ktype> &q, Vec3<ktype> p, int leg) {
  auto len = config.len_[leg];
  auto p_hip = config.p_hip_[leg];
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

// void Kinematics::dotjacobian(Mat3<ktype> &dj, Vec3<ktype> q, Vec3<ktype> dq,
//                              int leg) {
//   auto len = config.len_[leg];
//   auto p_hip = config.p_hip_[leg];
//   float l1 = len[0];
//   float l2 = len[1];
//   float l3 = len[2];
//   float lx = len[3];
//   float ly = len[4];
//   float s1 = sin(q[0]);
//   float c1 = cos(q[0]);
//   float s2 = sin(q[1]);
//   float c2 = cos(q[1]);
//   float s3 = sin(q[2]);
//   float c3 = cos(q[2]);
//   float dq1 = dq[0];
//   float dq2 = dq[1];
//   float dq3 = dq[2];
//   int s; // side sign
//   ((leg == 0) || (leg == 2)) ? s = -1 : s = 1;
//   dj(0, 0) =
//       dq3 * (l3 * (c3 * s1 + c1 * c2 * s3) + lx * (s1 * s3 - c1 * c2 * c3)) +
//       dq1 * (l3 * (c1 * s3 + c2 * c3 * s1) - lx * (c1 * c3 - c2 * s1 * s3) +
//              l2 * c2 * s1 - ly * s1 * s2) +
//       dq2 *
//           (ly * c1 * c2 + l2 * c1 * s2 + l3 * c1 * c3 * s2 + lx * c1 * s2 *
//           s3);
//   dj(0, 1) = dq2 * s1 * (l2 * c2 - ly * s2 + l3 * c2 * c3 + lx * c2 * s3) +
//              dq1 * c1 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3) +
//              dq3 * s1 * s2 * (lx * c3 - l3 * s3);
//   dj(0, 2) =
//       dq2 * (lx * c3 * s1 * s2 - l3 * s1 * s2 * s3) +
//       dq1 * (l3 * (c3 * s1 + c1 * c2 * s3) + lx * (s1 * s3 - c1 * c2 * c3)) +
//       dq3 * (l3 * (c1 * s3 + c2 * c3 * s1) - lx * (c1 * c3 - c2 * s1 * s3));
//   dj(1, 0) = 0;
//   dj(1, 1) = dq3 * c2 * (lx * c3 - l3 * s3) -
//              dq2 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3);
//   dj(1, 2) = dq2 * c2 * (lx * c3 - l3 * s3) - dq3 * s2 * (l3 * c3 + lx * s3);
//   dj(2, 0) =
//       dq3 * (l3 * (c1 * c3 - c2 * s1 * s3) + lx * (c1 * s3 + c2 * c3 * s1)) -
//       dq1 * (l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3) -
//              l2 * c1 * c2 + ly * c1 * s2) -
//       dq2 *
//           (ly * c2 * s1 + l2 * s1 * s2 + l3 * c3 * s1 * s2 + lx * s1 * s2 *
//           s3);
//   dj(2, 1) = dq2 * c1 * (l2 * c2 - ly * s2 + l3 * c2 * c3 + lx * c2 * s3) -
//              dq1 * s1 * (ly * c2 + l2 * s2 + l3 * c3 * s2 + lx * s2 * s3) +
//              dq3 * c1 * s2 * (lx * c3 - l3 * s3);
//   dj(2, 2) =
//       dq1 * (l3 * (c1 * c3 - c2 * s1 * s3) + lx * (c1 * s3 + c2 * c3 * s1)) -
//       dq3 * (l3 * (s1 * s3 - c1 * c2 * c3) - lx * (c3 * s1 + c1 * c2 * s3)) +
//       dq2 * (lx * c1 * c3 * s2 - l3 * c1 * s2 * s3);
// }
void Kinematics::clear() {
  config.Tlist.clear();
  config.Alist.clear();
  config.joint_seq.clear();
  config.joint_parent.clear();
  config.foot_parent.clear();
  data_.T_tree.clear();
  data_.T_foot.clear();
  data_.V_foot.clear();
  data_.V_tree.clear();
}

void Kinematics::updateTtree() {
  assert(config.joint_num == config.Alist.size() && "wrong dimension of Alist");
  for (int i = 0; i < config.joint_num; ++i) {
    data_.T_tree[i] = config.Tlist[i] * exp2rotm(config.Alist[i], data_.q[i]);
#ifdef DEBUG_KINE
    std::cout << "data_.T_tree[i]:\n" << data_.T_tree[i] << std::endl;
#endif
  }
}
// foot position
void Kinematics::updateFootPosition() {
  for (int i = 0; i < config.foot_num; ++i) {
    int parent = config.foot_parent[i];
    data_.T_foot[i] = config.Tlist_foot[i];
    for (int j = 0; j <= config.joint_num; ++j) {
      data_.T_foot[i] = data_.T_tree[parent] * data_.T_foot[i];
      parent = config.joint_parent[parent];
      if (parent <= -1) {
        break;
      }
    }
#ifdef DEBUG_KINE
    std::cout << "i=" << i << "T_foot[i]:\n" << data_.T_foot[i] << std::endl;
#endif
  }
}

void Kinematics::updateVtree() {
  Mat4<ktype> Ttmp;
  int parent;
  assert(data_.dq.rows() == config.joint_num &&
         "wrong dimension of dq in updateVtree");
  data_.V_tree[0] = config.Alist[0] * data_.dq[0];
  // get velocity for joint in joint frame
  for (int i = 1; i < config.joint_num; ++i) {
    Ttmp = data_.T_tree[i];
    parent = config.joint_parent[i];
    data_.V_tree[i] = AdT(invTransM(Ttmp)) * data_.V_tree[parent] +
                      config.Alist[i] * data_.dq[i];
  }
  // get velocity for foot in body frame
  for (int i = 0; i < config.foot_num; ++i) {
    Ttmp = config.Tlist_foot[i];
    parent = config.foot_parent[i];
    data_.V_foot[i] = AdT(invTransM(Ttmp)) * data_.V_tree[parent];
  }
}

// calculate jacobian for each foot,and combine them to be on whole body
// jacobian.
void Kinematics::jacobian() {
  /* calculate jacobian for each single foot */
  for (int foot = 0; foot < config.foot_num; ++foot) {
    data_.jacobian_foot[foot].resize(6, config.joint_num);
    int link = config.foot_parent[foot];
    Mat4<ktype> T_foot_2_inertia;
    T_foot_2_inertia = data_.T_foot[foot];
    T_foot_2_inertia.block(0, 3, 3, 1) << 0, 0, 0;
    Mat4<ktype> Ttmp, T_child;
    Ttmp = config.Tlist_foot[foot]; // tcp for foot
    T_child.setIdentity();
    // if joint i is not this foot's parent joint,then column i of jacobian is
    // zero
    for (int i = config.joint_num - 1; i >= 0; i--) {
      if (i != link || link <= -1) {
        data_.jacobian_foot[foot].block(0, i, 6, 1) = Vec6<ktype>::Zero();
        continue;
      } else {
        Ttmp = T_child * Ttmp;
        // get column i of jacobian in inertia frame
        data_.jacobian_foot[foot].block(0, i, 6, 1) =
            AdT(T_foot_2_inertia) * AdT(invTransM(Ttmp)) * config.Alist[link];
        // update T_child and link
        T_child = data_.T_tree[link];
        link = config.joint_parent[link];
      }
    }
  }
  /*combine jacobians to a bigger one*/
  for (int i = 0; i < config.foot_num; ++i) {
    data_.jacobian_whole.block(6 * i, 0, 6, config.joint_num) =
        data_.jacobian_foot[i];
  }
#ifdef DEBUG_KINE
  std::cout << "data_.jacobian_whole\n" << data_.jacobian_whole << std::endl;
#endif
}
// refer to designing document 20230202
void Kinematics::dotJacobian() {
  // updateVtree();
  /* calculate jacobian for each single foot */
  for (int foot = 0; foot < config.foot_num; ++foot) {
    data_.dot_jacobian_foot[foot].resize(6, config.joint_num);
    int link = config.foot_parent[foot];
    // formulation(9)
    Vec3<ktype> pe, pi, ve, vi, wi, delta_p, delta_v;
    Mat3<ktype> Ri;
    Ri = data_.T_foot[foot].block(0, 0, 3, 3);
    pe = data_.T_foot[foot].block(0, 3, 3, 1);
    ve = Ri * data_.V_foot[foot].block(3, 0, 3, 1);
    Mat4<ktype> Ttmp, T_i;
    T_i = data_.T_foot[foot] * invTransM(config.Tlist_foot[foot]);
    Ttmp.setIdentity();
    // if joint i is not this foot's parent joint,then column i of jacobian is
    // zero
    for (int i = config.joint_num - 1; i >= 0; i--) {
      if (i != link || link == -1) {
        data_.dot_jacobian_foot[foot].block(0, i, 6, 1) = Vec6<ktype>::Zero();
        continue;
      } else {
        T_i = T_i * invTransM(Ttmp);
        Ri = T_i.block(0, 0, 3, 3);
        pi = T_i.block(0, 3, 3, 1);
        vi = Ri * data_.V_tree[i].block(3, 0, 3, 1);
        wi = Ri * data_.V_tree[i].block(0, 0, 3, 1);
        delta_p = pi - pe;
        delta_v = vi - ve;
        Mat6<ktype> j_tmp;
        j_tmp.setZero();
        j_tmp.block(0, 0, 3, 3) = vec2so3(wi) * Ri;
        j_tmp.block(3, 3, 3, 3) = j_tmp.block(0, 0, 3, 3);
        j_tmp.block(3, 0, 3, 3) =
            vec2so3(delta_v) * Ri + vec2so3(delta_p) * vec2so3(wi) * Ri;
        data_.dot_jacobian_foot[foot].block(0, i, 6, 1) =
            j_tmp * config.Alist[i];
        // update T_child and link
        Ttmp = data_.T_tree[link];
        link = config.joint_parent[link];
      }
    }
  }
  /*combine dot jacobians to a bigger one*/
  // data_.dot_jacobian_whole.resize(6 * config.foot_num, config.joint_num);
  for (int i = 0; i < config.foot_num; ++i) {
#ifdef DEBUG_KINE
    std::cout << "data_.dot_jacobian_foot\n"
              << data_.dot_jacobian_foot[i] << std::endl;
#endif
    data_.dot_jacobian_whole.block(6 * i, 0, 6, config.joint_num) =
        data_.dot_jacobian_foot[i];
  }
#ifdef DEBUG_KINE
  std::cout << "data_.dot_jacobian_whole\n"
            << data_.dot_jacobian_whole << std::endl;
#endif
}
//
void Kinematics::getFootJacobian(Matxx<ktype> &j) {
  jacobian();
  j = data_.jacobian_whole;
};

void Kinematics::getFootDotJacobian(Matxx<ktype> &djacobian) {
  dotJacobian();
  djacobian = data_.dot_jacobian_whole;
};
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
