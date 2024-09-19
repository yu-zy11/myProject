#include "parallel_ankle/parallel_ankle.h"

#include <unistd.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>

#include "parallel_ankle/fk_net.h"

namespace core {
namespace estimator {

typedef Eigen::Matrix<double, 6, 1> Vector6d;

ParallelAnkleInfo ParallelAnkle::samInfo_ =
    ParallelAnkleInfo(Eigen::Vector3d(0.0063, -0.040, 0.1198), Eigen::Vector3d(0.0207, 0.0445, 0.215),
                      Eigen::Vector3d(0.0434, -0.040, -0.012), Eigen::Vector3d(0.0434, 0.0445, -0.012),
                      Eigen::Vector3d(0, 0, 0), Eigen::Matrix2d::Identity(), Eigen::Vector2d(0, 0),
                      Eigen::Vector2d(0.045, 0.045), Eigen::Vector2d(0.121, 0.216), false);

ParallelAnkleInfo ParallelAnkle::leftInfo_ = ParallelAnkleInfo(
    Eigen::Vector3d(0.0063, -0.040, 0.1198), Eigen::Vector3d(0.0207, 0.0445, 0.215),
    Eigen::Vector3d(0.0434, -0.040, -0.012), Eigen::Vector3d(0.0434, 0.0445, -0.012), Eigen::Vector3d(M_PI, 0, 0),
    ParallelAnkle::Matrix2d(0, -1, -1, 0), Eigen::Vector2d(14.2 * (M_PI / 180), 15.5 * (M_PI / 180)),
    Eigen::Vector2d(0.045, 0.045), Eigen::Vector2d(0.121, 0.216), false);

ParallelAnkleInfo ParallelAnkle::rightInfo_ = ParallelAnkleInfo(
    Eigen::Vector3d(0.0207, -0.0445, 0.215), Eigen::Vector3d(0.0063, 0.040, 0.1198),
    Eigen::Vector3d(0.0434, -0.0445, -0.012), Eigen::Vector3d(0.0434, 0.040, -0.012), Eigen::Vector3d(M_PI, 0, 0),
    ParallelAnkle::Matrix2d(-1, 0, 0, -1), Eigen::Vector2d(15.5 * (M_PI / 180), 14.2 * (M_PI / 180)),
    Eigen::Vector2d(0.045, 0.045), Eigen::Vector2d(0.216, 0.121), true);

ParallelAnkle::ParallelAnkle() : info_(samInfo_) {
  Eigen::Vector3d eul = info_.eulBias;
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eul(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eul(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eul(0), Eigen::Vector3d::UnitZ()));

  tranMat_ = yawAngle * pitchAngle * rollAngle;
}

ParallelAnkle::ParallelAnkle(u_int16_t id) : info_(!id ? leftInfo_ : rightInfo_) {
  Eigen::Vector3d eul = info_.eulBias;
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eul(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eul(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eul(0), Eigen::Vector3d::UnitZ()));

  tranMat_ = yawAngle * pitchAngle * rollAngle;
}

ParallelAnkle::ParallelAnkle(ParallelAnkleInfo info) : info_(info) {
  Eigen::Vector3d eul = info.eulBias;
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eul(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eul(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eul(0), Eigen::Vector3d::UnitZ()));

  tranMat_ = yawAngle * pitchAngle * rollAngle;
}

ParallelAnkle::~ParallelAnkle() {}

void ParallelAnkle::CalculateTranMatEulVel2W(const Eigen::Vector3d eul, Eigen::Matrix3d& mat) {
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eul(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eul(0), Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d ix(1, 0, 0);
  Eigen::Vector3d iy(0, 1, 0);
  Eigen::Vector3d iz(0, 0, 1);
  ix = yawAngle * pitchAngle * ix;
  iy = yawAngle * iy;

  mat << iz, iy, ix;
}

void ParallelAnkle::EulVel2W(Eigen::Vector3d& v3) {
  Eigen::Matrix3d mat;
  CalculateTranMatEulVel2W(v3, mat);
  v3 = mat * v3;
}

void ParallelAnkle::JointSystemConversion(Eigen::Vector2d& joint, const int mode, const bool dir) {
  const Eigen::Matrix2d jointDre = info_.jointDre;
  const Eigen::Vector2d jointBias = info_.jointBias;

  switch (mode) {
    case 1:
      if (!dir) {
        joint = jointDre * joint;
      } else {
        joint = jointDre.transpose() * joint;
      }
      break;

    default:
      if (!dir) {
        joint = jointDre * joint;
        joint += jointBias;
      } else {
        joint -= jointBias;
        joint = jointDre.transpose() * joint;
      }
      break;
  }
}

/**
 * @brief 用户坐标系=A {pitch, roll} --> 原始坐标系=B {pitch, roll}, yaw = 0.
 * @param eul A{pitch, roll} --> B{pitch, roll}
 */
void ParallelAnkle::CoordinateSystemConversion(Eigen::Vector2d& eul, const bool dir) {
  Eigen::Vector3d eul3v;
  eul3v << 0, eul;
  CoordinateSystemConversion(eul3v);
  eul << eul3v(1), eul3v(2);
}

/**
 * @brief 用户坐标系=A {yaw, pitch, roll} <--> 原始坐标系=B {yaw, pitch, roll}.
 * @param eul A{yaw, pitch, roll} --> B{yaw, pitch, roll}
 */
void ParallelAnkle::CoordinateSystemConversion(Eigen::Vector3d& eul, const bool dir) {
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eul(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eul(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eul(0), Eigen::Vector3d::UnitZ()));

  Eigen::Matrix3d rotationMatrix;
  rotationMatrix = yawAngle * pitchAngle * rollAngle;

  Eigen::Matrix3d tranMatrix = tranMat_;
  if (!dir)
    rotationMatrix = tranMatrix * rotationMatrix * tranMatrix.transpose();
  else
    rotationMatrix = tranMatrix.transpose() * rotationMatrix * tranMatrix;

  eul = rotationMatrix.eulerAngles(2, 1, 0);
  EulerAngleMinimization(eul);
}

/**
 * @brief 转换矩阵 = {用户坐标系=A {pitchVel, rollVel} --> 原始坐标系=B {pitchVel, rollVel}}, yaw = 0.
 * @param eul A{pitch, roll}
 */
void ParallelAnkle::GetTranMatrix(const Eigen::Vector2d eul, Eigen::Matrix2d& mat, const bool dir) {
  Eigen::Vector3d eulV3;
  Eigen::Matrix3d matM3;
  eulV3 << 0, eul;
  GetTranMatrix(eulV3, matM3);
  mat = matM3.block<2, 2>(1, 1);
}

/**
 * @brief 转换矩阵 = {用户坐标系=A {yawVel, pitchVel, rollVel} --> 原始坐标系=B {yawVel, pitchVel, rollVel}}.
 * @param eul A{yaw, pitch, roll}
 */
void ParallelAnkle::GetTranMatrix(const Eigen::Vector3d eul, Eigen::Matrix3d& mat, const bool dir) {
  Eigen::Vector3d eulA = eul;
  Eigen::Vector3d eulB = eul;
  Eigen::Matrix3d mat1, mat2;
  CoordinateSystemConversion(eulB);
  EulerAngleMinimization(eulB);
  CalculateTranMatEulVel2W(eulA, mat1);
  CalculateTranMatEulVel2W(eulB, mat2);
  Eigen::Matrix3d tranMatrix = tranMat_;
  mat = mat2.transpose() * tranMatrix * mat1;
  if (dir) mat = mat.transpose();
}

/**
 * @brief 用户坐标系=A {pitchVel, rollVel} --> 原始坐标系=B {pitchVel, rollVel}, yaw = 0.
 * @param eul A{pitch, roll}
 * @param eulVel A{ pitchVel, rollVel} --> B{pitchVel, rollVel}
 */
void ParallelAnkle::CoordinateSystemConversion(const Eigen::Vector2d eul, Eigen::Vector2d& eulVel, const bool dir) {
  Eigen::Matrix2d mat;
  GetTranMatrix(eul, mat);
  eulVel = mat * eulVel;
}

/**
 * @brief 用户坐标系=A {pitchVel, rollVel} --> 原始坐标系=B {pitchVel, rollVel}.
 * @param eul A{yaw, pitch, roll}
 * @param eulVel A{yawVel, pitchVel, rollVel} --> B{yawVel, pitchVel, rollVel}
 */
void ParallelAnkle::CoordinateSystemConversion(const Eigen::Vector3d eul, Eigen::Vector3d& eulVel, const bool dir) {
  Eigen::Matrix3d mat;
  GetTranMatrix(eul, mat);
  eulVel = mat * eulVel;
}

/**
 * @brief 最小化 |yaw| + |pitch| + |roll|.
 * @param eul {yaw, pitch, roll} --> {yaw, pitch, roll}_min
 */
void ParallelAnkle::EulerAngleMinimization(Eigen::Vector3d& eul) {
  double sum[2] = {0};
  for (size_t i = 0; i < 3; i++) {
    sum[0] += abs(eul(i));
    if (i == 1) {
      sum[1] += abs(-eul(i) + M_PI) < abs(-eul(i) - M_PI) ? abs(-eul(i) + M_PI) : abs(-eul(i) - M_PI);
    } else
      sum[1] += abs(eul(i) + M_PI) < abs(eul(i) - M_PI) ? abs(eul(i) + M_PI) : abs(eul(i) - M_PI);
  }

  if (sum[0] > sum[1]) {
    for (size_t i = 0; i < 3; i++) {
      if (i == 1) {
        eul(i) = abs(-eul(i) + M_PI) < abs(-eul(i) - M_PI) ? (-eul(i) + M_PI) : (-eul(i) - M_PI);
      } else
        eul(i) = abs(eul(i) + M_PI) < abs(eul(i) - M_PI) ? (eul(i) + M_PI) : (eul(i) - M_PI);
    }
  }
}

/**
 * @brief 原始坐标系= A{pitch, roll} --> 关节坐标系=D{joint1Angle, joint1Angle}.
 * @param eul A{pitch, roll}
 * @param ang D{joint1Angle, joint1Angle}
 */
void ParallelAnkle::EulToJoint(const Eigen::Vector2d eul_, Eigen::Vector2d& ang) {
  Eigen::Vector2d eul = eul_;
  CoordinateSystemConversion(eul);
  double labs[2], lbcs[2];
  labs[0] = info_.lab(0);
  labs[1] = info_.lab(1);
  lbcs[0] = info_.lbc(0);
  lbcs[1] = info_.lbc(1);
  Vector6d pointAs, pointCs;
  pointAs << info_.pointA1, info_.pointA2;
  pointCs << info_.pointC1, info_.pointC2;

  for (size_t i = 0; i < 2; i++) {
    double a, b, c, lab, lbc;
    Eigen::Vector3d pointA, pointC;

    lab = labs[i];
    lbc = lbcs[i];
    pointA = pointAs.block<3, 1>((3 * i), 0);
    pointC = pointCs.block<3, 1>((3 * i), 0);

    Eigen::AngleAxisd rotVec(eul(0), Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix3d pRotMat = Eigen::Matrix3d::Identity();
    pRotMat = rotVec.toRotationMatrix();

    rotVec = Eigen::AngleAxisd(eul(1), Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rRotMat = Eigen::Matrix3d::Identity();
    rRotMat = rotVec.toRotationMatrix();

    pointC = pRotMat * rRotMat * pointC;
    a = pointC(0) - pointA(0);
    b = pointC(2) - pointA(2);
    c = (pow(lbc, 2) - pow(lab, 2) - pow((pointC - pointA).norm(), 2)) / (2 * lab);
    ang(i) = asin(-c / sqrt(pow(a, 2) + pow(b, 2))) - atan(-a / b);
  }

  JointSystemConversion(ang, 0, 1);
}

/**
 * @brief 原始坐标系= A{pitch, roll} <-- 关节坐标系=D{joint1Angle, joint1Angle}.
 * @param eul A{pitch, roll}
 * @param ang D{joint1Angle, joint1Angle}
 */
void ParallelAnkle::JointToEul(const Eigen::Vector2d ang, Eigen::Vector2d& eul) {
  bool asym = info_.asym;
  Eigen::Vector2d ang_(ang);
  Eigen::VectorXd fkeul(eul);
  JointSystemConversion(ang_);
  ang_ -= info_.jointBias;
  if (asym) {
    double change = ang_(0);
    ang_(0) = ang_(1);
    ang_(1) = change;
  }

  FK(ang_, fkeul);
  eul << fkeul;
  if (asym) eul(1) = -eul(1);
  CoordinateSystemConversion(eul, 1);
}

/**
 * @brief  雅可比矩阵 J，{joint1AngleVel, joint1AngleVel} = J {pitchVel; rollVel}.
 * @param eul A{pitch, roll}
 * @param ang D{joint1Angle, joint1Angle}
 */
void ParallelAnkle::GetJacobiMatrix(const Eigen::Vector2d eul_, const Eigen::Vector2d ang, Eigen::Matrix2d& mat) {
  Eigen::Vector2d eul(eul_);
  Eigen::Vector2d thas(ang);
  CoordinateSystemConversion(eul);

  JointSystemConversion(thas);

  double labs[2], lbcs[2];
  labs[0] = info_.lab(0);
  labs[1] = info_.lab(1);
  lbcs[0] = info_.lbc(0);
  lbcs[1] = info_.lbc(1);
  Vector6d pointAs, pointCs;
  pointAs << info_.pointA1, info_.pointA2;
  pointCs << info_.pointC1, info_.pointC2;
  Eigen::Matrix2d J;

  for (size_t i = 0; i < 2; i++) {
    double lab, lbc;
    double xa, ya, za;
    double xb, yb, zb;
    double xc, yc, zc;
    double pitch, roll;
    double tha;
    double m1, m2, n1, p1, p2;
    double J11, J12;
    Eigen::Vector3d pointA, pointB, pointC;

    tha = thas(i);
    pitch = eul(0);
    roll = eul(1);
    lab = labs[i];
    lbc = lbcs[i];
    pointA = pointAs.block<3, 1>((3 * i), 0);
    pointC = pointCs.block<3, 1>((3 * i), 0);

    xa = pointA(0);
    xb = pointB(0);
    xc = pointC(0);
    ya = pointA(1);
    yb = pointB(1);
    yc = pointC(1);
    za = pointA(2);
    zb = pointB(2);
    zc = pointC(2);

    m1 = -xc * sin(pitch) + yc * cos(pitch) * sin(roll) + zc * cos(pitch) * cos(roll);
    m2 = yc * cos(roll) * sin(pitch) - zc * sin(pitch) * sin(roll);
    n1 = -(zc * cos(roll) + yc * sin(roll));
    p1 = -xc * cos(pitch) - yc * sin(pitch) * sin(roll) - zc * cos(roll) * sin(pitch);
    p2 = yc * cos(pitch) * cos(roll) - zc * cos(pitch) * sin(roll);

    pointB(0) = pointA(0) + cos(tha) * lab;
    pointB(1) = pointA(1);
    pointB(2) = pointA(2) - sin(tha) * lab;

    Eigen::AngleAxisd rotVec(eul(0), Eigen::Vector3d(0, 1, 0));
    Eigen::Matrix3d pRotMat = Eigen::Matrix3d::Identity();
    pRotMat = rotVec.toRotationMatrix();

    rotVec = Eigen::AngleAxisd(eul(1), Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rRotMat = Eigen::Matrix3d::Identity();
    rRotMat = rotVec.toRotationMatrix();
    pointC = pRotMat * rRotMat * pointC;

    xa = pointA(0);
    xb = pointB(0);
    xc = pointC(0);
    ya = pointA(1);
    yb = pointB(1);
    yc = pointC(1);
    za = pointA(2);
    zb = pointB(2);
    zc = pointC(2);

    J11 = m2 * (xb - xc) + n1 * (yb - yc) + p2 * (zb - zc);
    J11 = J11 / (-sin(tha) * lab * (xb - xc) - cos(tha) * lab * (zb - zc));
    J12 = m1 * (xb - xc) + p1 * (zb - zc);
    J12 = J12 / (-sin(tha) * lab * (xb - xc) - cos(tha) * lab * (zb - zc));
    J(i, 0) = J12;
    J(i, 1) = J11;
  }

  Eigen::Matrix2d abtran;
  Eigen::Matrix2d cdTran;
  GetTranMatrix(eul_, abtran);
  cdTran = info_.jointDre.transpose();
  mat = cdTran * J * abtran;
}

/**
 * @brief  雅可比矩阵 J，  D{joint1AngleVel, joint1AngleVel} = J A{pitchVel; rollVel}.
 * @param v2 A{pitch, roll} or D{joint1Angle, joint1Angle}
 * @param mode 0：v2 = {pitch, roll}， 1：v2 = {joint1Angle, joint1Angle}
 */
void ParallelAnkle::GetJacobiMatrix(const Eigen::Vector2d v2, Eigen::Matrix2d& mat, const int mode) {
  Eigen::Vector2d eul, ang;
  switch (mode) {
    case 1:
      ang = v2;
      JointToEul(v2, eul);
      break;
    default:
      eul = v2;
      EulToJoint(v2, ang);
      break;
  }

  GetJacobiMatrix(eul, ang, mat);
}

/**
 * @brief   A{pitchVel; rollVel}  -> D{joint1AngleVel, joint1AngleVel}
 * @param eul A{pitch, roll}
 * @param thas D{joint1Angle, joint1Angle}
 * @param eulVel A{pitchVel, rollVel}
 * @param thasVel A{joint1AngleVel, joint1AngleVel}
 */
void ParallelAnkle::JointVelToEulVel(const Eigen::Vector2d eul, const Eigen::Vector2d ang, Eigen::Vector2d& eulVel,
                                     const Eigen::Vector2d angVel) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, ang, J);
  eulVel = J.inverse() * angVel;
}

/**
 * @brief   A{pitchVel; rollVel}  <- D{joint1AngleVel, joint1AngleVel}
 * @param eul A{pitch, roll}
 * @param thas D{joint1Angle, joint1Angle}
 * @param eulVel A{pitchVel, rollVel}
 * @param thasVel A{joint1AngleVel, joint1AngleVel}
 */
void ParallelAnkle::EulVelToJointVel(const Eigen::Vector2d eul, const Eigen::Vector2d ang, const Eigen::Vector2d eulVel,
                                     Eigen::Vector2d& angVel) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, ang, J);
  angVel = J * eulVel;
}

/**
 * @brief   A{pitchVel; rollVel}  -> D{joint1AngleVel, joint1AngleVel}
 * @param v2 A{pitch, roll} or D{joint1Angle, joint1Angle}
 * @param eulVel A{pitchVel, rollVel}
 * @param thasVel A{joint1AngleVel, joint1AngleVel}
 * @param mode 0：v2 = {pitch, roll}， 1： v2 = {joint1Angle, joint1Angle}
 */
void ParallelAnkle::JointVelToEulVel(const Eigen::Vector2d eul, Eigen::Vector2d& eulVel, const Eigen::Vector2d angVel) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, J);
  eulVel = J.inverse() * angVel;
}

/**
 * @brief   A{pitchVel; rollVel}  <- D{joint1AngleVel, joint1AngleVel}
 * @param v2 A{pitch, roll} or D{joint1Angle, joint1Angle}
 * @param eulVel A{pitchVel, rollVel}
 * @param thasVel A{joint1AngleVel, joint1AngleVel}
 * @param mode 0：v2 = {pitch, roll}， 1： v2 = {joint1Angle, joint1Angle}
 */
void ParallelAnkle::EulVelToJointVel(const Eigen::Vector2d ang, const Eigen::Vector2d eulVel, Eigen::Vector2d& angVel) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(ang, J);
  angVel = J * eulVel;
}

void ParallelAnkle::JointTauToEulTau(const Eigen::Vector2d eul, const Eigen::Vector2d ang, Eigen::Vector2d& eulTau,
                                     const Eigen::Vector2d angTau) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, ang, J);
  eulTau = J.transpose() * angTau;
}

void ParallelAnkle::JointTauToEulTau(const Eigen::Vector2d eul, Eigen::Vector2d& eulTau, const Eigen::Vector2d angTau) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, J, 1);
  eulTau = J.transpose() * angTau;
}

void ParallelAnkle::EulTauToJointTau(const Eigen::Vector2d eul, const Eigen::Vector2d ang, const Eigen::Vector2d eulTau,
                                     Eigen::Vector2d& angTau) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, ang, J);
  J = Eigen::Matrix2d(J.transpose());
  J = Eigen::Matrix2d(J.inverse());
  angTau = J * eulTau;
}

void ParallelAnkle::EulTauToJointTau(const Eigen::Vector2d eul, const Eigen::Vector2d eulTau, Eigen::Vector2d& angTau) {
  Eigen::Matrix2d J;
  GetJacobiMatrix(eul, J);
  J = Eigen::Matrix2d(J.transpose());
  J = Eigen::Matrix2d(J.inverse());
  angTau = J * eulTau;
}

// return r-p
void ParallelAnkle::GetForwardQVT(Eigen::Vector2d& ang, Eigen::Vector2d& angVel, Eigen::Vector2d& tau) {
  Eigen::Vector2d eul, eulVel, eulTau;
  JointToEul(ang, eul);
  JointVelToEulVel(eul, ang, eulVel, angVel);
  JointTauToEulTau(eul, ang, eulTau, tau);

  eul = Eigen::Vector2d(eul);
  eulVel = Eigen::Vector2d(eulVel);
  eulTau = Eigen::Vector2d(eulTau);

  ang = eul;
  angVel = eulVel;
  tau = eulTau;
}

// input r-p
void ParallelAnkle::GetDecoupleQVT(Eigen::Vector2d& eul, Eigen::Vector2d& eulVel, Eigen::Vector2d& tau) {
  eul = Eigen::Vector2d(eul);
  eulVel = Eigen::Vector2d(eulVel);
  tau = Eigen::Vector2d(tau);

  Eigen::Vector2d ang, angVel, angTau;
  EulToJoint(eul, ang);
  EulVelToJointVel(eul, ang, eulVel, angVel);
  EulTauToJointTau(eul, ang, tau, angTau);

  eul = ang;
  eulVel = angVel;
  tau = angTau;
}

void ParallelAnkle::GetDecoupleQV(Eigen::Vector2d& eul, Eigen::Vector2d& eulVel) {
  eul = Eigen::Vector2d(eul);
  eulVel = Eigen::Vector2d(eulVel);

  Eigen::Vector2d ang, angVel;
  EulToJoint(eul, ang);
  EulVelToJointVel(eul, ang, eulVel, angVel);

  eul = ang;
  eulVel = angVel;
}

/**
 * @brief  采取数据{A{pitch, roll}; C{jointAngle1, joint2Angle}}.
 * @param limit {A{pitchDown, rollDown}; A{pitchUp, rollUp}}
 * @param interval 采样间隔
 * @param path 数据储存路径
 */
void ParallelAnkle::DataSampler(Eigen::Matrix2d limit, double interval, std::vector<std::string> path) {
  if (interval <= 0) {
    std::cout << "interval must be greater than 0 " << std::endl;
    return;
  }

  for (size_t i = 0; i < 2; i++) {
    std::ofstream out(path[i]);
    if (out.is_open()) {
      out << "";
      out.close();
      std::cout << "suc open " << path[i] << std::endl;
    } else
      std::cout << "fail open " << path[i] << std::endl;
  }

  double xd = limit(0, 0);
  double xu = limit(1, 0);
  double yd = limit(0, 1);
  double yu = limit(1, 1);

  std::ofstream out1(path[0]);
  std::ofstream out2(path[1]);
  double y = yd;
  out1 << std::to_string(0.0f) + " ";
  out2 << std::to_string(0.0f) + " ";
  while (y < yu) {
    out1 << std::to_string(y) + " ";
    out2 << std::to_string(y) + " ";
    y += interval;
  }

  out1 << "\n";
  out2 << "\n";

  double x = xd;
  Eigen::Vector2d eul, ang;
  while (x < xu) {
    y = yd;
    out1 << std::to_string(x) + " ";
    out2 << std::to_string(x) + " ";
    while (y < yu) {
      eul << x, y;
      EulToJoint(eul, ang);
      out1 << std::to_string(ang(0)) + " ";
      out2 << std::to_string(ang(1)) + " ";
      y += interval;
    }

    out1 << "\n";
    out2 << "\n";
    x += interval;
  }

  out1.close();
  out2.close();
}

void ParallelAnkle::DataSampler(const Eigen::Vector2d a, const Eigen::Vector2d w, const Eigen::Vector2d phi) {
  if (w(0) == 0 && w(1) == 0) {
    std::cout << "w is erorr." << std::endl;
    return;
  }

  double pitch, roll, pitchVel, rollVel;
  double t = 0;
  double dt = 0.0001;
  Eigen::Vector2d eul;
  Eigen::Vector2d FKEul;
  Eigen::Vector2d IKAng;
  Eigen::Vector2d IFAng;
  Eigen::Vector2d eulVel;
  Eigen::Vector2d angVel;
  Eigen::Vector2d lastAngVel;

  std::string path = "../src/parallel_ankle/data/joint_check.txt";
  std::ofstream out(path);
  if (out.is_open()) {
    out << "";
    std::cout << "suc open " << path << std::endl;
  } else {
    std::cout << "fail open " << path << std::endl;
    return;
  }

  lastAngVel.setZero();
  while (abs(w(0) * t) < 2 * M_PI && abs(w(1) * t) < 2 * M_PI) {
    pitch = a(0) * sin(w(0) * t + phi(0));
    pitchVel = w(0) * a(0) * cos(w(0) * t + phi(0));

    roll = a(1) * sin(w(1) * t + phi(1));
    rollVel = w(1) * a(1) * cos(w(1) * t + phi(1));
    eul << pitch, roll;
    eulVel << pitchVel, rollVel;
    EulToJoint(eul, IKAng);
    EulVelToJointVel(eul, eulVel, angVel);
    if (t == 0)
      IFAng = IKAng;
    else
      IFAng += lastAngVel * dt;
    JointToEul(IKAng, FKEul);
    out << t << " ";
    out << eul(0) << " ";
    out << eul(1) << " ";
    out << FKEul(0) << " ";
    out << FKEul(1) << " ";
    out << IFAng(0) << " ";
    out << IFAng(1) << " ";
    out << IKAng(0) << " ";
    out << IKAng(1) << " ";
    out << "\n";
    t += dt;
    lastAngVel = angVel;
    std::cout << abs(w(0) * t) << std::endl;
  }

  out.close();
}

void ParallelAnkle::test() {
  Eigen::Vector2d eul(0.4, 0.3);
  Eigen::Vector2d eulVel(0.3, 0.2);
  Eigen::Vector2d tau(0.2, 0.1);

  if (false) {
    eul = -eul;
    eulVel = -eulVel;
    tau = -tau;
  }

  if (info_.asym) {
    eul(1) = -eul(1);
    eulVel(1) = -eulVel(1);
    tau(1) = -tau(1);
  }

  eul = Eigen::Vector2d(eul);
  eulVel = Eigen::Vector2d(eulVel);
  tau = Eigen::Vector2d(tau);

  std::cout << "INIT eul angle : " << eul(0) << " " << eul(1) << "\t\t";
  std::cout << "INIT eul speed : " << eulVel(0) << " " << eulVel(1) << "\t\t";
  std::cout << "INIT eul torque : " << tau(0) << " " << tau(1) << std::endl;
  GetDecoupleQVT(eul, eulVel, tau);
  std::cout << "IK joint angle : " << eul(0) << " " << eul(1) << "\t";
  std::cout << "IK joint speed : " << eulVel(0) << " " << eulVel(1) << "\t";
  std::cout << "IK joint torque : " << tau(0) << " " << tau(1) << std::endl;
  GetForwardQVT(eul, eulVel, tau);
  std::cout << "FK   eul angle : " << eul(0) << " " << eul(1) << "\t";
  std::cout << "FK   eul speed : " << eulVel(0) << " " << eulVel(1) << "\t";
  std::cout << "FK   eul torque : " << tau(0) << " " << tau(1) << std::endl;
}

void ParallelAnkle::JacobiMatrixTest() {
  Eigen::Vector2d eul(0, 0.0);
  Eigen::Vector2d ang(0, 0.0);
  Eigen::Vector2d nEul(0, 0.0);
  Eigen::Vector2d nAng(0, 0.0);
  Eigen::Vector2d eulVel(0, 0.0);
  Eigen::Vector2d jAngVel(0, 0.0);
  Eigen::Vector2d mAngVel(0, 0.0);
  Eigen::Matrix2d limit_;
  Eigen::Matrix2d mat;
  double dt = 0.001;
  int counter = 0;

  srand(time(NULL));  // 设置随机数种子为当前时间
  double randomFloat;
  double maxDis = 0;
  double meanDis = 0;
  Eigen::Vector2d maxDisE;
  Eigen::Vector2d maxDisEV;
  for (size_t i = 0; i < 10000; i++) {
    double redund = 2;
    limit_ << -35 + redund, -20 + redund, 45 - redund, 18 - redund;
    limit_ = limit_ * M_PI / 180;

    randomFloat = rand() / static_cast<double>(RAND_MAX);
    eul(0) = (limit_(1, 0) - limit_(0, 0)) * randomFloat + limit_(0, 0);
    randomFloat = rand() / static_cast<double>(RAND_MAX);
    eul(1) = (limit_(1, 1) - limit_(0, 1) * randomFloat + limit_(0, 1));

    limit_ << -1, -1, 1, 1;
    randomFloat = rand() / static_cast<double>(RAND_MAX);
    eulVel(0) = (limit_(1, 0) - limit_(0, 0)) * randomFloat + limit_(0, 0);
    randomFloat = rand() / static_cast<double>(RAND_MAX);
    eulVel(1) = 0.5 > abs(randomFloat) ? 1 - abs(eulVel(0)) : -1 + abs(eulVel(0));

    const Eigen::Vector2d cEul = eul;
    EulToJoint(eul, ang);
    EulVelToJointVel(eul, eulVel, jAngVel);

    nEul = cEul + eulVel * dt;
    EulToJoint(nEul, nAng);
    mAngVel = (nAng - ang) / dt;

    counter++;
    double dis = (jAngVel - mAngVel).norm();
    meanDis = (counter - 1) * (meanDis) / counter + dis / counter;
    if (dis > maxDis) {
      maxDis = dis;
      maxDisE = eul;
      maxDisEV = eulVel;
    }

    std::cout << "dis : " << dis << std::endl;
    std::cout << "meanDis : " << meanDis << std::endl;
    std::cout << "maxDis : " << maxDis << std::endl;
    usleep(50);
  }
}
}  // namespace estimator
}  // namespace core
