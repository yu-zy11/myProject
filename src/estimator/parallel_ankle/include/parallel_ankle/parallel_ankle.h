#ifndef CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_PARALLEL_ANKLE_H_
#define CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_PARALLEL_ANKLE_H_

#include <Eigen/Dense>
#include <vector>

// TODO(qsy): Format the code
namespace core {
namespace estimator {

struct ParallelAnkleInfo {
  Eigen::Vector3d pointA1;
  Eigen::Vector3d pointA2;
  Eigen::Vector3d pointC1;
  Eigen::Vector3d pointC2;
  Eigen::Vector3d eulBias;
  Eigen::Matrix2d jointDre;
  Eigen::Vector2d jointBias;
  Eigen::Vector2d lab;
  Eigen::Vector2d lbc;
  bool asym;
  ParallelAnkleInfo()
      : pointA1(), pointA2(), pointC1(), pointC2(), eulBias(), jointDre(), jointBias(), lab(), lbc(), asym() {}
  ParallelAnkleInfo(Eigen::Vector3d pointA1_, Eigen::Vector3d pointA2_, Eigen::Vector3d pointC1_,
                    Eigen::Vector3d pointC2_, Eigen::Vector3d eulBias_, Eigen::Matrix2d jointDre_,
                    Eigen::Vector2d jointBias_, Eigen::Vector2d lab_, Eigen::Vector2d lbc_, bool asym_) {
    pointA1 = pointA1_;
    pointA2 = pointA2_;
    pointC1 = pointC1_;
    pointC2 = pointC2_;
    eulBias = eulBias_;
    jointDre = jointDre_;
    jointBias = jointBias_;
    lab = lab_;
    lbc = lbc_;
    asym = asym_;
  }
};

class ParallelAnkle {
 private:
  static ParallelAnkleInfo samInfo_;
  static ParallelAnkleInfo leftInfo_;
  static ParallelAnkleInfo rightInfo_;
  const ParallelAnkleInfo info_;
  Eigen::Matrix3d tranMat_;

 protected:
  static Eigen::Matrix2d Matrix2d(double x11, double x12, double x21, double x22) {
    Eigen::Matrix2d mat;
    mat << x11, x12, x21, x22;
    return mat;
  }

  void CalculateTranMatEulVel2W(Eigen::Vector3d eul, Eigen::Matrix3d& mat);
  void EulVel2W(Eigen::Vector3d& v3);
  void JointSystemConversion(Eigen::Vector2d& joint, const int mode = 0, const bool dir = 0);
  void CoordinateSystemConversion(Eigen::Vector2d& eul, const bool dir = 0);
  void CoordinateSystemConversion(Eigen::Vector3d& eul, const bool dir = 0);
  void GetTranMatrix(const Eigen::Vector2d eul, Eigen::Matrix2d& mat, const bool dir = 0);
  void GetTranMatrix(const Eigen::Vector3d eul, Eigen::Matrix3d& mat, const bool dir = 0);
  void CoordinateSystemConversion(const Eigen::Vector2d eul, Eigen::Vector2d& eulVel, const bool dir = 0);
  void CoordinateSystemConversion(const Eigen::Vector3d eul, Eigen::Vector3d& eulVel, const bool dir = 0);
  void EulerAngleMinimization(Eigen::Vector3d& eul);

 public:
  ParallelAnkle();
  ParallelAnkle(u_int16_t id);
  ParallelAnkle(ParallelAnkleInfo info);
  ~ParallelAnkle();

  void EulToJoint(const Eigen::Vector2d eul_, Eigen::Vector2d& ang);
  void JointToEul(const Eigen::Vector2d ang, Eigen::Vector2d& eul);
  void GetJacobiMatrix(const Eigen::Vector2d eul_, const Eigen::Vector2d ang, Eigen::Matrix2d& mat);
  void GetJacobiMatrix(const Eigen::Vector2d v2, Eigen::Matrix2d& mat, const int mode = 0);

  void JointVelToEulVel(const Eigen::Vector2d eul, const Eigen::Vector2d ang, Eigen::Vector2d& eulVel,
                        const Eigen::Vector2d angVel);
  void JointVelToEulVel(const Eigen::Vector2d ang, Eigen::Vector2d& eulVel, const Eigen::Vector2d angVel);
  void EulVelToJointVel(const Eigen::Vector2d eul, const Eigen::Vector2d ang, const Eigen::Vector2d eulVel,
                        Eigen::Vector2d& angVel);
  void EulVelToJointVel(const Eigen::Vector2d eul, const Eigen::Vector2d eulVel, Eigen::Vector2d& angVel);

  void JointTauToEulTau(const Eigen::Vector2d eul, const Eigen::Vector2d ang, Eigen::Vector2d& eulTau,
                        const Eigen::Vector2d angTau);
  void JointTauToEulTau(const Eigen::Vector2d eul, Eigen::Vector2d& eulTau, const Eigen::Vector2d angTau);
  void EulTauToJointTau(const Eigen::Vector2d eul, const Eigen::Vector2d ang, const Eigen::Vector2d eulTau,
                        Eigen::Vector2d& angTau);
  void EulTauToJointTau(const Eigen::Vector2d eul, const Eigen::Vector2d eulTau, Eigen::Vector2d& angTau);

  void GetForwardQVT(Eigen::Vector2d& ang, Eigen::Vector2d& angVel, Eigen::Vector2d& tau);
  void GetDecoupleQVT(Eigen::Vector2d& eul, Eigen::Vector2d& eulVel, Eigen::Vector2d& tau);
  void GetDecoupleQV(Eigen::Vector2d& eul, Eigen::Vector2d& eulVel);
  void DataSampler(Eigen::Matrix2d limit, double interval, std::vector<std::string> path);
  void DataSampler(const Eigen::Vector2d a, const Eigen::Vector2d w, const Eigen::Vector2d phi);
  void test();
  void JacobiMatrixTest();
};
}  // namespace estimator
}  // namespace core

#endif  // CORE_ESTIMATOR_PARALLEL_ANKLE_INCLUDE_PARALLEL_ANKLE_PARALLEL_ANKLE_H_
