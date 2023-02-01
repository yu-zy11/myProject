#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <Eigen/Dense>
#include <vector>
#define DEBUG_KINE
namespace ROBOTICS {
using std::vector;
using Mat4f = Eigen::Matrix4f;
using Mat3f = Eigen::Matrix3f;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
constexpr float pi{3.1415926};
/*!
 *@brief brief description
 *@param Tlist_: initial homegenious transformation matrix of body frame
 *{i} relative to body {i-1}
 *@param Alist_:screw of each joint in body frame,if joint i rotates along axis
 * y,then alist<<0 1 0 0 0 0;
 *@param joint_seq_:sequence of each link,0 repsents base
 *@param parent_seq_:parent sequence of each joint
 *@param p_hip_:hip positions for each leg
 *
 */
struct KineConfig {
  bool use_floating_Base{true};
  int joint_num;
  int foot_num;
  vector<Mat4f> Tlist;
  vector<Vec6f> Alist;

  vector<int> joint_seq;
  vector<int> parent_seq;
  vector<int> child_seq;
  vector<int> foot_link;
  vector<Mat4f> Tlist_foot;

  vector<Vec3f> p_hip;
  vector<Vec3f> p_hip_;
  vector<Vec5f> len_; // [l1(>0) l2(>0) l3(>0) lx ly]
};
struct KineData {
  vector<Mat4f> Ttree;
  vector<Mat4f> T_foot;
  Eigen::Matrix<float, -1, 1> q;
  Eigen::Matrix<float, -1, 1> dq;
  vector<Eigen::Matrix<float, 6, -1>> jacobian_foot;
  Eigen::Matrix<float, -1, -1> jacobian_whole;
  vector<Eigen::Matrix<float, 6, -1>> dot_jacobian_foot;
  Eigen::Matrix<float, -1, -1> dot_jacobian_whole;
};

class Kinemetics {
public:
  Kinemetics(bool is_floating_base) {
    config_.use_floating_Base = is_floating_base;
    init();
  }
  void setJointPosition(const Eigen::Matrix<float, -1, 1> q);
  void setJointVelocity(const Eigen::Matrix<float, -1, 1> dq);
  void updateData();
  void fkine();
  void jacobian();
  // void dotJacobian();
  void fkine(Vec3f &p, Vec3f q, int leg);
  void ikine(Vec3f &q, Vec3f p, int leg);

  void jacobian(Mat3f &j, Vec3f q, int leg);
  void dotjacobian(Mat3f &dj, Vec3f q, Vec3f dq, int leg);

private:
  void addFloatingBase();
  void init();
  void clearConfig();
  KineConfig config_;
  KineData data_;
  Vec3f q_last_;
};

} // namespace ROBOTICS

#endif
