#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <Eigen/Dense>
#include <vector>
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
 *@param Tlist_init_: initial homegenious transformation matrix of body frame
 *{i} relative to body {i-1}
 *@param Alist_:screw of each joint in body frame,if joint i rotates along axis
 * y,then alist<<0 1 0 0 0 0;
 *@param joint_seq_:sequence of each link,0 repsents base
 *@param parent_seq_:parent sequence of each joint
 *@param p_hip_:hip positions for each leg
 *
 */
struct KineConfig {
  vector<Mat4f> Tlist_init_;
  vector<Vec6f> Alist_;
  vector<int> joint_seq_;
  vector<int> parent_seq_;
  vector<Vec3f> p_hip_;
  vector<Vec5f> len_; // [l1(>0) l2(>0) l3(>0) lx ly]
};
class Kinemetics {
public:
  Kinemetics() { init(); }
  void init();
  void fkine(Vec3f &p, Vec3f q, int leg);

  void ikine(Vec3f &q, Vec3f p, int leg);

  void jacobian(Mat3f &j, Vec3f q, int leg);
  void dotjacobian(Mat3f &dj, Vec3f q, Vec3f dq, int leg);
  // void dot_jacobian();

private:
  vector<Mat4f> Ttree_;
  KineConfig config_;
};

} // namespace ROBOTICS

#endif
