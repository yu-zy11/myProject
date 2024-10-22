#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "robot_model.h"
#include "robotic_types.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>
// #define DEBUG_KINE
namespace ROBOTICS {
using std::vector;
using Vec5f = Eigen::Matrix<float, 5, 1>;
constexpr float pi{3.1415926};
/*!
 *@brief brief description
 *@param Tlist: initial (htm)homegenious transformation matrix of body frame
 *{i} relative to body {i-1}
 *@param Alist:screw of each joint in body frame,if joint i rotates along axis
 * y,then alist[i]=0 1 0 0 0 0;
 *@param joint_seq:sequence of each link
 *@param joint_parent:parent sequence of each joint
 *@param p_hip_:hip positions for each leg
 *@param Tlist_foot:include htm of foot to its parent joint frame
 *
 */
template <typename T> struct KineConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool use_floating_base{true};
  int joint_num;
  int foot_num;
  vector<Mat4<T>, Eigen::aligned_allocator<Mat4<T>>> Tlist;
  vector<Vec6<T>, Eigen::aligned_allocator<Vec6<T>>> Alist;
  vector<Mat4<T>, Eigen::aligned_allocator<Mat4<T>>> Tlist_foot;

  vector<int> joint_seq;
  vector<int> joint_parent;
  vector<int> foot_seq;
  vector<int> foot_parent;

  vector<Vec3<T>, Eigen::aligned_allocator<Vec3<T>>> p_hip;
  vector<Vec3<T>, Eigen::aligned_allocator<Vec3<T>>> p_hip_;
  vector<Vec5f, Eigen::aligned_allocator<Vec5f>>
      len_; // [l1(>0) l2(>0) l3(>0) lx ly]
};

/*!
 *@brief brief description
 *@param T_tree: htm of body frame {i} to its parent
 *@param T_foot: htm of foot to inertia;
 *@param V_tree: motion screw of joint i in body frame{i}
 *@param V_foot: motion screw of foot i in foot frame
 */
template <typename T> struct KineData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector<Mat4<T>, Eigen::aligned_allocator<Mat4<T>>> T_tree;
  vector<Mat4<T>, Eigen::aligned_allocator<Mat4<T>>> T_foot;
  vector<Vec6<T>, Eigen::aligned_allocator<Vec6<T>>> V_tree;
  vector<Vec6<T>, Eigen::aligned_allocator<Vec6<T>>> V_foot;
  Vecx<T> q;
  Vecx<T> dq;
  Vecx<T> q_last;
  Vecx<T> dq_last;
  vector<Mat6x<T>, Eigen::aligned_allocator<Mat6x<T>>> jacobian_foot;
  vector<Mat6x<T>, Eigen::aligned_allocator<Mat6x<T>>> dot_jacobian_foot;
  Matxx<T> jacobian_whole;
  Matxx<T> dot_jacobian_whole;
};

class Kinematics {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Kinematics(bool is_floating_base) {
    config.use_floating_base = is_floating_base;
    init();
  }
  Kinematics(bool floating, RobotModel &rmodel) {
    model = rmodel;
    config.use_floating_base = floating;
    init();
  }

  void update(const Vecx<ktype> &q, const Vecx<ktype> &dq);
  void getFootPosition(Vec6<ktype> &p, const int &index);
  void getFootJacobian(Matxx<ktype> &j);
  void getFootDotJacobian(Matxx<ktype> &dot_jacobian);

  void fkine(Vec3<ktype> &p, Vec3<ktype> q, int leg);
  void ikine(Vec3<ktype> &q, Vec3<ktype> p, int leg);
  void jacobian(Mat3<ktype> &j, Vec3<ktype> q, int leg);
  void dotjacobian(Mat3<ktype> &dj, Vec3<ktype> q, Vec3<ktype> dq, int leg);

private:
  void init();
  void resize();
  void clear();
  void addFloatingBase();
  void updateTtree();
  void updateFootPosition();
  void updateVtree(); // recursive Newton-Euler
  void jacobian();
  void dotJacobian();
  KineConfig<ktype> config;
  KineData<ktype> data_;
  Vec3<ktype> q_last_;
  RobotModel model;
};

} // namespace ROBOTICS

#endif
