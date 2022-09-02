#include "modules/locomotion/locomotion_core/controller/detail_simulation/detail_simulation.h"
#include <iostream>
#include "modules/locomotion/locomotion_core/controller/detail_simulation/linearDynamics/getRegressionMatrix/getRegressionMatrix.h"
namespace detail_simulation {
// detail simulation
DetailSimulation::DetailSimulation() {
  // dynamic parameters from identification
  m_link_inertial_ident.row(0) << 0.00547946, -0.00010364, 0.00000154, 0.00535782, 0.00004378, 0.00408357, 0.00862853,
      -0.06500852, 0.03588819, 2.43400000, 0.04409195, -0.00563441, 0.00741283, 0.02231120, 0.01956673, 0.03459354,
      -0.03551281, 0.24694425, -0.11513556, 3.52777512, 0.05429213, 0.00148881, 0.00102650, 0.02013905, 0.01735705,
      0.03485171, 0.01300000, 0.01300000, -0.09100000, 0.65000000;

  m_link_inertial_ident.row(1) << 0.00586110, 0.00010364, 0.00000707, 0.00535782, 0.00002270, 0.00408357, 0.00862853,
      0.06736205, 0.03605757, 2.43400000, 0.04009443, 0.00097549, 0.00354873, 0.01848465, -0.01573020, 0.02430161,
      -0.02968446, -0.22634664, -0.06832000, 3.41600002, 0.06169590, -0.00559855, -0.00291553, 0.01557770, -0.01140912,
      0.04785802, -0.01300000, -0.01300000, -0.09100000, 0.65000000;

  m_link_inertial_ident.row(2) << 0.00555006, 0.00010364, -0.00000707, 0.00535782, -0.00002270, 0.00408357, -0.00862853,
      -0.06900522, -0.02207250, 2.43400000, 0.03197600, 0.00896909, -0.00456622, 0.00958092, 0.01181418, 0.02916909,
      -0.06580080, 0.24042661, -0.07966606, 3.43466580, 0.05729757, -0.00416281, 0.00248505, 0.01492187, 0.00824880,
      0.04362913, 0.01362668, 0.01362668, -0.09538679, 0.68133420;

  m_link_inertial_ident.row(3) << 0.00292640, -0.00010364, 0.00000707, 0.00535782, 0.00004379, 0.00408357, -0.00862853,
      0.04406101, -0.00914008, 2.43400000, 0.04411651, -0.01177535, -0.01325751, 0.02993860, -0.01969123, 0.03942492,
      -0.01110876, -0.24561857, -0.12060667, 3.50883670, 0.05100121, 0.00313293, -0.00047580, 0.01664064, -0.01265723,
      0.03537458, -0.01089354, -0.01300000, -0.09100000, 0.65000000;

  m_rotor_inertial_ident.row(0) << 0.00003050, -0.00012342, 0.00014724, 18268.21437915, 0.00001502, 18268.21437383,
      -0.01100077, -0.00112191, 0.00133849, 0.09999999, 0.00199759, -0.00022576, -0.00007821, 0.00022572, -0.00048625,
      0.00195842, 0.00300000, 0.00941299, 0.00300000, 0.10000000, 0.00067178, -0.00015406, -0.00022864, 0.00047027,
      -0.00013024, 0.00059967, 0.00072836, 0.00286686, 0.00057455, 0.03822487;
  m_rotor_inertial_ident.row(1) << 0.00002982, 0.00016438, 0.00009520, 46052.56354976, -0.00001293, 46052.56356460,
      -0.01100083, 0.00149429, 0.00086539, 0.10000000, 0.00199345, 0.00030179, -0.00007400, 0.00019119, 0.00045575,
      0.00196399, 0.00277081, -0.01229773, 0.00300000, 0.10000000, 0.00107471, 0.00006270, 0.00002747, 0.00054217,
      -0.00017973, 0.00055804, 0.00029056, -0.00612742, -0.00183255, 0.10000000;
  m_rotor_inertial_ident.row(2) << 0.00002946, 0.00007674, 0.00017251, 36711.03065709, -0.00001094, 36711.03063736,
      0.01100089, -0.00069754, -0.00156815, 0.10000000, 0.00190523, -0.00002637, 0.00001566, 0.00010198, -0.00030913,
      0.00182521, -0.00061176, 0.00927429, 0.00300000, 0.10000000, 0.00300089, -0.00025256, 0.00011298, 0.00042681,
      0.00089563, 0.00281081, 0.00119132, 0.00750000, -0.00300000, 0.10000000;
  m_rotor_inertial_ident.row(3) << 0.00001908, -0.00015196, 0.00000216, 14878.62472903, 0.00000027, 14878.62474811,
      0.01100146, 0.00138126, -0.00001962, 0.10000000, 0.00362529, 0.00011508, 0.00000777, 0.00015105, 0.00056750,
      0.00355529, -0.00107251, -0.01206386, 0.00300000, 0.10000000, 0.00036761, -0.00017433, 0.00016509, 0.00044668,
      0.00017413, 0.00039401, 0.00112065, -0.00039995, 0.00143629, 0.05716330;
  m_muvc_ident.row(0) << 0.17710, 0.19341, 0.21757, 0.54744, 0.49683, 0.67348;
  m_muvc_ident.row(1) << 0.16625, 0.15358, 0.32676, 0.49352, 0.51031, 0.81247;
  m_muvc_ident.row(2) << 0.22208, 0.12465, 0.27295, 0.55293, 0.48512, 0.74481;
  m_muvc_ident.row(3) << 0.22191, 0.18512, 0.23830, 0.58583, 0.53225, 0.39442;

  // ideal parameters from CAD
  m_link_inertial_CAD.row(0) << 0.00291906572670800, -0.000103636268930000, 1.53663647000000e-06, 0.00535781520728400,
      4.37826474460000e-05, 0.00408356579712400, 0.00862853000000000, 0.0211295540000000, -0.00121456600000000,
      2.43400000000000, 0.0244141444111400, -0.000358905865570000, -0.000447241904480000, 0.0197254466316650,
      0.00581318748832000, 0.0105585697637650, 0.00262684500000000, 0.127995770000000, -0.0808827200000000,
      3.54500000000000, 0.0252893396819000, -5.45710000000000e-09, -2.95412286000000e-05, 0.0253185327275000,
      -1.61176220000000e-06, 6.83381589999983e-05, -0.000361900000000000, -6.30000000000000e-06, -0.123195800000000,
      0.700000000000000;

  m_link_inertial_CAD.row(1) << 0.00291906565726001, 0.000103636254750000, 7.07363647000000e-06, 0.00535781520728400,
      2.27023545500000e-05, 0.00408356572767601, 0.00862853000000000, -0.0211295500000000, -0.00121456600000000,
      2.43400000000000, 0.0243911209810250, 0.000363827233280000, -0.000370520875985000, 0.0197252247836500,
      -0.00582493960736000, 0.0105812232240650, 0.00435680500000000, -0.127960320000000, -0.0833890350000000,
      3.54500000000000, 0.0252893396819000, -5.45710000000000e-09, -2.95412286000000e-05, 0.0253185327275000,
      -1.61176220000000e-06, 6.83381589999983e-05, -0.000361900000000000, -6.30000000000000e-06, -0.123195800000000,
      0.700000000000000;

  m_link_inertial_CAD.row(2) << 0.00291906572670800, 0.000103636268930000, -7.07363647000000e-06, 0.00535781520728400,
      -2.27023525540000e-05, 0.00408356579712400, -0.00862853000000000, 0.0211295540000000, -0.00121456600000000,
      2.43400000000000, 0.0296877959856790, -0.000206376379590000, 1.70804746400000e-05, 0.0250371514016920,
      0.00720444102375200, 0.0106081955565870, 0.00394907000000000, 0.127663651000000, -0.104893096000000,
      3.62300000000000, 0.0221335533500000, -1.69910000000000e-08, -7.39332820000000e-05, 0.0221595031383000,
      -1.60014000000000e-06, 6.29699283000012e-05, -0.000499100000000000, -7.00000000000000e-06, -0.114114000000000,
      0.700000000000000;

  m_link_inertial_CAD.row(3) << 0.00291906572670800, -0.000103636268930000, 7.07363647000000e-06, 0.00535781520728400,
      4.37896474460000e-05, 0.00408356579712400, -0.00862853000000000, -0.0211295540000000, 0.00121456600000000,
      2.43400000000000, 0.0297010151714390, 0.000285212138528000, 7.83990655280000e-05, 0.0250713214531190,
      -0.00723665646853200, 0.0106292941177440, 0.00547797600000000, -0.127631044000000, -0.100911419000000,
      3.62300000000000, 0.0221335533500000, -1.69910000000000e-08, -7.39332820000000e-05, 0.0221595031383000,
      -1.60014000000000e-06, 6.29699283000012e-05, -0.000499100000000000, -7.00000000000000e-06, -0.114114000000000,
      0.700000000000000;

  m_rotor_inertial_CAD.setZero();
  //   m_muvc_CAD.row(0) << 0.17710, 0.19341, 0.21757, 0.54744, 0.49683, 0.67348;
  //   m_muvc_CAD.row(1) << 0.16625, 0.15358, 0.32676, 0.49352, 0.51031, 0.81247;
  //   m_muvc_CAD.row(2) << 0.22208, 0.12465, 0.27295, 0.55293, 0.48512, 0.74481;
  //   m_muvc_CAD.row(3) << 0.22191, 0.18512, 0.23830, 0.58583, 0.53225, 0.39442;
  m_muvc_CAD.setZero();
  m_ddq.setZero();
}
void DetailSimulation::detail_trajectory(Vec3<float>& rpy_des, Vec3<float>& pos_des) {
  static float time{0.0};
  constexpr float step{0.002};
  constexpr float duration_trans{2.0};
  constexpr float time_test{4.0};
  constexpr float duration_test{12.0};
  float r_max = 15.0 / 180 * 3.1415926;
  float p_max = 20.0 / 180 * 3.1415926;
  if (time < time_test) {
  } else if (time < time_test + duration_test) {
    (time < time_test + duration_trans) ? rpy_des[1] = (time - time_test) / duration_trans * p_max : rpy_des[1] = p_max;
  } else if (time < time_test + 2 * duration_test) {
    (time < time_test + duration_test + duration_trans)
        ? rpy_des[1] = -(time - time_test - duration_test) / duration_trans * p_max
        : rpy_des[1] = -p_max;
    pos_des[2] = pos_des[2] - 0.05;
  } else if (time < time_test + 3 * duration_test) {
    (time < time_test + 2 * duration_test + duration_trans)
        ? rpy_des[0] = -(time - time_test - 2 * duration_test) / duration_trans * r_max
        : rpy_des[0] = -r_max;
  } else if (time < time_test + 4 * duration_test) {
    (time < time_test + 3 * duration_test + duration_trans)
        ? rpy_des[0] = (time - time_test - 3 * duration_test) / duration_trans * r_max
        : rpy_des[0] = r_max;
  } else {
    time = time_test;
  }

  time += step;
}

void DetailSimulation::calculateJointStates(Vec12<float> q, Vec12<float> dq)  // leg=0 1 2 3
{
  m_q = q;
  m_dq = dq;
  // m_ddq=cmd_data.joint_acceleration;
  m_ddq = (1 - alpha_ddq) * m_ddq + alpha_ddq * (m_dq - m_dq_last) / step;
  m_dq_last = m_dq;
}
void DetailSimulation::inverseDynamics(Vec12<float> q, Vec12<float> dq, int mode)  // mode=0:CAD; 1:ident
{
  calculateJointStates(q, dq);  // leg=0 1 2 3
  double q1, q2, q3, v_q1, v_q2, v_q3, a_q1, a_q2, a_q3;
  constexpr double g{9.81};
  double Y_data[180];
  int Y_size[2];
  for (int leg = 0; leg < 4; leg++) {
    a_q1 = static_cast<double>(m_ddq[3 * leg]);
    a_q2 = static_cast<double>(m_ddq[3 * leg + 1]);
    a_q3 = static_cast<double>(m_ddq[3 * leg + 2]);
    v_q1 = static_cast<double>(m_dq[3 * leg]);
    v_q2 = static_cast<double>(m_dq[3 * leg + 1]);
    v_q3 = static_cast<double>(m_dq[3 * leg + 2]);
    q1 = static_cast<double>(m_q[3 * leg]);
    q2 = static_cast<double>(m_q[3 * leg + 1]);
    q3 = static_cast<double>(m_q[3 * leg + 2]);
    getRegressionMatrix(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, leg, Y_data, Y_size);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 60; j++) { m_y_matrix(i, j) = Y_data[3 * j + i]; }
    }
    m_y_matrix_link = m_y_matrix.block<3, 30>(0, 0);
    m_y_matrix_rotor = m_y_matrix.block<3, 30>(0, 30);
    if (mode == 1) {
      m_tau_link.segment(3 * leg, 3) = m_y_matrix_link * m_link_inertial_ident.row(leg).transpose();
      m_tau_rotor.segment(3 * leg, 3) = m_y_matrix_rotor * m_rotor_inertial_ident.row(leg).transpose();
    } else if (mode == 0) {
      m_tau_link.segment(3 * leg, 3) = m_y_matrix_link * m_link_inertial_CAD.row(leg).transpose();
      m_tau_rotor.segment(3 * leg, 3) = m_y_matrix_rotor * m_rotor_inertial_CAD.row(leg).transpose();
    } else {
      std::cout << "wrong input mode in DetailSimulation::inverseDynamics!\n ";
      assert(false);
    }
  }
  // calculate friction
  calculateFrinction(mode);
}

void DetailSimulation::calculateFrinction(int mode) {
  Vec3<float> muv, muc, v_q, sign_v_q;
  for (int leg = 0; leg < 4; leg++) {
    if (mode == 0) {
      muv = m_muvc_CAD.col(leg).segment(0, 3);
      muc = m_muvc_CAD.col(leg).segment(3, 3);
    } else if (mode == 1) {
      muv = m_muvc_ident.col(leg).segment(0, 3);
      muc = m_muvc_ident.col(leg).segment(3, 3);
    } else {
      std::cout << "wrong input mode in DetailSimulation::calculateFrinction!\n ";
      assert(false);
    }
    std::cout << "muv=" << muv << "  muc=" << muc << std::endl;
    v_q = m_dq.segment(3 * leg, 3);
    sign_v_q << sign(v_q[0]), sign(v_q[1]), sign(v_q[2]);
    m_friction.segment(3 * leg, 3) = muv.asDiagonal() * v_q + muc.asDiagonal() * sign_v_q;
  }
}
void DetailSimulation::getTorque(Vec12<float>& torque_link, Vec12<float>& torque_rotor, Vec12<float>& torque_friction) {
  torque_link = m_tau_link;
  torque_rotor = m_tau_rotor;
  torque_friction = m_friction;
}

int DetailSimulation::sign(float n) {
  int ans;
  if (n > 1e-8) {
    ans = 1;
  } else if (n < -1e-8) {
    ans = -1;
  } else {
    ans = 0;
  }
  return ans;
}

}  // namespace detail_simulation