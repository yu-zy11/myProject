//
// File: getYmatrix_leg1.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 01-Sep-2022 00:43:55
//

// Include Files
#include <cmath>
#include <string.h>
#include "getRegressionMatrix.h"
#include "getYmatrix_leg1.h"

// Function Definitions

//
// GETYMATRIX_LEG1
//     Y = GETYMATRIX_LEG1(A_Q1,A_Q2,A_Q3,G,Q1,Q2,Q3,V_Q1,V_Q2,V_Q3)
// Arguments    : double a_q1
//                double a_q2
//                double a_q3
//                double g
//                double q1
//                double q2
//                double q3
//                double v_q1
//                double v_q2
//                double v_q3
//                double Y[180]
// Return Type  : void
//
void getYmatrix_leg1(double a_q1, double a_q2, double a_q3, double g, double q1,
                     double q2, double q3, double v_q1, double v_q2, double v_q3,
                     double Y[180])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t17;
  double t18;
  double t19;
  double t20;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t31;
  double t32;
  double t34;
  double t35;
  double t36;
  double t37_tmp;
  double t37;
  double t40;
  double t43;
  double t44;
  double t45;
  double t47;
  double t50;
  double t51;
  double t53;
  double t57;
  double t59_tmp;
  double t62;
  double t63;
  double t65;
  double t68;
  double t70;
  double t71;
  double t72;
  double t73;
  double t74;
  double t75;
  double Y_tmp_tmp;
  double Y_tmp;
  double b_Y_tmp;
  double c_Y_tmp;
  double d_Y_tmp;

  //     This function was generated by the Symbolic Math Toolbox version 8.2.
  //     05-Aug-2022 23:01:03
  t2 = std::cos(q2);
  t3 = std::sin(q2);
  t4 = q2 * 2.0;
  t5 = v_q2 * v_q2;
  t6 = std::sin(q1);
  t7 = std::cos(q1);
  t8_tmp = g * t7;
  t10 = t4 + q3 * 2.0;
  t11 = std::sin(t10);
  t12 = q2 + q3;
  t13 = std::cos(t12);
  t14 = std::sin(t12);
  t15 = v_q2 / 2.0;
  t17 = t15 + v_q3 / 2.0;
  t18 = std::cos(t10);
  t19 = a_q1 / 2.0;
  t20 = a_q1 * t18 / 2.0;
  t12 = q3 + t4;
  t22 = std::cos(t12);
  t23 = (-q1 + q2) + q3;
  t24 = std::cos(q3);
  t25 = v_q3 * v_q3;
  t26 = (q1 + q2) + q3;
  t27 = std::sin(q3);
  t28 = std::sin(t12);
  t29 = a_q1 * 0.01288225;
  t31 = q1 * 8.0;
  t12 = q2 * 8.0;
  t32 = std::cos(t12);
  t34 = std::sin(t12);
  t35 = q2 * 16.0;
  t36 = t8_tmp - a_q1 * 0.227;
  t37_tmp = t8_tmp * 0.1135;
  t37 = t29 - t37_tmp;
  t10 = t4 + q3 * 16.0;
  t40 = std::sin(t10);
  t12 = q2 + q3 * 8.0;
  t43 = std::cos(t12);
  t44 = std::sin(t12);
  t45 = std::cos(t10);
  t47 = t15 + v_q3 * 4.0;
  t50 = v_q2 * 4.0 + v_q3 * 32.0;
  t51 = a_q1 * t45 / 2.0;
  t53 = v_q3 * 8.0 + v_q2;
  t15 = std::sin(t4);
  t57 = v_q1 * v_q1;
  t10 = std::cos(t4);
  t59_tmp = t15 * t57;
  t4 = t59_tmp / 2.0;
  t62 = a_q1 * 227.0 + t8_tmp * 2000.0;
  t63 = t11 * t57 / 2.0;
  t65 = g * std::cos(t23) / 2.0;
  t68 = g * std::sin(t23) / 2.0;
  t70 = std::sin(t35);
  t71 = std::cos(t35);
  t72 = t57 * t70 * 4.0;
  t12 = t40 * t57;
  t73 = t12 / 2.0;
  t74 = a_q2 + a_q3;
  t75 = t12 * 4.0;
  Y[0] = a_q1;
  memset(&Y[1], 0, 20U * sizeof(double));
  Y[21] = t8_tmp;
  Y[22] = 0.0;
  Y[23] = 0.0;
  Y[24] = -g * t6;
  Y[25] = 0.0;
  Y[26] = 0.0;
  Y[27] = 0.0;
  Y[28] = 0.0;
  Y[29] = 0.0;
  Y_tmp_tmp = t2 * t3 * v_q1 * v_q2;
  Y_tmp = Y_tmp_tmp * 2.0;
  b_Y_tmp = a_q1 * (t2 * t2);
  Y[30] = b_Y_tmp - Y_tmp;
  Y[31] = t4;
  Y[32] = 0.0;
  c_Y_tmp = t3 * t5;
  Y[33] = -a_q2 * t2 + c_Y_tmp;
  Y[34] = -a_q1 * t2;
  Y[35] = 0.0;
  Y[36] = a_q1 * t15 + t10 * v_q1 * v_q2 * 2.0;
  Y[37] = -t57 * t10;
  Y[38] = 0.0;
  Y[39] = 0.0;
  Y[40] = a_q2;
  Y[41] = 0.0;
  d_Y_tmp = t2 * t5;
  Y[42] = -a_q2 * t3 - d_Y_tmp;
  Y[43] = -a_q1 * t3;
  Y[44] = 0.0;
  Y[45] = a_q1 * (t3 * t3) + Y_tmp;
  Y[46] = -t4;
  Y[47] = 0.0;
  Y_tmp = g * t3;
  Y[48] = (a_q2 * t2 * -0.1135 + c_Y_tmp * 0.1135) + Y_tmp * t6;
  Y[49] = t2 * t62 * -0.0005;
  Y[50] = 0.0;
  Y[51] = t36;
  Y[52] = 0.0;
  Y[53] = 0.0;
  c_Y_tmp = a_q2 * t3;
  t10 = g * t2 * t6;
  Y[54] = (c_Y_tmp * -0.1135 - d_Y_tmp * 0.1135) - t10;
  Y[55] = t3 * t62 * -0.0005;
  Y[56] = 0.0;
  Y[57] = t37;
  Y[58] = 0.0;
  Y[59] = 0.0;
  t15 = t11 * v_q1;
  t4 = t15 * v_q2;
  t15 *= v_q3;
  Y[60] = ((t19 + t20) - t4) - t15;
  Y[61] = t63;
  Y[62] = t63;
  t23 = t14 * t17;
  t35 = a_q3 * t13;
  Y[63] = ((-a_q2 * t13 - t35) + t23 * v_q2 * 2.0) + t23 * v_q3 * 2.0;
  t23 = -a_q1 * t13;
  Y[64] = t23;
  Y[65] = t23;
  t23 = t18 * v_q1;
  Y[66] = (a_q1 * t11 + t23 * v_q2 * 2.0) + t23 * v_q3 * 2.0;
  t23 = -t18 * t57;
  Y[67] = t23;
  Y[68] = t23;
  Y[69] = 0.0;
  Y[70] = t74;
  Y[71] = t74;
  t23 = t13 * t17;
  t12 = a_q3 * t14;
  Y[72] = ((-a_q2 * t14 - t12) - t23 * v_q2 * 2.0) - t23 * v_q3 * 2.0;
  t23 = -a_q1 * t14;
  Y[73] = t23;
  Y[74] = t23;
  Y[75] = ((t19 - t20) + t4) + t15;
  Y[76] = -t63;
  Y[77] = -t63;
  t15 = t22 * v_q1;
  t4 = g * std::cos(t26) / 2.0;
  Y[78] = ((((((((((t65 - a_q2 * t13 * 0.1135) - t35 * 0.1135) + a_q1 * t27 *
                  0.226) + a_q1 * t28 * 0.226) - t4) + t5 * t14 * 0.1135) + t14 *
              t25 * 0.1135) + t14 * v_q2 * v_q3 * 0.227) + t15 * v_q2 * 0.452) +
           t15 * v_q3 * 0.226) + t24 * v_q1 * v_q3 * 0.226;
  t15 = a_q2 * t27;
  t23 = -t65 - a_q1 * t13 * 0.1135;
  t35 = t22 * t57;
  Y[79] = (((((t23 - t15 * 0.452) - a_q3 * t27 * 0.226) - t4) - t24 * t25 *
            0.226) - t35 * 0.226) - t24 * v_q2 * v_q3 * 0.452;
  Y[80] = ((((t23 - t15 * 0.226) - t4) + t5 * t24 * 0.226) - t35 * 0.113) - t24 *
    t57 * 0.113;
  Y[81] = ((a_q1 * -0.227 + t8_tmp) + c_Y_tmp * 0.226) + d_Y_tmp * 0.226;
  t15 = a_q1 * t3;
  Y[82] = t15 * 0.226;
  Y[83] = 0.0;
  t4 = t28 * v_q1;
  t23 = g * std::sin(t26) / 2.0;
  Y[84] = ((((((((((t68 - a_q2 * t14 * 0.1135) - t12 * 0.1135) - a_q1 * t22 *
                  0.226) - a_q1 * t24 * 0.226) - t23) - t5 * t13 * 0.1135) - t13
              * t25 * 0.1135) - t13 * v_q2 * v_q3 * 0.227) + t27 * v_q1 * v_q3 *
            0.226) + t4 * v_q2 * 0.452) + t4 * v_q3 * 0.226;
  t4 = a_q2 * t24;
  t35 = -t68 - a_q1 * t14 * 0.1135;
  t12 = t28 * t57;
  Y[85] = (((((t35 + t4 * 0.452) + a_q3 * t24 * 0.226) - t23) - t25 * t27 *
            0.226) - t12 * 0.226) - t27 * v_q2 * v_q3 * 0.452;
  Y[86] = ((((t35 + t4 * 0.226) - t23) + t5 * t27 * 0.226) - t27 * t57 * 0.113)
    - t12 * 0.113;
  Y[87] = (((((t29 - c_Y_tmp * 0.025651) + b_Y_tmp * 0.051076) - t37_tmp) -
            d_Y_tmp * 0.025651) + t10 * 0.226) - Y_tmp_tmp * 0.102152;
  Y[88] = ((a_q2 * 0.051076 - t15 * 0.025651) + t59_tmp * 0.025538) + Y_tmp * t7
    * 0.226;
  Y[89] = 0.0;
  Y[90] = a_q1 * 64.0;
  memset(&Y[91], 0, 20U * sizeof(double));
  Y[111] = g * std::cos(t31) * 8.0;
  Y[112] = 0.0;
  Y[113] = 0.0;
  Y[114] = g * std::sin(t31) * -8.0;
  Y[115] = 0.0;
  Y[116] = 0.0;
  Y[117] = 0.0;
  Y[118] = 0.0;
  Y[119] = 0.0;
  Y_tmp = t32 * t34 * v_q1 * v_q2 * 16.0;
  Y[120] = a_q1 * (t32 * t32) - Y_tmp;
  Y[121] = t72;
  Y[122] = 0.0;
  b_Y_tmp = a_q2 * t32;
  c_Y_tmp = t5 * t34;
  Y[123] = b_Y_tmp * -8.0 + c_Y_tmp * 64.0;
  Y[124] = a_q1 * t32 * -8.0;
  Y[125] = 0.0;
  Y[126] = a_q1 * t70 + t71 * v_q1 * v_q2 * 16.0;
  Y[127] = t57 * t71 * -8.0;
  Y[128] = 0.0;
  Y[129] = 0.0;
  Y[130] = a_q2 * 64.0;
  Y[131] = 0.0;
  d_Y_tmp = a_q2 * t34;
  t10 = t5 * t32;
  Y[132] = d_Y_tmp * -8.0 - t10 * 64.0;
  Y[133] = a_q1 * t34 * -8.0;
  Y[134] = 0.0;
  Y[135] = a_q1 * (t34 * t34) + Y_tmp;
  Y[136] = -t72;
  Y[137] = 0.0;
  Y_tmp = g * t6;
  Y[138] = (b_Y_tmp * -0.908 + c_Y_tmp * 7.264) + Y_tmp * t34;
  Y[139] = t32 * t62 * -0.004;
  Y[140] = 0.0;
  Y[141] = t36;
  Y[142] = 0.0;
  Y[143] = 0.0;
  Y[144] = (d_Y_tmp * -0.908 - t10 * 7.264) - Y_tmp * t32;
  Y[145] = t34 * t62 * -0.004;
  Y[146] = 0.0;
  Y[147] = t37;
  Y[148] = 0.0;
  Y[149] = 0.0;
  b_Y_tmp = t40 * v_q1;
  c_Y_tmp = b_Y_tmp * v_q2;
  b_Y_tmp = b_Y_tmp * v_q3 * 8.0;
  Y[150] = ((t19 + t51) - c_Y_tmp) - b_Y_tmp;
  Y[151] = t73;
  Y[152] = t75;
  d_Y_tmp = a_q3 * t43;
  Y[153] = ((-a_q2 * t43 - d_Y_tmp * 8.0) + t44 * t47 * v_q2 * 2.0) + t44 * t50 *
    v_q3 * 2.0;
  Y[154] = -a_q1 * t43;
  Y[155] = a_q1 * t43 * -8.0;
  t10 = t45 * v_q1;
  Y[156] = (a_q1 * t40 + t10 * v_q2 * 2.0) + t10 * v_q3 * 16.0;
  Y[157] = -t45 * t57;
  Y[158] = t45 * t57 * -8.0;
  Y[159] = 0.0;
  Y[160] = a_q2 + a_q3 * 8.0;
  Y[161] = a_q2 * 8.0 + a_q3 * 64.0;
  t10 = a_q3 * t44;
  Y[162] = ((-a_q2 * t44 - t10 * 8.0) - t43 * t47 * v_q2 * 2.0) - t43 * t50 *
    v_q3 * 2.0;
  Y[163] = -a_q1 * t44;
  Y[164] = a_q1 * t44 * -8.0;
  Y[165] = ((t19 - t51) + c_Y_tmp) + b_Y_tmp;
  Y[166] = -t73;
  Y[167] = -t75;
  b_Y_tmp = t44 * t53;
  Y[168] = (((a_q2 * t43 * -0.1135 - d_Y_tmp * 0.908) + Y_tmp * t44) + b_Y_tmp *
            v_q2 * 0.1135) + b_Y_tmp * v_q3 * 0.908;
  b_Y_tmp = t43 * t62;
  Y[169] = b_Y_tmp * -0.0005;
  Y[170] = b_Y_tmp * -0.004;
  Y[171] = t36;
  Y[172] = 0.0;
  Y[173] = 0.0;
  b_Y_tmp = t43 * t53;
  Y[174] = (((a_q2 * t44 * -0.1135 - t10 * 0.908) - Y_tmp * t43) - b_Y_tmp *
            v_q2 * 0.1135) - b_Y_tmp * v_q3 * 0.908;
  Y_tmp = t44 * t62;
  Y[175] = Y_tmp * -0.0005;
  Y[176] = Y_tmp * -0.004;
  Y[177] = t37;
  Y[178] = 0.0;
  Y[179] = 0.0;
}

//
// File trailer for getYmatrix_leg1.cpp
//
// [EOF]
//
