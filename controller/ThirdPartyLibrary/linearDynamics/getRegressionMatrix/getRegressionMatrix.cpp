//
// File: getRegressionMatrix.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 01-Sep-2022 00:43:55
//

// Include Files
#include <string.h>
#include "getRegressionMatrix.h"
#include "getYmatrix_leg1.h"
#include "getYmatrix_leg2.h"
#include "getYmatrix_leg3.h"
#include "getYmatrix_leg4.h"

// Function Definitions

//
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
//                double leg
//                double Y_data[]
//                int Y_size[2]
// Return Type  : void
//
void getRegressionMatrix(double a_q1, double a_q2, double a_q3, double g, double
  q1, double q2, double q3, double v_q1, double v_q2, double v_q3, double leg,
  double Y_data[], int Y_size[2])
{
  double dv0[180];
  if (leg == 0.0) {
    getYmatrix_leg1(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, dv0);
    Y_size[0] = 3;
    Y_size[1] = 60;
    memcpy(&Y_data[0], &dv0[0], 180U * sizeof(double));
  } else if (leg == 1.0) {
    getYmatrix_leg2(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, dv0);
    Y_size[0] = 3;
    Y_size[1] = 60;
    memcpy(&Y_data[0], &dv0[0], 180U * sizeof(double));
  } else if (leg == 2.0) {
    getYmatrix_leg3(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, dv0);
    Y_size[0] = 3;
    Y_size[1] = 60;
    memcpy(&Y_data[0], &dv0[0], 180U * sizeof(double));
  } else {
    getYmatrix_leg4(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, dv0);
    Y_size[0] = 3;
    Y_size[1] = 60;
    memcpy(&Y_data[0], &dv0[0], 180U * sizeof(double));
  }
}

//
// File trailer for getRegressionMatrix.cpp
//
// [EOF]
//
