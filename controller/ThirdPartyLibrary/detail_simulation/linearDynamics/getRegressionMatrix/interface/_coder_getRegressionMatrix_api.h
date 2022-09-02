/*
 * File: _coder_getRegressionMatrix_api.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 01-Sep-2022 00:43:55
 */

#ifndef _CODER_GETREGRESSIONMATRIX_API_H
#define _CODER_GETREGRESSIONMATRIX_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_getRegressionMatrix_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void getRegressionMatrix(real_T a_q1, real_T a_q2, real_T a_q3, real_T g,
  real_T q1, real_T q2, real_T q3, real_T v_q1, real_T v_q2, real_T v_q3, real_T
  leg, real_T Y_data[], int32_T Y_size[2]);
extern void getRegressionMatrix_api(const mxArray * const prhs[11], int32_T nlhs,
  const mxArray *plhs[1]);
extern void getRegressionMatrix_atexit(void);
extern void getRegressionMatrix_initialize(void);
extern void getRegressionMatrix_terminate(void);
extern void getRegressionMatrix_xil_terminate(void);

#endif

/*
 * File trailer for _coder_getRegressionMatrix_api.h
 *
 * [EOF]
 */
