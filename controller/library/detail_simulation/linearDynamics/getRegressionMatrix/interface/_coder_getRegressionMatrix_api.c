/*
 * File: _coder_getRegressionMatrix_api.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 01-Sep-2022 00:43:55
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_getRegressionMatrix_api.h"
#include "_coder_getRegressionMatrix_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131467U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "getRegressionMatrix",               /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *a_q1, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 0U, (int32_T *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *a_q1
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *a_q1, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(a_q1), &thisId);
  emlrtDestroyArray(&a_q1);
  return y;
}

/*
 * Arguments    : const real_T u_data[]
 *                const int32_T u_size[2]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[2])&u_size[0], 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[11]
 *                int32_T nlhs
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void getRegressionMatrix_api(const mxArray * const prhs[11], int32_T nlhs, const
  mxArray *plhs[1])
{
  real_T (*Y_data)[180];
  real_T a_q1;
  real_T a_q2;
  real_T a_q3;
  real_T g;
  real_T q1;
  real_T q2;
  real_T q3;
  real_T v_q1;
  real_T v_q2;
  real_T v_q3;
  real_T leg;
  int32_T Y_size[2];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  (void)nlhs;
  st.tls = emlrtRootTLSGlobal;
  Y_data = (real_T (*)[180])mxMalloc(sizeof(real_T [180]));

  /* Marshall function inputs */
  a_q1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "a_q1");
  a_q2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "a_q2");
  a_q3 = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "a_q3");
  g = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "g");
  q1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "q1");
  q2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "q2");
  q3 = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "q3");
  v_q1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "v_q1");
  v_q2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "v_q2");
  v_q3 = emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "v_q3");
  leg = emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "leg");

  /* Invoke the target function */
  getRegressionMatrix(a_q1, a_q2, a_q3, g, q1, q2, q3, v_q1, v_q2, v_q3, leg,
                      *Y_data, Y_size);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Y_data, Y_size);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void getRegressionMatrix_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  getRegressionMatrix_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void getRegressionMatrix_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void getRegressionMatrix_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_getRegressionMatrix_api.c
 *
 * [EOF]
 */
