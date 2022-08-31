/*
 * File: _coder_getRegressionMatrix_mex.cpp
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 01-Sep-2022 00:43:55
 */

/* Include Files */
#include "_coder_getRegressionMatrix_api.h"
#include "_coder_getRegressionMatrix_mex.h"

/* Function Declarations */
static void getRegressionMatrix_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[11]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[11]
 * Return Type  : void
 */
static void getRegressionMatrix_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[11])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 11) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 11, 4,
                        19, "getRegressionMatrix");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 19,
                        "getRegressionMatrix");
  }

  /* Call the function. */
  getRegressionMatrix_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray * const plhs[]
 *                int32_T nrhs
 *                const mxArray * const prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(getRegressionMatrix_atexit);

  /* Module initialization. */
  getRegressionMatrix_initialize();

  /* Dispatch the entry-point. */
  getRegressionMatrix_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  getRegressionMatrix_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_getRegressionMatrix_mex.cpp
 *
 * [EOF]
 */
