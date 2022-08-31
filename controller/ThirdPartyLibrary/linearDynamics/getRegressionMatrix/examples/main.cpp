//
// File: main.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 01-Sep-2022 00:43:55
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "getRegressionMatrix.h"
#include "main.h"
#include "getRegressionMatrix_terminate.h"
#include "getRegressionMatrix_initialize.h"

// Function Declarations
static double argInit_real_T();
static void main_getRegressionMatrix();

// Function Definitions

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_getRegressionMatrix()
{
  double Y_data[180];
  int Y_size[2];

  // Initialize function 'getRegressionMatrix' input arguments.
  // Call the entry-point 'getRegressionMatrix'.
  getRegressionMatrix(argInit_real_T(), argInit_real_T(), argInit_real_T(),
                      argInit_real_T(), argInit_real_T(), argInit_real_T(),
                      argInit_real_T(), argInit_real_T(), argInit_real_T(),
                      argInit_real_T(), argInit_real_T(), Y_data, Y_size);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  getRegressionMatrix_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_getRegressionMatrix();

  // Terminate the application.
  // You do not need to do this more than one time.
  getRegressionMatrix_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
