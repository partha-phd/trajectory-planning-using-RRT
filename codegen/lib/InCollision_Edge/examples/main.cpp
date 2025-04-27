//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 24-Oct-2018 10:30:03
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
#include "rt_nonfinite.h"
#include "InCollision_Edge.h"
#include "main.h"
#include "InCollision_Edge_terminate.h"
#include "InCollision_Edge_initialize.h"

// Function Declarations
static void argInit_1x2_real_T(double result[2]);
static void argInit_1xd2_real_T(double result_data[], int result_size[2]);
static void argInit_80x80x21_real_T(double result[134400]);
static boolean_T argInit_boolean_T();
static double argInit_real_T();
static void argInit_struct0_T(struct0_T *result);
static struct1_T argInit_struct1_T();
static void main_InCollision_Edge();

// Function Definitions

//
// Arguments    : double result[2]
// Return Type  : void
//
static void argInit_1x2_real_T(double result[2])
{
  int idx1;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_1xd2_real_T(double result_data[], int result_size[2])
{
  int idx1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 1;
  result_size[1] = 2;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx1] = argInit_real_T();
  }
}

//
// Arguments    : double result[134400]
// Return Type  : void
//
static void argInit_80x80x21_real_T(double result[134400])
{
  int idx0;
  int idx1;
  int idx2;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 80; idx0++) {
    for (idx1 = 0; idx1 < 80; idx1++) {
      for (idx2 = 0; idx2 < 21; idx2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[(idx0 + 80 * idx1) + 6400 * idx2] = argInit_real_T();
      }
    }
  }
}

//
// Arguments    : void
// Return Type  : boolean_T
//
static boolean_T argInit_boolean_T()
{
  return false;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : struct0_T *result
// Return Type  : void
//
static void argInit_struct0_T(struct0_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result->vel = argInit_real_T();
  result->psi = argInit_real_T();
  result->ax = argInit_real_T();
  result->ay = argInit_real_T();
  result->xCG = argInit_real_T();
  result->yCG = argInit_real_T();
  result->lf = argInit_real_T();
  result->lr = argInit_real_T();
  result->w = argInit_real_T();
  result->steerMax = argInit_real_T();
}

//
// Arguments    : void
// Return Type  : struct1_T
//
static struct1_T argInit_struct1_T()
{
  struct1_T result;

  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result.maxIter = argInit_real_T();
  result.minThresh = argInit_real_T();
  result.minRes = argInit_real_T();
  result.minDist = argInit_real_T();
  result.stopThresh = argInit_real_T();
  result.plotGraph = argInit_boolean_T();
  result.minAngle = argInit_real_T();
  return result;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_InCollision_Edge()
{
  double p_pos[2];
  double unusedU0_data[2];
  int unusedU0_size[2];
  double l_angle;
  double p_angle;
  static double dv0[134400];
  struct0_T r0;
  struct1_T r1;
  double col;
  double p_step[2];
  double p_pos_step[2];

  // Initialize function 'InCollision_Edge' input arguments.
  // Initialize function input argument 'p_pos'.
  argInit_1x2_real_T(p_pos);

  // Initialize function input argument 'unusedU0'.
  argInit_1xd2_real_T(unusedU0_data, unusedU0_size);
  l_angle = argInit_real_T();

  // Initialize function input argument 'gridConverted'.
  // Initialize function input argument 'EGO'.
  // Initialize function input argument 'param'.
  // Call the entry-point 'InCollision_Edge'.
  p_angle = argInit_real_T();
  argInit_80x80x21_real_T(dv0);
  argInit_struct0_T(&r0);
  r1 = argInit_struct1_T();
  InCollision_Edge(p_pos, unusedU0_data, unusedU0_size, l_angle, &p_angle,
                   argInit_real_T(), dv0, &r0, &r1, argInit_real_T(), &col,
                   p_step, p_pos_step);
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
  InCollision_Edge_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_InCollision_Edge();

  // Terminate the application.
  // You do not need to do this more than one time.
  InCollision_Edge_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
