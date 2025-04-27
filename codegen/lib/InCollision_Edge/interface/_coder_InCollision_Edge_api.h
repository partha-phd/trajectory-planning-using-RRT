/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_InCollision_Edge_api.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 24-Oct-2018 10:30:03
 */

#ifndef _CODER_INCOLLISION_EDGE_API_H
#define _CODER_INCOLLISION_EDGE_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_InCollision_Edge_api.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T vel;
  real_T psi;
  real_T ax;
  real_T ay;
  real_T xCG;
  real_T yCG;
  real_T lf;
  real_T lr;
  real_T w;
  real_T steerMax;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  real_T maxIter;
  real_T minThresh;
  real_T minRes;
  real_T minDist;
  real_T stopThresh;
  boolean_T plotGraph;
  real_T minAngle;
} struct1_T;

#endif                                 /*typedef_struct1_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void InCollision_Edge(real_T p_pos[2], real_T unusedU0_data[], int32_T
  unusedU0_size[2], real_T l_angle, real_T *p_angle, real_T angle_diff, real_T
  gridConverted[134400], struct0_T EGO, struct1_T param, real_T sampleTime,
  real_T *col, real_T p_step[2], real_T p_pos_step[2]);
extern void InCollision_Edge_api(const mxArray * const prhs[9], int32_T nlhs,
  const mxArray *plhs[4]);
extern void InCollision_Edge_atexit(void);
extern void InCollision_Edge_initialize(void);
extern void InCollision_Edge_terminate(void);
extern void InCollision_Edge_xil_terminate(void);

#endif

/*
 * File trailer for _coder_InCollision_Edge_api.h
 *
 * [EOF]
 */
