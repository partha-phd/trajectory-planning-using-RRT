/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_InCollision_Edge_api.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 24-Oct-2018 10:30:03
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_InCollision_Edge_api.h"
#include "_coder_InCollision_Edge_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "InCollision_Edge",                  /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2];
static const mxArray *b_emlrt_marshallOut(const real_T u[2]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *unusedU0,
  const char_T *identifier, real_T **y_data, int32_T y_size[2]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T **y_data, int32_T y_size[2]);
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *l_angle,
  const char_T *identifier);
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *p_pos,
  const char_T *identifier))[2];
static const mxArray *emlrt_marshallOut(const real_T u);
static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *gridConverted, const char_T *identifier))[134400];
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[134400];
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *EGO, const
  char_T *identifier, struct0_T *y);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static struct1_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
  const char_T *identifier);
static struct1_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static boolean_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2];
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T **ret_data, int32_T ret_size[2]);
static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[134400];
static boolean_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[2]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2]
{
  real_T (*y)[2];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u[2]
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const real_T u[2])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv0[2] = { 0, 0 };

  static const int32_T iv1[2] = { 1, 2 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m1, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m1, *(int32_T (*)[2])&iv1[0], 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *unusedU0
 *                const char_T *identifier
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *unusedU0,
  const char_T *identifier, real_T **y_data, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  real_T *r0;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(unusedU0), &thisId, &r0, y_size);
  *y_data = r0;
  emlrtDestroyArray(&unusedU0);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T **y_data, int32_T y_size[2])
{
  real_T *r1;
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, &r1, y_size);
  *y_data = r1;
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *l_angle
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *l_angle,
  const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(l_angle), &thisId);
  emlrtDestroyArray(&l_angle);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *p_pos
 *                const char_T *identifier
 * Return Type  : real_T (*)[2]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *p_pos,
  const char_T *identifier))[2]
{
  real_T (*y)[2];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(p_pos), &thisId);
  emlrtDestroyArray(&p_pos);
  return y;
}
/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *gridConverted
 *                const char_T *identifier
 * Return Type  : real_T (*)[134400]
 */
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *gridConverted, const char_T *identifier))[134400]
{
  real_T (*y)[134400];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(gridConverted), &thisId);
  emlrtDestroyArray(&gridConverted);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[134400]
 */
  static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[134400]
{
  real_T (*y)[134400];
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *EGO
 *                const char_T *identifier
 *                struct0_T *y
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *EGO, const
  char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  j_emlrt_marshallIn(sp, emlrtAlias(EGO), &thisId, y);
  emlrtDestroyArray(&EGO);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct0_T *y
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[10] = { "vel", "psi", "ax", "ay", "xCG", "yCG",
    "lf", "lr", "w", "steerMax" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 10, fieldNames, 0U, (int32_T *)&dims);
  thisId.fIdentifier = "vel";
  y->vel = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "vel")), &thisId);
  thisId.fIdentifier = "psi";
  y->psi = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "psi")), &thisId);
  thisId.fIdentifier = "ax";
  y->ax = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2,
    "ax")), &thisId);
  thisId.fIdentifier = "ay";
  y->ay = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3,
    "ay")), &thisId);
  thisId.fIdentifier = "xCG";
  y->xCG = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "xCG")), &thisId);
  thisId.fIdentifier = "yCG";
  y->yCG = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5,
    "yCG")), &thisId);
  thisId.fIdentifier = "lf";
  y->lf = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6,
    "lf")), &thisId);
  thisId.fIdentifier = "lr";
  y->lr = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7,
    "lr")), &thisId);
  thisId.fIdentifier = "w";
  y->w = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 8, "w")),
    &thisId);
  thisId.fIdentifier = "steerMax";
  y->steerMax = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    9, "steerMax")), &thisId);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *param
 *                const char_T *identifier
 * Return Type  : struct1_T
 */
static struct1_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
  const char_T *identifier)
{
  struct1_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = l_emlrt_marshallIn(sp, emlrtAlias(param), &thisId);
  emlrtDestroyArray(&param);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : struct1_T
 */
static struct1_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  struct1_T y;
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[7] = { "maxIter", "minThresh", "minRes",
    "minDist", "stopThresh", "plotGraph", "minAngle" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 7, fieldNames, 0U, (int32_T *)&dims);
  thisId.fIdentifier = "maxIter";
  y.maxIter = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "maxIter")), &thisId);
  thisId.fIdentifier = "minThresh";
  y.minThresh = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    1, "minThresh")), &thisId);
  thisId.fIdentifier = "minRes";
  y.minRes = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2,
    "minRes")), &thisId);
  thisId.fIdentifier = "minDist";
  y.minDist = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3,
    "minDist")), &thisId);
  thisId.fIdentifier = "stopThresh";
  y.stopThresh = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    4, "stopThresh")), &thisId);
  thisId.fIdentifier = "plotGraph";
  y.plotGraph = m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    5, "plotGraph")), &thisId);
  thisId.fIdentifier = "minAngle";
  y.minAngle = f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6,
    "minAngle")), &thisId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : boolean_T
 */
static boolean_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[2]
 */
static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2]
{
  real_T (*ret)[2];
  static const int32_T dims[2] = { 1, 2 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  ret = (real_T (*)[2])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T **ret_data
 *                int32_T ret_size[2]
 * Return Type  : void
 */
  static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T **ret_data, int32_T ret_size[2])
{
  static const int32_T dims[2] = { 1, 2 };

  const boolean_T bv0[2] = { false, true };

  int32_T iv2[2];
  emlrtCheckVsBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0], (boolean_T *)&bv0[0], iv2);
  ret_size[0] = iv2[0];
  ret_size[1] = iv2[1];
  *ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[134400]
 */
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[134400]
{
  real_T (*ret)[134400];
  static const int32_T dims[3] = { 80, 80, 21 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 3U, *(int32_T (*)[3])&dims[0]);
  ret = (real_T (*)[134400])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : boolean_T
 */
  static boolean_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "logical",
    false, 0U, (int32_T *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[9]
 *                int32_T nlhs
 *                const mxArray *plhs[4]
 * Return Type  : void
 */
void InCollision_Edge_api(const mxArray * const prhs[9], int32_T nlhs, const
  mxArray *plhs[4])
{
  real_T (*p_step)[2];
  real_T (*p_pos_step)[2];
  real_T (*p_pos)[2];
  real_T (*unusedU0_data)[2];
  int32_T unusedU0_size[2];
  real_T l_angle;
  real_T p_angle;
  real_T angle_diff;
  real_T (*gridConverted)[134400];
  struct0_T EGO;
  struct1_T param;
  real_T sampleTime;
  real_T col;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  p_step = (real_T (*)[2])mxMalloc(sizeof(real_T [2]));
  p_pos_step = (real_T (*)[2])mxMalloc(sizeof(real_T [2]));

  /* Marshall function inputs */
  p_pos = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "p_pos");
  c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "unusedU0", (real_T **)
                     &unusedU0_data, unusedU0_size);
  l_angle = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "l_angle");
  p_angle = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "p_angle");
  angle_diff = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "angle_diff");
  gridConverted = g_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "gridConverted");
  i_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "EGO", &EGO);
  param = k_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "param");
  sampleTime = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "sampleTime");

  /* Invoke the target function */
  InCollision_Edge(*p_pos, *unusedU0_data, unusedU0_size, l_angle, &p_angle,
                   angle_diff, *gridConverted, EGO, param, sampleTime, &col,
                   *p_step, *p_pos_step);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(col);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*p_step);
  }

  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(*p_pos_step);
  }

  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(p_angle);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void InCollision_Edge_atexit(void)
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
  InCollision_Edge_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void InCollision_Edge_initialize(void)
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
void InCollision_Edge_terminate(void)
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
 * File trailer for _coder_InCollision_Edge_api.c
 *
 * [EOF]
 */
