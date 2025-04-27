//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: InCollision_Edge_types.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 24-Oct-2018 10:30:03
//
#ifndef INCOLLISION_EDGE_TYPES_H
#define INCOLLISION_EDGE_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

typedef struct {
  double vel;
  double psi;
  double ax;
  double ay;
  double xCG;
  double yCG;
  double lf;
  double lr;
  double w;
  double steerMax;
} struct0_T;

typedef struct {
  double maxIter;
  double minThresh;
  double minRes;
  double minDist;
  double stopThresh;
  boolean_T plotGraph;
  double minAngle;
} struct1_T;

#endif

//
// File trailer for InCollision_Edge_types.h
//
// [EOF]
//
