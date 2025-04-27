//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: InCollision_Edge.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 24-Oct-2018 10:30:03
//
#ifndef INCOLLISION_EDGE_H
#define INCOLLISION_EDGE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "InCollision_Edge_types.h"

// Function Declarations
extern void InCollision_Edge(const double p_pos[2], const double unusedU0_data[],
  const int unusedU0_size[2], double l_angle, double *p_angle, double angle_diff,
  const double gridConverted[134400], const struct0_T *EGO, const struct1_T
  *param, double sampleTime, double *col, double p_step[2], double p_pos_step[2]);

#endif

//
// File trailer for InCollision_Edge.h
//
// [EOF]
//
