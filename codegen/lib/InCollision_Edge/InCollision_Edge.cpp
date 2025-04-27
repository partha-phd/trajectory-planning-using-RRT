//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: InCollision_Edge.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 24-Oct-2018 10:30:03
//

// Include Files
#include <cmath>
#include <float.h>
#include "rt_nonfinite.h"
#include "InCollision_Edge.h"
#include "InCollision_Edge_emxutil.h"

// Function Declarations
static double rt_remd_snf(double u0, double u1);
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_remd_snf(double u0, double u1)
{
  double y;
  double b_u1;
  double q;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    y = rtNaN;
  } else {
    if (u1 < 0.0) {
      b_u1 = std::ceil(u1);
    } else {
      b_u1 = std::floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      q = std::abs(u0 / u1);
      if (std::abs(q - std::floor(q + 0.5)) <= DBL_EPSILON * q) {
        y = 0.0 * u0;
      } else {
        y = std::fmod(u0, u1);
      }
    } else {
      y = std::fmod(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// Checking for the change in the steering deviation
// Arguments    : const double p_pos[2]
//                const double unusedU0_data[]
//                const int unusedU0_size[2]
//                double l_angle
//                double *p_angle
//                double angle_diff
//                const double gridConverted[134400]
//                const struct0_T *EGO
//                const struct1_T *param
//                double sampleTime
//                double *col
//                double p_step[2]
//                double p_pos_step[2]
// Return Type  : void
//
void InCollision_Edge(const double p_pos[2], const double [], const int [2],
                      double l_angle, double *p_angle, double angle_diff, const
                      double gridConverted[134400], const struct0_T *EGO, const
                      struct1_T *param, double sampleTime, double *col, double
                      p_step[2], double p_pos_step[2])
{
  double m;
  double delta1;
  emxArray_real_T *t;
  int i;
  double x;
  double ex;
  emxArray_real_T *t_x;
  int b_i;
  signed char n;
  double eyOrtho;
  boolean_T exitg1;
  double P[8];
  double P_grid[8];
  double probOccupancy[5];
  boolean_T y;
  boolean_T b_x[5];
  boolean_T exitg2;

  //  Function - Collision check along the edge--------------------------------
  if ((*p_angle >= 0.0) && (*p_angle <= 90.0) && (l_angle >= 270.0) && (l_angle <=
       360.0)) {
    if (std::abs(angle_diff) > EGO->steerMax) {
      *p_angle = l_angle + EGO->steerMax;
    }
  } else if ((l_angle >= 0.0) && (l_angle <= 90.0) && (*p_angle >= 270.0) &&
             (*p_angle <= 360.0)) {
    if (std::abs(angle_diff) > EGO->steerMax) {
      *p_angle = l_angle - EGO->steerMax;
    }
  } else if ((std::abs(angle_diff) > EGO->steerMax) && (angle_diff < 0.0)) {
    *p_angle = l_angle + EGO->steerMax;
  } else {
    if ((std::abs(angle_diff) > EGO->steerMax) && (angle_diff > 0.0)) {
      *p_angle = l_angle - EGO->steerMax;
    }
  }

  if (*p_angle < 0.0) {
    *p_angle += 360.0;
  }

  *col = 0.0;
  m = std::ceil(param->minDist / param->minRes);
  delta1 = m;
  if (m < 0.0) {
    delta1 = 0.0;
  }

  emxInit_real_T(&t, 2);
  i = t->size[0] * t->size[1];
  t->size[0] = 1;
  t->size[1] = (int)delta1;
  emxEnsureCapacity_real_T(t, i);
  if (t->size[1] >= 1) {
    t->data[t->size[1] - 1] = param->minDist;
    if (t->size[1] >= 2) {
      t->data[0] = 0.0;
      if (t->size[1] >= 3) {
        if ((param->minDist < 0.0) && (std::abs(param->minDist) >
             8.9884656743115785E+307)) {
          delta1 = param->minDist / ((double)t->size[1] - 1.0);
          i = t->size[1];
          for (b_i = 0; b_i <= i - 3; b_i++) {
            t->data[b_i + 1] = delta1 * (1.0 + (double)b_i);
          }
        } else {
          delta1 = param->minDist / ((double)t->size[1] - 1.0);
          i = t->size[1];
          for (b_i = 0; b_i <= i - 3; b_i++) {
            t->data[b_i + 1] = (1.0 + (double)b_i) * delta1;
          }
        }
      }
    }
  }

  if (!((!rtIsInf(*p_angle)) && (!rtIsNaN(*p_angle)))) {
    ex = rtNaN;
  } else {
    x = rt_remd_snf(*p_angle, 360.0);
    delta1 = std::abs(x);
    if (delta1 > 180.0) {
      if (x > 0.0) {
        x -= 360.0;
      } else {
        x += 360.0;
      }

      delta1 = std::abs(x);
    }

    if (delta1 <= 45.0) {
      x *= 0.017453292519943295;
      n = 0;
    } else if (delta1 <= 135.0) {
      if (x > 0.0) {
        x = 0.017453292519943295 * (x - 90.0);
        n = 1;
      } else {
        x = 0.017453292519943295 * (x + 90.0);
        n = -1;
      }
    } else if (x > 0.0) {
      x = 0.017453292519943295 * (x - 180.0);
      n = 2;
    } else {
      x = 0.017453292519943295 * (x + 180.0);
      n = -2;
    }

    if (n == 0) {
      ex = std::cos(x);
    } else if (n == 1) {
      ex = -std::sin(x);
    } else if (n == -1) {
      ex = std::sin(x);
    } else {
      ex = -std::cos(x);
    }
  }

  if (!((!rtIsInf(*p_angle)) && (!rtIsNaN(*p_angle)))) {
    x = rtNaN;
  } else {
    x = rt_remd_snf(*p_angle, 360.0);
    delta1 = std::abs(x);
    if (delta1 > 180.0) {
      if (x > 0.0) {
        x -= 360.0;
      } else {
        x += 360.0;
      }

      delta1 = std::abs(x);
    }

    if (delta1 <= 45.0) {
      x *= 0.017453292519943295;
      n = 0;
    } else if (delta1 <= 135.0) {
      if (x > 0.0) {
        x = 0.017453292519943295 * (x - 90.0);
        n = 1;
      } else {
        x = 0.017453292519943295 * (x + 90.0);
        n = -1;
      }
    } else if (x > 0.0) {
      x = 0.017453292519943295 * (x - 180.0);
      n = 2;
    } else {
      x = 0.017453292519943295 * (x + 180.0);
      n = -2;
    }

    if (n == 0) {
      x = std::sin(x);
    } else if (n == 1) {
      x = std::cos(x);
    } else if (n == -1) {
      x = -std::cos(x);
    } else {
      x = -std::sin(x);
    }
  }

  emxInit_real_T(&t_x, 2);
  i = t_x->size[0] * t_x->size[1];
  t_x->size[0] = 1;
  t_x->size[1] = t->size[1];
  emxEnsureCapacity_real_T(t_x, i);
  b_i = t->size[0] * t->size[1];
  for (i = 0; i < b_i; i++) {
    t_x->data[i] = t->data[i] * ex;
  }

  b_i = t->size[0] * t->size[1] - 1;
  i = t->size[0] * t->size[1];
  t->size[0] = 1;
  emxEnsureCapacity_real_T(t, i);
  for (i = 0; i <= b_i; i++) {
    t->data[i] *= x;
  }

  delta1 = ex * 6.123233995736766E-17 + x;
  eyOrtho = -ex + x * 6.123233995736766E-17;
  for (i = 0; i < 2; i++) {
    p_step[i] = 0.0;
    p_pos_step[i] = 0.0;
  }

  i = 1;
  exitg1 = false;
  while ((!exitg1) && (i - 1 <= (int)(m + -1.0) - 1)) {
    p_pos_step[0] = p_pos[0] + t_x->data[i];
    p_pos_step[1] = p_pos[1] + t->data[i];
    P[0] = (p_pos_step[0] + ex * EGO->lf) + EGO->w / 2.0 * delta1;
    P[1] = (p_pos_step[1] + x * EGO->lf) + EGO->w / 2.0 * eyOrtho;
    P[2] = (p_pos_step[0] + ex * EGO->lf) - EGO->w / 2.0 * delta1;
    P[3] = (p_pos_step[1] + x * EGO->lf) - EGO->w / 2.0 * eyOrtho;
    P[4] = (p_pos_step[0] - ex * EGO->lr) + EGO->w / 2.0 * delta1;
    P[5] = (p_pos_step[1] - x * EGO->lr) + EGO->w / 2.0 * eyOrtho;
    P[6] = (p_pos_step[0] - ex * EGO->lr) - EGO->w / 2.0 * delta1;
    P[7] = (p_pos_step[1] - x * EGO->lr) - EGO->w / 2.0 * eyOrtho;

    //  Conversion to grid
    p_step[0] = rt_roundd_snf(p_pos_step[0] / 0.5);
    p_step[1] = rt_roundd_snf(p_pos_step[1] / 0.5);
    for (b_i = 0; b_i < 8; b_i++) {
      P_grid[b_i] = rt_roundd_snf(P[b_i] / 0.5);
    }

    //  Checking for occupancy value
    probOccupancy[0] = gridConverted[(((int)p_step[0] + 80 * ((int)p_step[1] - 1))
      + 6400 * ((int)sampleTime - 1)) - 1];
    probOccupancy[1] = gridConverted[(((int)P_grid[0] + 80 * ((int)P_grid[1] - 1))
      + 6400 * ((int)sampleTime - 1)) - 1];
    probOccupancy[2] = gridConverted[(((int)P_grid[2] + 80 * ((int)P_grid[3] - 1))
      + 6400 * ((int)sampleTime - 1)) - 1];
    probOccupancy[3] = gridConverted[(((int)P_grid[4] + 80 * ((int)P_grid[5] - 1))
      + 6400 * ((int)sampleTime - 1)) - 1];
    probOccupancy[4] = gridConverted[(((int)P_grid[6] + 80 * ((int)P_grid[7] - 1))
      + 6400 * ((int)sampleTime - 1)) - 1];

    //      if param.plotGraph
    //          figure(3);
    //          plotObject_2(1) = plot([P(1) P(3)], [P(2) P(4)], 'Color', 'b', 'LineWidth', 2); 
    //          plotObject_2(2) = plot([P(1) P(5)], [P(2) P(6)], 'Color', 'b');
    //          plotObject_2(3) = plot([P(3) P(7)], [P(4) P(8)], 'Color', 'b');
    //          plotObject_2(4) = plot([P(7) P(5)], [P(8) P(6)], 'Color', 'b');
    //      end
    for (b_i = 0; b_i < 5; b_i++) {
      b_x[b_i] = (probOccupancy[b_i] > param->minThresh);
    }

    y = false;
    b_i = 0;
    exitg2 = false;
    while ((!exitg2) && (b_i < 5)) {
      if (b_x[b_i]) {
        y = true;
        exitg2 = true;
      } else {
        b_i++;
      }
    }

    if (y) {
      *col = 1.0;

      //          if param.plotGraph
      //              delete(plotObject_2);
      //          end
      exitg1 = true;
    } else {
      i++;
    }
  }

  emxFree_real_T(&t_x);
  emxFree_real_T(&t);
}

//
// File trailer for InCollision_Edge.cpp
//
// [EOF]
//
