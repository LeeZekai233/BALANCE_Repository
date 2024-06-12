/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: leg_pos.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-Jun-2024 22:14:42
 */

/* Include Files */
#include "leg_pos.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2(i, i1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * LEG_POS
 *     POS = LEG_POS(PHI1,PHI4)
 *
 * Arguments    : double phi1
 *                double phi4
 *                double pos[2]
 * Return Type  : void
 */
void leg_pos(double phi1, double phi4, double pos[2])
{
  double a;
  double b_a;
  double t14;
  double t15;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t8;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2024-06-09 22:00:09 */
  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t6 = t2 * 0.15;
  t8 = t4 * 0.15;
  t14 = t4 * 0.081;
  t15 = t5 * 0.081;
  t5 = t8 - t5 * 0.15;
  a = (t3 * 0.15 - t6) + 0.15;
  b_a = t14 - t15;
  t4 = (t3 * 0.081 - t2 * 0.081) + 0.081;
  t5 = t5 * t5 + a * a;
  t4 = atan(1.0 / (t4 + t5) *
            ((t15 - t14) + sqrt((b_a * b_a + t4 * t4) - t5 * t5))) *
       2.0;
  t5 = t8 + sin(t4) * 0.27;
  t4 = (t6 + cos(t4) * 0.27) - 0.075;
  pos[0] = sqrt(t5 * t5 + t4 * t4);
  pos[1] = rt_atan2d_snf(t5, t4);
}

/*
 * File trailer for leg_pos.c
 *
 * [EOF]
 */
