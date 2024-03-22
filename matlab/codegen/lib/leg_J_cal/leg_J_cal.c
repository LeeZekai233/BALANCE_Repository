/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: leg_J_cal.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 24-Jan-2024 20:37:46
 */

/* Include Files */
#include "leg_J_cal.h"
#include <math.h>

/* Function Definitions */
/*
 * leg_J_cal
 *     J = leg_J_cal(PHI1,PHI4)
 *
 * Arguments    : double phi1
 *                double phi4
 *                double J[4]
 * Return Type  : void
 */
void leg_J_cal(double phi1, double phi4, double J[4])
{
  double t106;
  double t10_tmp;
  double t12_tmp;
  double t2;
  double t21;
  double t22;
  double t23;
  double t24;
  double t28;
  double t3;
  double t30;
  double t32;
  double t38;
  double t4;
  double t44;
  double t47;
  double t48;
  double t5;
  double t52;
  double t53;
  double t59;
  double t60;
  double t70;
  double t71;
  double t76;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2024-01-24 20:35:20 */
  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t10_tmp = t2 * 0.15;
  t12_tmp = t4 * 0.15;
  t21 = t2 * 0.0864;
  t22 = t3 * 0.0864;
  t23 = t4 * 0.0864;
  t24 = t5 * 0.0864;
  t28 = t12_tmp - t5 * 0.15;
  t30 = (t3 * 0.15 - t10_tmp) + 0.15;
  t32 = t23 - t24;
  t38 = (t22 - t21) + 0.0864;
  t44 = t28 * t28 + t30 * t30;
  t47 = t2 * t28 * 0.3 + t4 * t30 * 0.3;
  t48 = t3 * t28 * 0.3 + t5 * t30 * 0.3;
  t52 = 1.0 / (t38 + t44);
  t53 = t52 * t52;
  t59 = sqrt((t32 * t32 + t38 * t38) - t44 * t44);
  t60 = 1.0 / t59;
  t30 = (t24 - t23) + t59;
  t28 = atan(t52 * t30) * 2.0;
  t70 = cos(t28);
  t71 = sin(t28);
  t76 = 1.0 / (t53 * (t30 * t30) + 1.0);
  t106 =
      (t23 + t47) * t53 * t30 +
      t52 * (t21 -
             t60 * ((t2 * t32 * 0.1728 + t4 * t38 * 0.1728) - t44 * t47 * 2.0) /
                 2.0);
  t28 =
      (t24 + t48) * t53 * t30 +
      t52 * (t22 -
             t60 * ((t3 * t32 * 0.1728 + t5 * t38 * 0.1728) - t44 * t48 * 2.0) /
                 2.0);
  t21 = t12_tmp + t71 * 0.288;
  t23 = (t10_tmp + t70 * 0.288) - 0.075;
  t59 = t70 * t76;
  t47 = t59 * t28;
  t60 = t71 * t76;
  t4 = t60 * t28;
  t28 = (-t10_tmp - t70 * 0.288) + 0.075;
  t2 = t28 * t28;
  t52 = 1.0 / t28;
  t30 = t21 * t21;
  t53 = 1.0 / sqrt(t30 + t23 * t23);
  t48 = 1.0 / (t30 + t2);
  t59 = t10_tmp - t59 * t106 * 0.576;
  t28 = t12_tmp - t60 * t106 * 0.576;
  J[0] = t53 * (t21 * t59 * 2.0 - t23 * t28 * 2.0) / 2.0;
  t30 = t21 * (1.0 / t2);
  J[1] = -t2 * t48 * (t52 * t59 - t30 * t28);
  J[2] = t53 * (t21 * t47 * 1.152 - t23 * t4 * 1.152) / 2.0;
  J[3] = t2 * t48 * (t52 * (0.0 - t47 * 0.576) + t30 * (t4 * 0.576));
}

/*
 * File trailer for leg_J_cal.c
 *
 * [EOF]
 */
