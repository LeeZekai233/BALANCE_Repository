/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lqr_k.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 18-Mar-2024 16:21:49
 */

/* Include Files */
#include "lqr_k.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * LQR_K
 *     K = LQR_K(L0)
 *
 * Arguments    : double L0
 *                double K[12]
 * Return Type  : void
 */
void lqr_k(double L0, double K[12])
{
  double t2;
  double t3;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2024-03-18 16:21:07 */
  t2 = L0 * L0;
  t3 = rt_powd_snf(L0, 3.0);
  K[0] = ((L0 * -188.0979417294775 + t2 * 196.2711201387192) -
          t3 * 102.1898825273429) -
         4.0444121665275814;
  K[1] = ((L0 * 889.395127069674 - t2 * 2615.6322745719) +
          t3 * 2535.188316302409) -
         14.508601442326089;
  K[2] = ((L0 * -27.238914598312 + t2 * 3.4206070806111648) -
          t3 * 1.3307700052804621) +
         0.32095098946980122;
  K[3] = ((L0 * 137.7749599043191 - t2 * 372.64676999781028) +
          t3 * 356.54333514254972) +
         0.45381509174984069;
  K[4] = ((L0 * -17.572099196096321 + t2 * 6.94704206930822) +
          t3 * 16.713686389144168) -
         1.803965837261339;
  K[5] = ((L0 * 197.03480757629529 - t2 * 691.11777594285388) +
          t3 * 729.93905142376923) -
         0.26797893632203129;
  K[6] = ((L0 * -21.508085724457629 - t2 * 10.745747937731959) +
          t3 * 46.309977855738957) -
         4.2054198883019076;
  K[7] = ((L0 * 306.74254739534513 - t2 * 1077.6546512317209) +
          t3 * 1142.8529673953) +
         0.67019773680314454;
  K[8] = ((L0 * -49.600450813044468 + t2 * 46.862666981597933) -
          t3 * 23.6227186268123) +
         29.91830985525225;
  K[9] = ((L0 * 146.84610685892849 + t2 * 17.732739412644261) -
          t3 * 237.79628209031921) +
         22.056563271908619;
  K[10] = ((L0 * -15.891732670145769 + t2 * 35.62274317381668) -
           t3 * 34.3525933185872) +
          5.9500628114089684;
  K[11] = ((L0 * -15.9840850382652 + t2 * 124.6211686324778) -
           t3 * 163.97155244350691) +
          3.2052607282776351;
}

/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */
