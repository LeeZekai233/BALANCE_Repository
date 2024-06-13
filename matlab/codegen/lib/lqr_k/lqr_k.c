/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lqr_k.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 14-Jun-2024 06:37:58
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
  /*     2024-06-12 15:11:56 */
  t2 = L0 * L0;
  t3 = rt_powd_snf(L0, 3.0);
  K[0] = ((L0 * -334.62862824681548 + t2 * 550.30561457754879) -
          t3 * 426.84149209452983) +
         0.34607515200551842;
  K[1] = ((L0 * 556.935247561363 - t2 * 2018.4845265092381) +
          t3 * 2161.6714771288321) +
         14.550610573294749;
  K[2] = ((L0 * -37.190884158808942 + t2 * 17.156734055617608) -
          t3 * 14.66331401971593) +
         0.30752245092349839;
  K[3] = ((L0 * 86.095104486204221 - t2 * 275.76751808717933) +
          t3 * 283.10008865262591) +
         2.3866750744414289;
  K[4] = ((L0 * -49.655966031210532 + t2 * 108.78603145555761) -
          t3 * 88.7430197071483) -
         0.48975566425563211;
  K[5] = ((L0 * 92.52235515291953 - t2 * 408.80734448516728) +
          t3 * 473.31913086822732) +
         6.6669873893078853;
  K[6] = ((L0 * -74.653879981929521 + t2 * 156.74450923617781) -
          t3 * 127.4546300410605) -
         2.3093047244316112;
  K[7] = ((L0 * 123.64115946715989 - t2 * 590.76917856936166) +
          t3 * 699.30144928633092) +
         14.96094589719929;
  K[8] = ((L0 * -82.68351273086715 - t2 * 36.950794733110847) +
          t3 * 148.0946834762787) +
         50.767605140381043;
  K[9] = ((L0 * 379.96917963846641 - t2 * 669.11283476433744) +
          t3 * 418.64464220821282) +
         41.97368420547263;
  K[10] = ((L0 * -10.12408340457603 + t2 * 8.7100936492048024) -
           t3 * 1.4465034674904571) +
          5.2774670210423071;
  K[11] = ((L0 * 14.401329736688609 - t2 * 1.8269366667257649) -
           t3 * 20.35566562957807) +
          3.013639578127294;
}

/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */
