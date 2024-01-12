#include <leg_task.h>


void VMC_data_get(leg_state_t *leg , float phi4_angle, 
                                    float phi4_gyro,
                                    float phi1_angle,
                                    float phi1_gyro)
{
    leg->last_dl0 = leg->this_dl0;
    leg->phi4 = phi4_angle;
    leg->phi1 = phi1_angle;
    leg->dphi4 = phi4_gyro;
    leg->dphi1 = phi1_gyro;

    leg_pos(leg->phi1, leg->phi4, leg->pos);
    leg->l0 = leg->pos[0];
    leg->phi0 = leg->pos[1];
    leg_spd(leg->dphi1, leg->dphi4, leg->phi1, leg->phi4, leg->spd);
    leg->dl0 = leg->spd[0];
    leg->this_dl0 = leg->dl0;
    leg->dphi0 = leg->spd[1];
    leg->ddl0 = (leg->this_dl0 - leg->last_dl0) / (TIME_STEP * 0.001);//可能要加低通滤波器

    leg_J_cal(phi1_angle,phi4_angle,leg->J);
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            
                leg->j[j][i] = leg->J[i * 2 + j];
            
            
        }
            
    }

    
}

mat Jacobian,
    JacobianT,
    JacobinT_inv,
    mat_F,
    mat_T;
    

static float  Jacobian_data[4];
static float  JacobianT_data[4];
static float  JacobinT_inv_data[4];
static float  mat_F_data[2];
static float  mat_T_data[2];

void FN_calculate(leg_state_t *leg,float MT1_torque,float MT4_torque)
{
    float costheta = cosf(b_chassis.balance_loop.theta);
    float sintheta = sinf(b_chassis.balance_loop.theta);

    //dtheta的计算
    static float this_dtheta, last_dtheta;
      last_dtheta = this_dtheta;
      this_dtheta = b_chassis.balance_loop.dtheta;
      b_chassis.balance_loop.ddtheta = (this_dtheta - last_dtheta) / ((TIME_STEP * 0.001));
    //ddzw的计算
      leg->ddzw = b_chassis.balance_loop.ddz - leg->ddl0 * costheta + \
                  2 * leg->dl0 * b_chassis.balance_loop.dtheta * sintheta + \
                    leg->l0 * b_chassis.balance_loop.ddtheta * sintheta + \
                    leg->l0 * (b_chassis.balance_loop.dtheta * b_chassis.balance_loop.dtheta) * costheta;
    //P和Tp的计算
    mat_init(&Jacobian,2,2,(float *)Jacobian_data);
    mat_init(&JacobianT,2,2,(float *)JacobianT_data);
    mat_init(&JacobinT_inv,2,2,(float *)JacobinT_inv_data);
    mat_init(&mat_F,2,1,(float *)mat_F_data);
    mat_init(&mat_T,2,1,(float *)mat_T_data);

    Jacobian_data[0] = leg->j[0][0];
    Jacobian_data[1] = leg->j[0][1];
    Jacobian_data[2] = leg->j[1][0];
    Jacobian_data[3] = leg->j[1][1];

    mat_T_data[0] = MT1_torque;
    mat_T_data[1] = MT4_torque;

    //求得VMC逆转换矩阵
    // FTp = (J')\[MT1;MT4];
    mat_trans(&Jacobian,&JacobianT);
    mat_inv(&JacobianT,&JacobinT_inv);
    mat_mult(&JacobinT_inv,&mat_T,&mat_F);

    leg->F_fdb = mat_F.pData[0];
    leg->Tp_fdb = mat_F.pData[1];

    float P = leg->F_fdb*costheta + (leg->Tp_fdb*sintheta)/leg->l0;
    //支持力的计算
      leg->leg_FN = WHEEL_MASS * leg->ddzw + P + WHEEL_MASS * 9.81;


      
}






/*
 * LEG_POS
 *     POS = LEG_POS(PHI1,PHI4)
 *
 * Arguments    : float phi1
 *                float phi4
 *                float pos[2]
 * Return Type  : void
 */
void leg_pos(float phi1, float phi4, float pos[2])
{
    float a;
    float b_a;
    float t14;
    float t15;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t8;
    /*     This function was generated by the Symbolic Math Toolbox version 23.2.
     */
     /*     2023-10-29 16:09:03 */
    t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t6 = t2 * 0.15;
  t8 = t4 * 0.15;
  t14 = t4 * 0.0864;
  t15 = t5 * 0.0864;
  t5 = t8 - t5 * 0.15;
  a = (t3 * 0.15 - t6) + 0.15;
  b_a = t14 - t15;
  t4 = (t3 * 0.0864 - t2 * 0.0864) + 0.0864;
  t5 = t5 * t5 + a * a;
  t4 = atan(1.0 / (t4 + t5) *
            ((t15 - t14) + sqrt((b_a * b_a + t4 * t4) - t5 * t5))) *
       2.0;
  t5 = t8 + sin(t4) * 0.288;
  t4 = (t6 + cos(t4) * 0.288) - 0.075;
    pos[0] = sqrt(t5 * t5 + t4 * t4);
    pos[1] = atan2f(t5, t4);
}

/*
 * File trailer for leg_pos.c
 *
 * [EOF]
 */


void leg_spd(float dphi1, float dphi4, float phi1, float phi4,
             float spd[2])
{
  float t10_tmp;
  float t12_tmp;
  float t2;
  float t21;
  float t22;
  float t23;
  float t24;
  float t28;
  float t3;
  float t30;
  float t32;
  float t38;
  float t4;
  float t44;
  float t47;
  float t48;
  float t5;
  float t52;
  float t53;
  float t59;
  float t60;
  float t70;
  float t71;
  float t76;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-21 22:52:09 */
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
  t47 =
      (t23 + t47) * t53 * t30 +
      t52 * (t21 -
             t60 * ((t2 * t32 * 0.1728 + t4 * t38 * 0.1728) - t44 * t47 * 2.0) /
                 2.0);
  t28 =
      (t24 + t48) * t53 * t30 +
      t52 * (t22 -
             t60 * ((t3 * t32 * 0.1728 + t5 * t38 * 0.1728) - t44 * t48 * 2.0) /
                 2.0);
  t4 = t12_tmp + t71 * 0.288;
  t21 = (t10_tmp + t70 * 0.288) - 0.075;
  t59 = t70 * t76;
  t23 = t59 * t28;
  t53 = t71 * t76;
  t2 = t53 * t28;
  t28 = (-t10_tmp - t70 * 0.288) + 0.075;
  t60 = t28 * t28;
  t52 = 1.0 / t28;
  t30 = t4 * t4;
  t28 = 1.0 / sqrt(t30 + t21 * t21);
  t48 = 1.0 / (t30 + t60);
  t59 = t10_tmp - t59 * t47 * 0.576;
  t30 = t12_tmp - t53 * t47 * 0.576;
  spd[0] = dphi4 * t28 * (t4 * t23 * 1.152 - t21 * t2 * 1.152) / 2.0 +
           dphi1 * t28 * (t4 * t59 * 2.0 - t21 * t30 * 2.0) / 2.0;
  t28 = t4 * (1.0 / t60);
  spd[1] = -dphi1 * t60 * t48 * (t52 * t59 - t28 * t30) +
           dphi4 * t60 * t48 * (t52 * (0.0 - t23 * 0.576) + t28 * (t2 * 0.576));
}
/*
 * File trailer for leg_spd.c
 *
 * [EOF]
 */



 /* Function Definitions */
 /*
  * LEG_CONV
  *     T = LEG_CONV(F,Tp,PHI1,PHI4)
  *
  * Arguments    : float F
  *                float Tp
  *                float phi1
  *                float phi4
  *                float T[2]
  * Return Type  : void
  */
void leg_conv(float F, float Tp, float phi1, float phi4, float T[2])
{
    float t104;
  float t106_tmp;
  float t108_tmp;
  float t120_tmp;
  float t123_tmp;
  float t124_tmp;
  float t131_tmp;
  float t16_tmp;
  float t174;
  float t175;
  float t18_tmp;
  float t33_tmp;
  float t34_tmp;
  float t35_tmp;
  float t36_tmp;
  float t51_tmp;
  float t54_tmp;
  float t5_tmp;
  float t64_tmp;
  float t75_tmp;
  float t7_tmp;
  float t81_tmp;
  float t82_tmp;
  float t91_tmp;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-21 22:52:10 */
  t174 = cos(phi1);
  t5_tmp = cos(phi4);
  t175 = sin(phi1);
  t7_tmp = sin(phi4);
  t16_tmp = t174 * 0.15;
  t18_tmp = t175 * 0.15;
  t33_tmp = t174 * 0.0864;
  t34_tmp = t5_tmp * 0.0864;
  t35_tmp = t175 * 0.0864;
  t36_tmp = t7_tmp * 0.0864;
  t104 = t18_tmp - t7_tmp * 0.15;
  t51_tmp = (t5_tmp * 0.15 - t16_tmp) + 0.15;
  t54_tmp = t35_tmp - t36_tmp;
  t64_tmp = (t34_tmp - t33_tmp) + 0.0864;
  t75_tmp = t104 * t104 + t51_tmp * t51_tmp;
  t81_tmp = t174 * t104 * 0.3 + t175 * t51_tmp * 0.3;
  t82_tmp = t5_tmp * t104 * 0.3 + t7_tmp * t51_tmp * 0.3;
  t51_tmp = 1.0 / (t64_tmp + t75_tmp);
  t91_tmp = t51_tmp * t51_tmp;
  t104 = sqrt((t54_tmp * t54_tmp + t64_tmp * t64_tmp) - t75_tmp * t75_tmp);
  t106_tmp = 1.0 / t104;
  t108_tmp = (t36_tmp - t35_tmp) + t104;
  t120_tmp = atan(t51_tmp * t108_tmp) * 2.0;
  t123_tmp = cos(t120_tmp);
  t124_tmp = sin(t120_tmp);
  t131_tmp = 1.0 / (t91_tmp * (t108_tmp * t108_tmp) + 1.0);
  t120_tmp =
      (t35_tmp + t81_tmp) * t91_tmp * t108_tmp +
      t51_tmp *
          (t33_tmp - t106_tmp *
                         ((t174 * t54_tmp * 0.1728 + t175 * t64_tmp * 0.1728) -
                          t75_tmp * t81_tmp * 2.0) /
                         2.0);
  t33_tmp =
      (t36_tmp + t82_tmp) * t91_tmp * t108_tmp +
      t51_tmp * (t34_tmp -
                 t106_tmp *
                     ((t5_tmp * t54_tmp * 0.1728 + t7_tmp * t64_tmp * 0.1728) -
                      t75_tmp * t82_tmp * 2.0) /
                     2.0);
  t35_tmp = t18_tmp + t124_tmp * 0.288;
  t81_tmp = (t16_tmp + t123_tmp * 0.288) - 0.075;
  t104 = (-t16_tmp - t123_tmp * 0.288) + 0.075;
  t174 = t104 * t104;
  t175 = 1.0 / t104;
  t104 = t35_tmp * t35_tmp;
  t106_tmp = t123_tmp * t131_tmp;
  t91_tmp = t16_tmp - t106_tmp * t120_tmp * 0.576;
  t82_tmp = t124_tmp * t131_tmp;
  t120_tmp = t18_tmp - t82_tmp * t120_tmp * 0.576;
  t108_tmp = F * (1.0 / sqrt(t104 + t81_tmp * t81_tmp));
  t51_tmp = Tp * t174 * (1.0 / (t104 + t174));
  t104 = t35_tmp * (1.0 / t174);
  T[0] = t108_tmp * (t35_tmp * t91_tmp * 2.0 - t81_tmp * t120_tmp * 2.0) / 2.0 -
         t51_tmp * (t175 * t91_tmp - t104 * t120_tmp);
  T[1] = t108_tmp *
             (t106_tmp * t35_tmp * t33_tmp * 1.152 -
              t82_tmp * t81_tmp * t33_tmp * 1.152) /
             2.0 +
         t51_tmp * (t175 * (0.0 - t106_tmp * t33_tmp * 0.576) +
                    t104 * (t82_tmp * t33_tmp * 0.576));
}

/*
 * File trailer for leg_conv.c
 *
 * [EOF]
 */




/* Function Definitions */
/*
 * leg_J_cal
 *     J = leg_J_cal(PHI1,PHI4)
 *
 * Arguments    : float phi1
 *                float phi4
 *                float J[4]
 * Return Type  : void
 */
void leg_J_cal(float phi1, float phi4, float J[4])
{
  float t106;
  float t10_tmp;
  float t12_tmp;
  float t2;
  float t21;
  float t22;
  float t23;
  float t24;
  float t28;
  float t3;
  float t30;
  float t32;
  float t38;
  float t4;
  float t44;
  float t47;
  float t48;
  float t5;
  float t52;
  float t53;
  float t59;
  float t60;
  float t70;
  float t71;
  float t76;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-21 22:52:11 */
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
