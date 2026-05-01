#ifndef __POWERCONTROL_TASK_H__
#define __POWERCONTROL_TASK_H__

#include "main.h"

//功控拟合公式选取  科大ROBOWALKER 0     RPS 1
#define PWC_FITTING_FORMULA     1


/**科大公式 PWC PoWerControl*/  
/**P = k0 + k1*I + k2*u + k3*I*u + k4*I**2 + k5*u**2 */ 
/**I输出电流+-A u电机轮子边缘线速度 +-10*/
#define PWC_K0 (float)0.6641993422153354
#define PWC_K1 (float)7.866580814387202e-06
#define PWC_K2 (float)0.040045908022913024
#define PWC_K3 (float)0.006057734368395625
#define PWC_K4 (float)2.458905951400473e-07
#define PWC_K5 (float)2.449555275893558
#define PWC_R2 (float)0.9741              //相关系数

/**自己公式 */
//电机热功率 P_heat = I**2*F2 + I*F1 + F0
//P = 
//新舵轮
#define FACTOR_2    (float)2.3833721747452773e-07
#define FACTOR_1	  (float)1.4505538281165652e-05
#define FACTOR_0    (float)0.9097964196761054
#define I_TIMES_V_TO_WATT   (float)0.00642213410783477
	//py获得的参数 旧全向哨兵
// #define FACTOR_2    (float)1.8816743761221132e-07
// #define FACTOR_1	  (float)-9.87128088082091e-05
// #define FACTOR_0    (float)0.523
// #define I_TIMES_V_TO_WATT   (float)0.00646251
	//旧参数
//#define FACTOR_2    (float)1.8e-07f
//#define FACTOR_1	(float)-5.017e-04f
//#define FACTOR_0    (float)0.523
//#define I_TIMES_V_TO_WATT   (float)0.0062618f

//GM6020系数
#define GM6020_K0 (float)1.0015598852257386
#define GM6020_K1 (float)-0.0011271350592013643
#define GM6020_K2 (float)0.16286428624826996
#define GM6020_K3 (float)0.6309113499764698
#define GM6020_K4 (float)0.0012542303316232962
#define GM6020_K5 (float)7.632464716389212
//GM6020系数
#define GM6020_F0	(float)0.0
#define GM6020_F1	(float)0.0
#define GM6020_F2	(float)0.0
#define GM6020_ItV	(float)0.0
/***************************Function*************************** */
float PowerControl_MaxPower_Get(Chassis_t *pChassis);
float PowerControl_SpeedLimitRate_Get(Chassis_t *pChassis);
void PowerControl_DrivingPower_ReDistribution(Chassis_t *pChassis);
void PowerControl_CurrentLimitRate_Get(Chassis_t *pChassis);
void PowerControl_Chassis_SpeedRef_Limit(Chassis_t *pChassis);
void PowerControl_Chassis_PID_Out_Limit(Chassis_t *pChassis);
void PowerControl_Power_Detect(Chassis_t *pChassis);

#endif


