#ifndef __LEG_TASK_H
#define __LEG_TASK_H
#include <stm32f4xx.h>
#include "Chassis_Task.h"


void leg_conv(float F, float Tp, float phi1, float phi4,
                     float T[2]);
void leg_pos(float phi1, float phi4, float* pos1,float* pos2);//输入phi1和phi4，得出phi0和l0
void leg_spd(float dphi1, float dphi4, float phi1, float phi4,
             Leg_State_t* Leg_State);//输入phi1，phi4，dphi1，dphi4，得出dphi0和dl0
void lqr_k(float L0, float K[12]);
void leg_J_cal(float phi1, float phi4, float J[4]);
void VMC_Data_Get(Leg_State_t* Leg_State , float phi4, float dphi4, float phi1, float dphi1);

#endif
