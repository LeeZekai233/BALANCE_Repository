#ifndef __LEG_TASK_H
#define __LEG_TASK_H
#include <stm32f4xx.h>
#include "Chassis_Task.h"

void leg_conv(double F, double Tp, double phi1, double phi4,
                     double T[2]);
void leg_pos(double phi1, double phi4, float* pos1,float* pos2);//输入phi1和phi4，得出phi0和l0
void leg_spd(double dphi1, double dphi4, double phi1, double phi4,
             Leg_State_t* Leg_State);//输入phi1，phi4，dphi1，dphi4，得出dphi0和dl0
void lqr_k(double L0, double K[12]);//后续double全改float


#endif
