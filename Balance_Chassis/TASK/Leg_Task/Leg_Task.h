#ifndef __LEG_TASK_H
#define __LEG_TASK_H
#include <stm32f4xx.h>
#include "PID.h"

typedef struct
{
	float pos[2]; //pos=[l0; phi0];
	float spd[2]; //spd[2]=[dl0; dphi0];
	float T_Set[2];//T[2]=[motor4;motor1];

	//支持力解算用计算变量
	float J[4];   //解雅可比矩阵的中间变量
	float j[2][2];//最终的雅可比矩阵
	float F_fdb;
	float Tp_fdb;
    float dtheta;
    float ddtheta;

    
	float phi4;
	float phi1;
	float dphi4;
	float dphi1;

	float this_dl0;
	float last_dl0;

	float l0;
	float dl0;
	float ddl0;
	float phi0;
	float dphi0;
    
	float Leg_F;
	float ddzw;
	float Leg_FN;
	float Leg_Final_FN;

	uint8_t Wheel_State;

	PID_t Leg_Length_PID;//腿长PID
    float spring_FN;
    float theta;
    
    
	
}Leg_State_t;//腿状态，采用论文中VMC的字母


void leg_conv(float F, float Tp, float phi1, float phi4,
                     float T[2]);
void leg_pos(float phi1, float phi4, float pos[2]);
void leg_spd(float dphi1, float dphi4, float phi1, float phi4, float spd[2]);
void lqr_k(float L0, float K[12]);
void leg_J_cal(float phi1, float phi4, float J[4]);
void VMC_Data_Get(Leg_State_t* Leg_State , float phi4, float dphi4, float phi1, float dphi1);
float Get_Left_GasSpring_FN(float Left_Leglength);
float Get_Right_GasSpring_FN(float Right_Leglength);


#endif
