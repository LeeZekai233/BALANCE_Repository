#ifndef SHOOTER_KALMAN_FILTER_H
#define SHOOTER_KALMAN_FILTER_H

#include "stm32f4xx.h"                  // Device header


#define SHOOT_ERROR_MEA       2
#define SHOOT_ERRPR_EST_INIT  5

typedef __packed struct{
	float Error_Mea;//测量误差
	float Error_Est;//估计误差
	float Error_Est_Last;
	float Kalman_Gain;
	float X_hat;//估计真实值
	float X_hat_Last;
}First_Order_Kalman_Filter_t;

extern First_Order_Kalman_Filter_t First_Order_Kalman_Filter;

float First_Order_Kalman_Filter_Cal(First_Order_Kalman_Filter_t *_First_Order_Kalman_Filter,float _Z/*测量值*/);


#endif

