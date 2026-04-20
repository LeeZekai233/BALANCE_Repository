#include "main.h"


float prev_omega_left,prev_omega_right;

//通过差分计算加速度，用于打滑检测
float difference_left_calc(float omega,float dt)
{
    float acceleration = (omega - prev_omega_left) / dt;
	prev_omega_left = omega;
    return acceleration;
}


float difference_right_calc(float omega,float dt)
{
    float acceleration = (omega - prev_omega_right) / dt;
	prev_omega_right = omega;
    return acceleration;
}

