#ifndef __GENERIC_ENCODER_H
#define __GENERIC_ENCODER_H
#include <stm32f4xx.h>



typedef struct
{
    
    float Single_Angle_fdb;	     //当前单圈角度   单位°
    float Multi_Angle_fdb;       //当前多圈角度   单位°
    
    float Angular_Vel_fdb;		//电机当前转速  rad/s
    
    uint8_t online_flag;
    
    float Torque;             //力矩
    
    uint32_t temperature;    //温度
}Encoder_t;






#endif
