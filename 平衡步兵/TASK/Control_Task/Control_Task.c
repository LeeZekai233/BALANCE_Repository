#include "main.h"

uint32_t time_tick = 0;

void Contorl_Task(Balance_Chassis_t* Chassis)
{
    time_tick++;
    if(time_tick%2==0)
    {
        Chassis_Task(Chassis);
        CAN1_Send_Task_1(Chassis->joint_T[0],Chassis->joint_T[3]);
    }
    
    if(time_tick%2==1)
    {
        CAN1_Send_Task_2(Chassis->joint_T[1],Chassis->joint_T[2]);
        CAN2_Send_Task(Chassis->driving_T[0],Chassis->driving_T[1]);//Òª¸Ä
    }
    
//    if(time_tick%5==0)
//    {
//        usart_gimbal_send()
//    }
}



void Control_Task_Init(Balance_Chassis_t* Chassis)
{
    Chassis_Param_Init(Chassis);
}


