#ifndef __CAN_BUS_TASK_H
#define __CAN_BUS_TASK_H
#include "stm32f4xx.h"                  // Device header



void CAN1_Send_Task(int16_t Pitch_Current_Set,int16_t Left_Fric_Current_Set,int16_t Right_Fric_Current_Set);
void CAN2_Send_Task(float Yaw_Speed_Set,int16_t Poke_Current_Set);
void CAN1_Receive_Task(CanRxMsg* RxMsg);
void CAN2_Receive_Task(CanRxMsg* RxMsg);



#endif
