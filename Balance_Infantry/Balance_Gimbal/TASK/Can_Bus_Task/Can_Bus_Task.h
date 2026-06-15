#ifndef __CAN_BUS_TASK_H
#define __CAN_BUS_TASK_H
#include "stm32f4xx.h"                  // Device header

#define FRIC_LEFT_FEEDBACK_ID 0x201
#define FRIC_RIGHT_FEEDBACK_ID 0x202
#define PITCH_FEEDBACK_ID 0x205
#define POKE_FEEDBACK_ID 0x141
#define YAW_FEEDBACK_ID 0xA1


void CAN1_Send_Task(int16_t Pitch_Current_Set,int16_t Left_Fric_Current_Set,int16_t Right_Fric_Current_Set);
void CAN2_Send_Task(float Yaw_Current_Set,float Poke_Speed_Set);
void CAN1_Receive_Task(CanRxMsg* RxMsg);
void CAN2_Receive_Task(CanRxMsg* RxMsg);



#endif
