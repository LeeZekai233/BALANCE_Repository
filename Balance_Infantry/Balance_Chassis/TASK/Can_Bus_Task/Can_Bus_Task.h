#ifndef __CAN_BUS_TASK_H
#define __CAN_BUS_TASK_H
#include <stm32f4xx.h>
#include "Chassis_Task.h"


#define JOINT1_CONTROL_ID 0x01
#define JOINT2_CONTROL_ID 0x02
#define JOINT3_CONTROL_ID 0x03
#define JOINT4_CONTROL_ID 0x04


#define JOINT1_FEEDBACK_ID  0xA1
#define JOINT2_FEEDBACK_ID  0xA2
#define JOINT3_FEEDBACK_ID  0xA3
#define JOINT4_FEEDBACK_ID  0xA4


#define DRIVING_LEFT_FEEDBACK_ID 0x201
#define DRIVING_RIGHT_FEEDBACK_ID 0x202


#define JOINT1_ENCODER_OFFSET +1.57221067- 0.33405f//+2.5583837- 0.33405f//+2.5770793- 0.33405f
#define JOINT2_ENCODER_OFFSET +2.69538951+ 0.33405f//+2.70574403+ 0.33405f//+2.70037508+ 0.33405f
#define JOINT3_ENCODER_OFFSET -0.637522697- 0.75432f//-0.639727831- 0.75432f//-0.61681366- 0.75432f
#define JOINT4_ENCODER_OFFSET +1.73366463+ 0.75432f//+1.70634007+ 0.75432f//+1.7635777 + 0.75432f



void CAN1_Receive_Task(CanRxMsg* RxMsg,Balance_Chassis_t* Chassis);
void CAN2_Receive_Task(CanRxMsg* RxMsg,Balance_Chassis_t* Chassis);
void CAN1_Send_Task_1(float Joint_T_Set1,float Joint_T_Set4);
void CAN1_Send_Task_2(float Joint_T_Set2, float Joint_T_Set3);
void CAN2_Send_Task(float Driving_T_Set1, float Driving_T_Set2);

extern float temp;

#endif
