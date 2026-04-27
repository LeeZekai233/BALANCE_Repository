#ifndef __CAN_BUS_TASK_H
#define __CAN_BUS_TASK_H
#include <stm32f4xx.h>
#include "Chassis_Task.h"



void CAN_Receive_Task(CanRxMsg* RxMsg,Balance_Chassis_t* Chassis);
void CAN1_Send_Task_1(float Joint_T_Set1,float Joint_T_Set4);
void CAN1_Send_Task_2(float Joint_T_Set2, float Joint_T_Set3);
void CAN2_Send_Task(float Driving_I_Set1, float Driving_I_Set2);



#endif
