#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H
#include "stm32f4xx.h"                  // Device header


extern uint32_t time_tick;





void Control_Task(void);
void Control_Task_Init(void);
void Chassis_Mode_Select(void);
void Chassis_Reference_Update(void);
void Chassis_Task(void);

#endif
