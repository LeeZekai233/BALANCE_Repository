#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H
#include <stm32f4xx.h>
#include "Chassis_Task.h"

extern uint32_t time_tick;


void Contorl_Task(Balance_Chassis_t* Chassis);
void Control_Task_Init(Balance_Chassis_t* Chassis);


#endif
