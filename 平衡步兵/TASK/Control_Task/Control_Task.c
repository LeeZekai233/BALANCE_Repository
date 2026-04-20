#include "main.h"

uint32_t time_tick = 0;

void Contorl_Task(void)
{
    time_tick++;
    
}



void Control_Task_Init(Balance_Chassis_t* Chassis)
{
    Chassis_Param_Init(Chassis);
}


