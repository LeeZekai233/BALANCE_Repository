#ifndef __INFANTRY_MODE_SWITCH_TASK_H
#define __INFANTRY_MODE_SWITCH_TASK_H
#include "public.h"

#include "CHASSIS_TASK.H"

/*************************************************/
#define HIGH_SPEED 2.7
#define NORMAL_SPEED 1.9 




void infantry_mode_switch_task(void);


extern chassis_t chassis;
extern float leg_length;
extern u8 sperate_flag;









#endif
