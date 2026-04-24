#ifndef __MOTORRUN_H__
#define __MOTORRUN_H__

#include "public.h"

#define REDUCTION_RATIO 14

void Motor_Run(void);
void motor_run_init(void);

extern float motor620_set;


void motor620_run(void);
void Motor_620_Speed(void);
void Motor_620_Angle(int32_t angle);
void Motor_6020_Angle(int32_t angle);

#endif
