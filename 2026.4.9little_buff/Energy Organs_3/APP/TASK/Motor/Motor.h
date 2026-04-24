#ifndef __MOTORRUN_H__
#define __MOTORRUN_H__

#include "public.h"

#define REDUCTION_RATIO 57

void Motor_Run(void);
void motor_run_init(void);

extern float motor620_set;

void Motor_620_Speed(int32_t motor620_setbig,int32_t motor620_setsmall);

void motor620_run(void);
void Motor_620_Angle(int32_t angle);
void Motor_6020_Angle(int32_t angle);
void pid_clear(void);
#endif
