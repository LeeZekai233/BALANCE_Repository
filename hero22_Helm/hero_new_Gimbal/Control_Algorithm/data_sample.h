#ifndef __DATA_SAMPLE_H__
#define __DATA_SAMPLE_H__
#include <public.h>



void sample_task(int frequency);
void sample_control_task(void);
void sample_task_Init(void);


extern pid_t yaw_gimbal_speed_pid;
extern pid_t yaw_gimbal_angle_pid;
extern float sys_input;
extern int16_t disterb;


#endif

