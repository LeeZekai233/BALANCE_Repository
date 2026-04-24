#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H
#include "public.h"






void rand_big_energe(void);
void control_task(void);
void clean_time(void);
void control_task_Init(void);
//void energe_turn_input(void);


extern int time_tick;
extern int led_time1 ;
extern int last_led_time1 ;
extern int led_time2 ;
extern int last_led_time2 ;
extern int energe_mode ;
extern long long time_Big_energe;
extern long long time_Small_energe;

#endif

