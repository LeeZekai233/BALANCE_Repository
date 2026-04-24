#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H
#include "public.h"

typedef struct {
  uint8_t mode;  // 0为未击打，1为待击打，2为完成
} leaf;

// void clean_energe_leaf_flag(void);
void rand_energe_leaf(void);
void control_task(void);
void led_display_set();
void clean_time(void);
void control_task_Init(void);

extern int time_tick;
extern int all_finish_time[5];
extern int led_time1;
extern int last_led_time1;
extern int energe_mode;
extern int energe_leaf;
extern long long time_Big_energe;
extern leaf LED[5];
extern uint8_t n;
#endif
