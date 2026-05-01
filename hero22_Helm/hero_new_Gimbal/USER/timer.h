#ifndef __TIMER_H__
#define __TIMER_H__

void Tim6_Init(void);
void TIM2_Init(void);
void TIM5_PWM_Init(void);
#define     TIM6_IRQProcess     do{control_task();}while(0);


#endif
