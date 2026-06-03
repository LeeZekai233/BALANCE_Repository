#ifndef __LOG_TASK_H
#define __LOG_TASK_H
#include <stm32f4xx.h>

void Log_Task(USART_TypeDef* USARTx,uint8_t* USART_BUF,float Data_1,float Data_2,float Data_3,float Data_4,float Data_5,float Data_6,uint32_t time_tick);


#endif
