#ifndef __USART2_H
#define __USART2_H
#include <stm32f4xx.h>

#define USART2_TX_BUF_LENGTH       100
#define USART2_RX_BUF_LENGTH       100


void usart2_init(uint32_t baud_rate);


#endif
