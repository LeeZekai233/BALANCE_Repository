#ifndef __CAN1_H
#define __CAN1_H
#include <stm32f4xx.h>

void CAN1_Init(void);
void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);


#endif

