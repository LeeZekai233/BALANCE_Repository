#ifndef __CAN2_H
#define __CAN2_H
#include <stm32f4xx.h>


void CAN2_Init(void);
void CAN2_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);





#endif





