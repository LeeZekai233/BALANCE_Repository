#ifndef __TF02_H
#define __TF02_H
#include "stm32f4xx.h"                  // Device header

typedef struct
{
	uint8_t Header;
	uint16_t Distance_mm;
    uint32_t Heart_cnt;
	uint8_t Online_flag;
} TF02_t;


void TF02_Online_Handle(void);





#endif
