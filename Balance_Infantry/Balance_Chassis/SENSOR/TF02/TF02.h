#ifndef __TF02_H
#define __TF02_H
#include "stm32f4xx.h"                  // Device header



typedef struct
{
	uint8_t Header;//帧头
	uint16_t Distance;//距离
    uint16_t Strength;//信号强度
    float Temperature;//温度
    uint32_t Heart_cnt;
	uint8_t Online_flag;
} TF02_t;



void TF02_Online_Handle(TF02_t* TF02);
void TF02_Data_Handle(uint8_t* Data,TF02_t* TF02);


#endif
