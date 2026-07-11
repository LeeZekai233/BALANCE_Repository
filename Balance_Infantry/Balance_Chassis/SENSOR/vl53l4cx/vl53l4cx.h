#ifndef __VL53L4CX_H
#define __VL53L4CX_H

#include "stm32f4xx.h"                  // Device header


typedef struct
{
	uint16_t Distance;//距离
    uint32_t Heart_cnt;//
    uint8_t Online_flag;//在线标志位
}vl53l4cx_t;



uint8_t vl53l4cx_Online_Handle(vl53l4cx_t* vl53l4cx,uint32_t time_tick);
uint16_t vl53l4cx_Data_Get(CanRxMsg* RxMsg,vl53l4cx_t* vl53l4cx,uint32_t time_tick);

#endif
