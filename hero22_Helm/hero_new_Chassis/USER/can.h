#include "stm32f4xx_conf.h"
#ifndef __CAN_H__
#define __CAN_H__

#define     CAN1_Data_Receive_Progress      do{Can1ReceiveMsgProcess(&rx_message);}while(0);
#define     CAN2_Data_Receive_Progress      do{Can2ReceiveMsgProcess(&rx_message);}while(0);

void Can1_Init(uint8_t ts1,uint8_t ts2,uint16_t brp,uint8_t mode);
void Can2_Init(uint8_t ts1,uint8_t ts2,uint16_t brp,uint8_t mode);

#endif
