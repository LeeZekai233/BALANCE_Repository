#ifndef __USART_4_H__
#define __USART_4_H__

#include <stm32f4xx.h>

#define UART4_RX_BUF_LENGTH       200
#define UART4_TX_BUF_LENGTH       200

extern uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

void USART4_Init(u32 bound);
void Uart4DmaSendDataProc(u16 ndtr);
void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);

#endif /*_USART_4_H_*/
