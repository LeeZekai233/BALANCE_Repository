#ifndef __USART6_H
#define __USART6_H
#include <stm32f4xx.h>

// 缓冲区定义
#define BSP_USART6_DMA_RX_BUF_LEN  512
#define USART6_TX_BUF_LENGTH       150

extern uint8_t _USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
extern uint8_t USART6_Tx_Buf[USART6_TX_BUF_LENGTH];

// 函数声明
void USART6_Init(uint32_t baud_rate);
void USART6_DMA_Tx_Enable(uint16_t data_len);
void USART6_Send_Data(uint8_t *data, uint16_t len);

#endif





