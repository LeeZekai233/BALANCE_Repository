#ifndef __USART_O_H
#define __USART_O_H
#define USART3_DMA_RX_BUF_LEN 64
#define USART3_DMA_TX_BUF_LEN 64
#define ST_LENGTH 18
void usart_st_Init(void);
void RemoteStData(uint8_t *pData,uint8_t *tData);
extern uint8_t USART3_DMA_RX_BUF[2][USART3_DMA_RX_BUF_LEN];
extern uint8_t USART3_DMA_TX_BUF[2][USART3_DMA_RX_BUF_LEN];
extern uint16_t count_time;
extern uint8_t count_start;
#endif
