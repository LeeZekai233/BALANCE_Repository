#include "main.h"


USART_Gimbal_Data_t USART_Gimbal_Data;
USART_Chassis_Data_t USART_Chassis_Data;


void USART_Gimbal_Receive(uint8_t *DataAddress,USART_Gimbal_Data_t* USART_Gimbal_Data)
{
     memcpy(USART_Gimbal_Data,DataAddress,sizeof(*USART_Gimbal_Data));
}



void USART_Chassis_Send(USART_Chassis_Data_t *data)
{

    memcpy(UART4_DMA_TX_BUF, data, 43);
    DMA_Cmd(DMA1_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}

    DMA_ClearFlag(DMA1_Stream4,
                  DMA_FLAG_FEIF4 |
                  DMA_FLAG_DMEIF4 |
                  DMA_FLAG_TEIF4 |
                  DMA_FLAG_HTIF4 |
                  DMA_FLAG_TCIF4);

    DMA_SetCurrDataCounter(DMA1_Stream4, 43);
    DMA_Cmd(DMA1_Stream4, ENABLE);
}
