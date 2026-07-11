#include "main.h"



uint8_t _UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];	
uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];



USART_InitTypeDef uart4;
GPIO_InitTypeDef gpio;
NVIC_InitTypeDef nvic;


/* 
 * 注意：
 * 以下变量、宏和函数保持使用你原工程已有定义：
 *
 * UART4_RX_BUF_LENGTH
 * _UART4_DMA_RX_BUF
 * UART4_DMA_TX_BUF
 * Verify_CRC8_Check_Sum()
 * usart_chassis_receive()
 * Chassis.USART_Chassis_Data
 */


/* UART4 初始化函数
 * 如果你原来的函数名不是 UART4_Init，只保留你原来的函数名，
 * 把函数体替换成下面内容即可。
 */
void UART4_Init(uint32_t bound)
{
    DMA_InitTypeDef dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    uart4.USART_BaudRate = bound;
    uart4.USART_WordLength = USART_WordLength_8b;
    uart4.USART_StopBits = USART_StopBits_1;
    uart4.USART_Parity = USART_Parity_No;
    uart4.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &uart4);

    /******************** UART4 RX DMA：DMA1 Stream2 Channel4 ********************/

    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);

    DMA_Cmd(DMA1_Stream2, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE) {}

    DMA_DeInit(DMA1_Stream2);

    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_UART4_DMA_RX_BUF;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = UART4_RX_BUF_LENGTH;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &dma);

    DMA_ClearFlag(DMA1_Stream2,
                  DMA_FLAG_FEIF2 |
                  DMA_FLAG_DMEIF2 |
                  DMA_FLAG_TEIF2 |
                  DMA_FLAG_HTIF2 |
                  DMA_FLAG_TCIF2);

    DMA_Cmd(DMA1_Stream2, ENABLE);

    /******************** UART4 中断：IDLE 接收空闲中断 ********************/

    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 4;
    nvic.NVIC_IRQChannelSubPriority = 4;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

    /******************** UART4 TX DMA：DMA1 Stream4 Channel4 ********************/

    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    DMA_Cmd(DMA1_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}

    DMA_DeInit(DMA1_Stream4);

    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&UART4_DMA_TX_BUF[0];
    dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize = sizeof(UART4_DMA_TX_BUF);
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &dma);

    DMA_ClearFlag(DMA1_Stream4,
                  DMA_FLAG_FEIF4 |
                  DMA_FLAG_DMEIF4 |
                  DMA_FLAG_TEIF4 |
                  DMA_FLAG_HTIF4 |
                  DMA_FLAG_TCIF4);

    /* 启用你原来已经写好的 DMA1_Stream4_IRQHandler */
    nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

    USART_Cmd(UART4, ENABLE);
}


/* 
 * UART4 DMA 发送函数
 * 发送前把有效数据放入 UART4_DMA_TX_BUF，
 * 然后调用 UART4_DMA_Send(len)。
 *
 * 关键修复：
 * 不能直接发送 sizeof(UART4_DMA_TX_BUF)，
 * 必须发送实际有效长度 len。
 */
void UART4_DMA_Send(uint16_t len)
{
    if (len == 0)
    {
        return;
    }

    if (len > sizeof(UART4_DMA_TX_BUF))
    {
        len = sizeof(UART4_DMA_TX_BUF);
    }

    DMA_Cmd(DMA1_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}

    DMA_ClearFlag(DMA1_Stream4,
                  DMA_FLAG_FEIF4 |
                  DMA_FLAG_DMEIF4 |
                  DMA_FLAG_TEIF4 |
                  DMA_FLAG_HTIF4 |
                  DMA_FLAG_TCIF4);

    DMA1_Stream4->M0AR = (uint32_t)&UART4_DMA_TX_BUF[0];
    DMA_SetCurrDataCounter(DMA1_Stream4, len);

    DMA_Cmd(DMA1_Stream4, ENABLE);
}


uint16_t length = 0;


/* UART4 接收空闲中断 */
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        UART4->SR;
        UART4->DR;
        DMA_Cmd(DMA1_Stream2, DISABLE);
        while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE) {}

        length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);

        DMA_ClearFlag(DMA1_Stream2,
                      DMA_FLAG_FEIF2 |
                      DMA_FLAG_DMEIF2 |
                      DMA_FLAG_TEIF2 |
                      DMA_FLAG_HTIF2 |
                      DMA_FLAG_TCIF2);

        if (length == 25)
        {
            USART_Gimbal_Receive(_UART4_DMA_RX_BUF,&USART_Gimbal_Data);
        }
        memset(_UART4_DMA_RX_BUF, 0, UART4_RX_BUF_LENGTH);
        DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
        DMA_SetCurrDataCounter(DMA1_Stream2, UART4_RX_BUF_LENGTH);
        DMA_Cmd(DMA1_Stream2, ENABLE);
    }
}


/* UART4 TX DMA 发送完成中断 */
void DMA1_Stream4_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) != RESET)
    {
        DMA_Cmd(DMA1_Stream4, DISABLE);

        DMA_ClearFlag(DMA1_Stream4,
                      DMA_FLAG_FEIF4 |
                      DMA_FLAG_DMEIF4 |
                      DMA_FLAG_TEIF4 |
                      DMA_FLAG_HTIF4 |
                      DMA_FLAG_TCIF4);
    }
}


