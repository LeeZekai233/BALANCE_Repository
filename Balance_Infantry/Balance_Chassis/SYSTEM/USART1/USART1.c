#include "main.h"

/*****************************************USART1初始化**************************************************/
static uint8_t _USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN]; // 单缓冲区定义

void USART1_Init(uint32_t baud_rate) // 串口1 DMA2 channel 4接收  PB7
{
    GPIO_InitTypeDef GPIO_U1;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;

    // 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    
    // 配置PB7为USART1_RX（关键修正：必须设为输入模式！）
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // 复用功能映射

    GPIO_StructInit(&GPIO_U1);
    GPIO_U1.GPIO_Mode = GPIO_Mode_AF;
    GPIO_U1.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_U1.GPIO_OType = GPIO_OType_PP;
    GPIO_U1.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_U1);
    
    // 配置USART1
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Rx;  // 仅接收模式
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
    
    // 使能USART1的DMA接收请求
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    // 配置DMA
    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)_USART1_DMA_RX_BUF;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = BSP_USART1_DMA_RX_BUF_LEN;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;      // 普通模式（非循环）
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);

    // 配置USART1中断
    nvic.NVIC_IRQChannel = USART1_IRQn;                          
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;			
    NVIC_Init(&nvic);	

    // 使能空闲线检测（用于帧结束判断）
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}




void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        // ★ 关键步骤：先读SR再读DR清除IDLE标志
        (void)USART1->SR;
        (void)USART1->DR;
        
        // 暂停DMA传输以便安全读取数据
        DMA_Cmd(DMA2_Stream2, DISABLE);
        
        // 计算接收到的数据长度
        uint32_t this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
        
       
        if (_USART1_DMA_RX_BUF[0] == 0x59 && _USART1_DMA_RX_BUF[1] == 0x59)
        {
            // 正确解析16位距离值（注意高低字节顺序）
            Chassis.TF02.Distance = (uint16_t)(_USART1_DMA_RX_BUF[3] << 8) | _USART1_DMA_RX_BUF[2];
            Chassis.TF02.Heart_cnt = time_tick;
        }
        else
        {
            Chassis.TF02.Distance = 0;  // 无效帧
     
        // 重置DMA计数器并重新使能
        DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
        DMA_Cmd(DMA2_Stream2, ENABLE);
        }
    }
}
