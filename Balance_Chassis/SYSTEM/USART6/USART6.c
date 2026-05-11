#include "main.h"


uint8_t _USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
uint8_t USART6_Tx_Buf[USART6_TX_BUF_LENGTH];


/*******************************USART6初始化函数********************************/
void USART6_Init(uint32_t baud_rate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    
    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // USART6在GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // USART6挂载在APB2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   // USART6的DMA在DMA2
    
    // 2. 配置GPIO
    // USART6_TX: PC6, USART6_RX: PC7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 配置GPIO复用功能
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    
    // 3. 配置USART6
    USART_InitStructure.USART_BaudRate = baud_rate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART6, &USART_InitStructure);
    
    // 4. 配置DMA接收（USART6_RX使用DMA2 Stream1 Channel5）
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    DMA_DeInit(DMA2_Stream1);
    DMA_StructInit(&DMA_InitStructure);
    
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_USART6_DMA_RX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = BSP_USART6_DMA_RX_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream1, ENABLE);
    
    // 5. 配置DMA发送（USART6_TX使用DMA2 Stream6 Channel5）
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
    DMA_DeInit(DMA2_Stream6);
    
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART6_Tx_Buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = USART6_TX_BUF_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream6, DISABLE); // 默认禁用，发送时启用
    
    // 6. 配置USART6中断（空闲中断）
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
    
    // 7. 使能USART6
    USART_Cmd(USART6, ENABLE);
}
uint8_t temp_flagaaa;
// USART6中断处理函数
uint32_t USART6_Data_Length = 0;
void USART6_IRQHandler(void)
{
    temp_flagaaa++;
    
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
       (void)USART6->SR;
		(void)USART6->DR;
        // 停止DMA接收
        DMA_Cmd(DMA2_Stream1, DISABLE);
        DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
        // 计算接收到的数据长度
        USART6_Data_Length = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
        
        // 重新配置DMA接收
        DMA_SetCurrDataCounter(DMA2_Stream1, BSP_USART6_DMA_RX_BUF_LEN);
       
        
        // 处理接收到的数据（这里调用你的数据处理函数）
        if(USART6_Data_Length > (HEADER_LEN + CMD_LEN + CRC_LEN))
        {
            judgement_data_handle(_USART6_DMA_RX_BUF, USART6_Data_Length);
        }
         DMA_Cmd(DMA2_Stream1, ENABLE);
    }
}


// USART6 DMA发送使能函数
void USART6_DMA_Tx_Enable(uint16_t data_len)
{
    // 确保数据长度不超过缓冲区大小
    if(data_len > USART6_TX_BUF_LENGTH)
    {
        data_len = USART6_TX_BUF_LENGTH;
    }
    
    // 停止DMA
    DMA_Cmd(DMA2_Stream6, DISABLE);
    
    // 等待DMA禁用
    while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);
    
    // 清除传输完成标志
    DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
    
    // 设置传输数据量
    DMA_SetCurrDataCounter(DMA2_Stream6, data_len);
    
    // 使能DMA
    DMA_Cmd(DMA2_Stream6, ENABLE);
}


// 通过USART6发送数据的函数
void USART6_Send_Data(uint8_t *data, uint16_t len)
{
    // 将数据复制到发送缓冲区
    if(len > USART6_TX_BUF_LENGTH)
    {
        len = USART6_TX_BUF_LENGTH;
    }
    
    memcpy(USART6_Tx_Buf, data, len);
    
    // 启用DMA发送
    USART6_DMA_Tx_Enable(len);
}

