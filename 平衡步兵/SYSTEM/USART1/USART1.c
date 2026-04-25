#include <main.h>
void Remote_DT7_To_USART_Chassis_Data(Remote_DT7_t* ,USART_Chassis_Data_t* );//不加这个报警告，原因未知

/*****************************************USART1初始化**************************************************/
static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];
void USART1_Init(uint32_t baud_rate)//串口1 DMA2 channel 4 8字节 	PB7
{
    GPIO_InitTypeDef GPIO_U1;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);//PB7

    GPIO_StructInit(&GPIO_U1);
    GPIO_U1.GPIO_Pin = GPIO_Pin_7;
    GPIO_U1.GPIO_Mode = GPIO_Mode_AF;
    GPIO_U1.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_U1.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_U1);
    
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
    
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART1_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(_USART1_DMA_RX_BUF)/2;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
    
    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
    DMA_Cmd(DMA2_Stream2, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;                          
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;			
    NVIC_Init(&nvic);	

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}




void USART1_IRQHandler(void)
{
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART1->SR;
		(void)USART1->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			//DMA失能
			DMA_Cmd(DMA2_Stream2, DISABLE);
			//清除相应标志位（普通模式必须）
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
			//计算包长度
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			//设置缓冲区长度
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			//切换缓冲区
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			//DMA使能
			DMA_Cmd(DMA2_Stream2, ENABLE);
			//包长度正确进行数据处理
            if(this_time_rx_len == RC_DATA_FRAME_LENGTH)
			{
                 DT7_Remote_Data_Dispose(_USART1_DMA_RX_BUF[0],&Remote_DT7_data);
                 Key_Mouse_State_Update(&Remote_DT7_data);
            
               //  Remote_DT7_To_USART_Chassis_Data(&Remote_DT7_data,&Chassis.USART_Chassis_Data);
			}
		}
		else
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
           if(this_time_rx_len == RC_DATA_FRAME_LENGTH)
           {
                DT7_Remote_Data_Dispose(_USART1_DMA_RX_BUF[1],&Remote_DT7_data);
                Key_Mouse_State_Update(&Remote_DT7_data);
            
           //     Remote_DT7_To_USART_Chassis_Data(&Remote_DT7_data,&Chassis.USART_Chassis_Data);
           }
	    }
	}
}


