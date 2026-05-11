#include "main.h"

uint8_t _UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];	
uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

void USART4_Init(u32 bound)
{
    USART_InitTypeDef uart4;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC,&gpio);

    uart4.USART_BaudRate = bound;          // speed 10byte/ms
    uart4.USART_WordLength = USART_WordLength_8b;
    uart4.USART_StopBits = USART_StopBits_1;
    uart4.USART_Parity = USART_Parity_No;
    uart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4,&uart4);

    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);


    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr		= (uint32_t)(&UART4->DR);
    dma.DMA_Memory0BaseAddr   		= (uint32_t)&_UART4_DMA_RX_BUF;
    dma.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize			 	= UART4_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
    dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 					= DMA_Mode_Normal;
    dma.DMA_Priority 				= DMA_Priority_Medium;
    dma.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &dma);
    DMA_Cmd(DMA1_Stream2, ENABLE);

    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    DMA_Cmd(DMA1_Stream4, DISABLE);                           // 关DMA通道
    DMA_DeInit(DMA1_Stream4);
    while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	= (uint32_t)(&UART4->DR);
    dma.DMA_Memory0BaseAddr   	= (uint32_t)&UART4_DMA_TX_BUF[0];
    dma.DMA_DIR 			   				 = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize					= sizeof(UART4_DMA_TX_BUF);
    dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 								= DMA_Mode_Normal;
    dma.DMA_Priority 						= DMA_Priority_Medium;
    dma.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4,&dma);

    DMA_Cmd(DMA1_Stream4, DISABLE);                           // 关DMA通道
    nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // 发送DMA通道的中断配置
    nvic.NVIC_IRQChannelPreemptionPriority = 1;     // 优先级设置
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

    USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
    USART_Cmd(UART4,ENABLE);

}

void Uart4DmaSendDataProc(u16 ndtr)
{
    DMA_Cmd(DMA1_Stream4, DISABLE);                      //关闭DMA传输
    DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}  //确保DMA可以被设置
    DMA_SetCurrDataCounter(DMA1_Stream4,ndtr);          //数据传输量
    DMA_Cmd(DMA1_Stream4, ENABLE);                      //开启DMA传输
}


void UART4_IRQHandler(void)
{
	u16 static length=0;
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //接收中断
    {
      (void)UART4->SR;
      (void)UART4->DR;
      DMA_Cmd(DMA1_Stream2, DISABLE);
      DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
      length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);
      DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
      DMA_Cmd(DMA1_Stream2, ENABLE);
      if(Verify_CRC8_Check_Sum(_UART4_DMA_RX_BUF,length))
      {
          usart_chassis_receive(_UART4_DMA_RX_BUF,&Chassis.USART_Chassis_Data);
      }
      memset(_UART4_DMA_RX_BUF,0,100);
      DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
      DMA_Cmd(DMA1_Stream2, ENABLE);
    }
}

void DMA1_Stream4_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam3传输完成
    {
      DMA_Cmd(DMA1_Stream4, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//清除DMA1_Steam3传输完成标志
    }
}
