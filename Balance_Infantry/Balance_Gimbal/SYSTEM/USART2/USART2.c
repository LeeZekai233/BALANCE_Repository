#include "main.h"

uint8_t USART2_DMA_TX_BUF[USART2_TX_BUF_LENGTH];
uint8_t _USART2_DMA_RX_BUF[USART2_RX_BUF_LENGTH];

void usart2_init(uint32_t baud_rate)
{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		NVIC_InitTypeDef nvic;
		DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
    
    USART_DeInit(USART2);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usart);
    

    
    DMA_DeInit(DMA1_Stream5);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART2_DMA_RX_BUF[0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(_USART2_DMA_RX_BUF);
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
    DMA_Init(DMA1_Stream5, &dma);
    
    
    
		nvic.NVIC_IRQChannel = USART2_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 3;   //pre-emption priority 
		nvic.NVIC_IRQChannelSubPriority = 3;		    //subpriority 
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
		
		
		


  DMA_Cmd(DMA1_Stream6, DISABLE);                           // 关DMA通道
  DMA_DeInit(DMA1_Stream6);
  while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART2->DR);
  dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART2_DMA_TX_BUF[0];
  dma.DMA_DIR 			   				 = DMA_DIR_MemoryToPeripheral;
  dma.DMA_BufferSize					= sizeof(USART2_DMA_TX_BUF);
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
  DMA_Init(DMA1_Stream6,&dma);

//	DMA_Cmd(DMA1_Stream6, ENABLE);                           // 关DMA通道
  nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // 发送DMA通道的中断配置
  nvic.NVIC_IRQChannelPreemptionPriority = 0;     // 优先级设置
  nvic.NVIC_IRQChannelSubPriority = 0;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
  
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
		USART_Cmd(USART2, ENABLE);
        USART_ClearFlag(USART2, USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE); 
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(DMA1_Stream5, ENABLE);

}



void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE)!= RESET)
	{
		USART_ReceiveData(USART2); //一定要读一次，不然可能会丢第一个字节，原因未知
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);//清除中断标志位
		DMA_Cmd(DMA1_Stream5,DISABLE);  
		USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);
        uint8_t length = USART2_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream5);
		Vision_Process_General_Message_New(_USART2_DMA_RX_BUF,length,&My_Auto_Shoot);
            
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
        DMA_SetCurrDataCounter(DMA1_Stream5,USART2_RX_BUF_LENGTH);
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(DMA1_Stream5,ENABLE);//重新置位后，地址指针变成0
	}
}


void DMA1_Stream6_IRQHandler(void)
{
    // 检查是否是传输完成中断
    if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
    {
        // 1. 清除中断标志位
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

    }
}


