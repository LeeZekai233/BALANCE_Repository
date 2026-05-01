#include "public.h"

uint8_t _USART3_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];//ch100单缓冲接收区
static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];//双缓冲接收区
volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
volatile unsigned char _USART6_RX_BUF[2][VTM_RC_FRAME_LENGTH]; //double sbus rx buffer to save data
uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];
static uint8_t _UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];
uint8_t _USART2_DMA_RX_BUF[BSP_USART2_DMA_RX_BUF_LEN];
uint8_t USART2_DMA_TX_BUF[USART2_TX_BUF_LENGTH];
uint8_t _USART6_DMA_RX_BUF[BSP_USART6_RX_BUF_LENGTH];
uint8_t _USART6_DMA_TX_BUF[BSP_USART6_TX_BUF_LENGTH];

//#if EN_UART5_DMA_SECOND_FIFO == 1	
//uint8_t _UART5_DMA_RX_BUF[2][BSP_UART5_DMA_RX_BUF_LEN];
//#else
//uint8_t _UART5_DMA_RX_BUF[100];
//#endif
static uint8_t UART5_DMA_TX_BUF[UART5_TX_BUF_LENGTH];


#if 0
#pragma import(__use_no_semihosting)  
/**
************************************************************************************************************************
* @Name     : fputc/_sys_exit
* @brief    : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
* @param    : ch
* @param    : FILE *f
* @retval   : void
* @Note     : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
************************************************************************************************************************
**/ 
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2 ->DR = (u8) ch;      
	return ch;
}
#endif










#if EN_USART3
void Usart3_Init(uint32_t Baud_rate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
    
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
    GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
    
    USART_InitStruct.USART_BaudRate=Baud_rate;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_StopBits=USART_StopBits_1;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b;
    USART_Init(USART3,&USART_InitStruct);
    
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
    USART_ClearFlag(USART3,USART_IT_IDLE | USART_FLAG_TC);
    
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART3,ENABLE);
    
    DMA_InitStruct.DMA_BufferSize=BSP_USART3_DMA_RX_BUF_LEN;
    DMA_InitStruct.DMA_Channel=DMA_Channel_4;
    DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_FIFOMode=DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)_USART3_RX_BUF;
    DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;
    DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)(&(USART3->DR));
    DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
    DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority=DMA_Priority_High;
    DMA_Init(DMA1_Stream1,&DMA_InitStruct);
    
    DMA_Cmd(DMA1_Stream1,ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel=USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
    NVIC_Init(&NVIC_InitStruct);
    
}
#endif

void USART3_IRQHandler()
{
    if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE)!=RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_IDLE);
        
        USART_ReceiveData(USART3);
        
        USART_Cmd(USART3,DISABLE);
        DMA_Cmd(DMA1_Stream1,DISABLE);
        
        USART3_Data_Receive_Process
        Equipment_Counter_Make_Zero(&Peripheral_State.Gimbal_Yaw_Gyro);
        USART_Cmd(USART3,ENABLE);
        DMA_Cmd(DMA1_Stream1,ENABLE);
        
    }
}


//void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
//{
// 
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
//	
//	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
//		
//	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
// 
//	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
//}	  

#if EN_USART1

void Usart1_Init(uint32_t Baud_rate)
{
    
    GPIO_InitTypeDef GPIO_U1;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
    GPIO_StructInit(&GPIO_U1);
    GPIO_U1.GPIO_Pin = GPIO_Pin_7;
    GPIO_U1.GPIO_Mode = GPIO_Mode_AF;
    GPIO_U1.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_U1.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_U1);
    
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = 100000;
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
    dma.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(sbus_rx_buffer)/2;
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
    
    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&sbus_rx_buffer[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
    DMA_Cmd(DMA2_Stream2, ENABLE);
    
    nvic.NVIC_IRQChannel = USART1_IRQn;                          
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&nvic);    

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}
#endif

char Serial_RxPacket[100];
uint8_t Serial_RxFlag = 0;

//串口接收中断服务函数
void USART1_IRQHandler(void)
{
	static uint32_t this_time_rx_len1 = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART1->SR;
		(void)USART1->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
			this_time_rx_len1 = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream2, ENABLE);
			Equipment_Counter_Make_Zero(&Peripheral_State.Remote_Control);
					USART1_Data_Receive_Process_0

		}
		
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
			this_time_rx_len1 = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
			Equipment_Counter_Make_Zero(&Peripheral_State.Remote_Control);
					USART1_Data_Receive_Process_1
		}
	}       


}

#if EN_USART6
void usart6_init()
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef dma;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
	
		//串口1对应引脚复用映射
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6为USART1
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7复用为USART1
		
		//USART1端口配置
		gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6与GPIOC7
		gpio.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		gpio.GPIO_PuPd 	= GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOC,&gpio); //初始化PC6，PC7

    USART_DeInit(USART6);
//    USART_StructInit(&usart);
    usart.USART_BaudRate = 921600;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);   


		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    
		DMA_DeInit(DMA2_Stream1);
    dma.DMA_Channel = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr   	= (uint32_t)&_USART6_DMA_RX_BUF[0];
    dma.DMA_DIR 			    = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize			= BSP_USART6_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
    dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 				= DMA_Mode_Normal;
    dma.DMA_Priority 			= DMA_Priority_Medium;
    dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &dma);
    DMA_Cmd(DMA2_Stream1, ENABLE);
		nvic.NVIC_IRQChannel = USART6_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 2;   //pre-emption priority 
		nvic.NVIC_IRQChannelSubPriority = 2;		    //subpriority 
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled


	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE) {}
	dma.DMA_Channel = DMA_Channel_5;
	dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
	dma.DMA_Memory0BaseAddr   	= (uint32_t)&_USART6_DMA_TX_BUF[0];
	dma.DMA_DIR 			   				 = DMA_DIR_MemoryToPeripheral;
	dma.DMA_BufferSize					= 0;//sizeof(UART4_DMA_TX_BUF);
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
	DMA_Init(DMA2_Stream6,&dma);

	DMA_Cmd(DMA2_Stream6, DISABLE);                           // 关DMA通道
	nvic.NVIC_IRQChannel = DMA2_Stream6_IRQn;   // 发送DMA通道的中断配置
	nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);

	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
		
		
    USART_Cmd(USART6,ENABLE);
}

//串口接收中断服务函数
void USART6_IRQHandler(void)

{
	static uint32_t this_time_rx_len6 = 0;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		(void)USART6->SR;
		(void)USART6->DR;
		DMA_Cmd(DMA2_Stream1, DISABLE); 
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);  //************************************
		this_time_rx_len6 = BSP_USART6_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1);

		USART6_Data_Receive_Process
		Equipment_Counter_Make_Zero(&Peripheral_State.VTM_Remote);
		DMA_SetCurrDataCounter(DMA2_Stream1,BSP_USART6_RX_BUF_LENGTH);
		DMA_Cmd(DMA2_Stream1, ENABLE);
	}       
}

void DMA2_Stream6_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA1_Steam3传输完成
    {
      DMA_Cmd(DMA2_Stream6, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);//清除DMA1_Steam3传输完成标志
    }
}

void Uart6SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = _USART6_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA发送方式

  Uart6DmaSendDataProc(DMA2_Stream6,nSendCount); //开始一次DMA传输！
}

 void Uart6DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
    DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);
    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}


#endif


#if EN_UART4
	/*-----UART4_TX-----PC10-----*/
/*-----UART4_RX-----PC11-----*/
	void uart4_init(u32 bound)
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
  nvic.NVIC_IRQChannelPreemptionPriority =3;
  nvic.NVIC_IRQChannelSubPriority =3;
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
  dma.DMA_BufferSize					= 0;//sizeof(UART4_DMA_TX_BUF);
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
  nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
  nvic.NVIC_IRQChannelSubPriority = 2;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

  USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
  USART_Cmd(UART4,ENABLE);

}

uint8_t length=0;
uint8_t RxData;
u8 Usar4_Link_State=0;

void UART4_IRQHandler(void)
{
	u16 static length=0;
  if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //接收中断
    {
			
				//Equipment_Counter_Make_Zero(&Peripheral_State.Equipment_Visual_Equipment_Auto_Aim);
			
			
      (void)UART4->SR;
      (void)UART4->DR;
      DMA_Cmd(DMA1_Stream2, DISABLE);
      DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
      length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);
      DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
			//视觉数据处理,数据存在发送给下板的自瞄数据结构体内
			UART4_Data_Receive_Process
			Equipment_Counter_Make_Zero(&Peripheral_State.Equipment_Visual_Equipment_Auto_Aim);
			
      DMA_Cmd(DMA1_Stream2, ENABLE);
			if(length==82)
			{
//				memcpy(&dat_3, &_UART4_DMA_RX_BUF[6], sizeof(id0x91_t));
			}
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





#endif

void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = UART4_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA发送方式

  Uart4DmaSendDataProc(DMA1_Stream4,nSendCount); //开始一次DMA传输！
}

 void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
    DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);
    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}



#if EN_USART2
void Usart2_Init(uint32_t baud_rate)
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
    
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    
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
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &dma);
    
    DMA_Cmd(DMA1_Stream5, ENABLE);
    
		nvic.NVIC_IRQChannel = USART2_IRQn;                          
		nvic.NVIC_IRQChannelPreemptionPriority = 1;   //pre-emption priority 
		nvic.NVIC_IRQChannelSubPriority = 1;		    //subpriority 
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	
		
		
		
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

  DMA_Cmd(DMA1_Stream6, DISABLE);                           // 关DMA通道
  DMA_DeInit(DMA1_Stream6);
  while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART2->DR);
  dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART2_DMA_TX_BUF[0];
  dma.DMA_DIR 			   				 = DMA_DIR_MemoryToPeripheral;
  dma.DMA_BufferSize					= 0;//sizeof(USART1_DMA_TX_BUF);
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
  nvic.NVIC_IRQChannelPreemptionPriority = 1;     // 优先级设置
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);


		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
		USART_Cmd(USART2, ENABLE);

}
float Pitch_Angle_Last;

uint8_t length_2=0;
//串口接收中断服务函数
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_IDLE)!= RESET)//
	{
		USART_ReceiveData(USART2); //一定要读一次，不然可能会丢第一个字节，原因未知
		USART_ClearITPendingBit(USART2,USART_IT_IDLE);//清除中断标志位
		DMA_Cmd(DMA1_Stream5,DISABLE);  
		USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);
		Pitch_Angle_Last=My_Auto_Snipe.Auto_Aim.Pitch_Angle;
		USART2_Data_Receive_Process_0
//		if(My_Auto_Snipe.Auto_Aim.Pitch_Angle!=Pitch_Angle_Last)
//		{
//			Send_Radar(gimbal_gyro.yaw_angle , 
//                      gimbal_gyro.pitch_angle , 
//                      gimbal_gyro.roll_angle , 
//                      judge_rece_mesg.game_robot_state.robot_id , 
//                      judge_rece_mesg.shoot_data.initial_speed, gimbal_data.ctrl_mode , USART2_DMA_TX_BUF); 
//		}
		
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(DMA1_Stream5,ENABLE);//重新置位后，地址指针变成0
	}



		
}       

void DMA1_Stream6_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA1_Steam6传输完成,DMA_FLAG_TCIF4为DMA通道4传输完成标志位
    {
      DMA_Cmd(DMA1_Stream6, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA1_Steam3传输完成标志
    }
}


void Uart2DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}



//发送单字节
void Uart2SendByteInfoProc(u8 nSendInfo)
{
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = USART2_DMA_TX_BUF;
  *pBuf++ = nSendInfo;

  Uart2DmaSendDataProc(DMA1_Stream6,1); //开始一次DMA传输！

}

//发送多字节
void Uart2SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = USART2_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA发送方式

  Uart2DmaSendDataProc(DMA1_Stream6,nSendCount); //开始一次DMA传输！
}



void USART2_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); // 等待上一次传输完成
    USART_SendData(USART2, byte); // 发送数据
}



void USART2_SendBuffer(uint8_t *buffer, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        USART2_SendByte(buffer[i]); // 发送当前字节
    }
}

#endif
