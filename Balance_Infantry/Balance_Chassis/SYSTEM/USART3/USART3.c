#include "main.h"


uint8_t CH040_Rx_Buffer[CH040_RX_BUFF_SIZE];
imu_data_t imu_data;
CH040DATA_t CH040DATA;
/********************************USART3初始化*****************************************/
void USART3_Init(uint32_t baud_rate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

    /* Enable the USART OverSampling by 8 */
    USART_InitStructure.USART_BaudRate = baud_rate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

	/*使能空闲帧中断*/
   USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
   USART_ClearFlag(USART3,USART_FLAG_TC|USART_FLAG_IDLE);
    /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 

    DMA_InitStructure.DMA_BufferSize = CH040_RX_BUFF_SIZE ;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR));
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* Configure RX DMA */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)CH040_Rx_Buffer ; 
    DMA_Init(DMA1_Stream1,&DMA_InitStructure);
    /* Enable DMA USART RX Stream */
    DMA_Cmd(DMA1_Stream1,ENABLE);
    /* Enable USART DMA RX Requsts */
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    /* Enable USART */
    USART_Cmd(USART3, ENABLE);
}




/****************************************CH040陀螺仪数据解析函数**************************************************/
float Last_Yaw_Angle = 0 , Diff = 0;
int16_t Yaw_Circle_Count=0;
void CH040_Data_Get(imu_data_t* imu_data , CH040DATA_t* CH040DATA)
{
    CH040DATA->Pitch_Angle = imu_data->eul[1];
    CH040DATA->Roll_Angle = imu_data->eul[0];
    CH040DATA->Yaw_Angle = imu_data->eul[2];
    
    
    CH040DATA->X_Acc = imu_data->acc[0];
    CH040DATA->Y_Acc = imu_data->acc[1];
    CH040DATA->Z_Acc = imu_data->acc[2];
    
    
    CH040DATA->Pitch_Gyro_Omega = imu_data->gyr[0];
    CH040DATA->Roll_Gyro_Omega = imu_data->gyr[1];
    CH040DATA->Yaw_Gyro_Omega =  -imu_data->gyr[2];
    
    Diff=CH040DATA->Yaw_Angle - Last_Yaw_Angle;
    if(Diff<-180.0f) Yaw_Circle_Count++;
    else if(Diff>180.0f) Yaw_Circle_Count--;
    
    CH040DATA->Yaw_Multi_Angle = CH040DATA->Yaw_Angle + Yaw_Circle_Count * 360.0f;
    Last_Yaw_Angle = CH040DATA->Yaw_Angle;
}
    



/**********************************************USART3接收中断************************************************/
uint8_t USART3_Data_Length;
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)
	{
	  USART_ReceiveData(USART3);	
      USART_ClearITPendingBit(USART3,USART_IT_IDLE);
	  DMA_Cmd(DMA1_Stream1,DISABLE);
	  USART_DMACmd(USART3,USART_DMAReq_Rx,DISABLE);
      USART3_Data_Length=CH040_RX_BUFF_SIZE-DMA_GetCurrDataCounter(DMA1_Stream1);
      if(USART3_Data_Length==CH040_DATA_FARMER_LENGHT)
      {
         memcpy(&imu_data,&CH040_Rx_Buffer[CH040_FRAMER_HEADER_LENGHT],sizeof(imu_data_t));
         CH040_Data_Get(&imu_data , &Chassis.Chassis_GYRO);
      }
      DMA_SetCurrDataCounter(DMA1_Stream1,CH040_RX_BUFF_SIZE);
      USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
      DMA_Cmd(DMA1_Stream1,ENABLE);
    }
}


