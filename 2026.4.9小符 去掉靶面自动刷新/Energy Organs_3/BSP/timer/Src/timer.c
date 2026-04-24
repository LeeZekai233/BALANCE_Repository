#include "timer.h"

/***********************************************
		Advanced-control timers (TIM1 and TIM8)
************************************************/
#if EN_TIM8
void TIM8_Configuration(void)
{
		TIM_TimeBaseInitTypeDef tim;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);    
		tim.TIM_Period = 0xFFFFFFFF;     
		tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
		tim.TIM_ClockDivision = TIM_CKD_DIV1;	
		tim.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_TimeBaseInit(TIM8, &tim);
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_PrescalerConfig(TIM8, 0, TIM_PSCReloadMode_Update);
		TIM_UpdateDisableConfig(TIM8, ENABLE);
		TIM_Cmd(TIM8,ENABLE);	   
}
#endif

#if EN_TIM1
void TIM1_Configuration(void)
{
		TIM_TimeBaseInitTypeDef tim;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);    
		tim.TIM_Period = 0xFFFFFFFF;     
		tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
		tim.TIM_ClockDivision = TIM_CKD_DIV1;	
		tim.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_TimeBaseInit(TIM8, &tim);
		TIM_ARRPreloadConfig(TIM1, ENABLE);	
		TIM_PrescalerConfig(TIM1, 0, TIM_PSCReloadMode_Update);
		TIM_UpdateDisableConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1,ENABLE);	   
}
#endif

/**********Please write the IRQHandler under the here.**********/
#if EN_TIM1_IRQ

#endif

#if EN_TIM8_IRQ

#endif

/**************************************************
		General-purpose timers (TIMx)
***************************************************/
#if EN_TIM2
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
			
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2,ENABLE);	
}
#endif

#if EN_TIM3
void TIM3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);      

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	
	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.235us
	TIM_TimeBaseStructure.TIM_Prescaler = 8 - 1; // 0.095us
	
//	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.26us
//	TIM_TimeBaseStructure.TIM_Prescaler = 7 - 1; // 0.097us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure); 

//    TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE 主输出使能
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH3预装载使能	 
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
    
    DMA_InitTypeDef DMA_InitStructure;
    
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* 复位初始化 DMA 数据流 */
    DMA_DeInit(DMA1_Stream7); 
    /* 确保 DMA 数据流复位完成 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
	DMA_InitStructure.DMA_BufferSize = WS28_SENDBUFF_SIZE;
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM3->CCR3);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC16;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);

    /* 等待 DMA 数据流有效 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != ENABLE);
    
    DMA_Cmd(DMA1_Stream7, ENABLE);   
    
    
	TIM_DMACmd(TIM3, TIM_DMA_CC3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	
	//结构体初始化
//	TIM_TimeBaseInitTypeDef  TimeBaseStructure;
//  GPIO_InitTypeDef  GPIO_InitStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructuce;                   //PWM输出
//	DMA_InitTypeDef DMA_InitStructure;
//	
//	//开启时钟                                              
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);     //使能时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
//	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);    
//	//设置io端复用                                          
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);   
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);     //端口复用
//	//GPIO初始化                                            
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;    
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;              //复用
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//GPIO_Speed_50MHz例程
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	
//	//设置计时器频率（1000*1680）/84000000	
////	TimeBaseStructure.TIM_Period=1000-1;
////	TimeBaseStructure.TIM_Prescaler=1680-1;
//	TimeBaseStructure.TIM_Period=105-1;
//	TimeBaseStructure.TIM_Prescaler=0;
//	TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
//	TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
//	TIM_TimeBaseInit(TIM3,&TimeBaseStructure);
//	
//	//调节占空比
//	TIM_OCInitStructuce.TIM_OCMode=TIM_OCMode_PWM2;//TIM_OCMode_PWM1例程                 	//脉冲宽度模式
//	TIM_OCInitStructuce.TIM_OutputState=TIM_OutputState_Enable;      //输出状态
//	TIM_OCInitStructuce.TIM_OutputNState=TIM_OutputNState_Disable;   //互补通道的输出状态
//	TIM_OCInitStructuce.TIM_Pulse=0;	                             //占空比
//	TIM_OCInitStructuce.TIM_OCPolarity=TIM_OCPolarity_High;           //输出极性
//	TIM_OCInitStructuce.TIM_OCNPolarity=TIM_OCNPolarity_High;        //互补通道输出极性
//  TIM_OCInitStructuce.TIM_OCIdleState=TIM_OCIdleState_Reset;       //空闲状态
//	TIM_OCInitStructuce.TIM_OCNIdleState=TIM_OCNIdleState_Set;       //互补通道空闲状态
//	//结构体初始化，pre预装载使能
//	TIM_OC3Init(TIM3,&TIM_OCInitStructuce);
//	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
//	TIM_OC4Init(TIM3,&TIM_OCInitStructuce);
//	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
//	//arr预装载使能，计时器使能
//	TIM_ARRPreloadConfig(TIM3,ENABLE);
//	TIM_Cmd(TIM3,ENABLE);
//	
//	DMA_DeInit(DMA1_Stream7);
//  DMA_StructInit(&DMA_InitStructure);//默认

//	DMA_InitStructure.DMA_Channel = DMA_Channel_5;//通道5，流7控制Tim-CH3
//	DMA_InitStructure.DMA_PeripheralBaseAddr = TIM3->CCR1;//外设地址
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer;//存储器地址
//	DMA_InitStructure.DMA_DIR	=	DMA_DIR_MemoryToPeripheral;//存储器到外设
//	DMA_InitStructure.DMA_BufferSize	=	0;	//
//	DMA_InitStructure.DMA_PeripheralInc	=	DMA_PeripheralInc_Disable;//外设地址 一直是tim3-ch3 不自增
//	DMA_InitStructure.DMA_MemoryInc	=	DMA_MemoryInc_Enable;//存储器地址 数组 需要自增
//	DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize	=	DMA_MemoryDataSize_HalfWord;//都是半字传输 16
//	DMA_InitStructure.DMA_Mode	= DMA_Mode_Normal	;
//	DMA_InitStructure.DMA_Priority	=	DMA_Priority_High;

//  DMA_Init(DMA1_Stream7, &DMA_InitStructure);
//	
//	TIM_DMACmd(TIM3,TIM_DMA_Update, ENABLE);	//使能DMA的中断

}
#endif

#if EN_TIM4
void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    tim.TIM_Period = 0xFFFFFFFF; //ARR的值 寄存器周期   
    tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM4, ENABLE);	//允许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_Cmd(TIM4,ENABLE);	
		
		TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}
#endif

#if EN_TIM5
void TIM5_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    nvic.NVIC_IRQChannel = TIM5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    tim.TIM_Period = 0xFFFFFFFF; //ARR的值 寄存器周期   
    tim.TIM_Prescaler = 168-1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM5, ENABLE);	//允许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM5, &tim);
    TIM_Cmd(TIM5,ENABLE);	
		
		TIM_ITConfig(TIM5, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
}
#endif

/**********Please write the IRQHandler under the here.**********/
#if EN_TIM2_IRQ
void TIM4_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
				TIM_ClearFlag(TIM2, TIM_FLAG_Update);
				TIM2_IRQProcess;
		}
}
#endif

#if EN_TIM3_IRQ
void TIM3_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM3_IRQProcess;
		}
//	 if(DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)!=RESET)
//	 {
//		  DMA_Cmd(DMA1_Stream7, DISABLE);                      //关闭DMA传输
//      DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);//清除DMA1_Stream7传输完成标志
//			DMA_SetCurrDataCounter(DMA1_Stream7,buffersize);//设置发送信息最大容量
//		  DMA_Cmd(DMA1_Stream7, ENABLE);//使能dma
//	 }
}
#endif

#if EN_TIM4_IRQ
void TIM4_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
				TIM_ClearFlag(TIM4, TIM_FLAG_Update);
				TIM4_IRQProcess;
		}
}
#endif

#if EN_TIM5_IRQ
void TIM5_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
				TIM_ClearFlag(TIM5, TIM_FLAG_Update);
				TIM5_IRQProcess;
		}
}
#endif

/***************************************************
		Basic timers (TIM6 and TIM7)
****************************************************/
#if EN_TIM6
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    tim.TIM_Period = 1000-1;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
	
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}
#endif

#if EN_TIM7
void TIM7_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    tim.TIM_Period = 1000-1;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM7,&tim);
	
    TIM_Cmd(TIM7, ENABLE);	 
    TIM_ITConfig(TIM7, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);	
}
#endif

/**********Please write the IRQHandler under the here.**********/
#if EN_TIM6_IRQ

void TIM6_DAC_IRQHandler(void)  
{	
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
				TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
				TIM_ClearFlag(TIM6, TIM_FLAG_Update);
				TIM6_IRQProcess;
		}
}
#endif

#if EN_TIM7_IRQ
void TIM7_IRQHandler(void)  
{	
    if (TIM_GetITStatus(TIM7,TIM_IT_Update)!= RESET) 
	  {
				TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
				TIM_ClearFlag(TIM7, TIM_FLAG_Update);
				TIM7_IRQProcess;
		}
}
#endif
