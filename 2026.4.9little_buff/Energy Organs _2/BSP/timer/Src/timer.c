#include "timer.h"

/***********************************************
		Advanced-control timers (TIM1 and TIM8)
************************************************/
#if EN_TIM8
/*????*/
void TIM8_Configuration(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);      

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	
	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.235us
	TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1; // 0.095us
	
//	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.26us
//	TIM_TimeBaseStructure.TIM_Prescaler = 7 - 1; // 0.097us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); 
 

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); 	
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
    DMA_InitTypeDef DMA_InitStructure;
    
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* 复位初始化 DMA 数据流 */
    DMA_DeInit(DMA2_Stream2); 
    /* 确保 DMA 数据流复位完成 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
	DMA_InitStructure.DMA_BufferSize = WS28_SENDBUFF_SIZE*2;
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer2;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM8->CCR1);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC16;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* 等待 DMA 数据流有效 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != ENABLE);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);   
    
	TIM_DMACmd(TIM8, TIM_DMA_CC1, ENABLE);

	TIM_Cmd(TIM8, ENABLE);  
}

#endif

#if EN_TIM1
void TIM1_Configuration(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);      

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	
	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.235us
	TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1; // 0.095us
	
//	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.26us
//	TIM_TimeBaseStructure.TIM_Prescaler = 7 - 1; // 0.097us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); 
 

	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
    DMA_InitTypeDef DMA_InitStructure;
    
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* 复位初始化 DMA 数据流 */
    DMA_DeInit(DMA2_Stream2); 
    /* 确保 DMA 数据流复位完成 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
	DMA_InitStructure.DMA_BufferSize = WS28_SENDBUFF_SIZE;
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer3;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM1->CCR2);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC16;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* 等待 DMA 数据流有效 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != ENABLE);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);   
    
	TIM_DMACmd(TIM1, TIM_DMA_CC2, ENABLE);

	TIM_Cmd(TIM1, ENABLE);  //??TIM8

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
     DMA_InitTypeDef DMA_InitStructure;

void TIM2_Configuration(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);      

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
	
	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.235us
	TIM_TimeBaseStructure.TIM_Prescaler = 8 - 1; // 0.095us
	
//	TIM_TimeBaseStructure.TIM_Period = 13 - 1; // 1.26us
//	TIM_TimeBaseStructure.TIM_Prescaler = 7 - 1; // 0.097us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 

//    TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE 主输出使能
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH3预装载使能	 
	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIMx在ARR上的预装载寄存器
	

    
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* 复位初始化 DMA 数据流 */
    DMA_DeInit(DMA1_Stream6); 
    /* 确保 DMA 数据流复位完成 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
	DMA_InitStructure.DMA_BufferSize = WS28_SENDBUFF_SIZE;
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer2;//LED_BYTE_Buffer1;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM2->CCR2);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC16;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);

    /* 等待 DMA 数据流有效 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != ENABLE);
    
    DMA_Cmd(DMA1_Stream6, ENABLE);   
    
    
	TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}
#endif

#if EN_TIM3
void TIM3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);      

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	
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
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); 

//    TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE 主输出使能
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH3预装载使能	 
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
    
    DMA_InitTypeDef DMA_InitStructure;
    
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* 复位初始化 DMA 数据流 */
    DMA_DeInit(DMA1_Stream5); 
    /* 确保 DMA 数据流复位完成 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
	DMA_InitStructure.DMA_BufferSize = WS28_SENDBUFF_SIZE;
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer2;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM3->CCR2);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC16;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

    /* 等待 DMA 数据流有效 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != ENABLE);
    
    DMA_Cmd(DMA1_Stream5, ENABLE);   
    
    
	TIM_DMACmd(TIM3, TIM_DMA_CC2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
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
	
		GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef tim;
		TIM_OCInitTypeDef TIM_OCInitStructure;
	  		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_Init(GPIOA, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_Init(GPIOA, &GPIO_InitStructure);  
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

    tim.TIM_Period = 13-1; //ARR的值 寄存器周期   
    tim.TIM_Prescaler = 8-1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM5, ENABLE);	//允许或禁止定时器工作时向ARR缓冲器中写入新值，在更新发生时载入覆盖以前的值
    TIM_TimeBaseInit(TIM5, &tim);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM5, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure); 
//	TIM_OC4Init(TIM5, &TIM_OCInitStructure); 

	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //CH3预装载使能	
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //CH3预装载使能	
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //CH3预装载使能	
//	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);  //CH3预装载使能	

	TIM_ARRPreloadConfig(TIM5, ENABLE); //使能TIMx在ARR上的预装载寄存器
	    DMA_InitTypeDef DMA_InitStructure;
			 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		/* 复位初始化 DMA 数据流 */
    DMA_DeInit(DMA1_Stream2); 
    /* 确保 DMA 数据流复位完成 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE);
	DMA_InitStructure.DMA_BufferSize = WS28_SENDBUFF_SIZE;
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer1;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM5->CCR1);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC16;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(DMA1_Stream2, &DMA_InitStructure);
    	//TIM_Cmd(TIM5, ENABLE);
    /* 等待 DMA 数据流有效 */
//    while (DMA_GetCmdStatus(DMA1_Stream7) != ENABLE);
    
//    DMA_Cmd(DMA1_Stream2, ENABLE);   
    
	TIM_DMACmd(TIM5, TIM_DMA_CC1, ENABLE);
	
	
//	
//	  DMA_DeInit(DMA1_Stream4);
//		DMA_InitStructure.DMA_Channel = DMA_Channel_6;
//		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer1;
//		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM5->CCR2);
//		DMA_Init(DMA1_Stream4, &DMA_InitStructure);
//			TIM_DMACmd(TIM5, TIM_DMA_CC2, ENABLE);
			
			DMA_DeInit(DMA1_Stream4);//原来为stream4
		DMA_InitStructure.DMA_Channel = DMA_Channel_6;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer1;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM5->CCR2);//原来为CCR2
		DMA_Init(DMA1_Stream4,&DMA_InitStructure);

				TIM_DMACmd(TIM5, TIM_DMA_CC2, ENABLE);	
//TIM_Cmd(TIM5, ENABLE);
DMA_DeInit(DMA1_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LED_BYTE_Buffer1;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM5->CCR4);
		DMA_Init(DMA1_Stream1, &DMA_InitStructure);
				TIM_DMACmd(TIM5, TIM_DMA_CC4, ENABLE);
TIM_Cmd(TIM5, ENABLE);
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
