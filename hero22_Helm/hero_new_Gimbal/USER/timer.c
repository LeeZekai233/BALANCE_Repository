#include "public.h"

#if EN_TIM6
void Tim6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel=TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority=3;
    NVIC_Init(&NVIC_InitStruct);
    
    TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period=1000-1;
    TIM_TimeBaseInitStruct.TIM_Prescaler=84-1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStruct);
    
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    
    TIM_Cmd(TIM6,ENABLE);
    TIM_ClearFlag(TIM6,TIM_FLAG_Update);
}        
#endif
void TIM6_DAC_IRQHandler()
{
    if(TIM_GetFlagStatus(TIM6,TIM_FLAG_Update)!=RESET)
    {
        TIM6_IRQProcess
        TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
        TIM_ClearFlag(TIM6,TIM_FLAG_Update);
    }
}

#if EN_TIM2
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    
    TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period=0xFFFFFFFF;
    TIM_TimeBaseInitStruct.TIM_Prescaler=84-1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
    
    TIM_Cmd(TIM2,ENABLE);
}

#endif

void TIM5_PWM_Init(void)
{
	//结构体初始化
	TIM_TimeBaseInitTypeDef  TimeBaseStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	//设置io端复用 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	//GPIO初始化 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//设置计时器频率（20000*84）/84000000
	TimeBaseStructure.TIM_Period = 20000 - 1;
	TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5,&TimeBaseStructure);
	//调节占空比
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//向上计数，CNT > ARR有效（高电平）
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//输出状态
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	//互补通道输出状态
	TIM_OCInitStructure.TIM_Pulse=1000;		//CCR
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;	//输出极性
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_Low;	//互补通道输出极性
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;	//空闲状态
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Set;	//互补通道空闲状态
	//结构体初始化，pre预装载使能
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
	//arr预装载使能，计时器使能
	TIM_ARRPreloadConfig(TIM5,DISABLE);
	TIM_Cmd(TIM5,ENABLE);	
	
}

 void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM5,Compare);
}

void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM5,Compare);
}







