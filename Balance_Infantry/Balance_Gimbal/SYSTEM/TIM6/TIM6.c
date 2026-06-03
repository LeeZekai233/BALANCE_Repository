#include "main.h"

void TIM6_Init(void)
{
	//结构体初始化
	TIM_TimeBaseInitTypeDef  TimeBaseStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	//使能时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	//配置时钟
	TimeBaseStructure.TIM_Period=84-1;
	TimeBaseStructure.TIM_Prescaler=1000-1;
	TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TimeBaseStructure);
	//使能时钟中断
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	//配置优先级
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//使能时钟
	TIM_Cmd(TIM6,ENABLE);
}



void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update))
	{
//		Contorl_Task(&Chassis);
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);//清除中断标志位
	}
}




