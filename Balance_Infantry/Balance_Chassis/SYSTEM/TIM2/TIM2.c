#include "main.h"


void TIM2_Init(void)
{
	//结构体初始化
	TIM_TimeBaseInitTypeDef  TimeBaseStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	//使能时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//配置时钟
	TimeBaseStructure.TIM_Period=84-1;
	TimeBaseStructure.TIM_Prescaler=1000-1;
	TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TimeBaseStructure);
	//使能时钟中断
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//配置优先级
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//使能时钟
	TIM_Cmd(TIM2,ENABLE);
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
//        if(DaMiao_8009.ERR != DM_ENABLE)
//        {
//        DaMiao_8009_Enable(CAN1,0x201);
//        }

//        if(DaMiao_8009.ERR == DM_ENABLE)
//        {
////            DaMiao_8009_Speed_Send(CAN1,&DaMiao_8009,0x201);
//    //        DaMiao_8009_Position_Send(CAN1,&DaMiao_8009,0x101);
//        }
        

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志位
	}
}
