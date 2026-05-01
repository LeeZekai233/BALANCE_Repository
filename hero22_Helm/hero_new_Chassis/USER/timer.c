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








