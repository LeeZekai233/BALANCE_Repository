#include "public.h"

/***********************功能说明***********************/
/*
    1.初始化can配置:  Can_Init(),可以通过这些参数设置can通信的波特率,波特率 = APB1时钟频率 / (CAN_Prescaler * (1 + CAN_BS1 + CAN_BS2))
                      42/(3*(1+4+9))=1Mbps
    
*/

void Can1_Init(uint8_t ts1,uint8_t ts2,uint16_t brp,uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    CAN_InitTypeDef CAN_InitStruct;
    CAN_FilterInitTypeDef CAN_FilterInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
    
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA,&GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStruct);
    
    CAN_InitStruct.CAN_ABOM=ENABLE;
    CAN_InitStruct.CAN_AWUM=DISABLE;
    CAN_InitStruct.CAN_Mode=mode;
    CAN_InitStruct.CAN_NART=DISABLE;
    CAN_InitStruct.CAN_Prescaler=brp;
    CAN_InitStruct.CAN_RFLM=DISABLE;
    CAN_InitStruct.CAN_TTCM=DISABLE;
    CAN_InitStruct.CAN_TXFP=DISABLE;
    CAN_InitStruct.CAN_BS1=ts1;
    CAN_InitStruct.CAN_BS2=ts2;
    CAN_InitStruct.CAN_SJW=CAN_SJW_1tq;
    CAN_Init(CAN1,&CAN_InitStruct);
    
    CAN_FilterInitStruct.CAN_FilterActivation=ENABLE;
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
    CAN_FilterInitStruct.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStruct.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStruct.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStruct.CAN_FilterNumber=0;
    CAN_FilterInitStruct.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInit(&CAN_FilterInitStruct);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel=CAN1_RX0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
    NVIC_Init(&NVIC_InitStruct);
    
    
}

/***********************功能说明***********************/
/*

*/
void CAN1_RX0_IRQHandler()
{
    CanRxMsg rx_message;
    if(CAN_GetFlagStatus(CAN1,CAN_FLAG_FMP0)!=RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
        CAN_ClearFlag(CAN1,CAN_IT_FMP0);
        CAN_Receive(CAN1,CAN_FIFO0,&rx_message);
        CAN1_Data_Receive_Progress        
    }
}



void Can2_Init(uint8_t ts1,uint8_t ts2,uint16_t brp,uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    CAN_InitTypeDef CAN_InitStruct;
    CAN_FilterInitTypeDef CAN_FilterInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
    
    GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);
    
    CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStruct);
    
    CAN_InitStruct.CAN_ABOM=ENABLE;
    CAN_InitStruct.CAN_AWUM=DISABLE;
    CAN_InitStruct.CAN_Mode=mode;
    CAN_InitStruct.CAN_NART=DISABLE;
    CAN_InitStruct.CAN_Prescaler=brp;
    CAN_InitStruct.CAN_RFLM=DISABLE;
    CAN_InitStruct.CAN_TTCM=DISABLE;
    CAN_InitStruct.CAN_TXFP=DISABLE;
    CAN_InitStruct.CAN_BS1=ts1;
    CAN_InitStruct.CAN_BS2=ts2;
    CAN_InitStruct.CAN_SJW=CAN_SJW_1tq;
    CAN_Init(CAN2,&CAN_InitStruct);
    
    CAN_FilterInitStruct.CAN_FilterActivation=ENABLE;
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;
    CAN_FilterInitStruct.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStruct.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStruct.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStruct.CAN_FilterNumber=14;
    CAN_FilterInitStruct.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInit(&CAN_FilterInitStruct);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel=CAN2_RX1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
    NVIC_Init(&NVIC_InitStruct);
    
    
}

void CAN2_RX1_IRQHandler()
{
    CanRxMsg rx_message;
    if(CAN_GetFlagStatus(CAN2,CAN_FLAG_FMP1)!=RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);
        CAN_ClearFlag(CAN2,CAN_IT_FMP1);
        CAN_Receive(CAN2,CAN_FIFO1,&rx_message);
        CAN2_Data_Receive_Progress        
    }
}
