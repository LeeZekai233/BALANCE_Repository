#include "main.h"


void CAN2_Init(void)
{
	//初始化结构体
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef   CAN_InitStructure;
	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);
    //CAN单元配置
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=ENABLE;//失能禁止自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
	//设置波特率42m/((1+6+7)*6)
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;
	CAN_InitStructure.CAN_Prescaler=3;
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber=14;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannel=CAN2_TX_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
}

//uint8_t CAN2_Send_Msg(uint8_t* msg,uint8_t len)
//{
//	uint8_t mbox;
//	uint16_t i=0;
//	CanTxMsg TxMessage;
//	TxMessage.StdId=0x12;
//	TxMessage.ExtId=0x12;
//	TxMessage.IDE=CAN_Id_Standard;
//	TxMessage.RTR=CAN_RTR_Data;
//	TxMessage.DLC=len;
//	for(i=0;i<len;i++)
//	TxMessage.Data[i]=msg[i];
//	mbox=CAN_Transmit(CAN2,&TxMessage);
//	i=0;
//	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0xfff))
//	if(i>=0xfff) return 1;
//	return 0;
//}

//uint8_t CAN2_Receive_Msg(uint8_t* buf)
//{
//	uint32_t i;
//	CanRxMsg RxMessage;
//	if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;
//	CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);
//	for(i=0;i<8;i++)buf[i]=RxMessage.Data[i];
//	return RxMessage.DLC;
//}

//void CAN2_TX_IRQHandler(void) //CAN TX
//{
//	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
//		{
//			CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
//		}
//}


void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
        CAN2_Receive_Task(&rx_message,&Chassis);
        Can_SuperCap_message_Process(&can_capacitance_message,&rx_message);
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
    }	
}

