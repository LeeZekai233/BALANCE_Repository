#include "main.h"

void CAN1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruecture;
	CAN_InitTypeDef CAN_InitStruecture;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruecture);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
	CAN_DeInit(CAN1);
	
	CAN_StructInit(&CAN_InitStruecture);
	
	CAN_InitStruecture.CAN_ABOM=ENABLE;
	CAN_InitStruecture.CAN_AWUM=DISABLE;
	CAN_InitStruecture.CAN_NART=ENABLE;
	CAN_InitStruecture.CAN_RFLM=DISABLE;
	CAN_InitStruecture.CAN_TTCM=DISABLE;
	CAN_InitStruecture.CAN_TXFP=DISABLE;
	
	CAN_InitStruecture.CAN_Mode=CAN_Mode_Normal;
	CAN_InitStruecture.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStruecture.CAN_BS1=CAN_BS1_9tq;
	CAN_InitStruecture.CAN_BS2=CAN_BS2_4tq;
	CAN_InitStruecture.CAN_Prescaler=3;
	CAN_Init(CAN1,&CAN_InitStruecture);
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
//	NVIC_InitStructure.NVIC_IRQChannel=CAN1_TX_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
}


//uint8_t CAN1_Receive_Msg(uint8_t* buf)
//{
//	CanRxMsg RxMessage;
//	CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
//	return 1;
//}


//void CAN1_TX_IRQHandler(void) //CAN TX
//{
//	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
//		{
//			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//		}
//}


CanRxMsg rx_message_1;
void CAN1_RX0_IRQHandler(void)
{   
	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
			CAN_Receive(CAN1, CAN_FIFO0, &rx_message_1);
			//CAN1数据处理
//			CAN_Motor_Data_Receive(&rx_message_1,&Gimbal.Yaw_Motor_Encoder,&Gimbal.Pitch_Motor_Encoder,Chassis.Chassis_M2006_Encoder);
 //           DaMiao_8009_Information_Receive(&rx_message_1,&DaMiao_8009);
			CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
			CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
		}
}

//    /**********************
//    *@Brief:can发送函数
//    *@Call:内部或外部
//    *@Param:
//    *@Note:无
//    *@RetVal:无
//    **********************/
//    void CAN_Send_Data(CAN_TypeDef *CANx,int _DLC,int Std_ID,u8* adress)
//    {
//    CanTxMsg CanTxMsg;

//    CanTxMsg.IDE = CAN_Id_Standard;
//    CanTxMsg.RTR = CAN_RTR_Data;
//    CanTxMsg.DLC = _DLC;
//    CanTxMsg.StdId = Std_ID;
//    memcpy(&CanTxMsg.Data[0],adress,_DLC);

//    while((CANx->TSR&CAN_TSR_TME)==0);//等待发送邮箱空闲
//    CAN_Transmit(CANx,&CanTxMsg);
//    }





