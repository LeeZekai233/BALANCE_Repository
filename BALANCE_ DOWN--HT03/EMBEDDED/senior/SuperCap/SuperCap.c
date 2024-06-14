#include "SuperCap.h"

SuperCap_Send_t Super_Cap_Send;
volatile usart_capacitance_message_t usart_capacitance_message;
uint8_t Capacitance_Message_Buf[100];


volatile can_capacitance_message_t can_capacitance_message;


/************************************* USART 通信  ************************************************/

void POWER_Control(SuperCap_Send_t *S)
{
	if(usart_chassis_data.chassis_mode==CHASSIS_RELAX)
	S->Stop_Control_Flag=0;
	else 
	S->Stop_Control_Flag=1;
	
	S->chassis_power_buffer=usart_chassis_data.chassis_power_buffer;
	S->chassis_power_limit =usart_chassis_data.chassis_power_limit;

	memcpy(&Capacitance_Message_Buf,(uint8_t *)S,sizeof(SuperCap_Send_t));
    Uart2SendBytesInfoProc((uint8_t *)Capacitance_Message_Buf,sizeof(Capacitance_Message_Buf));
}

void SuperCap_message_Process(volatile usart_capacitance_message_t *v,uint8_t *data)
{
	memcpy((uint8_t *)v,data,sizeof(usart_capacitance_message));
}


/************************************* CAN 通信  ************************************************/

void Can_SuperCap_message_Process(volatile can_capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x123:
	{
		memcpy((uint8_t *)v,msg->Data,8);
	}
	break;
	case 0x124:
	{
		memcpy((uint8_t *)v+8,msg->Data,8);
	}
	break;
	case 0x125:
	{
		memcpy((uint8_t *)v+16,msg->Data,8);
	}
	break;
	case 0x126:
	{
		memcpy((uint8_t *)v+24,msg->Data,8);
	}
	break;
	case 0x127:
	{
		memcpy((uint8_t *)v+32,msg->Data,8);
	}
	break;
	case 0x128:
	{
		memcpy((uint8_t *)v+40,msg->Data,8);
	}
	break;


	default:
		break;
	}
}


    
void CAN_POWER_Control(CAN_TypeDef *CANx ,SuperCap_Send_t *SC)
{
		if(usart_chassis_data.chassis_mode==CHASSIS_RELAX)
            SC->Stop_Control_Flag=0;
        else 
            SC->Stop_Control_Flag=1;
	
            SC->chassis_power_buffer=usart_chassis_data.chassis_power_buffer;
            SC->chassis_power_limit =usart_chassis_data.chassis_power_limit;

		memcpy(&Capacitance_Message_Buf,(uint8_t *)SC,sizeof(SuperCap_Send_t));
		
    CanTxMsg tx_message;
    tx_message.StdId = 0x300;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
		memcpy(tx_message.Data,(uint8_t *)SC,sizeof(SuperCap_Send_t));
    CAN_Transmit(CANx,&tx_message);
}
