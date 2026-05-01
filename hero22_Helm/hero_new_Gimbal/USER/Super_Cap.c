#include "public.h"

SuperCap_Send_t Super_Cap_Send;
can_capacitance_message_t can_capacitance_message;
uint8_t Capacitance_Message_Buf[100];
int supercap_cnt[6]={0};

void Can_SuperCap_message_Process(can_capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x123:
	{
		memcpy((uint8_t *)v,msg->Data,8);
		supercap_cnt[0]++;
		Equipment_Counter_Make_Zero(&Peripheral_State.Super_Cap);
	}
	break;
	case 0x124:
	{

		memcpy((uint8_t *)v+8,msg->Data,8);
		supercap_cnt[1]++;
	}
	break;
	case 0x125:
	{
		memcpy((uint8_t *)v+16,msg->Data,8);
		supercap_cnt[2]++;
	}
	break;
	case 0x126:
	{
		memcpy((uint8_t *)v+24,msg->Data,8);
		supercap_cnt[3]++;
	}
	break;
	case 0x127:
	{
		memcpy((uint8_t *)v+32,msg->Data,8);
		supercap_cnt[4]++;
	}
	break;
	case 0x128:
	{
		memcpy((uint8_t *)v+40,msg->Data,8);
		supercap_cnt[5]++;
	}
	break;


	default:
		break;
	}
}


void CAN_POWER_Control(CAN_TypeDef *CANx ,SuperCap_Send_t *SC)
{
	volatile static uint8_t supercap_send_delay=0;
	if(judge_rece_mesg.game_robot_state.power_management_chassis_output==1)
	{
		SC->Stop_Control_Flag=1;
	}
	else
		SC->Stop_Control_Flag=0;

		SC->chassis_power_buffer=judge_rece_mesg.power_heat_data.buffer_energy;
		SC->chassis_power_limit =judge_rece_mesg.game_robot_state.chassis_power_limit;

		memcpy(&Capacitance_Message_Buf,(uint8_t *)SC,sizeof(SuperCap_Send_t));
		
    CanTxMsg tx_message;
	tx_message.StdId = 0x100;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

	
	
	memcpy(tx_message.Data,(uint8_t *)SC,sizeof(SuperCap_Send_t));
    CAN_Transmit(CANx,&tx_message);
}