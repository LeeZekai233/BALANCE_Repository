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


void CAN2_SuperCap_Data(CanRxMsg *msg)
{
	memcpy(&Super_Cap_Send,msg->Data,sizeof(Super_Cap_Send));
}
