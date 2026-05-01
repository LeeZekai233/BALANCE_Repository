#ifndef __SUPER_CAP_H__
#define __SUPER_CAP_H__

typedef __packed struct
{
	uint8_t  Stop_Control_Flag;//开超电为1
	uint8_t StartWrirelessCharge;//开启无线充电为1
	uint16_t chassis_power_buffer; 
	uint16_t chassis_power_limit; 
}SuperCap_Send_t;

typedef __packed struct
{
	float  cap_voltage_filte;
	uint8_t mode;				//最简通信
	
}can_capacitance_message_t;

void Can_SuperCap_message_Process(can_capacitance_message_t *v,CanRxMsg * msg);
void CAN_POWER_Control(CAN_TypeDef *CANx ,SuperCap_Send_t *SC);
extern SuperCap_Send_t Super_Cap_Send;
extern can_capacitance_message_t can_capacitance_message;
void CAN2_SuperCap_Data(CanRxMsg *msg);


#endif