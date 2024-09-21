#ifndef _SUPERCAP_H
#define _SUPERCAP_H
#include "public.h"

 typedef __packed struct
{
		 uint8_t  Stop_Control_Flag;
     uint16_t chassis_power_buffer; 
		 uint16_t    chassis_power_limit; 
}SuperCap_Send_t;


 typedef __packed struct
{

	uint8_t mode;
	uint8_t mode_sure;
	
	float in_power;
	float in_v;
	float in_i;
	
	float out_power;
	float out_v;
	float out_i;

	float tempureture;
	float time;
	float this_time;
	
	float  cap_voltage_filte;
}usart_capacitance_message_t;


 typedef __packed struct
{
  

	
	uint8_t mode;
	uint8_t mode_sure;
	
	float in_power;
	float in_v;
	float in_i;
	
	float out_power;
	float out_v;
	float out_i;

	float tempureture;
	float time;
	float this_time;
	
		float  cap_voltage_filte;
}can_capacitance_message_t;

extern volatile usart_capacitance_message_t usart_capacitance_message;
extern volatile can_capacitance_message_t can_capacitance_message;
extern SuperCap_Send_t Super_Cap_Send;
extern uint8_t Capacitance_Message_Buf[100];
void POWER_Control(SuperCap_Send_t *S);
void SuperCap_message_Process(volatile usart_capacitance_message_t *v,uint8_t *data);
void CAN_POWER_Control(CAN_TypeDef *CANx,SuperCap_Send_t *SC);
void Can_SuperCap_message_Process(volatile can_capacitance_message_t *v,CanRxMsg * msg);

#endif
