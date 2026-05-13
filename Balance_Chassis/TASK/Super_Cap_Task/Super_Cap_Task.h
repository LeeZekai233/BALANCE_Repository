#ifndef __SUPER_CAP_TASK_H
#define __SUPER_CAP_TASK_H
#include <stm32f4xx.h>


typedef __packed struct
{
    uint8_t  Stop_Control_Flag;
    uint8_t  startWrelessCharge;
    uint16_t chassis_power_buffer;
    uint16_t chassis_power_limit;
} SuperCap_Send_t;




typedef __packed struct
{
	float  cap_voltage_filte;
    uint8_t mode;
} can_capacitance_message_t;



extern volatile can_capacitance_message_t can_capacitance_message;
extern SuperCap_Send_t Super_Cap_Send;
extern uint8_t Capacitance_Message_Buf[100];


void CAN_POWER_Control(CAN_TypeDef *CANx,SuperCap_Send_t *SC);
void Can_SuperCap_message_Process(volatile can_capacitance_message_t *v,CanRxMsg * msg);



#endif
