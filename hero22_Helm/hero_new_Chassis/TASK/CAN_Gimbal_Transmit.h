#ifndef __CAN_CHASSIS_TRANSMIT_H__
#define __CAN_CHASSIS_TRANSMIT_H__

#define G2C_ID 0x300
#define C2G_ID_Gryo 0x310
#define C2G_ID_Motor 0x320
typedef struct packed_
{
	int16_t Vx;
	int16_t Vy;
	int16_t Vw;
	uint8_t Chassis_Mode:4;
	uint8_t Chassis_Speed_Mode:2;
	uint8_t InputMode:2;
	uint8_t Move_State;
}G2C_t;

void CAN2_Chassis_Recieve(chassis_t *Chassis,CanRxMsg *msg);
void CAN2_Gimbal_Motor_Transmit(void);
void CAN2_Gimbal_Gryo_Transmit(void);
#endif
