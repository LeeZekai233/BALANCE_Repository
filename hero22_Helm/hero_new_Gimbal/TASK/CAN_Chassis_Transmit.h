#ifndef __CAN_CHASSIS_TRANSMIT_H__
#define __CAN_CHASSIS_TRANSMIT_H__

#define G2C_ID 0x300
#define C2G_ID_Gryo 0x310
#define C2G_ID_Motor 0x320

typedef struct
{
	int8_t Vx;
	int8_t Vy;
	int8_t Vw;
	uint8_t Chassis_Mode;
	uint16_t Res1;
	uint16_t Res2;
}G2C_t;

void CAN2_Chassis_Transmit(	float Vx,
							float Vy,
							float Vw,
							chassis_mode_e Chassis_Mode,
							chassis_speed_mode_e Chassis_Speed_Mode,
							chassis_move_state_e Chassis_Move_State,
							InputMode_e Input_Mode);
							
void CAN2_Chassis_Motor_Recieve(chassis_t *Chassis,CanRxMsg *msg);
void CAN2_Chassis_Gyro_Recieve(chassis_t *Chassis,CanRxMsg *msg);

#endif
