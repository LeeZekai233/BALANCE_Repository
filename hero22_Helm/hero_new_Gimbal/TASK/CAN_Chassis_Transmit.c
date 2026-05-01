#include "public.h"
/**
************************************************************************
* @brief:       int_to_float: 有符号整型转浮点数
* @param[in]:   x_int: 有符号原始值
* @param[in]:   x_min: 物理最小值
* @param[in]:   x_max: 物理最大值
* @param[in]:   bits:  总位数(如16)
* @retval:      线性映射浮点值
************************************************************************
**/
float int_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    int full_range = (1 << bits);
    int offset_int = x_int + (1 << (bits - 1));
    return (float)offset_int * span / (float)full_range + offset;
}

/**
************************************************************************
* @brief:       float_to_int: 浮点数转回有符号整型
* @param[in]:   x_float: 物理浮点值
* @param[in]:   x_min: 物理最小值
* @param[in]:   x_max: 物理最大值
* @param[in]:   bits:  总位数
* @retval:      有符号整型
************************************************************************
**/
int float_to_int(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    int full_range = (1 << bits);
    int half_range = (1 << (bits - 1));
    float temp = (x_float - x_min) * (float)full_range / span;
    int uint_val = (int)(temp + 0.5f);
    return uint_val - half_range;
}
void CAN2_Chassis_Transmit(	float Vx,
							float Vy,
							float Vw,
							chassis_mode_e Chassis_Mode,
							chassis_speed_mode_e Chassis_Speed_Mode,
							chassis_move_state_e Chassis_Move_State,
							InputMode_e Input_Mode)
{
	VAL_LIMIT(Vx,-12,12);
	VAL_LIMIT(Vy,-12,12);
	VAL_LIMIT(Vw,-12,12);
	VAL_LIMIT(Chassis_Mode,0,12);
	VAL_LIMIT(Chassis_Speed_Mode,1,4);
	VAL_LIMIT(Input_Mode,1,3);
	
	int16_t V_x=float_to_int(Vx,-12,12,16);
	int16_t V_y=float_to_int(Vy,-12,12,16);
	int16_t V_w=float_to_int(Vw,-12,12,16);
	
	CanTxMsg tx_message;
    tx_message.StdId = G2C_ID;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)V_x;
    tx_message.Data[1] = (uint8_t)(V_x >> 8);
    tx_message.Data[2] = (uint8_t)V_y;
    tx_message.Data[3] = (uint8_t)(V_y >> 8);
    tx_message.Data[4] = (uint8_t)V_w;
    tx_message.Data[5] = (uint8_t)(V_w >> 8);
    tx_message.Data[6] = Chassis_Mode|Chassis_Speed_Mode<<4|Input_Mode<<6;
    tx_message.Data[7] = Chassis_Move_State;
    CAN_Transmit(CAN2,&tx_message);
}

void CAN2_Chassis_Gyro_Recieve(chassis_t *Chassis,CanRxMsg *msg)
{
	memcpy(&Chassis->gyro_Pitch_angle,msg->Data,8);
}

void CAN2_Chassis_Motor_Recieve(chassis_t *Chassis,CanRxMsg *msg)
{
	memcpy(&Chassis->_3508_motor_speed_rpm,msg->Data,8);
}