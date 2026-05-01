#include "public.h"
G2C_t G2C_Data;
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
* @brief:       float_to_int: 浮点数转有符号整型
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

void CAN2_Chassis_Recieve(chassis_t *Chassis,CanRxMsg *msg)
{
	memcpy(&G2C_Data,msg->Data,sizeof(G2C_Data));
	Chassis->Vx=int_to_float(G2C_Data.Vx,-12,12,16);
	Chassis->Vy=int_to_float(G2C_Data.Vy,-12,12,16);
	Chassis->Vw=int_to_float(G2C_Data.Vw,-12,12,16);
	Chassis->ctrl_mode=G2C_Data.Chassis_Mode;
	Chassis->chassis_speed_mode=G2C_Data.Chassis_Speed_Mode;
	Chassis->Chassis_Move_State=G2C_Data.Move_State;
	RC_CtrlData.inputmode=G2C_Data.InputMode;
}

void CAN2_Gimbal_Gryo_Transmit(void)
{
	CanTxMsg tx_message;
    tx_message.StdId = C2G_ID_Gryo;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
	memcpy(tx_message.Data,&chassis_gyro.pitch_angle,8);
    CAN_Transmit(CAN2,&tx_message);
}

void CAN2_Gimbal_Motor_Transmit(void)
{
	CanTxMsg tx_message;
    tx_message.StdId = C2G_ID_Motor;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = Helm_chassis.Driving_Encoder[0].rate_rpm;
    tx_message.Data[1] = Helm_chassis.Driving_Encoder[0].rate_rpm>>8;
    tx_message.Data[2] = Helm_chassis.Driving_Encoder[1].rate_rpm;
    tx_message.Data[3] = Helm_chassis.Driving_Encoder[1].rate_rpm>>8;
    tx_message.Data[4] = Helm_chassis.Driving_Encoder[2].rate_rpm;
    tx_message.Data[5] = Helm_chassis.Driving_Encoder[2].rate_rpm>>8;
    tx_message.Data[6] = Helm_chassis.Driving_Encoder[3].rate_rpm;
    tx_message.Data[7] = Helm_chassis.Driving_Encoder[3].rate_rpm>>8;
    CAN_Transmit(CAN2,&tx_message);
}