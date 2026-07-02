#include "main.h"

DM_Motor_t Yaw_DM4310;

//达妙手册里提供的int和float的转换函数
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max- x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


int float_to_uint(float x, float x_min, float x_max, int bits)
{
 /// Converts afloat to anunsigned int, given range and number ofbits///
    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


/********************************
*@Brief： DM_Motor 接收函数
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DM_Motor_Information_Receive(CanRxMsg *msg,DM_Motor_t *DM_Motor,float offset)
{
	int8_t ERR_Flag = (msg->Data[0]>>4);
    if(ERR_Flag == (int8_t)0)
    {
        DM_Motor->ERR = DM_DISABLE;
    }
    else if(ERR_Flag == (int8_t)1)
    {
        DM_Motor->ERR = DM_ENABLE;
    }
	if(ERR_Flag == (int8_t)8)
	{
		DM_Motor->ERR = U_MAX;
	}
	else if(ERR_Flag == (int8_t)9)
	{
		DM_Motor->ERR = U_MIN;
	}
	else if(ERR_Flag == (int8_t)0xA)
	{
		DM_Motor->ERR = I_MAX;
	}
	else if(ERR_Flag == (int8_t)0xB)
	{
		DM_Motor->ERR = MOS_TEM_MAX;
	}
	else if(ERR_Flag == (int8_t)0xC)
	{
		DM_Motor->ERR = ROTOR_TEM_MAX;
	}
	else if(ERR_Flag == (int8_t)0xD)
	{
		DM_Motor->ERR = CAN_BAG_LOST;
	}
	else if(ERR_Flag == (int8_t)0xE)
	{
		DM_Motor->ERR = OVERLOAD;
	}
    
	DM_Motor->P_fdb = Normalize_Angle_PI( uint_to_float((msg->Data[1]<<8) | (msg->Data[2]), P_MIN , P_MAX , 16) + offset); //3.14由上位机决定
	DM_Motor->V_fdb = uint_to_float((msg->Data[3]<<4) | (msg->Data[4]>>4), -45.0f , 45.0f , 12);//45由上位机得
	DM_Motor->T_fdb = uint_to_float((msg->Data[4]&0x0f)<<8 | (msg->Data[5]), -40.0f , 40.0f , 12);
	DM_Motor->Temperature_MOS = msg->Data[6];
	DM_Motor->Temperature_Rotor = msg->Data[7];
	
	DM_Motor->Angle_Deg_fdb = DM_Motor->P_fdb * 57.2958f;
	
	if(DM_Motor->Angle_Deg_fdb_last - DM_Motor->Angle_Deg_fdb > 180) //达妙系列圈数
	{
		DM_Motor->round_cnt++;
	}
	if(DM_Motor->Angle_Deg_fdb_last - DM_Motor->Angle_Deg_fdb < -180 ) 
	{
		DM_Motor->round_cnt--;
	}
    DM_Motor->Angle_Deg_Total_fdb = DM_Motor->round_cnt*360.0f + DM_Motor->Angle_Deg_fdb;
	DM_Motor->Angle_Deg_fdb_last = DM_Motor->Angle_Deg_fdb;
}


/********************************
*@Brief： DM_Motor MIT模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DM_Motor_Information_Send(CAN_TypeDef* CANx,uint16_t CAN_ID,float P_des,float V_ref,float T_ref,float Kp,float Kd)
{
	CanTxMsg Motor_DM_Motor_CanTxMsg;

	Motor_DM_Motor_CanTxMsg.StdId = CAN_ID;
	Motor_DM_Motor_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DM_Motor_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DM_Motor_CanTxMsg.DLC = 0x08;
		
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp =float_to_uint(P_des, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(V_ref, V_MIN, V_MAX, 12);
    kp_tmp =float_to_uint(Kp, KP_MIN, KP_MAX, 12);
    kd_tmp =float_to_uint(Kd, KD_MIN, KD_MAX,12);
    tor_tmp = float_to_uint(T_ref,T_MIN, T_MAX, 12);
	Motor_DM_Motor_CanTxMsg.Data[0] = (int16_t)pos_tmp >>8;
	Motor_DM_Motor_CanTxMsg.Data[1] = (int16_t)pos_tmp;
	Motor_DM_Motor_CanTxMsg.Data[2] = (int16_t)(vel_tmp) >>4;
	Motor_DM_Motor_CanTxMsg.Data[3] = (((int16_t)(vel_tmp)&0x000f)<<4) | ((kp_tmp&0x0f00)>>8);
	Motor_DM_Motor_CanTxMsg.Data[4] = kp_tmp;
	Motor_DM_Motor_CanTxMsg.Data[5] = (kd_tmp&0x0ff0)>>4;
	Motor_DM_Motor_CanTxMsg.Data[6] = ((kd_tmp&0x000f)<<4) | (((int16_t)(tor_tmp)&0x0f00)>>8);
	Motor_DM_Motor_CanTxMsg.Data[7] = tor_tmp;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DM_Motor_CanTxMsg));
	
}
/********************************
*@Brief： DM_Motor 位置模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DM_Motor_Position_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float P_des,float V_des)
{
    CanTxMsg Motor_DM_Motor_Position_CanTxMsg;
    
	Motor_DM_Motor_Position_CanTxMsg.StdId = CAN_ID;
	Motor_DM_Motor_Position_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DM_Motor_Position_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DM_Motor_Position_CanTxMsg.DLC = 0x08;
	
	u8 *pos,*vel;
	pos=(uint8_t*)(&P_des);
	vel=(uint8_t*)(&V_des);
	
	Motor_DM_Motor_Position_CanTxMsg.Data[0] = *pos;
	Motor_DM_Motor_Position_CanTxMsg.Data[1] = *(pos+1);
	Motor_DM_Motor_Position_CanTxMsg.Data[2] = *(pos+2);
	Motor_DM_Motor_Position_CanTxMsg.Data[3] = *(pos+3);
	
	Motor_DM_Motor_Position_CanTxMsg.Data[4] = *vel;
	Motor_DM_Motor_Position_CanTxMsg.Data[5] = *(vel+1);
	Motor_DM_Motor_Position_CanTxMsg.Data[6] = *(vel+2);
	Motor_DM_Motor_Position_CanTxMsg.Data[7] = *(vel+3);
	
//	while((CAN2->TSR&CAN_TSR_TME)==0);
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DM_Motor_Position_CanTxMsg));
	
}

/********************************
*@Brief： DM_Motor 速度模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/

void DM_Motor_Speed_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float V_des)
{
    CanTxMsg Motor_DM_Motor_Speed_CanTxMsg;
    
	Motor_DM_Motor_Speed_CanTxMsg.StdId = CAN_ID;
	Motor_DM_Motor_Speed_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DM_Motor_Speed_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DM_Motor_Speed_CanTxMsg.DLC = 0x08;
	
	u8 *vel;
	vel=(uint8_t*)(&V_des);
	
	Motor_DM_Motor_Speed_CanTxMsg.Data[0] = *vel;
	Motor_DM_Motor_Speed_CanTxMsg.Data[1] = *(vel+1);
	Motor_DM_Motor_Speed_CanTxMsg.Data[2] = *(vel+2);
	Motor_DM_Motor_Speed_CanTxMsg.Data[3] = *(vel+3);
	
//	while((CAN2->TSR&CAN_TSR_TME)==0);
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DM_Motor_Speed_CanTxMsg));
	
}

/********************************
*@Brief：   达妙电机 	保存位置零点
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DM_Motor_Position0_offset(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
	
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFE;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg));
}


/********************************
*@Brief：   达妙电机 	使能
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DM_Motor_Enable(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
	
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFC;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg));
}


/********************************
*@Brief：   达妙电机 	失能
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DM_Motor_Disable(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
	
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFD;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg));
}



/********************************
*@Brief：   达妙电机 	清除错误信息
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DM_Motor_Clear_Error_Information(CAN_TypeDef* CANx, int16_t CAN_ID)
{		
	CanTxMsg Motor_DaMiao_Init_CanTxMsg;
	
	Motor_DaMiao_Init_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_Init_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_Init_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_Init_CanTxMsg.DLC = 0x08;
	
	Motor_DaMiao_Init_CanTxMsg.Data[0] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[1] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[2] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[3] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[4] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[5] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[6] = 0xFF;
	Motor_DaMiao_Init_CanTxMsg.Data[7] = 0xFB;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_Init_CanTxMsg));
}


// 角度归一化到 [-PI , PI]
float AngleWrap(float angle)
{
    if(angle > PI)  
    {
        angle -= 2*PI;
    }
    else if(angle < -PI) 
    {
        angle += 2*PI;
    }
    return angle;
}


void DM_Motor_To_Generic_Encoder(DM_Motor_t* DM_Motor,Encoder_t* Encoder)
{
    Encoder->Angle_Deg_fdb = DM_Motor->Angle_Deg_fdb;
    Encoder->Angle_Deg_Total_fdb = DM_Motor->Angle_Deg_Total_fdb;
    
    Encoder->Angle_Rad_fdb = DM_Motor->P_fdb;
    Encoder->Angle_Rad_Total_fdb = DM_Motor->P_fdb + DM_Motor->round_cnt*2*PI;
    
    Encoder->Omega_Rad_fdb = DM_Motor->V_fdb ;
    Encoder->temperature = DM_Motor->Temperature_Rotor ;//选用转子温度
    Encoder->Torque = DM_Motor->T_fdb ;
    Encoder->heart_cnt = time_tick;
}


