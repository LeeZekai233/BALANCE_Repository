#include "main.h"

DaMiao_8009_t Joint_Motor[4];

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
*@Brief： DaMiao_8009 接收函数
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DaMiao_8009_Information_Receive(CanRxMsg *msg,DaMiao_8009_t *DaMiao_8009,float offset)
{
	int8_t ERR_Flag = (msg->Data[0]>>4);
    if(ERR_Flag == (int8_t)0)
    {
        DaMiao_8009->ERR = DM_DISABLE;
    }
    else if(ERR_Flag == (int8_t)1)
    {
        DaMiao_8009->ERR = DM_ENABLE;
    }
	if(ERR_Flag == (int8_t)8)
	{
		DaMiao_8009->ERR = U_MAX;
	}
	else if(ERR_Flag == (int8_t)9)
	{
		DaMiao_8009->ERR = U_MIN;
	}
	else if(ERR_Flag == (int8_t)0xA)
	{
		DaMiao_8009->ERR = I_MAX;
	}
	else if(ERR_Flag == (int8_t)0xB)
	{
		DaMiao_8009->ERR = MOS_TEM_MAX;
	}
	else if(ERR_Flag == (int8_t)0xC)
	{
		DaMiao_8009->ERR = ROTOR_TEM_MAX;
	}
	else if(ERR_Flag == (int8_t)0xD)
	{
		DaMiao_8009->ERR = CAN_BAG_LOST;
	}
	else if(ERR_Flag == (int8_t)0xE)
	{
		DaMiao_8009->ERR = OVERLOAD;
	}
    
	DaMiao_8009->P_fdb = Normalize_Angle_PI( uint_to_float((msg->Data[1]<<8) | (msg->Data[2]), P_MIN , P_MAX , 16) + offset); //3.14由上位机决定
	DaMiao_8009->V_fdb = uint_to_float((msg->Data[3]<<4) | (msg->Data[4]>>4), -45.0f , 45.0f , 12);//45由上位机得
	DaMiao_8009->T_fdb = uint_to_float((msg->Data[4]&0x0f)<<8 | (msg->Data[5]), -40.0f , 40.0f , 12);
	DaMiao_8009->Temperature_MOS = msg->Data[6];
	DaMiao_8009->Temperature_Rotor = msg->Data[7];
	
	DaMiao_8009->Angle_Deg_fdb = DaMiao_8009->P_fdb * 57.2958f;
	
	if(DaMiao_8009->Angle_Deg_fdb_last - DaMiao_8009->Angle_Deg_fdb > 180) //达妙系列圈数
	{
		DaMiao_8009->round_cnt++;
	}
	if(DaMiao_8009->Angle_Deg_fdb_last - DaMiao_8009->Angle_Deg_fdb < -180 ) 
	{
		DaMiao_8009->round_cnt--;
	}
    DaMiao_8009->Angle_Deg_Total_fdb = DaMiao_8009->round_cnt*360.0f + DaMiao_8009->Angle_Deg_fdb;
	DaMiao_8009->Angle_Deg_fdb_last = DaMiao_8009->Angle_Deg_fdb;
}


/********************************
*@Brief： DaMiao_8009 MIT模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DaMiao_8009_Information_Send(CAN_TypeDef* CANx,uint16_t CAN_ID,float P_des,float V_ref,float T_ref,float Kp,float Kd)
{
	CanTxMsg Motor_DaMiao_8009_CanTxMsg;

	Motor_DaMiao_8009_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_8009_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_8009_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_8009_CanTxMsg.DLC = 0x08;
		
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp =float_to_uint(P_des, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(V_ref, V_MIN, V_MAX, 12);
    kp_tmp =float_to_uint(Kp, KP_MIN, KP_MAX, 12);
    kd_tmp =float_to_uint(Kd, KD_MIN, KD_MAX,12);
    tor_tmp = float_to_uint(T_ref,T_MIN, T_MAX, 12);
	Motor_DaMiao_8009_CanTxMsg.Data[0] = (int16_t)pos_tmp >>8;
	Motor_DaMiao_8009_CanTxMsg.Data[1] = (int16_t)pos_tmp;
	Motor_DaMiao_8009_CanTxMsg.Data[2] = (int16_t)(vel_tmp) >>4;
	Motor_DaMiao_8009_CanTxMsg.Data[3] = (((int16_t)(vel_tmp)&0x000f)<<4) | ((kp_tmp&0x0f00)>>8);
	Motor_DaMiao_8009_CanTxMsg.Data[4] = kp_tmp;
	Motor_DaMiao_8009_CanTxMsg.Data[5] = (kd_tmp&0x0ff0)>>4;
	Motor_DaMiao_8009_CanTxMsg.Data[6] = ((kd_tmp&0x000f)<<4) | (((int16_t)(tor_tmp)&0x0f00)>>8);
	Motor_DaMiao_8009_CanTxMsg.Data[7] = tor_tmp;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_8009_CanTxMsg));
	
}
/********************************
*@Brief： DaMiao_8009 位置模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/
void DaMiao_8009_Position_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float P_des,float V_des)
{
    CanTxMsg Motor_DaMiao_8009_Position_CanTxMsg;
    
	Motor_DaMiao_8009_Position_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_8009_Position_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_8009_Position_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_8009_Position_CanTxMsg.DLC = 0x08;
	
	u8 *pos,*vel;
	pos=(uint8_t*)(&P_des);
	vel=(uint8_t*)(&V_des);
	
	Motor_DaMiao_8009_Position_CanTxMsg.Data[0] = *pos;
	Motor_DaMiao_8009_Position_CanTxMsg.Data[1] = *(pos+1);
	Motor_DaMiao_8009_Position_CanTxMsg.Data[2] = *(pos+2);
	Motor_DaMiao_8009_Position_CanTxMsg.Data[3] = *(pos+3);
	
	Motor_DaMiao_8009_Position_CanTxMsg.Data[4] = *vel;
	Motor_DaMiao_8009_Position_CanTxMsg.Data[5] = *(vel+1);
	Motor_DaMiao_8009_Position_CanTxMsg.Data[6] = *(vel+2);
	Motor_DaMiao_8009_Position_CanTxMsg.Data[7] = *(vel+3);
	
//	while((CAN2->TSR&CAN_TSR_TME)==0);
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_8009_Position_CanTxMsg));
	
}

/********************************
*@Brief： DaMiao_8009 速度模式下的控制帧发送
*@Cal：   内部或外部
*@param:  无
*@Note:   无
*@RetVal: 无
********************************/

void DaMiao_8009_Speed_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float V_des)
{
    CanTxMsg Motor_DaMiao_8009_Speed_CanTxMsg;
    
	Motor_DaMiao_8009_Speed_CanTxMsg.StdId = CAN_ID;
	Motor_DaMiao_8009_Speed_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_8009_Speed_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_8009_Speed_CanTxMsg.DLC = 0x08;
	
	u8 *vel;
	vel=(uint8_t*)(&V_des);
	
	Motor_DaMiao_8009_Speed_CanTxMsg.Data[0] = *vel;
	Motor_DaMiao_8009_Speed_CanTxMsg.Data[1] = *(vel+1);
	Motor_DaMiao_8009_Speed_CanTxMsg.Data[2] = *(vel+2);
	Motor_DaMiao_8009_Speed_CanTxMsg.Data[3] = *(vel+3);
	
//	while((CAN2->TSR&CAN_TSR_TME)==0);
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_DaMiao_8009_Speed_CanTxMsg));
	
}

/********************************
*@Brief：   达妙电机 	保存位置零点
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DaMiao_8009_Position0_offset(CAN_TypeDef* CANx, int16_t CAN_ID)
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
void DaMiao_8009_Enable(CAN_TypeDef* CANx, int16_t CAN_ID)
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
void DaMiao_8009_Disable(CAN_TypeDef* CANx, int16_t CAN_ID)
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
void DaMiao_8009_Claer_Error_Information(CAN_TypeDef* CANx, int16_t CAN_ID)
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


void DaMiao_8009_To_Generic_Encoder(DaMiao_8009_t* DaMiao_8009,Encoder_t* Encoder)
{
    Encoder->Angle_Deg_fdb = DaMiao_8009->Angle_Deg_fdb;
    Encoder->Angle_Deg_Total_fdb = DaMiao_8009->Angle_Deg_Total_fdb;
    
    Encoder->Angle_Rad_fdb = DaMiao_8009->P_fdb;
    Encoder->Angle_Rad_Total_fdb = DaMiao_8009->P_fdb + DaMiao_8009->round_cnt*2*PI;
    
    Encoder->Omega_Rad_fdb = DaMiao_8009->V_fdb ;
    Encoder->temperature = DaMiao_8009->Temperature_Rotor ;//选用转子温度
    Encoder->Torque = DaMiao_8009->T_fdb ;
    Encoder->heart_cnt = time_tick;
}


