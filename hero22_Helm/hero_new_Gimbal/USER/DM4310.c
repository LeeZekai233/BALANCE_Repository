#include "public.h"


DaMiao_4310_t DM_4310;

float uint_to_float(int x_int, float x_min, float x_max, int bits){

 /// converts unsigned int to float, given range and number of bits ///
 float span = x_max- x_min;

 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;

 }

 int round_cnt;
 int DM4310_CNT;
 u16 Angle_Medium_Test;

void DM4310_Progress(volatile DaMiao_4310_t *v,CanRxMsg *msg)
{
	if(DM4310_CNT<50)
	{
		DM4310_CNT++;
//		round_cnt=v->P_fdb/360;
		if(v->P_fdb<0)
			round_cnt-=1;
	}
	
		v->ERR = NORMAL;
	int8_t ERR_Flag = (msg->Data[0]>>4);
	if(ERR_Flag == (int8_t)8)
	{
		v->ERR = U_MAX;
	}
	if(ERR_Flag == (int8_t)9)
	{
		v->ERR = U_MIN;
	}
	if(ERR_Flag == (int8_t)0xA)
	{
		v->ERR = I_MAX;
	}
	if(ERR_Flag == (int8_t)0xB)
	{
		v->ERR = MOS_TEM_MAX;
	}
	if(ERR_Flag == (int8_t)0xC)
	{
		v->ERR = ROTOR_TEM_MAX;
	}
	if(ERR_Flag == (int8_t)0xD)
	{
		v->ERR = CAN_BAG_LOST;
	}
	if(ERR_Flag == (int8_t)0xE)
	{
		v->ERR = OVERLOAD;
	}
	
	
//	static  p_fdb_last;
	int16_t static p_fdb_last;
//	v->angle = (float)(((int16_t)((msg->Data[1]<<8) | (msg->Data[2]))) +32767)/32767*180;   //+-32767   
//	v->filter_rate = ((float)(((int16_t)((msg->Data[3]<<4) | (msg->Data[4]>>4)))-2047)) *0.02198 ;  //+-45r/s
//	v->Torque = ((float)(((int16_t)(((msg->Data[4]&0x0f)<<8) | (msg->Data[5])))-2047) )*0.000977;
	
	v->P_fdb = (float)(  (int16_t)((msg->Data[1]<<8) | (msg->Data[2])));// * 0.0003814755474f  - 12.5    ) ;//* 0.0003814755f  ;   //+-32767   
	v->V_fdb = (float)(  (int16_t)((msg->Data[3]<<4) | (msg->Data[4]>>4)));// * 0.02197802198f -45    );  //+-45r/s
	v->T_fdb = (float)(  (int16_t)(((msg->Data[4]&0x0f)<<8) | (msg->Data[5]))) ;//* 0.00879 - 18); 
	
	Angle_Medium_Test=v->P_fdb;
	v->P_fdb = (uint_to_float(v->P_fdb,-3.14,3.14,16)/PI)*180;
	if(v->P_fdb < 0)
		v->P_fdb += 360;
//	if(((v->P_fdb - p_fdb_last) > 700) && (poke_benchmark != 0))	//在零点时反馈值会在0和-720之间跳动，故在拨盘开始转动后才开始计圈
//		round_cnt+=1;
	if( (v->P_fdb - p_fdb_last) >= 330 )	//在零点时反馈值会在0和-720之间跳动，故在拨盘开始转动后才开始计圈	
		round_cnt--;
	else if( (v->P_fdb - p_fdb_last) <= -330 )
		round_cnt++;
	v->Angle_fdb = round_cnt*360 + v->P_fdb;
	v->V_fdb = uint_to_float(v->V_fdb,-45,45,12)*10;	//速度反馈过小，卡弹反转结束判断不准确
	v->T_fdb = uint_to_float(v->T_fdb,-18,18,12);
	p_fdb_last = v->P_fdb;
}

//位置速度模式
void DM4310_angle_control(CAN_TypeDef *CANx,float angle,float speed,uint32_t id)
{
    CanTxMsg txmsg;
//	txmsg.StdId = 0x101;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
    angle = (angle/180.0)*PI;
    uint8_t *pbuf,*vbuf;		//定义指向uint8_t类型的指针
    pbuf=(uint8_t*)&angle;		//把angle的地址赋给pbuf
    vbuf=(uint8_t*)&speed;		//把speed的地址赋给vbuf
    
	txmsg.Data[0] = *pbuf;
	txmsg.Data[1] = *(pbuf+1);
	txmsg.Data[2] = *(pbuf+2);
	txmsg.Data[3] = *(pbuf+3);
	txmsg.Data[4] = *vbuf;
	txmsg.Data[5] = *(vbuf+1);
	txmsg.Data[6] = *(vbuf+2);
	txmsg.Data[7] = *(vbuf+3);
	
	CAN_Transmit(CANx,&txmsg);

}
/********************************
*@Brief：   达妙电机 	清楚错误
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void DaMiao_Exception_Clear(int16_t CAN_ID)
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
	
	CAN_TransmitStatus(CAN2,CAN_Transmit(CAN2,&Motor_DaMiao_Init_CanTxMsg));
}

//达妙电机使能帧
void DaMiao_Run_Init(CAN_TypeDef *CANx,uint16_t CAN_ID)
{		
	CanTxMsg txmsg;
	
	txmsg.StdId = CAN_ID;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.DLC = 0x08;
		
	txmsg.Data[0] = 0xFF;
	txmsg.Data[1] = 0xFF;
	txmsg.Data[2] = 0xFF;
	txmsg.Data[3] = 0xFF;
	txmsg.Data[4] = 0xFF;
	txmsg.Data[5] = 0xFF;
	txmsg.Data[6] = 0xFF;
	txmsg.Data[7] = 0xFC;
	
	CAN_Transmit(CANx,&txmsg);
}

void DaMiao_4310_Position0_offset(CAN_TypeDef *CANx,int16_t CAN_ID)
{		
	CanTxMsg txmsg;
	
	txmsg.StdId = CAN_ID;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	txmsg.DLC = 0x08;
		
	txmsg.Data[0] = 0xFF;
	txmsg.Data[1] = 0xFF;
	txmsg.Data[2] = 0xFF;
	txmsg.Data[3] = 0xFF;
	txmsg.Data[4] = 0xFF;
	txmsg.Data[5] = 0xFF;
	txmsg.Data[6] = 0xFF;
	txmsg.Data[7] = 0xFE;
	
	CAN_Transmit(CANx,&txmsg);
}


void DaMiao_4310_Information_Send(DaMiao_4310_t *DaMiao_4310,int16_t ID)
{
	
	CanTxMsg Motor_DaMiao_4310_CanTxMsg;
	Motor_DaMiao_4310_CanTxMsg.StdId = ID ;
	Motor_DaMiao_4310_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_DaMiao_4310_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_DaMiao_4310_CanTxMsg.DLC = 0x08;
		
	
	DaMiao_4310->P_ref = DaMiao_4310->Angle_ref;       //转换比
	
	Motor_DaMiao_4310_CanTxMsg.Data[0] = DaMiao_4310->P_ref >>8;
	Motor_DaMiao_4310_CanTxMsg.Data[1] = DaMiao_4310->P_ref;
	Motor_DaMiao_4310_CanTxMsg.Data[2] = DaMiao_4310->V_ref >>4;
	Motor_DaMiao_4310_CanTxMsg.Data[3] = ((DaMiao_4310->V_ref&0x000f)<<4) | ((DaMiao_4310->Kp&0x0f00)>>8);
	Motor_DaMiao_4310_CanTxMsg.Data[4] = DaMiao_4310->Kp;
	Motor_DaMiao_4310_CanTxMsg.Data[5] = (DaMiao_4310->Kd&0x0ff0)>>4;
	Motor_DaMiao_4310_CanTxMsg.Data[6] = ((DaMiao_4310->Kd&0x000f)<<4) | ((DaMiao_4310->T_ref&0x0f00)>>8);
	Motor_DaMiao_4310_CanTxMsg.Data[7] = DaMiao_4310->T_ref;
	
	CAN_TransmitStatus(CAN1,CAN_Transmit(CAN2,&Motor_DaMiao_4310_CanTxMsg));
	
}




int float_to_uint(float x, float x_min, float x_max, int bits)
{	
	 float span = x_max-x_min;
     float offset =x_min;
	 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


void DM_4310_MIT(CAN_TypeDef *CANx,float angle,float speed,float kp,float kd,float t,uint16_t id)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(angle,-3.14,3.14,16);
	vel_tmp = float_to_uint(speed,-45,45,12);
	kp_tmp = float_to_uint(kp,0,500,12);
	kd_tmp = float_to_uint(kd,0,5,12);
	tor_tmp = float_to_uint(t,-18,18,12);
	
	CanTxMsg txmsg;
//	txmsg.StdId = 0x101;
	txmsg.StdId = 0x02;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
//    angle = (angle/360.0)*PI;
//    uint8_t *pbuf,*vbuf,*kpbuf;		//定义指向uint8_t类型的指针
//    pbuf=(uint8_t*)&angle;		//把angle的地址赋给pbuf
//    vbuf=(uint8_t*)&speed;		//把speed的地址赋给vbuf
//	kpbuf=(uint8_t*)&kp;
    
	txmsg.Data[0] = (pos_tmp >> 8);
	txmsg.Data[1] = pos_tmp;
	txmsg.Data[2] = (vel_tmp >> 4);
	txmsg.Data[3] =  ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	txmsg.Data[4] =  kp_tmp;
	txmsg.Data[5] =  (kd_tmp>> 4);
	txmsg.Data[6] =  ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	txmsg.Data[7] =  tor_tmp;
	
	CAN_Transmit(CANx,&txmsg);
}

void DM_Speed_Mode(CAN_TypeDef *CANx,float _vel,uint16_t id)
{
	uint8_t *vbuf;
	vbuf = (uint8_t*)&_vel;
	
		
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x04;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	
	txmsg.Data[0] = *vbuf;
	txmsg.Data[1] = *(vbuf+1);
	txmsg.Data[2] = *(vbuf+2);
	txmsg.Data[3] = *(vbuf+3);
	txmsg.Data[4] =  0;
	txmsg.Data[5] =  0;
	txmsg.Data[6] =  0;
	txmsg.Data[7] =  0;
	
	CAN_Transmit(CANx,&txmsg);
}

