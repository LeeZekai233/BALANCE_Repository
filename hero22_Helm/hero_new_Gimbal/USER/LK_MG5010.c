#include "public.h"
int Angle_Kp ,Angle_Ki ,Speed_Kp ,Speed_Ki ,Iq_Kp ,Iq_Ki;


//void MF_EncoderProcess(Encoder *v, CanRxMsg * msg)//云台yaw，pitch共用
//{
//    
//	int i=0;
//	int32_t temp_sum = 0;
//    switch(msg->Data[0])
//		{
//			case 0x30:
//			{
//			Angle_Kp=msg->Data[2];
//			Angle_Ki=msg->Data[3];
//			Speed_Kp=msg->Data[4];
//			Speed_Ki=msg->Data[5];
//			Iq_Kp=msg->Data[6];
//			Iq_Ki=msg->Data[7];
//				break;
//			}
//            default:
//            {                
////	v->last_raw_value = v->raw_value;
//	v->cal_data.last_raw_value = v->cal_data.raw_value;			
////	v->raw_value = (msg->Data[7]<<8)|msg->Data[6];
//	v->cal_data.raw_value = (msg->Data[7]<<8)|msg->Data[6];			
////	v->diff = v->raw_value - v->last_raw_value;
//	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
//	if(v->cal_data.diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
//	{
//		v->cal_data.round_cnt++;
//		v->cal_data.ecd_raw_rate = v->cal_data.diff + 65536;
//	}
//	else if(v->cal_data.diff>32768)
//	{
//		v->cal_data.round_cnt--;
//		v->cal_data.ecd_raw_rate = v->cal_data.diff- 65536;
//	}		
//	else
//	{
//		v->cal_data.ecd_raw_rate = v->cal_data.diff;
//	}
//	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 65536;
//	//计算得到角度值，范围正负无穷大
//	v->ecd_angle = (float)(v->cal_data.raw_value - v->cal_data.ecd_bias)*0.0054931641f  + v->cal_data.round_cnt * 360;
//	//从电机编码器读取的速度
//	v->filter_rate = (int16_t)((msg->Data[5]<<8)|msg->Data[4]);
//	v->temperature = msg->Data[1];
//    }break;
//}
//}

void MF_EncoderTask(Encoder *v, CanRxMsg * msg,int offset)
{
	v->cal_data.can_cnt++;
	if(v->cal_data.can_cnt<=2){v->cal_data.ecd_bias = offset;}
    LK_MG5010Ecoder_Progress(v,msg);
	//MF_EncoderProcess(v, msg);
	// 码盘中间值设定也需要修改
	if (v->cal_data.can_cnt <= 10)
	{
		if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) < -32700)
		{
				v->cal_data.ecd_bias = offset + 65536;
		}
		else if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) > 32700)
		{
				v->cal_data.ecd_bias = offset - 65536;
		}
	}
}




void LK_MG5010Ecoder_Progress(Encoder *v, CanRxMsg * msg)
{
	switch(msg->Data[0])
	{
		case 0x30:
		{
			Angle_Kp=msg->Data[2];
			Angle_Ki=msg->Data[3];
			Speed_Kp=msg->Data[4];
			Speed_Ki=msg->Data[5];
			Iq_Kp=msg->Data[6];
			Iq_Ki=msg->Data[7];
		}
		break;
		
		default:
		{
			v->cal_data.last_raw_value = v->cal_data.raw_value;	
			v->cal_data.raw_value = msg->Data[7]<<8 | msg->Data[6];
			v->cal_data.diff = v->cal_data.raw_value  - v->cal_data.last_raw_value;
			if(v->cal_data.diff > 8191)
			{
				v->cal_data.round_cnt++;
			}
			else if(v->cal_data.diff < -8191)
			{
				v->cal_data.round_cnt--;
			}
			v->cal_data.ecd_value = v->cal_data.raw_value  + v->cal_data.round_cnt * 16383;
			v->ecd_angle  = (float)(v->cal_data.ecd_value - v->cal_data.ecd_bias) * 0.02197399744f + v->cal_data.round_cnt * 360;
			v->filter_rate = (int32_t)msg->Data[5]<<8|msg->Data[4];
			v->temperature = msg->Data[1];
		}
		break;
	}
}

//在上位机修改零点就不用再每次初始化零点
//void LK_MG5015_EncoderTask(volatile Encoder *v, CanRxMsg *msg, int offset)
//{
//	v->cal_data.can_cnt++;
//	if(v->cal_data.can_cnt <= 2){v->cal_data.ecd_bias = offset;}
//	LK_MG5010Ecoder_Progress(v,msg);
//}

//void LK_MG6015_Offset(int offset)
//{
//	CanTxMsg msg;
//	msg.StdId = 0x91;
//	
//}

void CAN_LK_MG5010_iqControl(CAN_TypeDef *CANx,int16_t iqControl,uint32_t id)
{
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	
	txmsg.Data[0] = 0xA1;
	txmsg.Data[1] = 0x00;
	txmsg.Data[2] = 0x00;
	txmsg.Data[3] = 0x00;
	txmsg.Data[4] = (uint8_t)iqControl;
	txmsg.Data[5] = (uint8_t)(iqControl>>8);
	CAN_Transmit(CANx,&txmsg);
}

void CAN_LK_MG5010_ERR_Clear(CAN_TypeDef *CANx,uint32_t id)
{
	CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	
	txmsg.Data[0] = 0x9B;
	CAN_Transmit(CANx,&txmsg);
}

void LK_5010_pitch_pid_Init(void)
{
    PID_struct_init(&gimbal_data.pid_init_pit_speed,POSITION_PID,2000,1,10,0,0);
    PID_struct_init(&gimbal_data.pid_init_pit_Angle,POSITION_PID,300,10,10,0.1,0);
}

