#include "public.h"
LK_M_t LK_M_Gimbal_Yaw;

void LK_Encoder_Process(Encoder_plus *v, CanRxMsg * msg)
{
    v->encoder_data.last_raw_value=v->encoder_data.raw_value;//更新过去的编码器原始值
    v->encoder_data.raw_value=(msg->Data[7]<<8)| msg->Data[6];//读取编码器原始值
    v->encoder_data.diff=v->encoder_data.raw_value-v->encoder_data.last_raw_value;//记录两次的编码器的差值判断后面是否超过了一圈
    if(v->encoder_data.diff<-32500)
    {
        v->encoder_data.round_cnt++;//差值为负说明正转过一圈
        v->encoder_data.ecd_raw_rate=v->encoder_data.diff+65536;//计算单圈编码器的值
    }
    else if(v->encoder_data.diff>32500)
    {
        v->encoder_data.round_cnt--;
        v->encoder_data.ecd_raw_rate=v->encoder_data.diff-65536;
    }
    v->encoder_data.ecd_value=v->encoder_data.raw_value+65536*v->encoder_data.round_cnt;//经过处理后的编码器的连续值,范围为正负无穷大
    v->ecd_angle=(v->encoder_data.raw_value-v->encoder_data.ecd_bias)*0.0054931641f+360*v->encoder_data.round_cnt;//经过处理后角度的连续值,范围为正负无穷大
    
    v->angle=v->encoder_data.ecd_raw_rate*0.0054931641f; //单圈正负角度值
    v->temperature=msg->Data[1];
    v->torque=(msg->Data[3]<<8)| msg->Data[2];
    v->filter_rate=(msg->Data[5]<<8)| msg->Data[4];
    
}

void LK_task(Encoder_plus *v, CanRxMsg * msg,int offset)
{
    v->encoder_data.can_cnt++;
    if(v->encoder_data.can_cnt<2) v->encoder_data.ecd_bias=offset;
    LK_Encoder_Process(v,msg);
    if(v->encoder_data.can_cnt<10)
    {
        if((v->encoder_data.ecd_bias-v->encoder_data.raw_value)<-32700)
        {
            v->encoder_data.ecd_bias=offset+65536;
        }
        else if((v->encoder_data.ecd_bias-v->encoder_data.raw_value)>32700)
        {
            v->encoder_data.ecd_bias=offset-65536;
        }
    }
}

void LK_ENABLE_RX(CAN_TypeDef *CANx,uint32_t id)
{
    CanTxMsg txmsg;
	txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
	
	txmsg.Data[0] = 0x9C;
	txmsg.Data[1] = 0x00;
	txmsg.Data[2] = 0x00;
	txmsg.Data[3] = 0x00;
	txmsg.Data[4] = 0x00;
	txmsg.Data[5] = 0x00;
	CAN_Transmit(CANx,&txmsg);
}

void LK_Read_Motor_State_2_Request(CAN_TypeDef *CANx,int16_t id)
{
	CanTxMsg LK_M_CanTxMsg;
	LK_M_CanTxMsg.StdId = 0x140+id;
	LK_M_CanTxMsg.IDE = CAN_Id_Standard;
	LK_M_CanTxMsg.RTR = CAN_RTR_Data;
	LK_M_CanTxMsg.DLC = 0x08;
	LK_M_CanTxMsg.Data[0] =0x9c;
	
	CAN_Transmit(CANx,&LK_M_CanTxMsg);
}

int64_t Mul_Angle_Meddium_Test,Mul_Angle_Meddium_Test_2;
int64_t Mul_Angle_Meddium;

void LK_M_Data_Process(CanRxMsg *msg,int16_t id,LK_M_t* LK_M)
{
	if((msg->StdId-0x140)==id)
	{
		
	
		switch(msg->Data[0])
		{
			case 0x30:
			{
				LK_M->Angle_Kp=msg->Data[2];
				LK_M->Angle_Ki=msg->Data[3];
				LK_M->Speed_Kp=msg->Data[4];
				LK_M->Speed_Ki=msg->Data[5];
				LK_M->Iq_Kp=msg->Data[6];
				LK_M->Iq_Ki=msg->Data[7];
				break;
			}
			case 0x33:
			{
				LK_M->Accel=msg->Data[4]
				|(msg->Data[5]<<8)
				|(msg->Data[6]<<16)
				|(msg->Data[7]<<24);
				break;
			}
			case 0x90:
			{
				LK_M->Encoder=msg->Data[2]|(msg->Data[3]<<8);
				LK_M->Encoder_raw=msg->Data[4]|(msg->Data[5]<<8);
				LK_M->Encoder_Offset=msg->Data[6]|(msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				break;
			}
			case 0x92:
			{
				Mul_Angle_Meddium=((int64_t)msg->Data[1]
				|((int64_t)(msg->Data[2])<<8)
				|((int64_t)(msg->Data[3])<<16)
				|((int64_t)(msg->Data[4])<<24)
				|((int64_t)(msg->Data[5])<<32)
				|((int64_t)(msg->Data[6])<<40)
				|((int64_t)(msg->Data[7])<<48));
				
				if(msg->Data[7]>0)
				{
					Mul_Angle_Meddium=Mul_Angle_Meddium|0xff00000000000000;
				}
				
				Mul_Angle_Meddium_Test=Mul_Angle_Meddium;
				LK_M->Motor_Angle=0.1*((float)Mul_Angle_Meddium/100+((float)(Mul_Angle_Meddium-Mul_Angle_Meddium/100*100)*0.01));
//				LK_M->Motor_Angle=0.001*((float)Mul_Angle_Meddium);
				
				
				break;
			}
			case 0x94:
			{
				LK_M->Circle_Angle=((int32_t)msg->Data[4])
				|((int32_t)(msg->Data[5])<<8)
				|((int32_t)(msg->Data[6])<<16)
				|((int32_t)(msg->Data[7])<<24);
				
				LK_M->Circle_Angle=0.01*LK_M->Circle_Angle;
				break;
			}
			case 0x9a:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->Voltage=((int16_t)msg->Data[3])|((int16_t)(msg->Data[4])<<8);
				
				LK_M->Voltage=0.1*LK_M->Voltage;
				
				if(((msg->Data[7])&1)==1)
				{
					LK_M->Voltage_State=Vol_Low;
				}
				else
				{
					LK_M->Voltage_State=Vol_Normal;
				}
				
				if((msg->Data[7]>>3)&1)
				{
					LK_M->Temp_State=Over_Temp;
				}
				else
				{
					LK_M->Temp_State=Temp_Normal;
				}
				break;
			}
			case 0x9c:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);;
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->T=0.0025*LK_M->T;
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				break;
			}
			case 0xa0:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->Power=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=0.1*(((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8));
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa1:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=0.1*(((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8));
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)(LK_M->Encoder+65524/2)/65535*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa2:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)(LK_M->Encoder+65524/2)/65535*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa3:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa4:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa5:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa6:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa7:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			case 0xa8:
			{
				LK_M->Temp=msg->Data[1];
				LK_M->T=((int16_t)msg->Data[2])|((int16_t)msg->Data[3]<<8);
				LK_M->LK_Speed=((int16_t)msg->Data[4])|((int16_t)msg->Data[5]<<8);
				LK_M->Encoder=((int16_t)msg->Data[6])|((int16_t)msg->Data[7]<<8);
				
				LK_M->Circle_Angle=(float)LK_M->Encoder/16383*360;
				LK_M->T=0.0025*LK_M->T;
				break;
			}
			default:
				break;
		}
	}
}

void LK_M_Read_MulAngle_Request(CAN_TypeDef *CANx,int16_t id)
{
	CanTxMsg LK_M_CanTxMsg;
	LK_M_CanTxMsg.StdId = 0x140+id;
	LK_M_CanTxMsg.IDE = CAN_Id_Standard;
	LK_M_CanTxMsg.RTR = CAN_RTR_Data;
	LK_M_CanTxMsg.DLC = 0x08;
	LK_M_CanTxMsg.Data[0] =0x94;
	
	CAN_Transmit(CANx,&LK_M_CanTxMsg);
}

