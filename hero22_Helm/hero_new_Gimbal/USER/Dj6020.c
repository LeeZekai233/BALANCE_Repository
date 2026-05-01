#include "public.h"
Encoder V;
Encoder Poke_3508;
pid_t p1;//angle
pid_t p2;//speed

extern chassis_t 	chassis;
extern pid_t mec[4];

float angle_last,angle_now,angle_end11,differ;
int cnt11;


void Set_dj6020_iq(CAN_TypeDef *CANx,int16_t motor1_iq,int16_t motor2_iq,int16_t motor3_iq,int16_t motor4_iq)
{
    CanTxMsg tx_message;
    
    tx_message.StdId=0x1FF;
    //tx_message.ExtId=;
    tx_message.DLC=0x08;
    tx_message.IDE=CAN_Id_Standard;
    tx_message.RTR=CAN_RTR_Data;
    
    tx_message.Data[0]=(motor1_iq>>8)&0xFF;
    tx_message.Data[1]=(motor1_iq)&0xFF;
    tx_message.Data[2]=(motor2_iq>>8)&0xFF;
    tx_message.Data[3]=(motor2_iq)&0xFF;
    tx_message.Data[4]=(motor3_iq>>8)&0xFF;
    tx_message.Data[5]=(motor3_iq)&0xFF;
    tx_message.Data[6]=(motor4_iq>>8)&0xFF;
    tx_message.Data[7]=(motor4_iq)&0xFF;
    
    CAN_Transmit(CANx,&tx_message);
}

//void GM6020EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset)
//{
//	v->cal_data.can_cnt++;
//    //v->cal_data.heart_cnt = time_tick;
//	if(v->cal_data.can_cnt<=2){v->cal_data.ecd_bias = offset;}
//	GM6020EncoderProcess(v, msg);
//	// 码盘中间值设定也需要修改
//	if (v->cal_data.can_cnt <= 10)
//	{
//		if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) < -4000)
//		{
//				v->cal_data.ecd_bias = offset + 8192;
//		}
//		else if ((v->cal_data.ecd_bias - v->cal_data.ecd_value) > 4000)
//		{
//				v->cal_data.ecd_bias = offset - 8192;
//		}
//	}
//}

void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	v->cal_data.last_raw_value = v->cal_data.raw_value;
	v->cal_data.raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
	if(v->cal_data.diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->cal_data.round_cnt++;
		v->cal_data.ecd_raw_rate = v->cal_data.diff + 8192;
	}
	else if(v->cal_data.diff>4096)
	{
		v->cal_data.round_cnt--;
		v->cal_data.ecd_raw_rate = v->cal_data.diff- 8192;
	}		
	else
	{
		v->cal_data.ecd_raw_rate = v->cal_data.diff;
	}
	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->cal_data.raw_value - v->cal_data.ecd_bias)*0.0439453125f  + v->cal_data.round_cnt * 360;
	v->filter_rate = (msg->Data[2]<<8)|msg->Data[3];
	if(v->filter_rate>=1000)
		v->filter_rate = v->filter_rate - 65535;
	v->temperature = msg->Data[6];
    v->Torque=(msg->Data[4]<<8)|msg->Data[5];
}

void GM6020_PID_task_Init(Encoder *V)
{

    //PID_struct_init(&p1,POSITION_PID,200,1,15,0.01,0);
    PID_struct_init(&p2,POSITION_PID,12000,10000,60,0.8,0);
    //pid_calc(&p2,V->cal_data.ecd_raw_rate,5);
    //pid_double_loop_cal(&p1,&p2,0,V->ecd_angle,&p1.out,V->cal_data.ecd_raw_rate,0);
    
}

void C620_3508_PID_Init(void)
{
    PID_struct_init(&chassis.pid_3508_motor_speed[0],POSITION_PID,15000,1500,5,0,0);
    PID_struct_init(&chassis.pid_3508_motor[0],POSITION_PID,200,8,0.3,0,0);
    
    PID_struct_init(&chassis.pid_3508_motor_speed[1],POSITION_PID,15000,1500,5,0,0);
    PID_struct_init(&chassis.pid_3508_motor[1],POSITION_PID,200,8,0.3,0,0);
    
    PID_struct_init(&chassis.pid_3508_motor_speed[2],POSITION_PID,15000,1000,5,0,0);
    PID_struct_init(&chassis.pid_3508_motor[2],POSITION_PID,200,8,0.3,0,0);
    
    PID_struct_init(&chassis.pid_3508_motor_speed[3],POSITION_PID,15000,1000,5,0,0);
    PID_struct_init(&chassis.pid_3508_motor[3],POSITION_PID,200,8,0.3,0,0);
    
    PID_struct_init(&mec[0],POSITION_PID,5000,5000,10,0.15,0);
    PID_struct_init(&mec[1],POSITION_PID,5000,5000,10,0.1,0);
    PID_struct_init(&mec[2],POSITION_PID,5000,5000,10,0.1,0);
    PID_struct_init(&mec[3],POSITION_PID,5000,5000,10,0.1,0);
}

void M3508orM2006EncoderTask(volatile Encoder *v, CanRxMsg * msg)
{
	v->cal_data.can_cnt++;   
    //v->cal_data.heart_cnt = time_tick;
	//(v->cal_data.can_cnt<=5)?GetEncoderBias(v,msg):EncoderProcess(v,msg);
    if(v->cal_data.can_cnt<=5)
    {
        GetEncoderBias(v,msg);
    }
    else if(v->cal_data.can_cnt>5)
    {
        EncoderProcess(v,msg);
    }
}

int32_t rpm_middle;
float rpm_filter_k = 1;
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
    
    
    
	int i=0;
	int32_t temp_sum = 0;    
	v->cal_data.last_raw_value = v->cal_data.raw_value;
	v->cal_data.raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->cal_data.diff = v->cal_data.raw_value - v->cal_data.last_raw_value;
	if(v->cal_data.diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->cal_data.round_cnt++;
		v->cal_data.ecd_raw_rate = v->cal_data.diff + 8192;
	}
	else if(v->cal_data.diff>4096)
	{
		v->cal_data.round_cnt--;
		v->cal_data.ecd_raw_rate = v->cal_data.diff- 8192;
	}		
	else
	{
		v->cal_data.ecd_raw_rate = v->cal_data.diff;
	}
	//计算得到连续的编码器输出值
	v->cal_data.ecd_value = v->cal_data.raw_value + v->cal_data.round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->cal_data.raw_value - v->cal_data.ecd_bias)*0.04394531f + v->cal_data.round_cnt * 360;
	v->cal_data.rate_buf[v->cal_data.buf_count++] = v->cal_data.ecd_raw_rate;
	if(v->cal_data.buf_count == RATE_BUF_SIZE)
	{
		v->cal_data.buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->cal_data.rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
	//v->filter_rate =(msg->Data[2]<<8)|msg->Data[3];// v->cal_data.ecd_raw_rate;
	
	v->rate_rpm = (msg->Data[2]<<8)|msg->Data[3];
	
//	rpm_middle = (msg->Data[2]<<8)|msg->Data[3];
	
//	v->rate_rpm = rpm_filter_k*rpm_middle + (1-rpm_filter_k)*v->rate_rpm;
	
	v->temperature = msg->Data[6];
    
    
    
//    angle_last=angle_now;
//        angle_now=Poke_3508.cal_data.raw_value*0.0439453125;
//        differ=(angle_now-angle_last);
//        if(differ<-320)
//        {
//            cnt11++;
//            angle_now=differ+360;
//        }
//        else if(differ>320)
//        {
//            cnt11--;
//            angle_now=differ-360;
//        }
//        angle_end11=angle_now+360*cnt11;
//        v->angle=angle_end11;
}

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{

            v->cal_data.ecd_bias = (msg->Data[0]<<8)|(msg->Data[1]);  //保存初始编码器值作为偏差  
            v->cal_data.ecd_value = v->cal_data.ecd_bias;
            v->cal_data.last_raw_value = v->cal_data.ecd_bias;
            v->cal_data.temp_count++;
}

void Set_C620andC610_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(motor1_iq >> 8);
    tx_message.Data[1] = (uint8_t)motor1_iq;
    tx_message.Data[2] = (uint8_t)(motor2_iq >> 8);
    tx_message.Data[3] = (uint8_t)motor2_iq;
    tx_message.Data[4] = (uint8_t)(motor3_iq >> 8);
    tx_message.Data[5] = (uint8_t)motor3_iq;
    tx_message.Data[6] = (uint8_t)(motor4_iq >> 8);
    tx_message.Data[7] = (uint8_t)motor4_iq;
    CAN_Transmit(CANx,&tx_message);
}

void Set_C620andC610_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(motor5_iq >> 8);
    tx_message.Data[1] = (uint8_t)motor5_iq;
    tx_message.Data[2] = (uint8_t)(motor6_iq >> 8);
    tx_message.Data[3] = (uint8_t)motor6_iq;
    tx_message.Data[4] = (uint8_t)(motor7_iq >> 8);
    tx_message.Data[5] = (uint8_t)motor7_iq;
    tx_message.Data[6] = (uint8_t)(motor8_iq >> 8);
    tx_message.Data[7] = (uint8_t)motor8_iq;
    CAN_Transmit(CANx,&tx_message);
}




