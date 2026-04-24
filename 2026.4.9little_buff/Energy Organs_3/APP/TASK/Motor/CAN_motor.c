#include <CAN_motor.h>

//u8 Temp_Buff[Buff_Len];
volatile Motor_Encoder Motor620_Encoder = {0};
volatile Motor_Encoder Motor_c610and620_Encoder1 = {0};    
volatile Motor_Encoder Motor6020_Encoder = {0};

void GetEncoderBias(volatile Motor_Encoder *v, CanRxMsg * msg)
{
	v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1]; 
	v->ecd_value = v->ecd_bias;                   
	v->raw_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;              
	v->temp_count++;    
	v->angle_bias = v->ecd_bias*0.04394531f;
}


void EncoderProcess(volatile Motor_Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum=0;
	
	
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)                             
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff +  8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->angle_bias = v->ecd_bias*0.04394531f;
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f + v->round_cnt * 360;

	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == 6)
	{
		v->buf_count = 0;
	}
	
	
	for(i = 0;i < 6; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int16_t)(temp_sum/6);	
	
	v->rotate_rate = (msg->Data[2]<<8)|msg->Data[3];
	
	v->ecd_current = (msg->Data[4]<<8)|msg->Data[5];
	v->torque_current = v->ecd_current * 20 / 16384;
	
	v->temperature = msg->Data[6];
	
	v->torque = 9549 * 24 * v->torque_current / v->rotate_rate;
}


void PitchEncoderProcess(volatile Motor_Encoder *v,CanRxMsg * msg)
{
	
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0439453125f  + v->round_cnt * 360;
	v->filter_rate = (msg->Data[2]<<8)|msg->Data[3];
//	if(v->filter_rate>=1000)
//		v->filter_rate = v->filter_rate - 65535;
	v->temperature = msg->Data[6];

}

int can1_count=0;

void Can1ReceiveMes(CanRxMsg *msg)
{
	can1_count++;
	
	if(msg->StdId==0x201)
	{
		PitchEncoderProcess(&Motor620_Encoder,msg);
		if(can1_count<=100)
		{
			
			
			if((Motor620_Encoder.ecd_bias - Motor620_Encoder.ecd_value) <-4000)Motor620_Encoder.ecd_bias = 4095 + 8192;
			else if((Motor620_Encoder.ecd_bias - Motor620_Encoder.ecd_value) > 4000)  Motor620_Encoder.ecd_bias = 4095 - 8192;
		}
	}
	

	if(msg->StdId==0x201)
	{
		if(can1_count<=2)  GetEncoderBias(&Motor6020_Encoder ,msg);
		else EncoderProcess(&Motor6020_Encoder ,msg);

	}   
    if(can1_count > 10000)
	{
		can1_count = 10000;
	}
}

void can1_count_reinit(void)
{
	can1_count=0;
}


void Motor_620_out1(CAN_TypeDef *CANx, int16_t cm1_iq1 , int16_t cm1_iq2 , int16_t cm1_iq3 , int16_t cm1_iq4 )
{
	  CanTxMsg Motor_620_CanTxMsg;
    Motor_620_CanTxMsg.StdId = 0x200;
    Motor_620_CanTxMsg.IDE = CAN_Id_Standard;
    Motor_620_CanTxMsg.RTR = CAN_RTR_Data;
    Motor_620_CanTxMsg.DLC = 0x08;
    Motor_620_CanTxMsg.Data[0] = (uint8_t)(cm1_iq1 >> 8);
    Motor_620_CanTxMsg.Data[1] = (uint8_t)cm1_iq1;
    Motor_620_CanTxMsg.Data[2] = (uint8_t)(cm1_iq2 >> 8);
    Motor_620_CanTxMsg.Data[3] = (uint8_t)cm1_iq2;
    Motor_620_CanTxMsg.Data[4] = (uint8_t)(cm1_iq3 >> 8);
    Motor_620_CanTxMsg.Data[5] = (uint8_t)cm1_iq3;
    Motor_620_CanTxMsg.Data[6] = (uint8_t)(cm1_iq4 >> 8);
    Motor_620_CanTxMsg.Data[7] = (uint8_t)cm1_iq4;
    CAN_Transmit(CANx,&Motor_620_CanTxMsg);
}


void Motor_6020_out(CAN_TypeDef *CANx, int16_t cm1_iq1 , int16_t cm1_iq2 , int16_t cm1_iq3 , int16_t cm1_iq4 )
{
	  CanTxMsg Motor_6020_CanTxMsg;
    Motor_6020_CanTxMsg.StdId = 0x1FF;
    Motor_6020_CanTxMsg.IDE = CAN_Id_Standard;
    Motor_6020_CanTxMsg.RTR = CAN_RTR_Data;
    Motor_6020_CanTxMsg.DLC = 0x08;
    Motor_6020_CanTxMsg.Data[0] = (uint8_t)(cm1_iq1 >> 8);
    Motor_6020_CanTxMsg.Data[1] = (uint8_t)cm1_iq1;
    Motor_6020_CanTxMsg.Data[2] = (uint8_t)(cm1_iq2 >> 8);
    Motor_6020_CanTxMsg.Data[3] = (uint8_t)cm1_iq2;
    Motor_6020_CanTxMsg.Data[4] = (uint8_t)(cm1_iq3 >> 8);
    Motor_6020_CanTxMsg.Data[5] = (uint8_t)cm1_iq3;
    Motor_6020_CanTxMsg.Data[6] = (uint8_t)(cm1_iq4 >> 8);
    Motor_6020_CanTxMsg.Data[7] = (uint8_t)cm1_iq4;
    CAN_Transmit(CANx,&Motor_6020_CanTxMsg);
}

void Motor_c610andc620_out1(CAN_TypeDef *CANx, int16_t cm1_iq1 , int16_t cm1_iq2 , int16_t cm1_iq3 , int16_t cm1_iq4 )
{
	  CanTxMsg Motor_c610andc620_CanTxMsg;
    Motor_c610andc620_CanTxMsg.StdId = 0x1FF;
    Motor_c610andc620_CanTxMsg.IDE = CAN_Id_Standard;
    Motor_c610andc620_CanTxMsg.RTR = CAN_RTR_Data;
    Motor_c610andc620_CanTxMsg.DLC = 0x08;
    Motor_c610andc620_CanTxMsg.Data[0] = (uint8_t)(cm1_iq1 >> 8);
    Motor_c610andc620_CanTxMsg.Data[1] = (uint8_t)cm1_iq1;
    Motor_c610andc620_CanTxMsg.Data[2] = (uint8_t)(cm1_iq2 >> 8);
    Motor_c610andc620_CanTxMsg.Data[3] = (uint8_t)cm1_iq2;
    Motor_c610andc620_CanTxMsg.Data[4] = (uint8_t)(cm1_iq3 >> 8);
    Motor_c610andc620_CanTxMsg.Data[5] = (uint8_t)cm1_iq3;
    Motor_c610andc620_CanTxMsg.Data[6] = (uint8_t)(cm1_iq4 >> 8);
    Motor_c610andc620_CanTxMsg.Data[7] = (uint8_t)cm1_iq4;
    CAN_Transmit(CANx,&Motor_c610andc620_CanTxMsg);
}



