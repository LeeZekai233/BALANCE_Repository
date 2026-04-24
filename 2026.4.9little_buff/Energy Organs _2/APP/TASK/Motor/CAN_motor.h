#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__


#include "public.h"
#define Buff_Len 4

typedef struct{
	int32_t raw_value;   							
	int32_t last_raw_value;							
	int32_t diff;									
	int32_t round_cnt;								
	float ecd_angle;								
	int32_t ecd_value;                    			
	int32_t rate_buf[6];	                		
	int16_t filter_rate;							
	int16_t rotate_rate;							
	int32_t angle_rate;								
	u32 temperature;                      			
	int32_t ecd_current;							
	double torque_current;							
	double torque;									
	                                                
	int32_t temp_count;                   			
	uint8_t buf_count;							    
	int32_t ecd_bias;
    double angle_bias;	
	int32_t ecd_raw_rate;								
} Motor_Encoder;

extern volatile Motor_Encoder Motor620_Encoder;
extern volatile Motor_Encoder Motor_c610and620_Encoder1;
extern volatile Motor_Encoder Motor6020_Encoder;



void GetEncoderBias(volatile Motor_Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Motor_Encoder *v, CanRxMsg * msg);
void PitchEncoderProcess(volatile Motor_Encoder *v,CanRxMsg * msg);

void Can1ReceiveMes(CanRxMsg *msg);
void Motor_620_out1(CAN_TypeDef *CANx, int16_t cm1_iq1 , int16_t cm1_iq2 , int16_t cm1_iq3 , int16_t cm1_iq4 );
void Motor_c610andc620_out1(CAN_TypeDef *CANx, int16_t cm1_iq1 , int16_t cm1_iq2 , int16_t cm1_iq3 , int16_t cm1_iq4 );
void Motor_6020_out(CAN_TypeDef *CANx, int16_t cm1_iq1 , int16_t cm1_iq2 , int16_t cm1_iq3 , int16_t cm1_iq4 );

#endif
