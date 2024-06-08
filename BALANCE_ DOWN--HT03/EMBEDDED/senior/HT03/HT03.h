#ifndef __HT03_H
#define __HT03_H
#include "public.h"

#define MOTOR_Reduction_ratio 6

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03


#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef STRUCT_MOTOR
#define STRUCT_MOTOR

#define RATE_BUF_SIZE 6
typedef struct 
{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t can_cnt;					//��¼������ʹ�ô������ڵ����ʼ��ɲ�������
    int32_t heart_cnt;
}Encoder_cal;


typedef struct{
	Encoder_cal cal_data;

    u8 if_online;
	int32_t filter_rate;											//�ٶ�
	double ecd_angle;											//�Ƕ�
	int16_t rate_rpm;

	double angle;
	double gyro;

	float Torque;
	u32 temperature;
	
	
}Encoder;



#endif

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);

void CAN_HT03_SendControlPara(CAN_TypeDef *CANx,uint32_t id,float f_p, float f_v, float f_kp, float f_kd, float f_t);
void HT03_START(CAN_TypeDef *CANx,uint32_t id);
void HT03_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);







#endif
