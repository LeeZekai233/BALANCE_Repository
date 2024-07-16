#ifndef __LK_TECH_H
#define __LK_TECH_H
#include "public.h"

typedef struct{
	
	uint8_t anglekp;
	uint8_t angleki;
	uint8_t speedkp;
	uint8_t speedki;
	uint8_t torquekp;
	uint8_t torqueki;
	
}PID9015Typedefine;

#ifndef STRUCT_MOTOR
#define STRUCT_MOTOR

#define RATE_BUF_SIZE 6
typedef struct 
{
	int32_t raw_value;   									//����������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //���������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t can_cnt;					//��¼������ʹ�ô������ڵ����ʼ��ɲ�������
    int32_t heart_cnt;
    
    float raw_angle_val;
    float last_raw_angle_val;
    float angle_diff;
}Encoder_cal;


typedef struct{
	Encoder_cal cal_data;

    u8 if_online;                        //����״̬                       
	int32_t filter_rate;				//dps
	double ecd_angle;					//��
	int16_t rate_rpm;                   //rpm

	double angle;                       //����
	double gyro;                        //���ٶ�

	float Torque;                       //����
	u32 temperature;
	
	
}Encoder;
	





#endif

void MF_18bit_EncoderProcess(volatile Encoder *v, CanRxMsg * msg,float Torque_Constant);//��̨yaw��pitch����
void MF_18bit_EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset,float Torque_Constant);
void CAN_MF_single_torsionControl(CAN_TypeDef *CANx ,float torque,uint32_t id,float Torque_Constant);
void CAN_MF_multiy_torsionControl(CAN_TypeDef *CANx ,float Torque_Constant,float torque1,float torque2,float torque3,float torque4);

void MG_18bit_EncoderProcess(volatile Encoder *v, CanRxMsg * msg,float Torque_Constant);
void MG_18bit_EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset,float Torque_Constant);
void CAN_MG_single_torsionControl(CAN_TypeDef *CANx ,float torque,uint32_t id,float Torque_Constant);
void CAN_MG_multiy_torsionControl(CAN_TypeDef *CANx ,float Torque_Constant,float torque1,float torque2,float torque3,float torque4);

void CAN_LK_TechCommand(CAN_TypeDef *CANx ,uint8_t command,uint32_t id);
void CAN_LK_TechsetpidCommand(CAN_TypeDef *CANx, float akp,
                           float aki,
                           float skp,
                           float ski,
                           float iqkp,
                           float iqki, uint32_t id);
void CAN_LK_TechangleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl,uint32_t id);
void CAN_LK_TechspeedControl(CAN_TypeDef *CANx ,uint32_t speedControl,uint32_t id);






#endif