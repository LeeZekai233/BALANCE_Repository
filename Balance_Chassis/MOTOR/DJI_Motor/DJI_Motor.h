#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#include <stm32f4xx.h>

/********************DJI Encoder******************************/
#define RATE_BUF_SIZE 6
#define M3508_ENCODER_TO_ANGLE 0.04394531f
#define RPM_TO_RAD_PER_SEC 0.10472f                  //轮子减速比16.875
#define RPM_TO_WHEEL_RAD_PER_SED      0.006206f      //电机反馈转速 rpm 到轮子角速度 rad/s的转换系数
#define M3508_CURRENT_TO_WHEEL_TORQUE 5.0625f        //反馈电流到驱动轮力矩的转换系数
#define M3508_ENCODER_TO_WHEEL        0.0592592f        //电机角度到驱动轮角度的系数
#define M3508_TORQUE_TO_IQ            3060.2f      //设定驱动轮力矩到M3508控制电流值
#define M3508_CURRETN_TO_TORQUE       0.3f

#define REDUCTION_RATIO_16            

typedef struct 
{
	 
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
	int32_t round_cnt;										//圈数
	int32_t can_cnt;					//记录函数的使用次数，在电机初始完成部分任务	
    int32_t heart_cnt;
	
}Encoder_cal;



typedef struct{
	Encoder_cal cal_data;

    uint8_t online_flag;
	int32_t filter_rate;											//速度,滤波用
	double ecd_angle;											    //角度,单位°
	int16_t rate_rpm;                                               //速度
	double angle;          //暂时不用                              
	double speed;           //暂时不用

	float Torque;    //暂时不用
	u32 temperature;
    float currtent;//转矩电流
	
}DJI_Encoder_t;
	



#ifndef GENERIC_ENCODER
#define GENERIC_ENCODER

typedef struct
{
    
    float Angle_Deg_fdb;	         //单圈角度反馈   单位°
    float Angle_Deg_Total_fdb;       //多圈角度反馈   单位°
    
    float Angle_Rad_fdb;             //单圈角度反馈   单位rad
    float Angle_Rad_Total_fdb;       //多圈角度反馈   单位rad
    
    float Omega_Rad_fdb;		//电机转速反馈  单位rad/s
    
    uint8_t online_flag;
    
    float Torque;             //力矩
    
    uint32_t temperature;    //温度
    
    uint32_t heart_cnt;//
}Encoder_t;//通用编码器

#endif



extern DJI_Encoder_t Driving_M3508[2];


void GetEncoderBias(volatile DJI_Encoder_t *v, CanRxMsg * msg);
void EncoderProcess(volatile DJI_Encoder_t *v, CanRxMsg * msg);
void GM6020EncoderProcess(volatile DJI_Encoder_t *v, CanRxMsg * msg);
void M3508orM2006EncoderTask(volatile DJI_Encoder_t *v, CanRxMsg * msg);
void GM6020EncoderTask(volatile DJI_Encoder_t *v, CanRxMsg * msg,int offset);
void Set_GM6020_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
void Set_GM6020_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6iq, int16_t motor7_iq, int16_t motor8_iq);
void Set_C620andC610_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
void Set_C620andC610_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq);
void M3508_Encoder_To_Generic_Encoder(DJI_Encoder_t* DJI_Encoder,Encoder_t* Encoder);
#endif
