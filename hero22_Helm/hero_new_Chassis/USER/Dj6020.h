#ifndef __DJ6020_H__
#define __DJ6020_H__

#define RATE_BUF_SIZE   6

#define GM6020_rx_id1  0x205
#define GM6020_rx_id2  0x206
#define GM6020_rx_id3  0x207
#define GM6020_rx_id4  0x208
#define GM6020_rx_id5  0x209
#define GM6020_rx_id6  0x20A
#define GM6020_rx_id7  0x20B
#define GM6020_tx_U_id1_to_id4  0x1FF
#define GM6020_tx_U_id5_to_id6  0x2FF
#define GM6020_tx_V_id1_to_id4  0x1FE
#define GM6020_tx_V_id5_to_id6  0x2FE

#define C620_3508_tx_id     0x200

#define C620_3508_rx_id1     0x201
#define C620_3508_rx_id2     0x202
#define C620_3508_rx_id3     0x203
#define C620_3508_rx_id4     0x204
#define C620_3508_rx_id5     0x205
#define C620_3508_rx_id6     0x206
#define C620_3508_rx_id7     0x207



#define PITCH_3508  0x207

//typedef struct
//{
//    uint16_t raw_angle;
//    uint16_t raw_speed;
//    uint16_t torque_current;
//    
//    
//    
//}GM6020_Encoder;

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

//    u8 if_online;
	int32_t filter_rate;											//速度
	double ecd_angle;											//角度
	int16_t rate_rpm;
	
	double angle;
	double gyro;

	int16_t Torque;
	uint32_t temperature;
	
	double Init_Angle;
}Encoder;


void Set_dj6020_iq(CAN_TypeDef *CANx,int16_t motor1_iq,int16_t motor2_iq,int16_t motor3_iq,int16_t motor4_iq);
void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void GM6020_PID_task_Init(Encoder *V);
//void GM6020EncoderTask(volatile Encoder *v, CanRxMsg * msg,int offset);
void M3508orM2006EncoderTask(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void C620_3508_PID_Init(void);
void Set_C620andC610_IQ1(CAN_TypeDef *CANx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq);
void Set_C620andC610_IQ2(CAN_TypeDef *CANx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq);

#endif

extern Encoder V;
