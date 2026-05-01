#ifndef __LK_MG5010_H__
#define __LK_MG5010_H__

typedef struct{
	
	uint8_t anglekp;
	uint8_t angleki;
	uint8_t speedkp;
	uint8_t speedki;
	uint8_t torquekp;
	uint8_t torqueki;
	
}PID9015Typedefine;

#define RATE_BUF_SIZE 6
//typedef struct 
//{
//	
//	int32_t raw_value;   									//编码器不经处理的原始值
//	int32_t last_raw_value;								//上一次的编码器原始值
//	int32_t ecd_value;                       //经过处理后连续的编码器值
//	int32_t diff;													//两次编码器之间的差值
//	int32_t temp_count;                   //计数用
//	uint8_t buf_count;								//滤波更新buf用
//	int32_t ecd_bias;											//初始编码器值	
//	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
//	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
//	int32_t round_cnt;										//圈数
//	int32_t can_cnt;					//记录函数的使用次数，在电机初始完成部分任务	
//    int32_t heart_cnt;
//	
//}Encoder_cal;

//typedef struct{
//	Encoder_cal cal_data;

////    u8 if_online;
//	int32_t filter_rate;											//速度
//	double ecd_angle;											//角度
//	int16_t rate_rpm;

//	double angle;
//	double gyro;

//	float Torque;
//	u32 temperature;
//	
//	
//}Encoder;


#endif

void MF_EncoderProcess(Encoder *v, CanRxMsg * msg);//云台yaw，pitch共用
void MF_EncoderTask(Encoder *v, CanRxMsg * msg,int offset);
void LK_MG5010Ecoder_Progress(Encoder *v, CanRxMsg * msg);
void CAN_LK_MG5010_iqControl(CAN_TypeDef *CANx,int16_t iqControl,uint32_t id);
void CAN_LK_MG5010_ERR_Clear(CAN_TypeDef *CANx,uint32_t id);
void LK_5010_pitch_pid_Init(void);


