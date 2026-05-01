#ifndef __LK_NEW_H__
#define __LK_NEW_H__

typedef struct
{
    int32_t ecd_bias;//编码器的初始值
    
    int32_t raw_value;
    int32_t last_raw_value;
    int32_t diff;
    
    int32_t ecd_value;//经过处理后的编码器的连续值,范围为正负无穷大
    
    int32_t ecd_raw_rate;//处理后得到的单圈原始值
    
    int32_t round_cnt;//圈数记录值
    
    int32_t rate_buffer[6];
    int32_t buffer_cnt;
    
    int32_t can_cnt;
    
}Encoder_cal_plus;

typedef struct
{
    Encoder_cal_plus encoder_data;
    
    int32_t filter_rate;//滤波后速率,实际为直接从报文解析出来的数据
    double ecd_angle;//编码器角度值
    
    double angle;//编码器转换后角度值
    int16_t rate_rpm;//最终转速
    
    float torque;//转矩电流值
    uint32_t temperature;//温度
    
    int32_t single_angle;
    
}Encoder_plus;

typedef enum
{
	Vol_Normal=0,
	Vol_Low,
}Voltage_State_e;

typedef enum
{
	Temp_Normal =0 ,
	Over_Temp,
}Temp_State_e;



typedef struct
{
	int16_t ID;
	
	uint8_t Angle_Kp;
	uint8_t Angle_Ki;
	uint8_t Speed_Kp;
	uint8_t Speed_Ki;
	uint8_t Iq_Kp;//转矩pid参数
	uint8_t Iq_Ki;
	
	float Accel;//加速度
	int16_t LK_Speed;//电机速度
	float Power;//功率
	
	float T;//转矩
	
	
	
	int16_t Encoder;//减去零偏后的编码器值
	int16_t Encoder_raw;//原始编码器值
	int16_t Encoder_Offset;//编码器零偏
	
	float Motor_Angle;//电机多圈角度
	float Circle_Angle;//电机单圈角度
	
	float Temp;//电机温度1°C/LSB
	float Voltage;//电压0.1V/LSB
	
	Temp_State_e Temp_State;//温度状态
	Voltage_State_e	Voltage_State;//电压状态
	
	
}LK_M_t;

void LK_task(Encoder_plus *v, CanRxMsg * msg,int offset);
void LK_Encoder_Process(Encoder_plus *v, CanRxMsg * msg);
void LK_ENABLE_RX(CAN_TypeDef *CANx,uint32_t id);
void LK_Read_Motor_State_2_Request(CAN_TypeDef *CANx,int16_t id);
void LK_M_Data_Process(CanRxMsg *msg,int16_t id,LK_M_t* LK_M);
void LK_M_Read_MulAngle_Request(CAN_TypeDef *CANx,int16_t id);


#endif

