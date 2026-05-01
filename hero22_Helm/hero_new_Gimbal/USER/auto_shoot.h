#ifndef __AUTO_SHOOT_H__
#define __AUTO_SHOOT_H__

typedef struct
{
	float Yaw_Angle;//目标yaw轴位置
	float Pitch_Angle;//目标Pitch轴位置
	
	float Yaw_Angle_Last;//上一时刻目标yaw轴位置
	float Pitch_Angle_Last;//上一时刻Pitch轴位置
	
	uint8_t enable_shoot;
	
	uint8_t  Flag_Get_Target;//目标锁定标志位，1：锁定目标，0：未识别到目标
	uint16_t Lost_Cnt;//目标丢失计数器
}Auto_Aim_t;


typedef struct
{
	float Yaw_Angle;//目标yaw轴位置
	float Pitch_Angle;//目标Pitch轴位置
	
	float Yaw_Angle_Last;//上一时刻目标yaw轴位置
	float Pitch_Angle_Last;//上一时刻Pitch轴位置
	
	u8 Enable_Shoot;//是否击打标志位
	
	float Yaw_Speed;//目标yaw轴速度
	float Pitch_Speed;//目标pitch轴速度
	
	u8 	Flag_Get_Target;//目标锁定标志位，1：锁定目标，0：未识别到目标
	u16 Lost_Cnt;//目标丢失计数器
	
 u8 Shoot_flag;
 u8 Last_Shoot_flag;

}Buff_t;



typedef struct
{
	Auto_Aim_t Auto_Aim;
	Buff_t Buff;
}Auto_Shoot_t;

typedef __packed struct
{
	uint8_t Header;
	float Pitch_Angle;
	float Yaw_Angle; 
	uint8_t enable_shoot;
	uint8_t state;			//1 正常		2 锁定
	
	uint16_t Check_Sum;
	uint8_t Tail;
}New_Auto_Aim_t;

typedef __packed struct
{
	float Pitch;
	float Yaw;
	float Roll;
	float Shoot_Speed;
	uint8_t 	Current_Color;//蓝：1，红0；
	uint8_t  mode;
	uint8_t game_state;
	uint8_t poke_state;
	float shoot_freqency;
}New_Auto_Aim_Send_t;	

typedef struct
{
	float Yaw_Angle;
	float Pitch_Angle;
	float Roll_Angle;
	uint8_t Robot_ID;
	uint8_t Scan_Flag;
	uint8_t Another_Priority;
}Visual_Data_Need_t;

extern Auto_Shoot_t My_Auto_Shoot;
extern New_Auto_Aim_t New_Auto_Aim;
extern New_Auto_Aim_Send_t New_Auto_Aim_Send;

void Vision_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Shoot_t *Auto_Shoot);
void send_protocol_New(float Yaw, float Pitch, float Roll, int id, float ammo_speed, uint8_t mode, u8* data);

#endif