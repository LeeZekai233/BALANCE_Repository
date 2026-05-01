#ifndef __RADAR_
#define __RADAR_

#include "public.h"

#define AUTO_StdID_Visual_Need_1  0x300
#define AUTO_StdID_Visual_Need_2  0x301

#define AUTO_StdID_Visual_Fdb_1  0x302
#define AUTO_StdID_Visual_Fdb_2  0x303
#define AUTO_StdID_Visual_Fdb_3  0x304
#define AUTO_StdID_Visual_Fdb_4  0x305

///***********************************autoshoot****************************************/

typedef struct
{
	float Yaw_Angle;//目标yaw轴位置
	float Pitch_Angle;//目标Pitch轴位置
	
//	float Yaw_Angle_Last;//上一时刻目标yaw轴位置
//	float Pitch_Angle_Last;//上一时刻Pitch轴位置
//	
//	uint8_t enable_shoot;
//	
//	uint8_t  Flag_Get_Target;//目标锁定标志位，1：锁定目标，0：未识别到目标
//	uint16_t Lost_Cnt;//目标丢失计数器
}Auto_Snipe_Aim_t;



//typedef struct
//{
//	float Yaw_Angle;//目标yaw轴位置
//	float Pitch_Angle;//目标Pitch轴位置
//	
//	float Yaw_Delta_Point;
//	float Pitch_Delta_Point;
//	
//	float Yaw_Angle_Last;//上一时刻目标yaw轴位置
//	float Pitch_Angle_Last;//上一时刻Pitch轴位置
//	
//	float Yaw_Speed;//目标yaw轴速度
//	float Pitch_Speed;//目标pitch轴速度
//	
//	uint8_t  Flag_Get_Target;//目标锁定标志位，1：锁定目标，0：未识别到目标
//	uint16_t Lost_Cnt;//目标丢失计数器
//	
//	uint16_t xy_o_time;
//	u8 xy_0_flag;
//	u8 buff_kf_flag;
//	
//}Buff_t;


typedef struct
{
	Auto_Snipe_Aim_t Auto_Aim;

}Auto_Snipe_t;

typedef __packed struct
{
	uint8_t Header;
	
	float Yaw_Angle; 
	float Pitch_Angle;
//	float buff_X;
//	float buff_Y;
//	uint8_t enable_shoot;
//  uint8_t if_receive_data;
	uint16_t Check_Sum;
//	uint8_t Tail;
}New_Auto_Snipe_Aim_t;

typedef __packed struct
{
	float Pitch;
	float Yaw;
//	float Roll;
	float Shoot_Speed;
//  uint8_t  mode;
//	uint16_t Check_Sum;
	uint8_t Move_Flag;
	
}New_Auto_Snipe_Aim_Send_t;	

//typedef struct
//{
//	float Yaw_Angle;
//	float Pitch_Angle;
//	float Roll_Angle;
//	uint8_t Robot_ID;
//	uint8_t Scan_Flag;
//	uint8_t Another_Priority;
//}Visual_Data_Need_t;

extern Auto_Snipe_t My_Auto_Snipe;
extern New_Auto_Snipe_Aim_t New_Auto_Snipe_Aim;
extern New_Auto_Snipe_Aim_Send_t New_Auto_Snipe_Aim_Send;



void Radar_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Snipe_t *Auto_Shoot);
void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, uint8_t *data);
void Send_Radar(float Yaw, float Pitch, float Roll, int id, float ammo_speed, uint8_t mode, u8* data);



#endif



