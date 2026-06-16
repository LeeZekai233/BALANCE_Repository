#ifndef __AUTO_SHOOT_TASK_H
#define __AUTO_SHOOT_TASK_H
#include "stm32f4xx.h"                  // Device header




/*接收*/
typedef struct
{
	float Yaw_Angle;//目标yaw轴位置
	float Pitch_Angle;//目标Pitch轴位置
	
	float Yaw_Angle_Last;//上一时刻目标yaw轴位置
	float Pitch_Angle_Last;//上一时刻Pitch轴位置
	
	uint8_t Enable_Shoot;//是否击打标志位
	uint8_t Link_State;//1:串口正常，2：识别到
	
	uint8_t Flag_Get_Target;//目标锁定标志位，1：锁定目标，0：未识别到目标
	uint16_t Lost_Cnt;//目标丢失计数器
	
}Auto_Aim_t;



typedef struct
{
	float Yaw_Angle;//目标yaw轴位置
	float Pitch_Angle;//目标Pitch轴位置
	
	float Yaw_Angle_Last;//上一时刻目标yaw轴位置
	float Pitch_Angle_Last;//上一时刻Pitch轴位置
	
	uint8_t Enable_Shoot;//是否击打标志位
	
	float Yaw_Speed;//目标yaw轴速度
	float Pitch_Speed;//目标pitch轴速度
	
	uint8_t 	Flag_Get_Target;//目标锁定标志位，1：锁定目标，0：未识别到目标
	uint16_t Lost_Cnt;//目标丢失计数器

 uint8_t Shoot_flag;
 uint8_t Last_Shoot_flag;

	float Buff_Shoot_Delay;	

	
}Buff_t;


typedef struct
{
	Auto_Aim_t Auto_Aim;
	Buff_t Buff;
}Auto_Shoot_t;
/*接收*/

typedef __packed struct
{
	uint8_t Header;
	
	float Pitch_Angle;
	float Yaw_Angle;
	uint8_t Enable_Shoot;
	uint8_t Link_State;//1：串口正常 2：识别到
	float Buff_Shoot_Delay;	
	
	u16 Check_Sum;
	uint8_t Tail;
}New_Auto_Aim_t;//中转结构体

typedef __packed struct
{
	float Pitch;
	float Yaw;
	float Roll;
	float Shoot_Speed;
	uint8_t Current_Color;
	uint8_t Mode;
	uint8_t Game_State;
	uint8_t Poke_State;
	float Shoot_Frequency;
}New_Auto_Aim_Send_t;	


extern Auto_Shoot_t My_Auto_Shoot;
extern Auto_Shoot_t Othter_Auto_Shoot;
extern New_Auto_Aim_t New_Auto_Aim;
extern New_Auto_Aim_Send_t New_Auto_Aim_Send;

void Vision_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Shoot_t *Auto_Shoot);
void send_protocol_New(float Yaw, float Pitch, float Roll,float Speed,uint8_t ID, uint8_t* data);


#endif


