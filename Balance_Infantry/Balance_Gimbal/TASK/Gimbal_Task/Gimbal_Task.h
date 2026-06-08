#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "stm32f4xx.h"                  // Device header

#include "USART3.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "PID.h"

#define YAW_ANGLE_OFFSET         0     //要调


typedef enum
{
    GIMBAL_RELAX  = 0,           //云台失能
    GIMBAL_INIT = 1,             //云台初始化
    GIMBAL_REMOTE = 2,           //遥控
    GIMBAL_KEY_MOUSE = 3,        //键鼠
    GIMBAL_AUTO_AIM = 4,         //自瞄
    GIMBAL_BIG_BUFF = 5,         //大符
    GIMBAL_SMALL_BUFF = 6,       //小符
    GIMBAL_SENTRY = 7,           //被致盲使用哨兵模式
}Gimbal_Mode_e;


typedef struct
{
    float Yaw_Angle_Ref;//角度参考值
	float Yaw_Angle_Fdb;//角度反馈值
	float Yaw_Angle_To_Chassis;//相对底盘的yaw轴角度，范围:-180~180
    float Yaw_Speed_Ref;//速度参考值
 	float Yaw_Speed_Fdb;//速度反馈值
    
    float Pitch_Angle_Ref;//角度参考值
	float Pitch_Angle_Fdb;//角度反馈值
    float Pitch_Speed_Ref;//速度参考值
	float Pitch_Speed_Fdb;//速度反馈值
    
    Gimbal_Mode_e   Remote_Gimbal_Mode;
    Gimbal_Mode_e   Last_Remote_Gimbal_Mode;
    Gimbal_Mode_e	Gimbal_Mode;//云台模式
	Gimbal_Mode_e	Gimbal_Mode_Last;//上一次云台模式
    CH040DATA_t     CH040_Data;//陀螺仪数据结构体
    Encoder_t       Yaw_Motor_Encoder;//Yaw电机编码器
    Encoder_t       Pitch_Motor_Encoder;//Pitch电机编码器
    
    PID_t          Pitch_Motor_Speed_PID;
    PID_t          Pitch_Motor_Angle_PID;
    PID_t          Yaw_Motor_Angle_PID;
    PID_t          Yaw_Motor_Speed_PID;
    PID_t          Yaw_Motor_Init_Angle_PID;//初始化闭环编码器，正常情况闭陀螺仪，参数不同
    PID_t          Yaw_Motor_Init_Speed_PID;
    int16_t        Pitch_Motor_Set_Current;//6020 pitch转矩电流值
    float          Yaw_Motor_Set_T;//DM4310力矩值
    uint8_t        Init_Finish_Flag;//初始化完成标志位
}Gimbal_t;


extern Gimbal_t Gimbal;


float Normalize_Angle_PI(float angle);
void Gimbal_Mode_Select(void);
void Gimbal_Reference_Update(void);
void Gimbal_Feedback_Update(void);
void Gimbal_Relax_Handle(void);
void Gimbal_Init_Handle(void);
void Gimbal_Remote_Handle(void);
void Gimbal_Control_Loop(void);
void Gimbal_Task(void);

#endif
