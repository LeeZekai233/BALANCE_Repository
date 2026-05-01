#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_conf.h"
#define GIMBAL_NORMAL_YAW_SENSITIVITY   0.00025
#define GIMBAL_NORMAL_PITCH_SENSITIVITY 0.00010 
#define GIMBAL_SNIPE_YAW_SENSITIVITY    0.00001f
#define GIMBAL_SNIPE_PITCH_SENSITIVITY  0.00001f
/**
  ******************************************************************************
																云台模式枚举,后面控制模式复杂之后再用
	 =============================================================================
 **/
typedef enum
{
  GIMBAL_RELAX         = 0,			//关控
	
  GIMBAL_INIT          = 1,			//初始化

  GIMBAL_FOLLOW_ZGYRO  = 3,			//跟随陀螺仪

  GIMBAL_AUTO_AIM	   = 4,				//自瞄

  GIMBAL_SNIPE    = 5,		//英雄吊射

  GIMBAL_NORMAL = 6,	//普通模式
	
  GIMBAL_RADAR_ASSISTANT_SNIPE = 7,	//雷达辅助吊射
	
  GIMBAL_MANUAL_SINPE = 8,
	
} gimbal_mode_e;

 /**
  ******************************************************************************
										云台输入与反馈结构体（输入，反馈，电机输出）
	 =============================================================================
 **/
typedef struct
{
  /* position loop */
  float yaw_angle_ref;				
  float pit_angle_ref;				
  float yaw_angle_fdb;				
  float pit_angle_fdb;
//  float small_gimbal_angle_ref;
//  float small_gimbal_angle_fdb;
  /* speed loop */
  float yaw_speed_ref;
  float pit_speed_ref;
  float yaw_speed_fdb;
  float pit_speed_fdb;
//  float small_gimbal_speed_ref;
//  float small_gimbal_speed_fdb;
  float scope_angle_Init;
  float scope_angle_ref;
  float scope_angle_fdb;
  float scope_speed_ref;
  float scope_speed_fdb;
  int16_t yaw_motor_input;
  int16_t pitch_motor_input;
//  int16_t small_pit_motor_input;
  int16_t scope_motor_input;
} gim_ref_and_fdb_t;

 /**
  ******************************************************************************
													云台外部控制信号输入结构体
	 =============================================================================
 **/
typedef struct 
{
  float pitch_angle_dynamic_ref;
  float yaw_angle_dynamic_ref;
  //float small_pitch_angle_dynamic_ref;
}gim_dynamic_ref_t;


 /**
  ******************************************************************************
													云台结构体
	 =============================================================================
 **/
typedef struct
{

  /* ctrl mode */
  gimbal_mode_e ctrl_mode;
  gimbal_mode_e last_ctrl_mode;
  
  gim_ref_and_fdb_t gim_ref_and_fdb;
  gim_ref_and_fdb_t gim_ref_and_fdb_last;
  gim_dynamic_ref_t gim_dynamic_ref;
	
    u8 if_finish_Init;		//初始化标志位，初始化完成后置1
	
  pid_t pid_init_yaw_Angle; 
  pid_t pid_init_pit_Angle; 
  
  pid_t pid_init_yaw_speed; 
  pid_t pid_init_pit_speed;
  

  pid_t pid_yaw_Angle; 
  pid_t pid_pit_Angle; 
  pid_t pid_yaw_speed; 
  pid_t pid_pit_speed;
//  pid_t pid_small_pit_Angle;
//  pid_t pid_small_pit_Speed;

  //英雄吊射模式下的参数
  pid_t pid_auto_yaw_Angle; 
  pid_t pid_auto_pit_Angle; 
  pid_t pid_auto_yaw_speed; 
  pid_t pid_auto_pit_speed; 
  //倍镜参数
  pid_t pid_scope_angle;
  pid_t pid_scope_speed;

  // 自瞄模式外环的参数
  pid_t pid_yaw_follow_1;
  pid_t pid_yaw_speed_follow_1;
  pid_t pid_yaw_follow_2;
  pid_t pid_yaw_speed_follow_2;
  pid_t pid_yaw_follow_3;
  pid_t pid_yaw_speed_follow_3;
  pid_t pid_yaw_follow_4;
  pid_t pid_yaw_speed_follow_4;
  
  pid_t pid_pit_follow_1;
  pid_t pid_pit_speed_follow_1;
  pid_t pid_pit_follow_2;
  pid_t pid_pit_speed_follow_2;
  pid_t pid_pit_follow_3;
  pid_t pid_pit_speed_follow_3;
  
		

}gimbal_t;

extern gimbal_t gimbal_data;
extern float yaw_sys_input;
extern uint8_t reversal_flag,reversing_flag;
extern float Follow_Angle_Medium;
extern Encoder Pitch_Encoder;
extern Encoder Scope_Encoder;

void Gimbal_parameter_Init(void);
void Gimbal_task(void);
void Gimbal_Follow_Gyro_Handle(void);
void Gimbal_Init_Handle(void);
void Gimbal_Snipe_Handle(void);
void Gimbal_AUTO_AIM_Handle(void);


#endif
