#ifndef __BOARD_CONNECTED_TASK_H
#define __BOARD_CONNECTED_TASK_H
#include <stm32f4xx.h>
#include "Remote_Task.h"

#define GIMBAL_SEND_DATA_LENGTH  26

typedef struct
{
	u8 if_follow_gim;//是否跟随云台
	u8 jump_cmd;//跳跃命令
	u8 overstep_cmd;//上台阶命令
	u8 Chassis_Mode;//底盘模式
	float Yaw_Encoder_Angle;//yaw轴电机编码器角度
	float Cmd_Leg_Length;//命令腿长
	float V_x;
	float V_y;
	float roll;
	int16_t rotate_speed;//小陀螺速度
	u8 Control_Mode;//控制模式
	u8 remote_online_flag;
	u8 fric_wheel_run;
	u8 Rollover_posture_cmd;
	u8 low_speed_cmd;
	u8 UI_auto_aim_state;
	u8 gimbal_data_if_finish_Init;
	float leg_single_angle_handle_left;
	float leg_single_angle_handle_right;
	u8 fn_2_trigger_flag;
	u8 lock_shoot_check;

}  USART_Chassis_Data_t;//底盘接收的云台数据



typedef __packed struct
{
	uint16_t shooter_id1_17mm_cooling_heat;//
	uint16_t shooter_barrel_heat_limit;//射击热量上限
	uint16_t shooter_barrel_cooling_value;  //机器人射击热量每秒冷却值
	float    bullet_speed_x_hat;
	float    bullet_speed;
	uint8_t  robot_level;
	uint8_t  power_management_chassis_output;
	uint16_t current_HP;
    uint8_t  robot_id;
	uint8_t  allow_gimbal_init;
    float remain_heat;
//	int16_t remain_heat;//剩余热量这里是老代码里Judeg_System里手动计算的，这里先注释掉
	uint8_t game_state;
	
} USART_Gimbal_Data_t;//发送给云台的数据



extern uint32_t gimbal_control_online_heart_cnt;//云台心跳检测


//extern USART_Chassis_Data_t usart_chassis_data;
//extern USART_Gimbal_Data USART_Gimbal_Data;	
//extern u8 gimbal_control_state;	
//extern u8 gimbal_control_state_longtime;


void usart_gimbal_send(
					   uint16_t shooter_id1_17mm_cooling_heat,
	                   uint16_t shooter_barrel_heat_limit,
	                   uint16_t shooter_barrel_cooling_value,
	                   uint8_t  robot_level,
                       float    bullet_speed_x_hat,
					   float    bullet_speed,
                       uint8_t  power_management_chassis_output,
					   uint16_t current_HP,
					   uint8_t  robot_id,
					   uint8_t  allow_gimbal_init,
                       float  remain_heat,
					   uint8_t  game_state,USART_Gimbal_Data_t* USART_Gimbal_Data);
					   
                       
void usart_chassis_receive(uint8_t *DataAddress,USART_Chassis_Data_t* USART_Chassis_Data);
uint8_t gimbal_control_online_detective(void);				   
void Remote_DT7_To_USART_Chassis_Data(Remote_DT7_t* Remote,USART_Chassis_Data_t* USART_Chassis_Data);


#endif
                       
