#ifndef __BOARD_CONNECTED_TASK_H
#define __BOARD_CONNECTED_TASK_H
#include <stm32f4xx.h>
#include "Remote_Task.h"



#define GIMBAL_DATA_LENGTH     25
#define CHASSIS_RECEIVE_LENGTH 32



typedef __packed struct
{
	uint8_t Chassis_Mode;//底盘模式
	float Yaw_Encoder_Angle;//yaw轴电机编码器角度
	float Cmd_Leg_Length;//命令腿长
    float V_y;
	float V_x;


	int16_t Jump_Height;//跳跃高度

	uint8_t remote_online_flag;
	uint8_t fric_wheel_run;

	uint8_t UI_auto_aim_state;
	uint8_t Gimbal_Init_Finish_Flag;
	float leg_single_angle_handle_left;
	float leg_single_angle_handle_right;
	uint8_t fn_2_trigger_flag;

}  USART_Chassis_Data_t;//底盘接收的云台数据



typedef __packed struct
{
	uint16_t shooter_id1_17mm_cooling_heat;//第一个17mm发射机构的射击热量
	uint16_t shooter_barrel_heat_limit;//射击热量上限
	uint16_t shooter_barrel_cooling_value;  //机器人射击热量每秒冷却值
	float    bullet_speed_x_hat;//没用
	float    bullet_speed;
	uint8_t  robot_level;
	uint8_t  power_management_chassis_output;
	uint16_t current_HP;
    uint8_t  robot_id;
	uint8_t  Gimbal_Init_Cmd;
    float Jump_Finish_Flag;
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
					   uint8_t  Gimbal_Init_Cmd,
                       float  remain_heat,
					   uint8_t  game_state,USART_Gimbal_Data_t* USART_Gimbal_Data);
					   
                       
void usart_chassis_receive(uint8_t *DataAddress,USART_Chassis_Data_t* USART_Chassis_Data);
uint8_t gimbal_control_online_detective(void);				   
void Remote_DT7_To_USART_Chassis_Data(Remote_DT7_t* Remote,USART_Chassis_Data_t* USART_Chassis_Data);


#endif
                       
