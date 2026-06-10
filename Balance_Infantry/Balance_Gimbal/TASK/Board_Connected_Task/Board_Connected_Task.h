#ifndef __BOARD_CONNECTED_TASK_H
#define __BOARD_CONNECTED_TASK_H
#include "stm32f4xx.h"                  // Device header


typedef __packed struct
{
	uint8_t if_follow_gim;//是否跟随云台
	uint8_t jump_cmd;//跳跃命令
	uint8_t overstep_cmd;//上台阶命令
	uint8_t Chassis_Mode;//底盘模式
	float Yaw_Encoder_Angle;//yaw轴电机编码器角度
	float Cmd_Leg_Length;//命令腿长
	float V_x;
	float V_y;
	float roll;
	int16_t rotate_speed;//小陀螺速度
	uint8_t Control_Mode;//控制模式 //暂时不用
	uint8_t remote_online_flag;
	uint8_t fric_wheel_run;
	uint8_t Rollover_posture_cmd;
	uint8_t low_speed_cmd;
	uint8_t UI_auto_aim_state;  //自瞄在不在
	uint8_t Gimbal_Init_Finish_Flag;//原gimbal_data_if_finish_Init，
	float leg_single_angle_handle_left;
	float leg_single_angle_handle_right;
	uint8_t fn_2_trigger_flag;
	uint8_t lock_shoot_check;
}  USART_Chassis_Data_t;//云台发送的底盘数据



typedef __packed struct
{
	uint16_t shooter_id1_17mm_cooling_heat;//第1个17mm发射机构的射击热量 
	uint16_t shooter_barrel_heat_limit;//射击热量上限
	uint16_t shooter_barrel_cooling_value;  //机器人射击热量每秒冷却值
	float    bullet_speed_x_hat;
	float    bullet_speed;
	uint8_t  robot_level;
	uint8_t  power_management_chassis_output;
	uint16_t current_HP;
    uint8_t  robot_id;
	uint8_t  Gimbal_Init_Cmd;
    float remain_heat;
//	int16_t remain_heat;//剩余热量这里是老代码里Judeg_System里手动计算的，这里先注释掉
	uint8_t game_state;
	
} USART_Gimbal_Data_t;//接收的底盘数据


extern USART_Chassis_Data_t USART_Chassis_Data;
extern USART_Gimbal_Data_t USART_Gimbal_Data;


void USART_Gimbal_Receive(uint8_t *DataAddress,USART_Gimbal_Data_t* USART_Gimbal_Data);
void USART_Chassis_Send(USART_Chassis_Data_t *data);

    
    
#endif
