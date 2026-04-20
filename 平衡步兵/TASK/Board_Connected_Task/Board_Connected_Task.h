#ifndef __BOARD_CONNECTED_TASK_H
#define __BOARD_CONNECTED_TASK_H
#include <stm32f4xx.h>

#define GIMBAL_SEND_DATA_LENGTH  22

typedef struct
{
	u8 Follow_Gimbal_Cmd;//是否跟随云台
	u8 Jump_Cmd;//跳跃命令
	u8 Overstep_Cmd;//上台阶命令
	u8 Chassis_Mode;//底盘控制模式
	float Yaw_Encoder_Ecd_Angle;//yaw轴电机编码器角度
	float Cmd_Leg_Length;//命令腿长
	float V_x;
	float V_y;
    float Omega;
	float Roll;
	int16_t Rotate_Rpeed;//小陀螺速度
	u8 Ctrl_Mode;//控制模式
	u8 Remote_Online_Flag;
	u8 Fric_Wheel_Run;
	u8 Rollover_Posture_Cmd;
	u8 low_speed_cmd;//有点问题
	u8 UI_auto_aim_state;
	u8 gimbal_data_if_finish_Init;
	float leg_single_angle_handle_left;
	float leg_single_angle_handle_right;
	u8 fn_2_trigger_flag;
	u8 lock_shoot_check;

} USART_Chassis_Data_t;//底盘接收的云台数据

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
//	int16_t remain_heat;//剩余热量这里是老代码里Judeg_System里手动计算的，这里先注释掉
	uint8_t game_state;
	
} usart_gimbal_data_t;//发送给云台的数据


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
					   uint8_t  game_state,usart_gimbal_data_t* usart_gimbal_data);
					   
                       
void usart_gimbal_receive(usart_gimbal_data_t *data,uint8_t *DataAddress);	
void gimbal_control_online_detective(void);					   
//extern USART_Chassis_Data_t usart_chassis_data;
//extern usart_gimbal_data_t usart_gimbal_data;	
//extern u8 gimbal_control_state;	
//extern u8 gimbal_control_state_longtime;



#endif
