#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;


		if(time_tick%10 == 0)
		usart_chassis_send(chassis.follow_gimbal,
							chassis.chassis_speed_mode ,
							chassis.ctrl_mode,
							yaw_Encoder.ecd_angle,
							leg_length,
							chassis.ChassisSpeed_Ref.left_right_ref,
							chassis.ChassisSpeed_Ref.forward_back_ref,
							chassis.ChassisSpeed_Ref.rotate_ref,
							judge_rece_mesg.power_heat_data.chassis_power,
							judge_rece_mesg.power_heat_data.chassis_power_buffer,
							judge_rece_mesg.game_robot_state.chassis_power_limit);
	
		
		
	if(time_tick%2 == 0)
	{
		 shoot_task();
			gimbal_task();
			can_bus_send_task();
	}
	if(time_tick%3 == 0)
	{
		send_protocol(gimbal_gyro.yaw_Angle,gimbal_gyro.pitch_Angle,gimbal_gyro.roll_Angle,judge_rece_mesg.game_robot_state.robot_id,24,0,UART4_DMA_TX_BUF);
	}
}

void control_task_Init(void)
{
		gimbal_parameter_Init();
	shot_param_init();
}
