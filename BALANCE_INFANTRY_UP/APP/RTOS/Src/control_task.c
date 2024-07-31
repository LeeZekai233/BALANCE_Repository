#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;
	  IWDG_ReloadCounter();            //喂狗

		if(time_tick%10 == 0)
		usart_chassis_send(chassis.follow_gimbal,
							RC_CtrlData.Key_Flag.Key_V_Flag ,
							chassis.ctrl_mode,
							yaw_Encoder.ecd_angle,
							leg_length,
							chassis.ChassisSpeed_Ref.left_right_ref,
							chassis.ChassisSpeed_Ref.forward_back_ref,
							chassis.ChassisSpeed_Ref.rotate_ref,
                            chassis.roll*PI/180.0f,
							judge_rece_mesg.power_heat_data.chassis_power,
							judge_rece_mesg.power_heat_data.chassis_power_buffer,
							judge_rece_mesg.game_robot_state.chassis_power_limit,
							RC_CtrlData.Key_Flag.Key_G_TFlag
                            );
	
		
		
	if(time_tick%2 == 0)
	{
		 shoot_task();
			gimbal_task();
			can_bus_send_task();
	}

	if(time_tick%4 == 0)
	{
        
		send_protocol_New(gimbal_gyro.yaw_Angle,gimbal_gyro.pitch_Angle,gimbal_gyro.roll_Angle,judge_rece_mesg.game_robot_state.robot_id,judge_rece_mesg.shoot_data.bullet_speed,gimbal_data.vision_mode,usart4.TX_BUFF);
        
    }
	
	  if(time_tick%100 == 0) //上传用户信息
    {
      Client_Send_Handle();
    }
    
    if(time_tick%100 == 50) //上传用户信息
    {
      Send_bullet_remaining_num();
    }
		
		if(time_tick%1000==0)
		{
			LED0_ON;
			HARD_WDG_ON;
		}
		if(time_tick%2000==0)
		{
			LED0_OFF;
			HARD_WDG_OFF;
		}
        
        online_detective(&general_friction.left_motor);
        online_detective(&general_friction.right_motor);
}

void control_task_Init(void)
{
		gimbal_parameter_Init();
	shot_param_init();
}
