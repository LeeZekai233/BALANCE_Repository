#include "public.h"
Remote_Type Remote_Type_e;
Remote_Type Remote_Type_e_Last;
extern chassis_t 	chassis;
extern int16_t wheel_rpm[4];
uint32_t time_tick=0;
int heat_tick;
extern _42mm_shoot_t _42mm_shoot;
extern uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

int flagg=0;
    
void control_task(void)
{
	//记录上一次遥控器类型
	Remote_Type_e_Last=Remote_Type_e;
	//心跳检测
	Peripheral_State_Judge(&Peripheral_State,0.005);
	if((Peripheral_State.Remote_Control.Link_State==CONNECTED &&Peripheral_State.VTM_Remote.Link_State==CONNECTED)
	 ||(Peripheral_State.Remote_Control.Link_State==DISCONNECT&&Peripheral_State.VTM_Remote.Link_State==DISCONNECT))
	{
		RC_CtrlData.inputmode=STOP;
	}
	else if(Peripheral_State.Remote_Control.Link_State==CONNECTED)
	{
		Remote_Type_e=DR16_Remote;
		SetInputMode(&RC_CtrlData);
		
	}
	else
	{
		Remote_Type_e=VTM_Remote;
		Set_Input_Mode_VTM();
		VTM_Switch_Action_Get();
	}
	
	if(Remote_Type_e!=Remote_Type_e_Last)
	{
		RC_CtrlData.RemoteSwitch.trigger=0;
	}
	
	Heat_Control();//热量限制 
    Hero_Mode_Select_Task();
    Gimbal_task();
    Chassis_ModeSelect();
    _42mm_Shoot_Task();
    
    
    if(time_tick % 2 == 0)
    {
        send_protocol_New(gimbal_gyro.yaw_angle , 
                          gimbal_gyro.pitch_angle , 
			  gimbal_gyro.roll_angle , 
			  judge_rece_mesg.game_robot_state.robot_id , 
			  judge_rece_mesg.shoot_data.initial_speed , gimbal_data.ctrl_mode , UART4_DMA_TX_BUF); 
	}
    	if(time_tick % 4  == 0)
	{
			Send_Radar(gimbal_gyro.yaw_angle , 
                      gimbal_gyro.pitch_angle , 
                      gimbal_gyro.roll_angle , 
                      judge_rece_mesg.game_robot_state.robot_id , 
                      judge_rece_mesg.shoot_data.initial_speed, gimbal_data.ctrl_mode , USART2_DMA_TX_BUF); 
		
		

	}
		if(RC_CtrlData.inputmode==STOP)
		{
			yaw_sys_input=0;
			gimbal_data.gim_ref_and_fdb.pitch_motor_input=0;
			gimbal_data.gim_ref_and_fdb.scope_motor_input=0;
			for(int i=0;i<4;i++)
            {
                chassis.current[i]=0;
            }
            _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=0;
            _42mm_shoot.left_friction_current[1]=0;
            _42mm_shoot.left_friction_current[0]=0;
            _42mm_shoot.right_friction_current[1]=0;
            _42mm_shoot.right_friction_current[0]=0;
            _42mm_shoot.down_friction_current[1]=0;
            _42mm_shoot.down_friction_current[0]=0;
			
		}
    
    can_send();
		
		if(time_tick%50==0)
		{
			Client_Send_Handle(); 	//向客户端发送数据
		}

    time_tick++;
    if(time_tick==100000)
    {
        time_tick=0;
//		CAN_LK_MG5010_ERR_Clear(CAN2,1);
    }
		
  
}