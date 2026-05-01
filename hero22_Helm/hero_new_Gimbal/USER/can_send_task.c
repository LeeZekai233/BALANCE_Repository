#include "public.h"

extern chassis_t 	chassis;
extern uint32_t time_tick;
extern _42mm_shoot_t _42mm_shoot;
extern SuperCap_Send_t Super_Cap_Send;
int16_t test_current;

void can_send(void)
{
    //控制云台Yaw
		CAN_LK_MG5010_iqControl(CAN2,yaw_sys_input,GIMBAL_YAW_MOTOR);
// 测试通信专用   CAN2_Chassis_Transmit(8.7,-5.56,6.28,CHASSIS_REVERSE_ROTATE,NORMAL_SPEED_MODE,REMOTE_INPUT,114,514);

      //控制发射机构
      if(time_tick%2==0)
      {   
		  //摩擦轮
          Set_C620andC610_IQ2(CAN1,_42mm_shoot.down_friction_current[0],
                                   _42mm_shoot.right_friction_current[0],
/*Pitch和摩擦轮一起发，降低总线占用，提高稳定性*/-gimbal_data.gim_ref_and_fdb.pitch_motor_input,gimbal_data.gim_ref_and_fdb.scope_motor_input);
		  
		//拨盘电机
		Set_C620andC610_IQ2(CAN2,_42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input,0,0,0);
		  

      }
	  else if(time_tick%2==1)
	  {
		  //摩擦轮
		Set_C620andC610_IQ1(CAN1,_42mm_shoot.left_friction_current[1],
                                 _42mm_shoot.down_friction_current[1],
                                 _42mm_shoot.right_friction_current[1],
                                 _42mm_shoot.left_friction_current[0]);
		      //控制底盘

    CAN2_Chassis_Transmit(chassis.Vcx,chassis.Vcy,chassis.Vw,chassis.ctrl_mode,chassis.chassis_speed_mode,chassis.Chassis_Move_State,RC_CtrlData.inputmode);

	  }
    //超颠
    if(time_tick%10==0)
    {
	  CAN_POWER_Control(CAN2,&Super_Cap_Send);
    }
    
    
}