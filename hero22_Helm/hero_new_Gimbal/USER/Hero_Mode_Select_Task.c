#include "public.h"
extern RC_Ctl_t RC_CtrlData;
extern chassis_t chassis;
extern _42mm_shoot_t _42mm_shoot;
extern gimbal_t gimbal_data;
uint8_t friction_on_flag=0;
uint16_t friction_off_cnt=0;
uint16_t delay_fire_flag;
uint32_t delay_cnt=0;
uint8_t Auto_shoot_flag;
uint8_t rotate_cnt = 0;		//小陀螺正反转计数，达到最大值255后，会变为0，符合奇偶数交替的要求
uint8_t rotate_flag;
#define CHASSIS_SPEED 3
float chassis_speed = 0;	//底盘速度键鼠输入

void Hero_Mode_Select_Task(void)
{
    switch(RC_CtrlData.inputmode)
    {
        case REMOTE_INPUT:
			if(gimbal_data.if_finish_Init != 1)
			{
				chassis.ctrl_mode = CHASSIS_RELAX;
			}
			switch (Remote_Type_e)
			{
				case DR16_Remote :
				{
					if(RC_CtrlData.inputmode!=RC_CtrlData.inputmode_last)
					{
						gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
						gimbal_data.gim_ref_and_fdb.yaw_angle_ref=gimbal_data.gim_ref_and_fdb.yaw_angle_fdb;
					}
					else
					{
						if(RC_CtrlData.rc.s1==3)
						{
							chassis.ctrl_mode=AUTO_FOLLOW_GIMBAL;//底盘跟随云台
							RC_CtrlData.RemoteSwitch.s3to2=0;
							if(trigger2_flag == 1)
							{
								_42mm_shoot.friction_state = FRICTION_ON;
								_42mm_shoot.ctrl_mode = _42MM_SHOOT_NORMAL;
							}		
							else 
							{
								_42mm_shoot.friction_state =FRICTION_OFF;
								_42mm_shoot.ctrl_mode =_42MM_SHOOT_RELAX ;
							}
							
							if(trigger1_flag == 1&&_42mm_shoot.friction_state == FRICTION_ON)
							{
								_42mm_shoot.shoot_flag = 1;		//开火标志位将在发射完成后清除
								trigger1_flag = 0;		//清除ch4下拉标志位
							}
							gimbal_data.ctrl_mode=GIMBAL_FOLLOW_ZGYRO;
						}
						else if(RC_CtrlData.rc.s1==1)
						{
							gimbal_data.ctrl_mode=GIMBAL_RADAR_ASSISTANT_SNIPE;//停止以进入雷达辅助吊射（调试专用！）
							RC_CtrlData.RemoteSwitch.s3to1=0;
							if(trigger2_flag == 1)
							{
								_42mm_shoot.friction_state = FRICTION_ON;
								_42mm_shoot.ctrl_mode = _42MM_SHOOT_NORMAL;
							}		
							else 
							{
								_42mm_shoot.friction_state =FRICTION_OFF;
								_42mm_shoot.ctrl_mode =_42MM_SHOOT_RELAX ;
							}
							
							if(trigger1_flag == 1&&_42mm_shoot.friction_state == FRICTION_ON)
							{
								_42mm_shoot.shoot_flag = 1;		//开火标志位将在发射完成后清除
								trigger1_flag = 0;		//清除ch4下拉标志位
							}
							
						}
						else
						{
							gimbal_data.ctrl_mode=GIMBAL_FOLLOW_ZGYRO;
							switch(RC_CtrlData.RemoteSwitch.trigger)
							{
								case 0:
									chassis.ctrl_mode=AUTO_FOLLOW_GIMBAL;
								break;
								case 1://拨轮向右
									chassis.ctrl_mode=CHASSIS_ROTATE;
								break;
								case 2://拨轮向左
									chassis.ctrl_mode=CHASSIS_REVERSE_ROTATE;
								break;
								default:
								break;
							}
							trigger1_flag=0;
							trigger2_flag=0;
							trigger1_cnt=0;
							trigger2_cnt=0;
								
						}
					}
					/****************↑底盘↑*************/
					
					/****************↓云台↓*************/
					if(!gimbal_data.if_finish_Init)
					{
						//未初始化完成进入初始化模式
						gimbal_data.ctrl_mode =GIMBAL_INIT;
						
					}
					else
					{
						//初始化完成进入跟随陀螺仪模式
						if(gimbal_data.ctrl_mode == GIMBAL_INIT)
						gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
					}
					//进吊射关底盘***********************/
					if(	gimbal_data.ctrl_mode==GIMBAL_RADAR_ASSISTANT_SNIPE||
						gimbal_data.ctrl_mode==GIMBAL_SNIPE)
					{
						chassis.ctrl_mode=CHASSIS_STOP;
					}
				}
				break;
				
				case VTM_Remote:
				{
					if(RC_CtrlData.inputmode!=RC_CtrlData.inputmode_last)
					{
						gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
						gimbal_data.gim_ref_and_fdb.yaw_angle_ref=gimbal_data.gim_ref_and_fdb.yaw_angle_fdb;
					}
					
					if(pause_trigger_flag==0 && gimbal_data.ctrl_mode != GIMBAL_FOLLOW_ZGYRO)
					{
						gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
					}
					if(vtm_remote_data.pause==1)
					{
						fn_1_trigger_flag=0;
						
						fn_2_trigger_flag=0;
						trigger_flag=0;
						fn_1_cnt=0;
						fn_2_cnt=0;
						trigger_cnt=0;
					}
					
					if(fn_1_trigger_flag==0)
					{
						gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
					}
					
					if(fn_1_trigger_flag==1)
					{
						gimbal_data.ctrl_mode = GIMBAL_AUTO_AIM;
					}
					
					if(fn_2_trigger_flag==0&&fn_1_trigger_flag==0&&pause_trigger_flag==1)
					{
						gimbal_data.ctrl_mode = GIMBAL_SNIPE;
						chassis.ctrl_mode=CHASSIS_STOP;
					}
					else if(fn_2_trigger_flag==1&&fn_1_trigger_flag==0&&pause_trigger_flag==1)
					{
						gimbal_data.ctrl_mode = GIMBAL_RADAR_ASSISTANT_SNIPE;
						chassis.ctrl_mode=CHASSIS_STOP;
					}
					else
					{
						chassis.ctrl_mode=AUTO_FOLLOW_GIMBAL;
					}
					
					
					if(!gimbal_data.if_finish_Init)
					{
						//未初始化完成进入初始化模式
						gimbal_data.ctrl_mode =GIMBAL_INIT;
						
					}
					else
					{
						//初始化完成进入跟随陀螺仪模式
						if(gimbal_data.ctrl_mode == GIMBAL_INIT)
						gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
					}
					
					if(trigger_flag==1)
					{
						switch(RC_CtrlData.RemoteSwitch.trigger)
						{
							case 0:
								chassis.ctrl_mode=AUTO_FOLLOW_GIMBAL;
							break;
							case 1://拨轮向右
								chassis.ctrl_mode=CHASSIS_ROTATE;
							break;
							case 2://拨轮向左
								chassis.ctrl_mode=CHASSIS_REVERSE_ROTATE;
							break;
							default:
							break;
						}
						trigger1_cnt=0;
						trigger2_cnt=0;
						trigger1_flag=0;
						trigger2_flag=0;
						
					}
					else
					{
						if(trigger2_flag == 1)
						{
							_42mm_shoot.friction_state = FRICTION_ON;
							_42mm_shoot.ctrl_mode = _42MM_SHOOT_NORMAL;
						}		
						else 
						{
							_42mm_shoot.friction_state =FRICTION_OFF;
							_42mm_shoot.ctrl_mode =_42MM_SHOOT_RELAX ;
						}
						
						if(trigger1_flag == 1&&_42mm_shoot.friction_state == FRICTION_ON)
						{
							_42mm_shoot.shoot_flag = 1;		//开火标志位将在发射完成后清除
							trigger1_flag = 0;				//清除ch4下拉标志位
						}
					}
				}
				break;
				default:
				break;
			}
			Remote_Input();
            break;
            
        case KEY_MOUSE_INPUT:
		{
			if(RC_CtrlData.inputmode!=RC_CtrlData.inputmode_last)
			{
				gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
				gimbal_data.gim_ref_and_fdb.yaw_angle_ref=gimbal_data.gim_ref_and_fdb.yaw_angle_fdb;
			}
				/****************↓云台↓*************/
				if(!gimbal_data.if_finish_Init)
				{
					//未初始化完成进入初始化模式
					gimbal_data.ctrl_mode =GIMBAL_INIT;
					
				}
				else
				{
					//初始化完成进入跟随陀螺仪模式
					if(gimbal_data.ctrl_mode == GIMBAL_INIT)
					gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
		
				}
				/****************↑云台↑*************/	
			
			
				/********************↓底盘↓*****************/
				if(gimbal_data.if_finish_Init != 1)	
				{
					chassis.ctrl_mode = CHASSIS_RELAX;		//云台初始化未完成，底盘不动
				}
				else	//云台初始化完成
				{
					if(gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)	//云台处于普通模式，则底盘自动跟随云台
					{
						chassis.ctrl_mode = AUTO_FOLLOW_GIMBAL;
						
						
					}
                    //按下Q键，云台进入吊射模式，底盘车轮锁死
					if(RC_CtrlData.Key_Flag.Key_Q_TFlag && gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
					{
						gimbal_data.ctrl_mode = GIMBAL_SNIPE;
						chassis.ctrl_mode = CHASSIS_STOP;
					}
					if(RC_CtrlData.Key_Flag.Key_Q_TFlag == 0 && gimbal_data.ctrl_mode == GIMBAL_SNIPE)
					{
						gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
						chassis.ctrl_mode= AUTO_FOLLOW_GIMBAL;
					}
					if(gimbal_data.ctrl_mode==GIMBAL_SNIPE)
					{
						
						if(RC_CtrlData.Key_Flag.Key_W_Flag)
						{
							gimbal_data.gim_ref_and_fdb.pit_angle_ref+=0.001;
						}
						if(RC_CtrlData.Key_Flag.Key_S_Flag)
						{
							gimbal_data.gim_ref_and_fdb.pit_angle_ref-=0.001;
						}
						if(RC_CtrlData.Key_Flag.Key_A_Flag)
						{
							gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=0.001;
						}
						if(RC_CtrlData.Key_Flag.Key_D_Flag)
						{
							gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=0.001;
						}
					}
					if(RC_CtrlData.Key_Flag.Key_CTRL_Flag)		
					{
						rotate_flag = 1;
						if(chassis.ctrl_mode == AUTO_FOLLOW_GIMBAL)
						{
							if(rotate_cnt % 2 == 1)
							chassis.ctrl_mode = CHASSIS_ROTATE;		//底盘小陀螺正传
							else
							chassis.ctrl_mode = CHASSIS_REVERSE_ROTATE;		//底盘小陀螺反转
						}
					}
					else
					{
						if(rotate_flag == 1)
						{
							rotate_cnt++;
							rotate_flag = 0;
						}
					}
					
					 if (RC_CtrlData.Key_Flag.Key_SHIFT_Flag)
					{
						chassis.chassis_speed_mode = HIGH_SPEED_MODE;	//按下SHIFT,底盘跑路模式
						chassis_speed = CHASSIS_SPEED*1.5;		//底盘速度赋值
					}
					else
					{
						chassis.chassis_speed_mode = NORMAL_SPEED_MODE;		//不按SHIFT,底盘普通模式
						chassis_speed = CHASSIS_SPEED;		//底盘速度赋值
					}
					

					
					
					///////////////////////////底盘速度赋值，在chassis_task中该值被传给了Vx，Vy//////////////////////////////////
				    if(RC_CtrlData.Key_Flag.Key_W_Flag&&!RC_CtrlData.Key_Flag.Key_S_Flag)
					{
						chassis.ChassisSpeed_Ref.forward_back_ref=chassis_speed;		//按下W键，前后ref置chassis_speed，即前进
					}
					else if(RC_CtrlData.Key_Flag.Key_S_Flag&&!RC_CtrlData.Key_Flag.Key_W_Flag)
					{
						chassis.ChassisSpeed_Ref.forward_back_ref = -chassis_speed;		//按下s键，前后ref置-chassis_speed，即后退
					}
					else
					{
						chassis.ChassisSpeed_Ref.forward_back_ref = 0;					//W、S均(未)按下，前后方向速度置零
					}
                    
                    
					if(RC_CtrlData.Key_Flag.Key_A_Flag&&!RC_CtrlData.Key_Flag.Key_D_Flag)
					{
//						if (chassis.chassis_speed_mode == HIGH_SPEED_MODE)
//						{
							chassis.ChassisSpeed_Ref.left_right_ref = -chassis_speed; 	//按下A键，高速模式下左右ref置-chassis_speed/2，即左平移
//						}
//						else
//						{
//							chassis.ChassisSpeed_Ref.left_right_ref = -chassis_speed*0.6;		//按下A键，普通模式下左右ref置-chassis_speed，即左平移
//						}
					}
					else if(RC_CtrlData.Key_Flag.Key_D_Flag&&!RC_CtrlData.Key_Flag.Key_A_Flag)
					{
//						if (chassis.chassis_speed_mode == HIGH_SPEED_MODE)
//						{
							chassis.ChassisSpeed_Ref.left_right_ref = chassis_speed;		//按下D键，高速模式下左右ref置chassis_speed，即右平移
//						}
//						else
//						{
//							chassis.ChassisSpeed_Ref.left_right_ref = chassis_speed*0.6;		//按下D键，普通模式下左右ref置chassis_speed，即右平移
//						}
					}
					else
					{
						chassis.ChassisSpeed_Ref.left_right_ref = 0;						//A、D键均(未)按下，左右移动速度置零
					}
					
					
					
//						if(RC_CtrlData.Key_Flag.Key_C_TFlag)
//						{
//							friction_on_flag =1;
////							RC_CtrlData.Key_Flag.Key_C_TFlag = 0;
//						}
//						else
//						{
//							friction_on_flag =0;
//						}
						if(RC_CtrlData.Key_Flag.Key_C_Flag)
						{
							friction_off_cnt++;
							if(friction_off_cnt > 500)
							{
								friction_on_flag = 0;
								RC_CtrlData.Key_Flag.Key_C_TFlag = 0;
							}
							else
							{
								friction_on_flag=1;
							}
						}
						else
						{
							friction_off_cnt = 0;
						}
						
						if(friction_on_flag)
						{
							_42mm_shoot.ctrl_mode = _42MM_SHOOT_NORMAL;
							_42mm_shoot.friction_state = FRICTION_ON;
						}
						else
						{
							_42mm_shoot.ctrl_mode = _42MM_SHOOT_RELAX;
							_42mm_shoot.friction_state = FRICTION_OFF;
						}
						
						if( RC_CtrlData.mouse.press_l==1 /*&& RC_CtrlData.mouse.last_press_l!=1*/)
						{   
							if(RC_CtrlData.mouse.last_press_l!=1)
								_42mm_shoot.shoot_flag = 1;
							else							
								_42mm_shoot.shoot_flag = 0;
							Auto_Shoot_Fire=1;
						}
						else
                        {						
							_42mm_shoot.shoot_flag = 0;
							Auto_Shoot_Fire=0;
                        }
						
                        
						if(RC_CtrlData.mouse.press_r)
						{
							if(RC_CtrlData.mouse.press_r&&My_Auto_Shoot.Auto_Aim.Flag_Get_Target==1)
							{
								Auto_shoot_flag= 1;
							}
						}
						else
						{
							Auto_shoot_flag =  0;
						}
						if(Auto_shoot_flag == 1)
						{
							gimbal_data.ctrl_mode = GIMBAL_AUTO_AIM;
						}
						else if(gimbal_data.ctrl_mode!=GIMBAL_SNIPE&&gimbal_data.ctrl_mode!=GIMBAL_RADAR_ASSISTANT_SNIPE)
						{
							gimbal_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
						}

				}
				/********************↑底盘↑*****************/
				
				//////////////////////////////////   云台键鼠赋值     ///////////////////////////////////////////////////
				if (gimbal_data.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
				{
					VAL_LIMIT(RC_CtrlData.mouse.x, -100, 100);		//鼠标x值大小限制
					VAL_LIMIT(RC_CtrlData.mouse.y, -100, 100);		//鼠标y值大小限制
									
				if(RC_CtrlData.Key_Flag.Key_V_TFlag)
				{
					if(reversal_flag == 0)
					{
						reversal_flag = 1;
						reversing_flag = 1;
						gimbal_data.gim_ref_and_fdb.yaw_angle_ref -= 180;
						Follow_Angle_Medium -= 180;
					}
						
				}
				else
				{
					if(reversal_flag == 1)
					{
						reversal_flag = 0;
						reversing_flag = 1;
						gimbal_data.gim_ref_and_fdb.yaw_angle_ref += 180;
						Follow_Angle_Medium += 180;
					}
					
				}
					
					gimbal_data.gim_ref_and_fdb.yaw_angle_ref -= RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT;		//云台Yaw轴键鼠赋值
					gimbal_data.gim_ref_and_fdb.pit_angle_ref += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT;			//云台Pitch轴键鼠赋值
				}
				
				//////////////////////////////////   云台键鼠赋值     ///////////////////////////////////////////////////

				if(RC_CtrlData.Key_Flag.Key_R_Flag)
				{
					UI.cnt=0;
				}
		}
		break;

				default :
				{
					yaw_sys_input=0;
					gimbal_data.gim_ref_and_fdb.scope_motor_input=0;
				}
				break;		
    }
	RC_CtrlData.inputmode_last=RC_CtrlData.inputmode;
}
