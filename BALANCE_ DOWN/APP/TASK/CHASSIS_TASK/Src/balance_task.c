#include "balance_task.h"



Balance_chassis_t b_chassis = { 0 };
    int Init_cnt;
    float V_T_gain;
    float V_Tp_gain;
    float balance_Tgain;
    float balance_Tpgain;

    float V_T_outlandgain ;
    float V_Tp_outlandgain ;
    float balance_Toutlandgain ;
    float balance_Tpoutlandgain ;


		
/**
************************************************************************************************************************
* @Name     : balance_param_init
* @brief    : 平衡底盘参数初始化
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void balance_param_init(void)
{
    memset(&b_chassis, 0, sizeof(Balance_chassis_t));
    b_chassis.chassis_dynemic_ref.leglength = 0.21;
    PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,20000,20000,1200,0,40000);
    PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 20000, 20000, 1200, 0, 40000);
    PID_struct_init(&b_chassis.leg_harmonize_pid, POSITION_PID, 2000, 2000, 150, 0, 3000);
    PID_struct_init(&b_chassis.vw_pid, POSITION_PID,5,5,2,0,0);
    PID_struct_init(&b_chassis.roll_pid, POSITION_PID, 50000, 20000, 800, 0, 12000);
	
		PID_struct_init(&b_chassis.pid_follow_gim, POSITION_PID, 500, 200, 7, 0, 10);
	
	PID_struct_init(&b_chassis.Init_Tp_pid, POSITION_PID, 500, 200, 40, 0, 60);
	
}

/**
************************************************************************************************************************
* @Name     : balance_chassis_task
* @brief    : 平衡底盘总控制任务
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void balance_chassis_task(void)
{
    balance_cmd_select();
    switch (b_chassis.ctrl_mode)
    {
    case CHASSIS_RELAX:
    {
			
        b_chassis.joint_T[0] = 0;
			  b_chassis.joint_T[1] = 0;
			  b_chassis.joint_T[2] = 0;
				b_chassis.joint_T[3] = 0;
				b_chassis.driving_T[0] = 0;
			  b_chassis.driving_T[1] = 0;
			Init_cnt = 0;
			balance_Tpgain = 0;
			balance_Tpoutlandgain = 0;
			b_chassis.chassis_ref.pitch = 0;
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;

    }
    break;
		case CHASSIS_STOP:
		{
			chassis_stop_handle();
			balance_task();
		}break;
    case CHASSIS_INIT:
    {
        chassis_Init_handle();

    }
    break;
		case CHASSIS_STAND_MODE:
		{
			chassis_standup_handle();
			balance_task();
		}break;
    case CHASSIS_SEPARATE:
    {
				chassis_seperate_handle();
        balance_task();
    }
    break;
    case MANUAL_FOLLOW_GIMBAL:
    {
			if(b_chassis.jump_flag)
			{
				balance_jump_handle();
				balance_task();
			}else
			{
				follow_gimbal_handle();
        balance_task();
			}
    }
    break;
    case CHASSIS_ROTATE:
    {
			chassis_rotate_handle();
			balance_task();
    }break;
		case CHASSIS_REVERSE:
		{
			chassis_side_handle();
			balance_task();
		}break;
    default:
        break;
    }
#if POWER_LIMIT == 1
    power_limit_handle();
#else
    b_chassis.max_speed = 2.4;
    b_chassis.min_speed = -2.4;
		b_chassis.Max_power_to_PM01 = input_power_cal();
//		b_chassis.predict_power = all_power_cal(balance_chassis.Driving_Encoder[0].Torque,-2.528,0.000494,1,balance_chassis.Driving_Encoder[0].rate_rpm) + all_power_cal(balance_chassis.Driving_Encoder[1].Torque,-2.528,0.000494,1,balance_chassis.Driving_Encoder[1].rate_rpm);

#endif 
}


/**
************************************************************************************************************************
* @Name     : balance_cmd_select
* @brief    : 平衡底盘模式与命令处理
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void balance_cmd_select(void)
{
    b_chassis.last_ctrl_mode = b_chassis.ctrl_mode;
	if((b_chassis.ctrl_mode != CHASSIS_INIT&&b_chassis.ctrl_mode != CHASSIS_STAND_MODE)||usart_chassis_data.chassis_mode == 0)
    b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;

    if (b_chassis.ctrl_mode != CHASSIS_INIT)
        {
            b_chassis.chassis_dynemic_ref.vy = usart_chassis_data.y/100.0f;
            b_chassis.chassis_dynemic_ref.vx = usart_chassis_data.x/100.0f;
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy,b_chassis.min_speed,b_chassis.max_speed);
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx,b_chassis.min_speed,b_chassis.max_speed);
        }
        if(b_chassis.ctrl_mode != CHASSIS_INIT&&usart_chassis_data.chassis_mode != CHASSIS_RELAX&&b_chassis.last_ctrl_mode == CHASSIS_RELAX&&usart_chassis_data.if_follow_gim)
        {
            b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
        }
        if (b_chassis.last_ctrl_mode == CHASSIS_RELAX && b_chassis.ctrl_mode != CHASSIS_RELAX)
	    {
		    b_chassis.ctrl_mode = CHASSIS_INIT;
	    }
        if (b_chassis.ctrl_mode == usart_chassis_data.chassis_mode && fabs(chassis_gyro.pitch_Angle) > 15 &&usart_chassis_data.chassis_mode != CHASSIS_RELAX)
        {
            b_chassis.ctrl_mode = CHASSIS_INIT;
        }
				if(usart_chassis_data.jump_cmd)
				{
					b_chassis.jump_flag = usart_chassis_data.jump_cmd;
				}
				b_chassis.chassis_dynemic_ref.leglength = usart_chassis_data.cmd_leg_length/100.0f;
				
				get_remote_angle();
				VAL_LIMIT(b_chassis.chassis_ref.remote_speed,b_chassis.min_speed,b_chassis.max_speed);
}

/**
************************************************************************************************************************
* @Name     : get_remote_angle
* @brief    : 获取底盘转角
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void get_remote_angle(void)
{
	float vy;
	float vx;
	float temp_angle;
	
	b_chassis.yaw_encoder_ecd_angle = usart_chassis_data.yaw_Encoder_ecd_angle;
	b_chassis.yaw_angle_0_2pi = convert_ecd_angle_to_0_2pi(b_chassis.yaw_encoder_ecd_angle,b_chassis.yaw_angle_0_2pi);
		if(b_chassis.yaw_angle_0_2pi>PI)
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi-(2*PI);}
		else
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi;}
		
	vy = b_chassis.chassis_dynemic_ref.vy;
	vx = b_chassis.chassis_dynemic_ref.vx;
	
	
	if(vy==0&&vx==0)
	{
		b_chassis.chassis_ref.remote_angle = 0;
		b_chassis.chassis_ref.remote_speed = 0;
	}else
	{
		b_chassis.chassis_ref.remote_speed = sqrt((vx*vx)+(vy*vy));
		temp_angle=atan2(vy,vx) - PI/2;
		if(temp_angle < -PI)
		{
			b_chassis.chassis_ref.remote_angle = temp_angle+2*PI;
		}else
		{
			b_chassis.chassis_ref.remote_angle = temp_angle;
		}
	}
	
	
}

/**
************************************************************************************************************************
* @Name     : chassis_standup_handle
* @brief    : 起立模式
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_standup_handle(void)
{
	b_chassis.chassis_ref.leglength = 0.15f;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
	
		if(fabs(b_chassis.balance_loop.state_err[4])<1.5*PI/180.0)
		b_chassis.ctrl_mode = usart_chassis_data.chassis_mode;
	
	
}


/**
************************************************************************************************************************
* @Name     : chassis_Init_handle
* @brief    : 初始化收腿
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_Init_handle(void)
{
		Init_cnt++;
    b_chassis.chassis_ref.leglength = 0.14f;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	VMC_data_get(&b_chassis.left_leg,-balance_chassis.joint_Encoder[1].angle,
                                        -balance_chassis.joint_Encoder[1].gyro,
                                        -balance_chassis.joint_Encoder[2].angle+PI,
                                        -balance_chassis.joint_Encoder[2].gyro);

        VMC_data_get(&b_chassis.right_leg,balance_chassis.joint_Encoder[0].angle,
                                        balance_chassis.joint_Encoder[0].gyro,
                                        balance_chassis.joint_Encoder[3].angle+PI,
                                        balance_chassis.joint_Encoder[3].gyro);
	float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f) - 1.57f;
	if((fabs(phi0) >= 4*PI/180))
	{
		float Init_Tp = pid_calc(&b_chassis.Init_Tp_pid,phi0,0);
		//双腿协调pid
    float harmonize_output = pid_calc(&b_chassis.leg_harmonize_pid, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
		
		//腿部竖直力F的计算
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength);
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength);
		
		leg_conv(b_chassis.left_leg.leg_F, Init_Tp-harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = 0 ;
		
		leg_conv(b_chassis.right_leg.leg_F, Init_Tp+ harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = 0;
		
		//电机输出限幅
    VAL_LIMIT(b_chassis.joint_T[1], -34 , 34);
    VAL_LIMIT(b_chassis.joint_T[2], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[0], -5, 5);

    VAL_LIMIT(b_chassis.joint_T[0], -34, 34);
    VAL_LIMIT(b_chassis.joint_T[3], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[1], -5, 5);
	}else
	{

		b_chassis.ctrl_mode = CHASSIS_STAND_MODE;
		Init_cnt = 0;
		
	}

    
    

}


/**
************************************************************************************************************************
* @Name     : chassis_seperate_handle
* @brief    : 单个底盘模式，测试用
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_seperate_handle(void)
{
    b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy;
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
    b_chassis.chassis_ref.vw = b_chassis.chassis_dynemic_ref.vw;
	b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
}


/**
************************************************************************************************************************
* @Name     : follow_gimbal_handle
* @brief    : 底盘跟随云台
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
float target_angle;
void follow_gimbal_handle(void)
{
		
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
//		b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
		if(fabs(b_chassis.balance_loop.dx) > 0.2||b_chassis.chassis_ref.vy != 0)
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
		
		if(fabs(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi)<PI/2)
		{
			target_angle = b_chassis.chassis_ref.remote_angle;
			b_chassis.chassis_ref.vy = b_chassis.chassis_ref.remote_speed;
		}else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi > 3*PI/2)
		{
			target_angle = b_chassis.chassis_ref.remote_angle-2*PI;
			b_chassis.chassis_ref.vy = b_chassis.chassis_ref.remote_speed;
		}else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi < -3*PI/2)
		{
			target_angle = b_chassis.chassis_ref.remote_angle+2*PI;
			b_chassis.chassis_ref.vy = b_chassis.chassis_ref.remote_speed;
		}
		else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi>0)
		{
			target_angle = b_chassis.chassis_ref.remote_angle - PI;
			b_chassis.chassis_ref.vy = -b_chassis.chassis_ref.remote_speed;
		}else if(b_chassis.chassis_ref.remote_angle-b_chassis.yaw_angle__pi_pi<0)
		{
			target_angle = b_chassis.chassis_ref.remote_angle + PI;
			b_chassis.chassis_ref.vy = -b_chassis.chassis_ref.remote_speed;
		}
		
		
		b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,target_angle); 
		VAL_LIMIT(b_chassis.chassis_ref.vw,-5,5);
		
		
}

/**
************************************************************************************************************************
* @Name     : balance_jump_handle
* @brief    : 跳跃任务
* @param		: jump_state
* @retval   : void
* @Note     :	jump_state用于记录跳跃状态
************************************************************************************************************************
**/
u8 jump_state = 0;
void balance_jump_handle(void)
{
	b_chassis.chassis_ref.vy = b_chassis.balance_loop.dx;
	b_chassis.chassis_ref.vx = 0;
	b_chassis.chassis_ref.vw = 0;
	b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
	if(jump_state == 0)
	{
		b_chassis.chassis_ref.leglength = 0.14f;
		if(fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<=0.02)
		{
			jump_state++;
		}
	}else if(jump_state == 1)
	{
		PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,20000,20000,4000,0,15000);
    PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 20000, 20000, 4000, 0, 15000);
		b_chassis.chassis_ref.leglength = 0.34f;
		if((fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<=0.01))
			jump_state++;
		
	}else if(jump_state == 2)
	{
		b_chassis.chassis_ref.leglength = 0.14f;
		if(fabs(b_chassis.balance_loop.L0 - b_chassis.chassis_ref.leglength)<=0.01)
		{
			PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,20000,20000,1200,0,40000);
			PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 20000, 20000, 1200, 0, 40000);
			jump_state=0;
			b_chassis.jump_flag = 0;
		}
	}
}


/**
************************************************************************************************************************
* @Name     : chassis_rotate_handle
* @brief    : 小陀螺
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_rotate_handle(void)
{
	if(b_chassis.yaw_angle_0_2pi>=PI)
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi-(2*PI);}
		else
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi;}
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy*sinf(b_chassis.yaw_angle__pi_pi)+b_chassis.chassis_dynemic_ref.vx*cosf(b_chassis.yaw_angle__pi_pi);
    b_chassis.chassis_ref.vx = 0;
		//b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
		
		b_chassis.chassis_ref.vw = usart_chassis_data.rotate_speed; 
		VAL_LIMIT(b_chassis.chassis_dynemic_ref.vw,-7,7);
		VAL_LIMIT(b_chassis.chassis_ref.vy,-0.5,0.5);
}


/**
************************************************************************************************************************
* @Name     : chassis_side_handle
* @brief    : 侧向对敌
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_side_handle(void)
{
	 float side_angle;
	if((b_chassis.yaw_angle_0_2pi>=0)&&((b_chassis.yaw_angle_0_2pi<=PI)))
		{side_angle = PI/2;}
		else
		{side_angle = 3*PI/2;}
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy;
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
		if(fabs(b_chassis.balance_loop.dx) > 0.2||b_chassis.chassis_ref.vy != 0)
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
		
		b_chassis.chassis_ref.vw = -pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle_0_2pi,side_angle); 
		VAL_LIMIT(b_chassis.chassis_ref.vw,-5,5);
}

/**
************************************************************************************************************************
* @Name     : chassis_stop_handle
* @brief    : 停止模式，大幅用
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void chassis_stop_handle(void)
{
    b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
}

/**
************************************************************************************************************************
* @Name     : balance_task
* @brief    : 平衡底盘解算
* @param		: void
* @retval   : void
* @Note     :	一定要注意弧度制的转化啊
							电机极性要仔细检查
							检查好各个传感器的单位与性能
************************************************************************************************************************
**/
void balance_task(void)
{
    /********************各个计算量的更新************************************/
        VMC_data_get(&b_chassis.left_leg,-balance_chassis.joint_Encoder[1].angle,
                                        -balance_chassis.joint_Encoder[1].gyro,
                                        -balance_chassis.joint_Encoder[2].angle+PI,
                                        -balance_chassis.joint_Encoder[2].gyro);

        VMC_data_get(&b_chassis.right_leg,balance_chassis.joint_Encoder[0].angle,
                                        balance_chassis.joint_Encoder[0].gyro,
                                        balance_chassis.joint_Encoder[3].angle+PI,
                                        balance_chassis.joint_Encoder[3].gyro);
   
    //计算状态变量    
    b_chassis.balance_loop.phi = chassis_gyro.pitch_Angle*PI/180.0f;
    b_chassis.balance_loop.dphi = chassis_gyro.pitch_Gyro*PI/180.0f;
    b_chassis.balance_loop.x = ((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R;
    b_chassis.balance_loop.dx = Mileage_kalman_filter.velocity;
    b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f) - 1.57f) - chassis_gyro.pitch_Angle*PI/180.0f);
    b_chassis.balance_loop.dtheta = (((b_chassis.left_leg.dphi0 + b_chassis.right_leg.dphi0)/2.0f) - chassis_gyro.pitch_Gyro*PI/180.0f);
    b_chassis.balance_loop.ddz = chassis_gyro.z_Acc*cos(chassis_gyro.pitch_Angle*PI/180.0f);
	
	  b_chassis.balance_loop.wheel_dx = ((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R;

    b_chassis.balance_loop.L0 = (b_chassis.left_leg.l0 + b_chassis.right_leg.l0)/2.0f;
    
    //计算支持力
    FN_calculate(&b_chassis.left_leg,-balance_chassis.joint_Encoder[2].Torque,-balance_chassis.joint_Encoder[1].Torque);//capacitance_message.out_power
    FN_calculate(&b_chassis.right_leg,balance_chassis.joint_Encoder[3].Torque,balance_chassis.joint_Encoder[0].Torque);//balance_chassis.Driving_Encoder[0].rate_rpm
																																																											 //balance_chassis.Driving_Encoder[0].Torque

    /*****************************************************************/

    //lqr参数根据腿长的变化获取
    lqr_k(b_chassis.balance_loop.L0, b_chassis.balance_loop.K);
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            
                b_chassis.balance_loop.k[j][i] = b_chassis.balance_loop.K[i * 2 + j];
            
                                                                                      
        }
            
    }
   
   //误差计算
    b_chassis.balance_loop.state_err[0] = -(b_chassis.balance_loop.theta);
	b_chassis.balance_loop.state_err[1] = -(b_chassis.balance_loop.dtheta);
	if(b_chassis.ctrl_mode == CHASSIS_ROTATE)
	{
		b_chassis.balance_loop.state_err[2] = -(b_chassis.balance_loop.x -b_chassis.chassis_ref.y_position)-1.2;
	}else
	{
		b_chassis.balance_loop.state_err[2] = -(b_chassis.balance_loop.x -b_chassis.chassis_ref.y_position);
	}
	
	b_chassis.balance_loop.state_err[3] = -(b_chassis.balance_loop.dx - b_chassis.chassis_ref.vy);
	b_chassis.balance_loop.state_err[4] = b_chassis.chassis_ref.pitch-(b_chassis.balance_loop.phi);
	b_chassis.balance_loop.state_err[5] = -(b_chassis.balance_loop.dphi);
	
    //对腿变化加速度的限制
	if(b_chassis.chassis_dynemic_ref.vy == 1.6)
	{
    VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.4, 1.4);
	}else if(b_chassis.chassis_dynemic_ref.vy == 2.4)
	{
		VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.2, 1.2);
	}else
	{
		VAL_LIMIT(b_chassis.balance_loop.state_err[3], -1.4, 1.4);
	}

    //lqr未离地增益计算
		
    V_T_gain = b_chassis.balance_loop.k[0][3] * b_chassis.balance_loop.state_err[3];
    V_Tp_gain = b_chassis.balance_loop.k[1][3] * b_chassis.balance_loop.state_err[3] ;
    balance_Tgain = b_chassis.balance_loop.k[0][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[0][1] * b_chassis.balance_loop.state_err[1] + b_chassis.balance_loop.k[0][2] * b_chassis.balance_loop.state_err[2] + b_chassis.balance_loop.k[0][4] * b_chassis.balance_loop.state_err[4] + b_chassis.balance_loop.k[0][5] * b_chassis.balance_loop.state_err[5];
    balance_Tpgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] + b_chassis.balance_loop.k[1][2] * b_chassis.balance_loop.state_err[2] + b_chassis.balance_loop.k[1][4] * b_chassis.balance_loop.state_err[4] + b_chassis.balance_loop.k[1][5] * b_chassis.balance_loop.state_err[5];
		
		
		
		
		
    //lqr离地增益计算
    V_T_outlandgain = 0;
    V_Tp_outlandgain = 0;
    balance_Toutlandgain = 0;
    balance_Tpoutlandgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] ;
		
    //lqr输出
    b_chassis.balance_loop.lqrOutT = balance_Tgain + V_T_gain;
    b_chassis.balance_loop.lqrOutTp = balance_Tpgain + V_Tp_gain;
		
		
    //双腿协调pid
    float harmonize_output = pid_calc(&b_chassis.leg_harmonize_pid, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
    //转向pid
    float vw_torque = pid_calc(&b_chassis.vw_pid, chassis_gyro.yaw_Gyro*PI/180.0f, b_chassis.chassis_ref.vw);
    //roll平衡pid
    float roll_F_output = pid_calc(&b_chassis.roll_pid,chassis_gyro.roll_Angle*PI/180.0f,0);
    
    
    //腿部竖直力F的计算
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength)+ (BODY_MASS/2) * 9.8 + roll_F_output;
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength) + (BODY_MASS/2)*9.8 - roll_F_output;
    
   
    //此处的T0为phi1电机的扭矩，另一个是phi4的
    if (wheel_state_estimate(&b_chassis.left_leg)||(b_chassis.ctrl_mode==CHASSIS_INIT))
    {
        leg_conv(b_chassis.left_leg.leg_F, b_chassis.balance_loop.lqrOutTp-harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = b_chassis.balance_loop.lqrOutT / 2.0f + vw_torque;
			
    }
    else
    {
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
        leg_conv(b_chassis.left_leg.leg_F, balance_Tpoutlandgain - harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = 0;
			
			
    }

    if (wheel_state_estimate(&b_chassis.right_leg)||(b_chassis.ctrl_mode==CHASSIS_INIT))
    {
        leg_conv(b_chassis.right_leg.leg_F, b_chassis.balance_loop.lqrOutTp+ harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = b_chassis.balance_loop.lqrOutT / 2.0f - vw_torque;
    }
    else
    {
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
        leg_conv(b_chassis.right_leg.leg_F, balance_Tpoutlandgain + harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = 0;
    }


    
    
    //电机输出限幅
    VAL_LIMIT(b_chassis.joint_T[1], -34 , 34);
    VAL_LIMIT(b_chassis.joint_T[2], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[0], -5, 5);

    VAL_LIMIT(b_chassis.joint_T[0], -34, 34);
    VAL_LIMIT(b_chassis.joint_T[3], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[1], -5, 5);

    
}


/**
************************************************************************************************************************
* @Name     : middle_angle_adjust_handle
* @brief    : 重心自适应算法
* @param		: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/

void middle_angle_adjust_handle(void)
{
	if(fabs(b_chassis.chassis_ref.vy)==0.0f && fabs(b_chassis.balance_loop.wheel_dx) < 0.20f && fabs(chassis_gyro.yaw_Gyro*PI/180.0f)<=0.04f)
	{
		if(b_chassis.balance_loop.wheel_dx > 0.03f)
		{
			b_chassis.chassis_ref.pitch +=  b_chassis.balance_loop.wheel_dx*0.001;
		}else if(b_chassis.balance_loop.wheel_dx < -0.03f)
		{
			b_chassis.chassis_ref.pitch +=  b_chassis.balance_loop.wheel_dx*0.001;
		}
	}
	   
}


/**
************************************************************************************************************************
* @Name     : power_limit_handle
* @brief    : 功率控制的主函数
* @param		: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/

void power_limit_handle(void)
{
    b_chassis.Max_power_to_PM01 = input_power_cal();
    get_speed_err_limite_rate(output_power_cal(capacitance_message.cap_voltage_filte));
}

/**
************************************************************************************************************************
* @Name     : input_power_cal
* @brief    : 计算发送给功率控制板的最大功率值
* @retval   : Max_Power
* @Note     : 在此处处理缓冲功率
************************************************************************************************************************
**/

float input_power_cal(void)
{
	  float judge_power = usart_chassis_data.chassis_power_limit +
                      (usart_chassis_data.chassis_power_buffer - 5) * 2;
//		float judge_power = 80;
    float Max_Power = judge_power;

    if (capacitance_message.cap_voltage_filte >= 23.0)
    {
        Max_Power = (23.7 - capacitance_message.cap_voltage_filte) * 150;
        VAL_LIMIT(Max_Power, 0, judge_power);
    }
    if (capacitance_message.cap_voltage_filte >= 23.7)
    {
        Max_Power = 0;
    }

    VAL_LIMIT(Max_Power, 0, 150);
    return Max_Power;
}

/**
************************************************************************************************************************
* @Name     : output_power_cal
* @brief    : 计算功率控制板输出的最大功率值
* @retval   : Max_Power
* @Note     : 在此处进行软件限制
************************************************************************************************************************
**/
float output_power_cal(float voltage)//限制电压防止电压过低导致电机复位
{ 
	int max_power=0;
  if(voltage>WARNING_VOLTAGE+3)
    max_power=150;
  else
    max_power=b_chassis.Max_power_to_PM01;
  VAL_LIMIT(max_power,0,150);
  return max_power;
//  return 80;
}


/**
************************************************************************************************************************
* @Name     : get_speed_err_limite_rate
* @brief    : 根据给定最大功率求出最优功率限制系数
* @param		: max_power
* @retval   : 二次方程的根（可以优化！10.13）
* @Note     :
************************************************************************************************************************
**/

void get_speed_err_limite_rate(float max_power)
{
    static float K1 = 0.0f;
    static float K2 = 0.0f;
    static float K3 = 0.0f;
    float w = ((balance_chassis.Driving_Encoder[0].rate_rpm + (-balance_chassis.Driving_Encoder[1].rate_rpm))/2.0f);
    float Vmax[2];
    b_chassis.predict_power = all_power_cal(b_chassis.balance_loop.lqrOutT,K1,K2,K3,w);
    Vmax_cal(b_chassis.balance_loop.k[0][3],max_power,balance_Tgain,K1,K2,K3,w,Vmax);
    if (Vmax[0]>0)
    {
        b_chassis.max_speed = Vmax[0];
        b_chassis.min_speed = Vmax[1];
    }else
    {
        b_chassis.max_speed = Vmax[1];
        b_chassis.min_speed = Vmax[0];
    }
    
}

/* Function Definitions */
/*
 * ALL_POWER_CAL
 *     P = ALL_POWER_CAL(T,K1,K2,K3,W)
 *
 * Arguments    : float T
 *                float k1
 *                float k2
 *                float k3
 *                float w
 * Return Type  : float
 */
float all_power_cal(float T, float k1, float k2, float k3, float w)
{
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-31 00:57:20 */
  return ((k3 + T * w * 0.10471204188481675) + T * T * k1) + k2 * (w * w);
}

/*
 * File trailer for all_power_cal.c
 *

/* Function Definitions */
/*
 * Vmax_cal
 *     Vmax = Vmax_cal(Kv,Pmax,bT_gain,K1,K2,K3,W)
 *
 * Arguments    : float Kv
 *                float Pmax
 *                float bT_gain
 *                float k1
 *                float k2
 *                float k3
 *                float w
 *                float Vmax[2]
 * Return Type  : void
 */
void Vmax_cal(float Kv, float Pmax, float bT_gain, float k1, float k2,
              float k3, float w, float Vmax[2])
{
  float Vmax_tmp;
  float b_Vmax_tmp;
  float t2;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-12-31 01:37:29 */
  t2 = w * w;
  t2 = sqrt(((Pmax * k1 * 16.0 - k1 * k3 * 16.0) - k1 * k2 * t2 * 16.0) +
            t2 * 0.043858446862750471) *
       191.0;
  Vmax_tmp = 1.0 / Kv * (1.0 / k1);
  b_Vmax_tmp = w * 40.0 + bT_gain * k1 * 764.0;
  Vmax[0] = Vmax_tmp * (b_Vmax_tmp - t2) * -0.00065445026178010475;
  Vmax[1] = Vmax_tmp * (b_Vmax_tmp + t2) * -0.00065445026178010475;
}

/*
 * File trailer for Vmax_cal.c
 *
 * [EOF]
 */

/**
************************************************************************************************************************
* @Name     : wheel_state_estimate
* @brief    : 底盘离地检测函数
* @param		: leg
* @retval   : wheel_state
* @Note     :
************************************************************************************************************************
**/

uint8_t wheel_state_estimate(leg_state_t *leg)
{
    if (leg->leg_FN < 15)
    {
			
        leg->wheel_state = 0;
        return 0;
    }
    else
    {
        leg->wheel_state = 1;
        return 1;
    }
}


/**
************************************************************************************************************************
* @Name     : lqr_k
* @brief    : 底盘倒立摆lqr计算函数
* @param		: double L0, double K[12]
* @retval   : void
* @Note     : This function was generated by the Symbolic Math Toolbox version 23.2.
************************************************************************************************************************
**/

void lqr_k(double L0, double K[12])
{
    double t2;
    double t3;
    /*     This function was generated by the Symbolic Math Toolbox version 23.2.
     */
     /*     2023-11-17 17:36:04 */
    t2 = L0 * L0;
    t3 = L0 * L0 * L0;


//		K[0] = ((L0 * -224.8656362888122 + t2 * 287.28601354985511) -
//          t3 * 167.080125914703) -
//         2.2708328063653309;
//  K[1] = ((L0 * 799.62215955328907 - t2 * 2421.0071871579148) +
//          t3 * 2369.2356209053869) -
//         18.715545190698261;
//  K[2] = ((L0 * -40.38018715277537 + t2 * 32.521969964233293) -
//          t3 * 21.552686854980259) +
//         0.78425211175398291;
//  K[3] = ((L0 * 141.5900544765947 - t2 * 419.48263098475292) +
//          t3 * 411.93317206269018) -
//         0.46043751213191358;
//  K[4] = ((L0 * -17.15704148443394 + t2 * 10.446769679445049) +
//          t3 * 12.03299467720135) -
//         2.279837861427144;
//  K[5] = ((L0 * 161.6106764263622 - t2 * 545.89886715268119) +
//          t3 * 568.97639790267169) -
//         4.3911422530775761;
//  K[6] = ((L0 * -23.559263858644151 - t2 * 0.25207525679081089) +
//          t3 * 36.613891289564663) -
//         5.0130786727043821;
//  K[7] = ((L0 * 271.99755480600669 - t2 * 928.21025842917868) +
//          t3 * 975.53282181648024) -
//         6.4120436957134306;
//  K[8] = ((L0 * -72.738997754922465 + t2 * 71.495363483826637) -
//          t3 * 27.011943732562759) +
//         37.21816652483988;
//  K[9] = ((L0 * 248.84544086263139 - t2 * 351.82171741912708) +
//          t3 * 163.43913537877319) +
//         34.99279463955817;
//  K[10] = ((L0 * -15.502668700859459 + t2 * 33.523719735351087) -
//           t3 * 31.186727381276171) +
//          5.6619913186474813;
//  K[11] = ((L0 * -10.663987348138379 + t2 * 80.881742458802307) -
//           t3 * 106.2776716888919) +
//          5.1536359190828316;

  
				
 if(b_chassis.ctrl_mode == CHASSIS_REVERSE)
 {
	K[0] = ((L0 * -164.21187446705741 + t2 * 333.27820048959512) -
          t3 * 290.16809333992671) -
         5.1206310043709564;
  K[1] = ((L0 * 89.49371109349967 - t2 * 346.09241702160728) +
          t3 * 383.610068918057) +
         9.7034793225372926;
  K[2] = ((L0 * -16.79114731066824 + t2 * 19.69386230401766) -
          t3 * 19.65858973774041) -
         0.80312713828875615;
  K[3] = ((L0 * 2.065544673417532 - t2 * 18.184763940192038) +
          t3 * 23.039726058905359) +
         2.599836187365606;
  K[4] = ((L0 * -19.701004773081081 + t2 * 48.415038801143361) -
          t3 * 43.452820262805) -
         2.44692532013582;
  K[5] = ((L0 * 0.35768472816694807 - t2 * 42.320916699973431) +
          t3 * 62.731181599873238) +
         4.7814071836321892;
  K[6] = ((L0 * -14.104823289185029 + t2 * 30.07134135094627) -
          t3 * 25.797390003849841) -
         3.570061632837104;
  K[7] = ((L0 * -6.9062378825656277 - t2 * 19.932846563720329) +
          t3 * 39.526916596188173) +
         5.6106681292998379;
  K[8] = ((L0 * -76.986765750688534 + t2 * 137.05658267993471) -
          t3 * 100.3414748531177) +
         22.228359887381661;
  K[9] = ((L0 * 113.1194124260669 - t2 * 272.89042610873332) +
          t3 * 242.16034253946171) +
         16.511505493436012;
  K[10] = ((L0 * -16.17794241325295 + t2 * 30.742057413672271) -
           t3 * 24.128403933004979) +
          4.8022621963146923;
  K[11] = ((L0 * 21.568035174876272 - t2 * 48.749030886242032) +
           t3 * 41.221623960507863) +
          2.35016748065256;
				}else
 {
	 K[0] = ((L0 * -186.74038705721131 + t2 * 201.97420397139749) -
          t3 * 108.3922719160114) -
         3.8868512085748912;
  K[1] = ((L0 * 866.963134767256 - t2 * 2557.5602592201781) +
          t3 * 2480.8038486245782) -
         13.96005646527167;
  K[2] = ((L0 * -27.184527508222011 + t2 * 5.4123157156028769) -
          t3 * 2.7363022656833622) +
         0.33232099020438249;
  K[3] = ((L0 * 135.0619938974605 - t2 * 369.51279819148158) +
          t3 * 355.004964540673) +
         0.57542436237342309;
  K[4] = ((L0 * -14.18588549123702 + t2 * 5.4735137000306606) +
          t3 * 13.73146407261625) -
         1.5295186838771559;
  K[5] = ((L0 * 160.7539783640141 - t2 * 563.88135325153826) +
          t3 * 595.66172507388114) -
         0.31967692492001731;
  K[6] = ((L0 * -21.101555682817771 - t2 * 7.4757065527223174) +
          t3 * 41.4083515635957) -
         3.9113935989267179;
  K[7] = ((L0 * 291.97127701953679 - t2 * 1026.339390410207) +
          t3 * 1088.270524848502) +
         0.41230278922931268;
  K[8] = ((L0 * -49.217478963434921 + t2 * 45.074543332746387) -
          t3 * 21.222317620313671) +
         29.707795474857338;
  K[9] = ((L0 * 150.50567379259061 - t2 * 0.69384412831275666) -
          t3 * 215.3668996951227) +
         22.524431646835819;
  K[10] = ((L0 * -15.679105646817719 + t2 * 34.796814060222538) -
           t3 * 33.3670837524549) +
          5.8739300117464914;
  K[11] = ((L0 * -15.1059497692952 + t2 * 119.6899044038893) -
           t3 * 157.75263393578209) +
          3.37703257938664;
 }
}

/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */


/**
************************************************************************************************************************
* @Name     : convert_ecd_angle_to_0_2pi
* @brief    : 将电机编码器的机械角度值（范围正负无穷大）解算为范围在0~2pi的角度值      
* @param		: ecd_angle 电机编码器的机械角度值  类型  double
* @param		: _0_2pi_angle 范围在0~2pi的角度值  类型  float
* @retval   : _0_2pi_angle 范围在0~2pi的角度值  类型  float
* @Note     : 
************************************************************************************************************************
**/
double convert_ecd_angle_to_0_2pi(double ecd_angle,float _0_2pi_angle)
{
	_0_2pi_angle=fmod(YAW_POLARITY*ecd_angle*ANGLE_TO_RAD,2*PI);	
	if(_0_2pi_angle<0)
		 _0_2pi_angle+=2*PI;

	return _0_2pi_angle;
}

