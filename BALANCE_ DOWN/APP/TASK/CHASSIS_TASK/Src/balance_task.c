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
    b_chassis.chassis_dynemic_ref.leglength = 0.23;
    PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,2000,2000,800,0,40000);
    PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 2000, 2000, 800, 0, 40000);
    PID_struct_init(&b_chassis.leg_harmonize_pid, POSITION_PID, 2000, 2000, 150, 0, 3000);
    PID_struct_init(&b_chassis.vw_pid, POSITION_PID,5,5,2,0,0);
    PID_struct_init(&b_chassis.roll_pid, POSITION_PID, 500, 200, 1000, 0, 0);
	
		PID_struct_init(&b_chassis.pid_follow_gim, POSITION_PID, 500, 200, 1000, 0, 0);
	
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
				follow_gimbal_handle();
        balance_task();
    }
    break;
    case CHASSIS_ROTATE:
    {
			chassis_rotate_handle();
			balance_task();
    }break;
    default:
        break;
    }
#if POWER_LIMIT == 1
    power_limit_handle();
#else
    b_chassis.max_speed = 3;
    b_chassis.min_speed = -3;
		b_chassis.Max_power_to_PM01 = 150;
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
    b_chassis.ctrl_mode = can_chassis_data.chassis_mode;

    if (b_chassis.ctrl_mode != CHASSIS_INIT)
        {
            b_chassis.chassis_dynemic_ref.vy = can_chassis_data.y/100.0f;
            b_chassis.chassis_dynemic_ref.vx = can_chassis_data.x/100.0f;
            b_chassis.chassis_dynemic_ref.vw = can_chassis_data.rotate_speed;
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy,b_chassis.min_speed,b_chassis.max_speed);
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx,-2,2);
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vw,-5,5);
        }
        if(b_chassis.ctrl_mode != CHASSIS_INIT&&can_chassis_data.chassis_mode != CHASSIS_RELAX&&b_chassis.last_ctrl_mode == CHASSIS_RELAX&&can_chassis_data.if_follow_gim)
        {
            b_chassis.ctrl_mode = can_chassis_data.chassis_mode;
        }
        if (b_chassis.last_ctrl_mode == CHASSIS_RELAX && b_chassis.ctrl_mode != CHASSIS_RELAX)
	    {
		    b_chassis.ctrl_mode = CHASSIS_INIT;
	    }
        if (b_chassis.ctrl_mode == CHASSIS_SEPARATE && fabs(chassis_gyro.pitch_Angle) > 12)
        {
            b_chassis.ctrl_mode = CHASSIS_INIT;
        }
				b_chassis.chassis_dynemic_ref.leglength = can_chassis_data.cmd_leg_length/100.0f;
				
				b_chassis.yaw_encoder_ecd_angle = can_chassis_data.yaw_Encoder_ecd_angle/10000.0f;
				b_chassis.yaw_angle_0_2pi = convert_ecd_angle_to_0_2pi(b_chassis.yaw_encoder_ecd_angle,b_chassis.yaw_angle_0_2pi);
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
	if(fabs(chassis_gyro.pitch_Angle)<3)
		b_chassis.ctrl_mode = can_chassis_data.chassis_mode;
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
	if((fabs(phi0) >= 4*PI/180)||Init_cnt<1000)
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
void follow_gimbal_handle(void)
{
	if(b_chassis.yaw_angle_0_2pi>=PI)
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi-(2*PI);}
		else
		{b_chassis.yaw_angle__pi_pi=b_chassis.yaw_angle_0_2pi;}
		
		b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy;
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
		b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
		
		b_chassis.chassis_ref.vw = pid_calc(&b_chassis.pid_follow_gim,b_chassis.yaw_angle__pi_pi,0); 
		VAL_LIMIT(b_chassis.chassis_dynemic_ref.vw,-5,5);
		
		
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
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
		b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
		
		b_chassis.chassis_ref.vw = can_chassis_data.rotate_speed; 
		VAL_LIMIT(b_chassis.chassis_dynemic_ref.vw,-5,5);
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

    b_chassis.balance_loop.L0 = (b_chassis.left_leg.l0 + b_chassis.right_leg.l0)/2.0f;
    
    //计算支持力
    FN_calculate(&b_chassis.left_leg,-balance_chassis.joint_Encoder[2].Torque,-balance_chassis.joint_Encoder[1].Torque);
    FN_calculate(&b_chassis.right_leg,balance_chassis.joint_Encoder[3].Torque,balance_chassis.joint_Encoder[0].Torque);
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
	b_chassis.balance_loop.state_err[2] = -(b_chassis.balance_loop.x -b_chassis.chassis_ref.y_position);
	b_chassis.balance_loop.state_err[3] = -(b_chassis.balance_loop.dx - b_chassis.chassis_ref.vy);
	b_chassis.balance_loop.state_err[4] = -(b_chassis.balance_loop.phi);
	b_chassis.balance_loop.state_err[5] = -(b_chassis.balance_loop.dphi);
    //对腿变化加速度的限制
    VAL_LIMIT(b_chassis.balance_loop.state_err[3], -2.5, 2.5);

    //lqr未离地增益计算
    V_T_gain = b_chassis.balance_loop.k[0][3] * b_chassis.balance_loop.state_err[3];
    V_Tp_gain = b_chassis.balance_loop.k[1][3] * b_chassis.balance_loop.state_err[3];
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
    
    //如果pitch角控制不住，下蹲获取平衡
//    if (fabs(b_chassis.balance_loop.phi*180/PI)>5)
//    {
//        
//        b_chassis.chassis_ref.leglength = 0.15;
//    }
//    else
//    {
//       
//       b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
//    }
    //腿部竖直力F的计算
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength)+ (BODY_MASS/2) * 9.8 + roll_F_output;
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength) + (BODY_MASS/2)*9.8 - roll_F_output;
    /*test*/
 
    /*float ddl = (b_chassis.left_leg.ddl0 + b_chassis.right_leg.ddl0) / 2;
    float dl = (b_chassis.left_leg.dl0 + b_chassis.right_leg.dl0) / 2;
    float ddXw = chassis_gyro.y_Acc - ddl * sinf(b_chassis.balance_loop.theta) - 2 * b_chassis.balance_loop.dtheta * cosf(b_chassis.balance_loop.theta) * dl - b_chassis.balance_loop.L0 * b_chassis.balance_loop.ddtheta * cosf(b_chassis.balance_loop.theta) + b_chassis.balance_loop.L0 * (b_chassis.balance_loop.dtheta) * (b_chassis.balance_loop.dtheta) * sinf((b_chassis.balance_loop.theta));
    float max_F = ((-b_chassis.balance_loop.lqrOutT * WHEEL_R + (-b_chassis.balance_loop.lqrOutTp) * b_chassis.balance_loop.L0 * cosf(b_chassis.balance_loop.theta)  ) - ddXw * WHEEL_MASS) / sinf(b_chassis.balance_loop.theta);
    */
   
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
        leg_conv(b_chassis.left_leg.leg_F, balance_Tpoutlandgain + harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
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
        leg_conv(b_chassis.right_leg.leg_F, balance_Tpoutlandgain - harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
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
    float Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit +
                      (judge_rece_mesg.power_heat_data.chassis_power_buffer - 5) * 2;

    if (capacitance_message.cap_voltage_filte >= 23.0)
    {
        Max_Power = (23.7 - capacitance_message.cap_voltage_filte) * 150;
        VAL_LIMIT(Max_Power, 0, judge_rece_mesg.game_robot_state.chassis_power_limit + (judge_rece_mesg.power_heat_data.chassis_power_buffer - 5) * 2);
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
    if (leg->leg_FN < 10)
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
  K[0] = ((L0 * -100.894454139715 + t2 * 186.15407202778019) -
          t3 * 164.71714791853231) -
         9.0643185539432256;
  K[1] = ((L0 * -22.24445981993027 + t2 * 37.0647970786585) -
          t3 * 23.324012572880282) +
         6.6478307573260293;
  K[2] = ((L0 * -8.8338359698040048 - t2 * 5.3289547189472906) +
          t3 * 5.332543582893269) -
         0.677454180483319;
  K[3] = ((L0 * -0.84493166249022367 + t2 * 1.0561848214883061) -
          t3 * 0.41856935352784452) +
         0.59344724851797026;
  K[4] = ((L0 * -2.0360080085351981 + t2 * 5.823074783540295) -
          t3 * 5.8316266624952089) -
         0.698720843852313;
  K[5] = ((L0 * -2.857758657196158 + t2 * 6.4174455395863541) -
          t3 * 5.4555519509322528) +
         0.56842253383360752;
  K[6] = ((L0 * -9.5181215673992856 + t2 * 25.891984355714168) -
          t3 * 25.66832429493542) -
         4.0099365908433109;
  K[7] = ((L0 * -16.3636975946947 + t2 * 37.713064921031268) -
          t3 * 32.796043727186067) +
         3.17599239852152;
  K[8] = ((L0 * -150.6500155179067 + t2 * 355.72917401723151) -
          t3 * 315.3589239118059) +
         28.307393517205121;
  K[9] = ((L0 * 61.732820887336288 - t2 * 180.83468117506251) +
          t3 * 182.944871604467) +
         35.710799348588;
  K[10] = ((L0 * -13.22708180796258 + t2 * 29.627764194465591) -
           t3 * 25.6677747739944) +
          2.8516020824915231;
  K[11] = ((L0 * 5.0203680463653813 - t2 * 14.0592425863903) +
           t3 * 13.82247388589254) +
          2.5451089583353692;
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

