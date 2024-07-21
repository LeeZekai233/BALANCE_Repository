#include "gimbal_task.h"

/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   该模块为步兵单云台模块，参数设置位于源文件上部分和头文件上部分：
						 源文件：兵种id（1：英雄，2：工程，3-4-5：步兵，6：飞机，7：哨兵）
						 
										云台限位设置
										视觉云台限位
										云台初始化反馈设置
										普通模式云台反馈设置
										自瞄或吊射云台反馈设置
										电机输出极性设置
										自瞄角度补偿
										pid设置
						 头文件：大幅部分参数设置
						 
	* @notice  该模块通用与所有兵种，请未来的代码维护者与开发人员维护模块的
						 独立性，维护各云台间的通用性，禁止将属于云台部分的逻辑与代码
						 写到别的模块内，该模块仅负责云台的控制，参考输入的赋值请移步
						 模式选择。
						 
	* @notice  云台模块的调用请移步至control_task，推荐云台计算频率为2ms
						 推荐云台电机发送频率为2ms。在controltask里放置gimbal_task
						 在control_task_Init里放置gimbal_parameter_Init
						 
	*	@introduction 本模块采用状态机的方式编写云台的各种模式与功能，控制信号
									的输入与模式的切换与选择来自mode_switch_tasks，全模块的
									变量由gimbal_t结构体包含，模块可自定义状态观测器的更新来
									源，对应接口为宏定义结尾为_FDB，可调用框架的通用传感器结
									构体变量如：
									#define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
									
 ===============================================================================
 **/
 
 
 
 
 
 
 
 /**
  ******************************************************************************
							参数设置
		兵种id（1：英雄，2：工程，3-4-5：步兵，6：飞机，7：哨兵）
		
		云台限位设置
		视觉云台限位
		云台初始化反馈设置
		普通模式云台反馈设置
		自瞄或吊射云台反馈设置
		电机输出极性设置
		自瞄角度补偿
		
	 =============================================================================
 **/
 
#define STANDARD 3

gimbal_t gimbal_data;
//云台限位
float pitch_min = 0;		
float pitch_max = 0;		


#if STANDARD == 3

		#define INFANTRY_PITCH_MAX 27.0f
		#define INFANTRY_PITCH_MIN -22.0f
        
        #define CHASSIS_RE_PITCH_MAX 27.0f
        #define CHASSIS_RE_PITCH_MIN -34.0f
        
    float pitch_middle = 0;
    float Pitch_min = INFANTRY_PITCH_MIN;
    float Pitch_max = INFANTRY_PITCH_MAX;

    #define VISION_PITCH_MIN            -22
    #define VISION_PITCH_MAX            27

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle   //步兵机械将电机反着装导致yaw轴电机向右编码器角度为负，与期望极性相反，需要加负号
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             gimbal_gyro.pitch_Angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             gimbal_gyro.pitch_Gyro

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        1
    
    #define YAW_BIG_FEED          1
    #define PITCH_BIG_FEED        1
    
    float Buff_Yaw_remain = 1.4;
    float Buff_pitch_remain= 1;

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;
    
    float big_buff_pit_fed = PITCH_BIG_FEED;
    float big_buff_yaw_fed = YAW_BIG_FEED;
#elif STANDARD == 4
#elif STANDARD == 5
#endif




 /**
  ******************************************************************************
																云台结构体初始化
		pid参数设置
		
	 =============================================================================
 **/
void gimbal_parameter_Init(void)
{
		//结构体内存置零
    memset(&gimbal_data, 0, sizeof(gimbal_t));
		

    /*******************************pid_Init**************************0*******/
#if STANDARD == 3
    // 初始化下的参数
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 500, 4,
                    15, 0.01f, 8); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 27000, 20000,
                    150, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 500, 4,
                    13, 0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

    // 跟随陀螺仪下的参数
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 500, 30,
                    20, 0.01f, 15); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 27000, 20000,
                    150, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
		PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 5000, 0,
                    12.5, 0.02f, 5);
    PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 29000, 10000,
                    400, 0.8f, 0); 
                    

    //自瞄陀螺下参数
    PID_struct_init ( &gimbal_data.pid_pit_rotate, POSITION_PID, 200, 30, 15
                    , 0.01, 12 );
										
    PID_struct_init ( &gimbal_data.pid_pit_speed_rotate, POSITION_PID, 27000, 25000, 300.0f, 0.001f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_rotate, POSITION_PID,  150,13,
                    9, 0.1, 10);//15 0 80
    PID_struct_init ( &gimbal_data.pid_yaw_speed_rotate, POSITION_PID, 29800, 29800,
                    400.0f, 0.8, 0 ); //160 0.8 40
                    
    //自瞄平移下参数
    PID_struct_init ( &gimbal_data.pid_pit_follow, POSITION_PID, 200, 30, 20
                    , 0.01, 15);
										
    PID_struct_init ( &gimbal_data.pid_pit_speed_follow, POSITION_PID, 27000, 25000, 300.0f, 0.001f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_follow, POSITION_PID,  150,16,
                    9, 0.1, 10);//15 0 80
    PID_struct_init ( &gimbal_data.pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    400.0f, 0.8, 0 ); //160 0.8 40
    //小幅下的参数            
    PID_struct_init(&gimbal_data.pid_pit_small_buff, POSITION_PID, 200, 5,
                    7.0f, 0.1f, 3); 
    PID_struct_init(&gimbal_data.pid_pit_speed_small_buff, POSITION_PID, 27000, 25000,
                    350.0f, 8.0f, 200); 
    PID_struct_init(&gimbal_data.pid_yaw_small_buff, POSITION_PID, 250, 5,
                    6.5f, 0.2f, 10); 
    PID_struct_init(&gimbal_data.pid_yaw_speed_small_buff, POSITION_PID, 25000, 5000,
                    400.0f, 8.0f, 200);
    //大幅下的参数
    PID_struct_init(&gimbal_data.pid_pit_big_buff, POSITION_PID, 200, 10,
                    8.0f, 0.05f, 20); 
    PID_struct_init(&gimbal_data.pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                    300.0f, 8.0f, 150); 
    PID_struct_init(&gimbal_data.pid_yaw_big_buff, POSITION_PID, 250, 5,
                    8.5f, 0.05f, 30); 
    PID_struct_init(&gimbal_data.pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
                    400.0f, 8.0f, 200);
#elif STANDARD == 4
#elif STANDARD == 5
#endif
    /************************************************************************/
}


 /**
  ******************************************************************************
																云台总控制任务		
	 =============================================================================
 **/
void gimbal_task(void)
{
    
    switch (gimbal_data.ctrl_mode)
    {
    case GIMBAL_RELAX:		//关控
    {
				//关控模式下，所有输入输出置零，初始化标志位清零
        memset(&gimbal_data.gim_ref_and_fdb, 0, sizeof(gim_ref_and_fdb_t));
			 gimbal_data.if_finish_Init = 0;
    }
        break;
    case GIMBAL_INIT:			//初始化
    {
        gimbal_init_handle();
    }
        break;
    case GIMBAL_FOLLOW_ZGYRO:		//跟随陀螺仪
    {
        gimbal_follow_gyro_handle();
    }
        break;
    case GIMBAL_AUTO_SMALL_BUFF:	//小福
    {
        auto_small_buff_handle();
    }
        break;
    case GIMBAL_AUTO_BIG_BUFF:		//大幅
    {
        auto_big_buff_handle();
    }
        break;
    default:
        break;
    }
      if(chassis.ctrl_mode==CHASSIS_REVERSE)
      {
          pitch_min = CHASSIS_RE_PITCH_MIN;
          pitch_max = CHASSIS_RE_PITCH_MAX;
      }else
      {
          pitch_min = Pitch_min;
          pitch_max = Pitch_max;
      }
         VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, pitch_min - chassis.roll, pitch_max - chassis.roll);		//pitch轴云台限幅
     
    
		gimbal_data.last_ctrl_mode = gimbal_data.ctrl_mode;//云台模式更新

}



/****************************big_or_small_buff_var************************************/
float last_pitch_angle;
float last_yaw_angle;
float yaw_angle_ref_aim,pit_angle_ref_aim;
float last_yaw,last_pit;
uint8_t first_flag = 0;
uint8_t ved = 0;
float Delta_Dect_Angle_Yaw,Delta_Dect_Angle_Pit;
/*************************************************************************************/


 /**
  ******************************************************************************		
																云台初始化任务		
	 =============================================================================
 **/
void gimbal_init_handle	( void )
{
    //大幅参数初始化
    first_flag = 0;
		ved = 0;
		//指定初始化给定与反馈
    int init_rotate_num = 0;
    gimbal_data.gim_ref_and_fdb.pit_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_INIT_ANGLE_FDB;
		
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_INIT_ANGLE_FDB;

    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_INIT_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_INIT_SPEED_FDB;
		//计算云台多圈，并补偿多余圈数避免编码器的累计值影响初始化
    init_rotate_num = (gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)/360;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = init_rotate_num*360;

   //通过劣弧转到正的角度
    if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>=181)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
    else if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<-179)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;
		//pitch轴与yaw轴双环pid计算
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_init_yaw_Angle,
                                                                      &gimbal_data.pid_init_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_init_pit_Angle,
                                                                      &gimbal_data.pid_init_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
	 //自主判断是否完成初始化
	if (fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=4&&fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<=1.5)
    {
			
        gimbal_data.if_finish_Init = 1;		//初始化标志位置1
                pitch_middle = 0;	//初始化完默认转普通模式，获取普通模式下的pitch反馈中值（步兵为陀螺仪，丝杆英雄为5015编码器）
			//计算pitch轴软件限位
                pitch_max = pitch_middle+Pitch_max;
                pitch_min = pitch_middle+Pitch_min;
        
    }
    																																		
}








 /**
  ******************************************************************************
																云台跟随gyro控制任务		
	 =============================================================================
 **/

float feed_forward;
float feed_forward_limit = 70;
float feed_forward_k = 3;
float rotate_feed_forward;
float rotate_torque_fed = -170;

void gimbal_follow_gyro_handle(void)
{
		//如果刚刚切换至该模式，该模式的增量式输入以当前传感器反馈赋初始值
		if(gimbal_data.last_ctrl_mode != GIMBAL_FOLLOW_ZGYRO)
		{
			gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref   = YAW_ANGLE_FDB;
			gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = PITCH_ANGLE_FDB;
		}
		//指定云台反馈
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_SPEED_FDB;
    if((RC_CtrlData.mouse.press_r)&&(gimbal_data.vision_mode==AIM_NORMAL)&&new_location.flag)//鼠标右键按下
    {

                
                
           
                        //切换云台反馈
									/**/
										float pitch,yaw;
										pitch = convert_ecd_angle_to__pi_pi(VISION_PITCH_ANGLE_FDB,pitch);
										yaw = convert_ecd_angle_to__pi_pi(VISION_YAW_ANGLE_FDB,yaw);
										gimbal_data.gim_ref_and_fdb.pit_angle_fdb = pitch;
                                        gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = yaw;
									/**/
                                    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
                        
                                    gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y + auto_aim_pitch_remain;
                                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x + auto_aim_Yaw_remain;
                                                
                                                if(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb > 180.0)
                                                {
                                                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
                                                }else if(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb < -180.0)
                                                {
                                                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;
                                                }
                                                //视觉模式下云台限位
                                                VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
                                                
                                                //pitch轴与yaw轴双环pid计算
                                                gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_rotate,
                                                                                                              &gimbal_data.pid_yaw_speed_rotate,
                                                                                                              gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                                                              gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                                                              &gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                                                              gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                                                              0)*YAW_MOTOR_POLARITY;
                                                gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_rotate,
                                                                                                              &gimbal_data.pid_pit_speed_rotate,
                                                                                                              gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                                                              gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                                                              &gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                                                              gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                                                              0 )*PITCH_MOTOR_POLARITY;
                    
                    gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = VISION_PITCH_ANGLE_FDB;
                    gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref = VISION_YAW_ANGLE_FDB;
										
//									if(fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb) < 0.5)
//									{
//										gimbal_data.if_auto_shoot = 1;
//									}else
//									{
//										gimbal_data.if_auto_shoot = 0;
//									}
									
                    
						
                    
									
									
										
                                
                
				
        
    }else
    {
			//普通模式下云台输入
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
			//pitch轴与yaw轴双环pid计算
        
        if(chassis.ctrl_mode==CHASSIS_ROTATE)
        {
            rotate_feed_forward = rotate_torque_fed;
            feed_forward = 0;
        }else
        {
           rotate_feed_forward = 0; 
           feed_forward = RC_CtrlData.mouse.x*feed_forward_k;
        }
        VAL_LIMIT(feed_forward,-feed_forward_limit,+feed_forward_limit);
        
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,
                                                                      &gimbal_data.pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                       feed_forward)*YAW_MOTOR_POLARITY + rotate_feed_forward;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }
}








 /**
  ******************************************************************************
																small_buff控制任务		
	 =============================================================================
 **/
float pit_forward_k = 0;
float yaw_forward_k = 0.01;
void auto_small_buff_handle(void)
{
     if(first_flag == 0)
	{
		last_pitch_angle=VISION_PITCH_ANGLE_FDB;
		last_yaw_angle=VISION_YAW_ANGLE_FDB;
		first_flag = 1;
	}
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
    if(new_location.xy_0_flag)
    {
        new_location.xy_o_time++;
    }else
    {
        new_location.xy_o_time=0;
    }
    if(new_location.xy_o_time<1)
    {
        ved = 1;
        if(last_yaw==new_location.x1&&last_pit==new_location.y1)
        {
            Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x1 ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
            Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y1 ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x + Buff_Yaw_remain;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y + Buff_pitch_remain;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;
        
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = yaw_angle_ref_aim;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = raw_data_to_pitch_angle(pit_angle_ref_aim)+Buff_pitch_remain;;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_small_buff,
                                                                      &gimbal_data.pid_yaw_speed_small_buff,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      yaw_forward_k*gimbal_data.pid_yaw_small_buff.out )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_small_buff,
                                                                      &gimbal_data.pid_pit_speed_small_buff,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      pit_forward_k*gimbal_data.pid_pit_small_buff.out )*PITCH_MOTOR_POLARITY;
}







 /**
  ******************************************************************************
																big_buff控制任务		
	 =============================================================================
 **/

void auto_big_buff_handle(void)
{
    if(first_flag == 0)
	{
		last_pitch_angle=VISION_PITCH_ANGLE_FDB;
		last_yaw_angle=VISION_YAW_ANGLE_FDB;
		first_flag = 1;
	}
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
    if(new_location.xy_0_flag)
    {
        new_location.xy_o_time++;
    }else
    {
        new_location.xy_o_time=0;
    }
    if(new_location.xy_o_time<1)
    {
        ved = 1;
        if(last_yaw==new_location.x1&&last_pit==new_location.y1)
        {
            Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x1 ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
            Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y1 ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
				
			yaw_angle_ref_aim=Delta_Dect_Angle_Yaw + new_location.x + Buff_Yaw_remain;
			pit_angle_ref_aim=Delta_Dect_Angle_Pit + new_location.y + Buff_pitch_remain;
        }
        last_yaw=new_location.x1;
		last_pit=new_location.y1;

        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = yaw_angle_ref_aim;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = raw_data_to_pitch_angle(pit_angle_ref_aim)+Buff_pitch_remain;;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
    if(ved==0)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = last_yaw_angle;
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = last_pitch_angle;
    }
    
    if(fabs(gimbal_data.pid_yaw_big_buff.err[0])<=0.7&&fabs(gimbal_data.pid_pit_big_buff.err[0])<=0.7)
    {
        big_buff_pit_fed = PITCH_BIG_FEED;
        big_buff_yaw_fed = YAW_BIG_FEED;
    }else
    {
        big_buff_pit_fed = 0;
        big_buff_yaw_fed = 0;
    }
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_big_buff,
                                                                      &gimbal_data.pid_yaw_speed_big_buff,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      gimbal_data.pid_yaw_big_buff.out*big_buff_yaw_fed )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_big_buff,
                                                                      &gimbal_data.pid_pit_speed_big_buff,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      gimbal_data.pid_pit_big_buff.out*big_buff_pit_fed )*PITCH_MOTOR_POLARITY;
}








 /**
  ******************************************************************************
																能量机关位姿解算	
	 =============================================================================
 **/
float raw_data_to_pitch_angle(float ecd_angle_pit)
{
  int shoot_angle_speed;
  float distance_s;
  float distance_x;
  float distance_y;
  float x1;
  float x2;
  float x3;
  float x4;
  float angle_tan;
  float shoot_radian;
  float shoot_angle;
  float real_angle;
	
  shoot_angle_speed=28;//judge_rece_mesg.shoot_data.bullet_speed;
  distance_s=6.9/cos(gimbal_gyro.pitch_Angle*ANGLE_TO_RAD);//*cos((get_yaw_angle-yaw_Angle)*ANGLE_TO_RAD));//(Gimbal_Auto_Shoot.Distance-7)/100;
	real_angle=ecd_angle_pit+RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA, distance_s );
	
  distance_x=(cos((ecd_angle_pit)*ANGLE_TO_RAD)*distance_s);
  distance_y=(sin((ecd_angle_pit)*ANGLE_TO_RAD)*distance_s);

  x1=shoot_angle_speed*shoot_angle_speed;
  x2=distance_x*distance_x;
  x3=sqrt(x2-(19.6*x2*((9.8*x2)/(2*x1)+distance_y))/x1);
  x4=9.8*x2;
  angle_tan=(x1*(distance_x-x3))/(x4);
  shoot_radian=atan(angle_tan);
  shoot_angle=shoot_radian*RAD_TO_ANGLE;
  return shoot_angle;
}















