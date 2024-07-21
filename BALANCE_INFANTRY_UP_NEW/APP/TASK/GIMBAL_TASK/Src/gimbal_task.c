#include "gimbal_task.h"

/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ��ģ��Ϊ��������̨ģ�飬��������λ��Դ�ļ��ϲ��ֺ�ͷ�ļ��ϲ��֣�
						 Դ�ļ�������id��1��Ӣ�ۣ�2�����̣�3-4-5��������6���ɻ���7���ڱ���
						 
										��̨��λ����
										�Ӿ���̨��λ
										��̨��ʼ����������
										��ͨģʽ��̨��������
										����������̨��������
										��������������
										����ǶȲ���
										pid����
						 ͷ�ļ���������ֲ�������
						 
	* @notice  ��ģ��ͨ�������б��֣���δ���Ĵ���ά�����뿪����Աά��ģ���
						 �����ԣ�ά������̨���ͨ���ԣ���ֹ��������̨���ֵ��߼������
						 д�����ģ���ڣ���ģ���������̨�Ŀ��ƣ��ο�����ĸ�ֵ���Ʋ�
						 ģʽѡ��
						 
	* @notice  ��̨ģ��ĵ������Ʋ���control_task���Ƽ���̨����Ƶ��Ϊ2ms
						 �Ƽ���̨�������Ƶ��Ϊ2ms����controltask�����gimbal_task
						 ��control_task_Init�����gimbal_parameter_Init
						 
	*	@introduction ��ģ�����״̬���ķ�ʽ��д��̨�ĸ���ģʽ�빦�ܣ������ź�
									��������ģʽ���л���ѡ������mode_switch_tasks��ȫģ���
									������gimbal_t�ṹ�������ģ����Զ���״̬�۲����ĸ�����
									Դ����Ӧ�ӿ�Ϊ�궨���βΪ_FDB���ɵ��ÿ�ܵ�ͨ�ô�������
									��������磺
									#define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
									
 ===============================================================================
 **/
 
 
 
 
 
 
 
 /**
  ******************************************************************************
							��������
		����id��1��Ӣ�ۣ�2�����̣�3-4-5��������6���ɻ���7���ڱ���
		
		��̨��λ����
		�Ӿ���̨��λ
		��̨��ʼ����������
		��ͨģʽ��̨��������
		����������̨��������
		��������������
		����ǶȲ���
		
	 =============================================================================
 **/
 
#define STANDARD 3

gimbal_t gimbal_data;
//��̨��λ
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

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle   //������е���������װ����yaw�������ұ������Ƕ�Ϊ���������������෴����Ҫ�Ӹ���
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
																��̨�ṹ���ʼ��
		pid��������
		
	 =============================================================================
 **/
void gimbal_parameter_Init(void)
{
		//�ṹ���ڴ�����
    memset(&gimbal_data, 0, sizeof(gimbal_t));
		

    /*******************************pid_Init**************************0*******/
#if STANDARD == 3
    // ��ʼ���µĲ���
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 500, 4,
                    15, 0.01f, 8); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 27000, 20000,
                    150, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 500, 4,
                    13, 0.15f, 8); 
    PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 29000, 10000,
                    150, 0.8f, 40); 

    // �����������µĲ���
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 500, 30,
                    20, 0.01f, 15); //15, 0.01f, 8
    PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 27000, 20000,
                    150, 0.001, 60); //170, 0.001f, 60
    //------------------------------------------------
		PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 5000, 0,
                    12.5, 0.02f, 5);
    PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 29000, 10000,
                    400, 0.8f, 0); 
                    

    //���������²���
    PID_struct_init ( &gimbal_data.pid_pit_rotate, POSITION_PID, 200, 30, 15
                    , 0.01, 12 );
										
    PID_struct_init ( &gimbal_data.pid_pit_speed_rotate, POSITION_PID, 27000, 25000, 300.0f, 0.001f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_rotate, POSITION_PID,  150,13,
                    9, 0.1, 10);//15 0 80
    PID_struct_init ( &gimbal_data.pid_yaw_speed_rotate, POSITION_PID, 29800, 29800,
                    400.0f, 0.8, 0 ); //160 0.8 40
                    
    //����ƽ���²���
    PID_struct_init ( &gimbal_data.pid_pit_follow, POSITION_PID, 200, 30, 20
                    , 0.01, 15);
										
    PID_struct_init ( &gimbal_data.pid_pit_speed_follow, POSITION_PID, 27000, 25000, 300.0f, 0.001f, 0 ); 

    PID_struct_init ( &gimbal_data.pid_yaw_follow, POSITION_PID,  150,16,
                    9, 0.1, 10);//15 0 80
    PID_struct_init ( &gimbal_data.pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    400.0f, 0.8, 0 ); //160 0.8 40
    //С���µĲ���            
    PID_struct_init(&gimbal_data.pid_pit_small_buff, POSITION_PID, 200, 5,
                    7.0f, 0.1f, 3); 
    PID_struct_init(&gimbal_data.pid_pit_speed_small_buff, POSITION_PID, 27000, 25000,
                    350.0f, 8.0f, 200); 
    PID_struct_init(&gimbal_data.pid_yaw_small_buff, POSITION_PID, 250, 5,
                    6.5f, 0.2f, 10); 
    PID_struct_init(&gimbal_data.pid_yaw_speed_small_buff, POSITION_PID, 25000, 5000,
                    400.0f, 8.0f, 200);
    //����µĲ���
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
																��̨�ܿ�������		
	 =============================================================================
 **/
void gimbal_task(void)
{
    
    switch (gimbal_data.ctrl_mode)
    {
    case GIMBAL_RELAX:		//�ؿ�
    {
				//�ؿ�ģʽ�£���������������㣬��ʼ����־λ����
        memset(&gimbal_data.gim_ref_and_fdb, 0, sizeof(gim_ref_and_fdb_t));
			 gimbal_data.if_finish_Init = 0;
    }
        break;
    case GIMBAL_INIT:			//��ʼ��
    {
        gimbal_init_handle();
    }
        break;
    case GIMBAL_FOLLOW_ZGYRO:		//����������
    {
        gimbal_follow_gyro_handle();
    }
        break;
    case GIMBAL_AUTO_SMALL_BUFF:	//С��
    {
        auto_small_buff_handle();
    }
        break;
    case GIMBAL_AUTO_BIG_BUFF:		//���
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
         VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, pitch_min - chassis.roll, pitch_max - chassis.roll);		//pitch����̨�޷�
     
    
		gimbal_data.last_ctrl_mode = gimbal_data.ctrl_mode;//��̨ģʽ����

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
																��̨��ʼ������		
	 =============================================================================
 **/
void gimbal_init_handle	( void )
{
    //���������ʼ��
    first_flag = 0;
		ved = 0;
		//ָ����ʼ�������뷴��
    int init_rotate_num = 0;
    gimbal_data.gim_ref_and_fdb.pit_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_INIT_ANGLE_FDB;
		
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = 0.0f;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_INIT_ANGLE_FDB;

    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_INIT_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_INIT_SPEED_FDB;
		//������̨��Ȧ������������Ȧ��������������ۼ�ֵӰ���ʼ��
    init_rotate_num = (gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)/360;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = init_rotate_num*360;

   //ͨ���ӻ�ת�����ĽǶ�
    if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>=181)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
    else if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<-179)
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;
		//pitch����yaw��˫��pid����
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
	 //�����ж��Ƿ���ɳ�ʼ��
	if (fabs(gimbal_data.gim_ref_and_fdb.pit_angle_ref - gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<=4&&fabs(gimbal_data.gim_ref_and_fdb.yaw_angle_ref - gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<=1.5)
    {
			
        gimbal_data.if_finish_Init = 1;		//��ʼ����־λ��1
                pitch_middle = 0;	//��ʼ����Ĭ��ת��ͨģʽ����ȡ��ͨģʽ�µ�pitch������ֵ������Ϊ�����ǣ�˿��Ӣ��Ϊ5015��������
			//����pitch�������λ
                pitch_max = pitch_middle+Pitch_max;
                pitch_min = pitch_middle+Pitch_min;
        
    }
    																																		
}








 /**
  ******************************************************************************
																��̨����gyro��������		
	 =============================================================================
 **/

float feed_forward;
float feed_forward_limit = 70;
float feed_forward_k = 3;
float rotate_feed_forward;
float rotate_torque_fed = -170;

void gimbal_follow_gyro_handle(void)
{
		//����ո��л�����ģʽ����ģʽ������ʽ�����Ե�ǰ��������������ʼֵ
		if(gimbal_data.last_ctrl_mode != GIMBAL_FOLLOW_ZGYRO)
		{
			gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref   = YAW_ANGLE_FDB;
			gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = PITCH_ANGLE_FDB;
		}
		//ָ����̨����
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_SPEED_FDB;
    if((RC_CtrlData.mouse.press_r)&&(gimbal_data.vision_mode==AIM_NORMAL)&&new_location.flag)//����Ҽ�����
    {

                
                
           
                        //�л���̨����
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
                                                //�Ӿ�ģʽ����̨��λ
                                                VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
                                                
                                                //pitch����yaw��˫��pid����
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
			//��ͨģʽ����̨����
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
			//pitch����yaw��˫��pid����
        
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
																small_buff��������		
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
																big_buff��������		
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
																��������λ�˽���	
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















