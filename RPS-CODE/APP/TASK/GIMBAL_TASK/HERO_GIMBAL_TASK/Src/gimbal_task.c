#include "gimbal_task.h"

/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  Sun
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ��ģ��Ϊ����Ӣ����̨ģ�飬��������λ��Դ�ļ��ϲ��ֺ�ͷ�ļ��ϲ��֣�
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
 
#define STANDARD 1

gimbal_t gimbal_data;
//��̨��λ
float pitch_min = 0;		
float pitch_max = 0;		

#if STANDARD == 1

		#define HERO_PITCH_MAX 6486
		#define HERO_PITCH_MIN -2981
		
    float pitch_middle = 0;
    float Pitch_min = HERO_PITCH_MIN;
    float Pitch_max = HERO_PITCH_MAX;
    
    #define VISION_PITCH_MIN            -25  //�Ӿ��Ĵ������ӽ�����������������ϵ
    #define VISION_PITCH_MAX            30

    #define YAW_INIT_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_INIT_ANGLE_FDB        gimbal_gyro.pitch_Angle
    #define YAW_INIT_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_INIT_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_ANGLE_FDB               gimbal_gyro.yaw_Angle
    #define PITCH_ANGLE_FDB             Pitch_Encoder.ecd_angle
    #define YAW_SPEED_FDB               gimbal_gyro.yaw_Gyro
    #define PITCH_SPEED_FDB             Pitch_Encoder.filter_rate

    #define VISION_YAW_ANGLE_FDB        gimbal_gyro.yaw_Angle
    #define VISION_PITCH_ANGLE_FDB      gimbal_gyro.pitch_Angle
    #define VISION_YAW_SPEED_FDB        gimbal_gyro.yaw_Gyro
    #define VISION_PITCH_SPEED_FDB      gimbal_gyro.pitch_Gyro

    #define YAW_AUTO_ANGLE_FDB          -yaw_Encoder.ecd_angle
    #define PITCH_AUTO_ANGLE_FDB        Pitch_Encoder.ecd_angle
    #define YAW_AUTO_SPEED_FDB          gimbal_gyro.yaw_Gyro
    #define PITCH_AUTO_SPEED_FDB        gimbal_gyro.pitch_Gyro

    #define YAW_MOTOR_POLARITY          -1
    #define PITCH_MOTOR_POLARITY        1

    float auto_aim_Yaw_remain = 0;
    float auto_aim_pitch_remain = 0;

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
		

    /*******************************pid_Init*********************************/
#if STANDARD == 1
    // ��ʼ���µĲ���
    PID_struct_init(&gimbal_data.pid_init_pit_Angle, POSITION_PID, 300, 10, 
                    -7,-0.05f,-60);		//MF5015+����˿��
  	PID_struct_init(&gimbal_data.pid_init_pit_speed, POSITION_PID, 800, 100,
                    7 , 0, 60);
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_init_yaw_Angle, POSITION_PID, 3000, 20, 
                    18 ,0.15f, 10);				//MF9025
	PID_struct_init(&gimbal_data.pid_init_yaw_speed, POSITION_PID, 2000, 50, 
                    12, 0.5f, 20 );

    //��ͨģʽ�µĲ���
    PID_struct_init(&gimbal_data.pid_pit_Angle, POSITION_PID, 10000, 3, 
                    50,0.001f,0.1f);	
	PID_struct_init(&gimbal_data.pid_pit_speed, POSITION_PID, 10000, 10,  
                    0.6f,0.001f,0.4f);
	PID_struct_init(&gimbal_data.pid_yaw_Angle, POSITION_PID, 257, 20,  
                    13 ,0.01f, 3);
	PID_struct_init(&gimbal_data.pid_yaw_speed, POSITION_PID, 3800, 10,  
                    20, 0.01f, 5 );
					
    //------------------------------------------------

    //����ģʽ�²���
    PID_struct_init(&gimbal_data.pid_pit_follow, POSITION_PID, 1000, 10,  
                    -20,-0.1,-10);		//MF5015+����˿��
  	PID_struct_init(&gimbal_data.pid_pit_speed_follow, POSITION_PID, 800, 100,
                    12 , 0, 50);
    //------------------------------------------------

    //����ģʽ�µĲ���
    PID_struct_init(&gimbal_data.pid_auto_pit_Angle, POSITION_PID, 200, 100, 
                    1.5f, 0.02f, 1);  
    PID_struct_init(&gimbal_data.pid_auto_pit_speed, POSITION_PID, 500, 50, 
                    1.5f, 0.01f, 1); 
    //------------------------------------------------
    PID_struct_init(&gimbal_data.pid_auto_yaw_Angle, POSITION_PID, 3000, 30, 
                    35, 0.2f, 2); 
    PID_struct_init(&gimbal_data.pid_auto_yaw_speed, POSITION_PID, 2000, 50,  
                    25, 0.1f, 1.9f);

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
    case GIMBAL_AUTO_ANGLE:		//Ӣ�۵���ģʽ
    {
        gimbal_auto_angle_handle();
    }
        break;

    default:
        break;
    }
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, pitch_min , pitch_max );		//pitch����̨�޷�
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
                pitch_middle = PITCH_ANGLE_FDB;	//��ʼ����Ĭ��ת��ͨģʽ����ȡ��ͨģʽ�µ�pitch������ֵ������Ϊ�����ǣ�˿��Ӣ��Ϊ5015��������
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
//    if(RC_CtrlData.mouse.press_r)//����Ҽ�����
//    {

//                if (new_location.flag)//�Ӿ����ʶ��
//                {
//										//�л���̨����
//                    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
//                    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
//                    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
//                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
//										//�л���̨����
//                    gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
//                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;
//										//�Ӿ�ģʽ����̨��λ
//                    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );
//                }
//				//pitch����yaw��˫��pid����
//        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
//                                                                      &gimbal_data.pid_yaw_speed_follow,
//                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
//                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
//																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
//                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
//                                                                      new_location.yaw_speed)*YAW_MOTOR_POLARITY;
//        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
//                                                                      &gimbal_data.pid_pit_speed,
//                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
//                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
//																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
//                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
//                                                                      0 )*PITCH_MOTOR_POLARITY;
//    }else
//    {
			//��ͨģʽ����̨����
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
			//pitch����yaw��˫��pid����
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,
                                                                      &gimbal_data.pid_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
																																			&gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
//    }
}















 /**
  ******************************************************************************
																Ӣ�۵����������		
	 =============================================================================
 **/
void gimbal_auto_angle_handle(void)
{
		//�ս��õ���ģʽ����������ֵ
		if(gimbal_data.last_ctrl_mode != GIMBAL_AUTO_ANGLE)
		{
			gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref = YAW_AUTO_ANGLE_FDB;
			gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref = PITCH_AUTO_ANGLE_FDB;
		}
		//��������
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb = PITCH_AUTO_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = YAW_AUTO_ANGLE_FDB;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb = PITCH_AUTO_SPEED_FDB;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = YAW_AUTO_SPEED_FDB;
    if(RC_CtrlData.mouse.press_r)//�Ҽ����£������Ӿ���������
    {
        if(new_location.flag)//�Ӿ����ʶ��
        {
						//��������
            gimbal_data.gim_ref_and_fdb.pit_angle_fdb = VISION_PITCH_ANGLE_FDB;
            gimbal_data.gim_ref_and_fdb.yaw_angle_fdb = VISION_YAW_ANGLE_FDB;
            gimbal_data.gim_ref_and_fdb.pit_speed_fdb = VISION_PITCH_SPEED_FDB;
            gimbal_data.gim_ref_and_fdb.yaw_speed_fdb = VISION_YAW_SPEED_FDB;
						//������ֵ
            gimbal_data.gim_ref_and_fdb.pit_angle_ref = new_location.y;
            gimbal_data.gim_ref_and_fdb.yaw_angle_ref = new_location.x;
						//���pitch�޷�
            VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref, VISION_PITCH_MIN , VISION_PITCH_MAX );

        }
				//˫��pid����
        gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_yaw_follow,
                                                                      &gimbal_data.pid_yaw_speed_follow,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      new_location.yaw_speed)*YAW_MOTOR_POLARITY;
        gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }else//��ʼ����΢��ģʽ
    {
				//������ֵ��˫��pid���
        gimbal_data.gim_ref_and_fdb.pit_angle_ref = gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref = gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref;
    gimbal_data.gim_ref_and_fdb.yaw_motor_input = pid_double_loop_cal(&gimbal_data.pid_auto_yaw_Angle,
                                                                      &gimbal_data.pid_auto_yaw_speed,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.yaw_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,
                                                                      0 )*YAW_MOTOR_POLARITY;
    gimbal_data.gim_ref_and_fdb.pitch_motor_input = pid_double_loop_cal(&gimbal_data.pid_auto_pit_Angle,
                                                                      &gimbal_data.pid_auto_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,                     
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_fdb,
                                                                      &gimbal_data.gim_ref_and_fdb.pit_speed_ref,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,
                                                                      0 )*PITCH_MOTOR_POLARITY;
    }
	
}


