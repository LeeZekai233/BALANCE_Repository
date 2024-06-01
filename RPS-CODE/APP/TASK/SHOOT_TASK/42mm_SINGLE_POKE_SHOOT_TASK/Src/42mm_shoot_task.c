#include "42mm_shoot_task.h"

/**
  ******************************************************************************
  * @file   42mm_shoot_task.c
  * @author  Sun
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   42mm���跢��ģ�飬��ģ�鶨����Ӣ��һ�����̹���˫Ħ���ֵ�
							����ơ���������λ��λ��Դ�ļ��ϲ���ע�⣡����Ӣ�۷�����
							���ٶȷ���ѡ��rate_rpm��filter_rate��
	
	* @notice  ��ģ�����������23����Ӣ�ۻ�е��������δ���Ĵ���ά�����뿪��
							��Աά��ģ��Ķ����ԣ�ά��ͨ���ԣ���ֹ�����ڸò��ֵ��߼������
						 д�����ģ���ڣ���ģ���������Ŀ��ƣ��ο�����ĸ�ֵ���Ʋ�
						 ģʽѡ��
						 
	* @notice  ����ģ��ĵ������Ʋ���control_task���Ƽ���̨����Ƶ��Ϊ2ms
						 �Ƽ���̨�������Ƶ��Ϊ2ms����controltask��shoot_task
						 ��control_task_Init�����shoot_param_init
						 
	*	@introduction ��ģ�����״̬���ķ�ʽ������ַ����״̬�������ź�
									��������ģʽ���л���ѡ������mode_switch_tasks��ȫģ���
									����_42mm_shoot�ṹ�������ģ����Զ���Ħ���ֵļ��ԣ�����
									�������������
									#define RIGHT_FRICTION_POLARITY -1
									
 ===============================================================================
 **/


_42mm_shoot_t _42mm_shoot;


 /**
  ******************************************************************************
																			��������
	Ħ����ת�����ã����ٶ�Ӧ10��12��14��16��
	Ӣ���²���ת��
	Ӣ���²�����������
	42mm���������ϲ��̽Ƕ�
	Ħ������ת��������
	 =============================================================================
 **/
#define FRICTION_SPEED_10 2000
#define FRICTION_SPEED_12 2100
#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

#define POKE_SPEED -200			//Ӣ���²���ת��
#define POKE_MAX_OUT 10500		//Ӣ���²�����������
#define ONE_POKE_ANGLE_42 1030	//42mm���������²��̽Ƕ�
//Ħ������ת��������
#define RIGHT_FRICTION_POLARITY 1
#define LEFT_FRICTION_POLARITY  1
#define POKE_POLARITY      -1



/******************** VARIABVLE ******************/
uint16_t frictionSpeed_42=0;		//42mm����
u8 over_heat = 0;                   //��������־λ
int lock_cnt = 0;
int reverse_cnt = 0;
int set_cnt = 0;
u8 poke_init_flag = 0;
u8 press_l_first_in = 0;
u8 press_r_first_in = 0;

int Stall_detection = 0;//��ת������
int Burst_count = 0;//��������
int bullet_lock_flag = 0;//���̶�ת��־λ
int bullet_locked_flag =0;

 /**
  ******************************************************************************
																��̨�ṹ���ʼ��
		pid��������
		
	 =============================================================================
 **/
void shoot_param_init(void)
{
		//�ṹ���ڴ�����
    memset(&_42mm_shoot,0,sizeof(_42mm_shoot_t));

   
	 //42mm����
	PID_struct_init(&_42mm_shoot.pid_downpoke_angle,   POSITION_PID, 700 , 1,  10  , 0.003f  , 50 );//�²���λ�û�
	PID_struct_init(&_42mm_shoot.pid_downpoke_speed,   POSITION_PID, 10000 , 1,  10  , 0.006f, 60 );//��
}


 /**
  ******************************************************************************
																��̨�ܿ�������		
	 =============================================================================
 **/
void shoot_task(void)
{
    Shoot_42mm_speed_Select(3050);//�����Զ�ѡ��
    heat_limit_42mm(RC_CtrlData.Key_Flag.Key_B_Flag);//��������
    shoot_friction_handle_42();//Ħ���ֿ�������
    shoot_bullet_handle_42();//���̿�������

}

 
 /**
  ******************************************************************************
																�����Զ�ѡ��
		������ڲ���Ϊ����Ħ�����ٶȣ���Ϊ0����Ĭ�ϲ���ϵͳѡ�񣬷�ִ֮�в����ڵ��ٶ�
	 =============================================================================
 **/
static void Shoot_42mm_speed_Select(uint16_t test_frictionSpeed_42) // 42mm����
{
    if (test_frictionSpeed_42 == 0)
    {
			//������ϵͳ�������޽���ѡ��
//        if (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit == 10)
//            frictionSpeed_42 = FRICTION_SPEED_10;
//        else if (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit == 16)
//            frictionSpeed_42 = FRICTION_SPEED_16;
//        else
//            frictionSpeed_42 = FRICTION_SPEED_10;
    }
    else
    {
        frictionSpeed_42 = test_frictionSpeed_42;
    }
}




 /**
  ******************************************************************************
																��������		
		������ڲ���Ϊ�Ƿ����������־��Ϊ1���ԣ�Ϊ0����
	 =============================================================================
 **/

void heat_limit_42mm(u8 ifignore)
{
   float residue_heart; //ʣ������
    residue_heart=(judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit           //ͨ������ϵͳ����ʣ������
                       -judge_rece_mesg.power_heat_data.shooter_id1_42mm_cooling_heat);
    if (ifignore)
    {
        over_heat = 0;
    }
    else
    {
        if (residue_heart >= 100)
            over_heat = 0;
        else
            over_heat = 1;
    }
}


 /**
  ******************************************************************************
																Ħ���ֿ�������	
	Ħ���������ں���ת����뷴ת����
	 =============================================================================
 **/
 void shoot_friction_handle_42(void)
{
    //Ħ����״̬�ж���ѡ��
    if (RC_CtrlData.inputmode != STOP)//�ǹؿ�
	{
        if((RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag))//����Ħ��������
        {
					LASER_ON();

						   _42mm_shoot.friction_state = NORMAL;

        }else
        {
             _42mm_shoot.friction_state = Stop;
			LASER_OFF();
        }
    }else
    {
        _42mm_shoot.friction_state = Stop;
    }

    //����Ħ����״̬��ʼִ��Ħ����
    switch (_42mm_shoot.friction_state)
    {
    case START:
     {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = LEFT_FRICTION_POLARITY*frictionSpeed_42;
     }
         break;
    case NORMAL:
    {
		
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = LEFT_FRICTION_POLARITY*frictionSpeed_42;
    }
         break;
    case BACK:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = -RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = -LEFT_FRICTION_POLARITY*frictionSpeed_42;
    }
		 break;
    default:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = 0;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = 0;
    }
        break;
    }

}


 /**
  ******************************************************************************
																���̿�������		
		�����߼����ϲ��̲��ýǶȻ��������²���Ϊ���ֵ�·�����������ٶȻ�����
							�������ƶ�ת�����������ջ����
	 =============================================================================
 **/
void shoot_bullet_handle_42(void)
{

	//�����ְ���߼�Ϊ��������������������һ������ź�
    if (RC_CtrlData.mouse.press_l == 1 || RC_CtrlData.RemoteSwitch.trigger == 1)//�������
    {
        if (press_l_first_in == 0)
        {
             press_l_first_in = 1;
             _42mm_shoot.shoot_flag = 1;
        }
        else
        {
             _42mm_shoot.shoot_flag = 0;
        }
    }
    else
    {
        press_l_first_in = 0;
    }
	if (RC_CtrlData.mouse.press_r == 1)
	{
		if (press_r_first_in == 0)
		{
			press_r_first_in = 1;
			_42mm_shoot.inverse_flag = 1;
		
		}
		else
		{
			_42mm_shoot.inverse_flag =0;
		}
	
	
	
	}
	else
	{
		press_r_first_in = 0;
	}
		
    if(_42mm_shoot.friction_state == NORMAL&&over_heat==0)
    {
			
			
        if(poke_init_flag == 0)
        {
					//���̲�����ʼ��
			_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref = general_poke.down_poke.ecd_angle;
          
            _42mm_shoot.pid_downpoke_speed.max_out = 6000;
            poke_init_flag = 1;
        }
        if(_42mm_shoot.shoot_flag)
        {
					//������䣬�²��̸����ۼӣ���������
			_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref += ONE_POKE_ANGLE_42 * POKE_POLARITY;
			
        }
		if(RC_CtrlData.Key_Flag.Key_B_Flag)//ǿ��������������
		{
			Burst_count++;
			if(Burst_count%40==0)
			{_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref += ONE_POKE_ANGLE_42 * POKE_POLARITY;Burst_count = 0;}
		
		}
        if(_42mm_shoot.shoot_flag)
			_42mm_shoot.pid_downpoke_speed.max_out=POKE_MAX_OUT;
		else
		{
			Stall_detection++;
			if(Stall_detection%150 == 0&&RC_CtrlData.Key_Flag.Key_Z_Flag==0)
			{
				if(general_poke.down_poke.filter_rate == 0&&general_poke.down_poke.Torque<-6750)//���̶�ת���
				{
					bullet_lock_flag = 1;//���̶�ת
					bullet_locked_flag = 1;//���̶�תUI��־λ
					_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref = general_poke.down_poke.ecd_angle;//��ת֮���̲�����ʼ��
				}
			
			}
		}
		if(bullet_lock_flag == 1)//��ת��תһ������
		{
			_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref -= ONE_POKE_ANGLE_42 * POKE_POLARITY * 2;
			bullet_lock_flag = 0;
		
		}
		if(_42mm_shoot.inverse_flag == 1)//�Ҽ���תһ������
		{
			_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref -= ONE_POKE_ANGLE_42 * POKE_POLARITY;
		}
		if(general_poke.down_poke.filter_rate <-50)
		{
			bullet_locked_flag = 0;//���¶�תUI��־λ
		}


		_42mm_shoot.shoot_ref_and_fdb.down_poke_angle_fdb = general_poke.down_poke.ecd_angle;
	    _42mm_shoot.shoot_ref_and_fdb.down_poke_angle_ref = _42mm_shoot.shoot_ref_and_fdb.down_poke_angle_dynamic_ref;
	    _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = pid_double_loop_cal(&_42mm_shoot.pid_downpoke_angle,
																			  &_42mm_shoot.pid_downpoke_speed,
																			  _42mm_shoot.shoot_ref_and_fdb.down_poke_angle_ref,
																			  _42mm_shoot.shoot_ref_and_fdb.down_poke_angle_fdb,
																			  &_42mm_shoot.shoot_ref_and_fdb.down_poke_speed_ref,
																			  _42mm_shoot.shoot_ref_and_fdb.down_poke_speed_fdb,
	                                                                          0);
    }else if(_42mm_shoot.friction_state == BACK)
		{
			_42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref-=0.5f;
			_42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = pid_double_loop_cal(&_42mm_shoot.pid_uppoke_angle,
                                                                                &_42mm_shoot.pid_uppoke_speed,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb,
                                                                                &_42mm_shoot.shoot_ref_and_fdb.up_poke_speed_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb,
                                                                                0);
		}else
    {
         poke_init_flag = 0;
         _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = 0;
    }
}

