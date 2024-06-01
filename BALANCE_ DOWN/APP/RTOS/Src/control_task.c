/**
  ******************************************************************************
  * @file    control_task.c
  * @author  Lee_Zekai
  * @version V1.0.0
  * @date    01-June-2024
  * @brief   ���ļ�Ϊ�����Ƴ���ļ��е��ã�void control_task(void)������TIM6�б�ʹ��
  *          ִ��Ƶ��Ϊ1000hz��������Ϊtime_tick��������������Դ�ļ��б����ü��㣬��
  *          ���ɱ���ֵ�������������У����øñ����Ը�������������з�Ƶ��
  *         
  @verbatim
 ===============================================================================
**/



/* Includes ------------------------------------------------------------------*/
#include "control_task.h"

/* counter variable */
int time_tick = 0;
/* ��ʱȫ�ֱ��� */
float lp_data = 0;


/**
************************************************************************************************************************
* @Name     : control_task
* @brief    : �����������
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void control_task(void)
{
/* ���������� */
	time_tick++;
/* ������Ź�ι�� */
	IWDG_ReloadCounter();
/* �Ե��������ǵļ��ٶȼƽ��е�ͨ�˲�  ����ʱ��0.001s*/
	lp_data = Lpf_1st_calcu(&ACC_LPF,chassis_gyro.x_Acc,15,0.001);
/* һά��������̼� */
	Mileage_kalman_filter_calc(&Mileage_kalman_filter,
								((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R,
								((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R,
								lp_data);
/* 500hz��Ƶ�µ����� */
	if(time_tick%2==0)
	{
		balance_chassis_task();//���̿�������
		can_bus_send_task();//���̵����������
		
	}
/* ��������ͨ�� */
	if(time_tick%10==9)
	{
        /* �ɳ����������
		power_data_set_handle(CAN2,(uint16_t)(b_chassis.Max_power_to_PM01));
        */
        
        /* �³���������� */
        POWER_Control(&Super_Cap_Send);
	}
	if(time_tick%10==5)
	{
        /* �ɳ����ȡ��Ϣ����
		power_data_read_handle(CAN2);
       */
        
        /* �����Ϸ�����̨ui */
		usart_gimbal_send(usart_capacitance_message.cap_voltage_filte);//
	}
/* ��������ָʾ����Ӳ�����Ź�ι�� */
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
/* ������챵��������� */
	online_detective(&balance_chassis.Driving_Encoder[0]);
    online_detective(&balance_chassis.Driving_Encoder[1]);
}

/**
************************************************************************************************************************
* @Name     : control_task_Init
* @brief    : �������������ʼ��
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void control_task_Init(void)
{
    /* ���̲�����ʼ�� */
	balance_param_init();
}
