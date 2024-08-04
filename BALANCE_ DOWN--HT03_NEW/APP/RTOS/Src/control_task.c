#include "control_task.h"


int time_tick = 0;


void control_task(void)
{
	time_tick++;
	IWDG_ReloadCounter();            //������Ź�ι��
	
	balance_chassis_task();//���̿�������
    
	if(time_tick%2==0)
	{
		can_bus_send_task();//���̵����������
	}
	
	if(time_tick%10==9)
	{
//		power_data_set_handle(CAN2,(uint16_t)(b_chassis.Max_power_to_PM01));//�����������
        POWER_Control(&Super_Cap_Send);
	}
	if(time_tick%5==0)
	{
//		power_data_read_handle(CAN2);//�����ȡ��Ϣ����
		usart_gimbal_send(usart_capacitance_message.cap_voltage_filte,usart_capacitance_message.in_v,(b_chassis.left_leg.phi4+b_chassis.right_leg.phi4)/2.0,(b_chassis.left_leg.phi1+b_chassis.right_leg.phi1)/2.0,(b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f,b_chassis.balance_loop.L0,b_chassis.jump_flag);//�����Ϸ�����̨ui
	}
	//��������ָʾ����Ӳ�����Ź�ι��
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
    if(time_tick%10==7)
    {
        
    }
	online_detective(&balance_chassis.Driving_Encoder[0]);
    online_detective(&balance_chassis.Driving_Encoder[1]);
    TF02_online_detective();
}

void control_task_Init(void)
{
	balance_param_init();
}
