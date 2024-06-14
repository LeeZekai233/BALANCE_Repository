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
	if(time_tick%10==5)
	{
//		power_data_read_handle(CAN2);//�����ȡ��Ϣ����
		usart_gimbal_send(usart_capacitance_message.cap_voltage_filte);//�����Ϸ�����̨ui
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
}

void control_task_Init(void)
{
	balance_param_init();
}
