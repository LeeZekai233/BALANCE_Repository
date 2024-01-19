#include "control_task.h"


int time_tick = 0;
float lp_data = 0;
float data;
void control_task(void)
{
	time_tick++;
	
	lp_data = Lpf_1st_calcu(&ACC_LPF,chassis_gyro.x_Acc,15,0.001);
	Mileage_kalman_filter_calc(&Mileage_kalman_filter,
								((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R,
								((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R,
								lp_data);
//	data = ((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R;
	if(time_tick%2==0)
	{
		balance_chassis_task();
		
	}
	can_bus_send_task();
	if(time_tick%10==9)
	{
		power_data_set_handle(CAN2,b_chassis.Max_power_to_PM01);
	}
	if(time_tick%10==5)
	{
		power_data_read_handle(CAN2);
	}
		
	
	
}

void control_task_Init(void)
{
	balance_param_init();
}
