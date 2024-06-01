/**
  ******************************************************************************
  * @file    control_task.c
  * @author  Lee_Zekai
  * @version V1.0.0
  * @date    01-June-2024
  * @brief   该文件为主控制程序的集中调用，void control_task(void)函数在TIM6中被使用
  *          执行频率为1000hz，计数器为time_tick变量，可在任意源文件中被调用计算，但
  *          不可被赋值，在下述任务中，将用该变量对各个控制任务进行分频。
  *         
  @verbatim
 ===============================================================================
**/



/* Includes ------------------------------------------------------------------*/
#include "control_task.h"

/* counter variable */
int time_tick = 0;
/* 临时全局变量 */
float lp_data = 0;


/**
************************************************************************************************************************
* @Name     : control_task
* @brief    : 控制任务调度
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void control_task(void)
{
/* 计数器计数 */
	time_tick++;
/* 软件看门狗喂狗 */
	IWDG_ReloadCounter();
/* 对底盘陀螺仪的加速度计进行低通滤波  采样时间0.001s*/
	lp_data = Lpf_1st_calcu(&ACC_LPF,chassis_gyro.x_Acc,15,0.001);
/* 一维卡尔曼里程计 */
	Mileage_kalman_filter_calc(&Mileage_kalman_filter,
								((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R,
								((balance_chassis.Driving_Encoder[0].gyro + (-balance_chassis.Driving_Encoder[1].gyro))/2.0f) * WHEEL_R,
								lp_data);
/* 500hz分频下的任务 */
	if(time_tick%2==0)
	{
		balance_chassis_task();//底盘控制任务
		can_bus_send_task();//底盘电机发送任务
		
	}
/* 超级电容通信 */
	if(time_tick%10==9)
	{
        /* 旧超电控制任务
		power_data_set_handle(CAN2,(uint16_t)(b_chassis.Max_power_to_PM01));
        */
        
        /* 新超电控制任务 */
        POWER_Control(&Super_Cap_Send);
	}
	if(time_tick%10==5)
	{
        /* 旧超电获取信息任务
		power_data_read_handle(CAN2);
       */
        
        /* 电容上发至云台ui */
		usart_gimbal_send(usart_capacitance_message.cap_voltage_filte);//
	}
/* 程序运行指示灯与硬件看门狗喂狗 */
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
/* 底盘轮毂电机心跳监测 */
	online_detective(&balance_chassis.Driving_Encoder[0]);
    online_detective(&balance_chassis.Driving_Encoder[1]);
}

/**
************************************************************************************************************************
* @Name     : control_task_Init
* @brief    : 控制任务参数初始化
* @param		: void
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void control_task_Init(void)
{
    /* 底盘参数初始化 */
	balance_param_init();
}
