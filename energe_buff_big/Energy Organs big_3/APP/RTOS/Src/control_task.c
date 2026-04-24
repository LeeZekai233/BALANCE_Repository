#include "control_task.h"
#include "stdbool.h"
int time_tick = 0;
int led_time1 = 1;
int last_led_time1 = 24 ;
int led_time2 = 1;
int last_led_time2 = 5 ;
uint8_t flag_R=0;
int turn_flag = 0;
int energe_mode = 0;
double turn_a=1.045, turn_w=2, turn_b=1.045;
double angle = 0;
long long time_Big_energe=0;
long long time_Small_energe=0;
int rand_time = 0;
int speed_small=0;
int speed_big=0;
uint8_t flag_turn_way=0;
uint8_t send_time=0;
uint8_t flag1=0;
uint8_t flag_color_ready_to_change=0;
bool state=1;
void rand_big_energe(void)
{

	turn_a = rand()%265/1000.0+0.78;
//	turn_a=0.9;
	turn_b = 2.09-turn_a;
	turn_w = rand()%116/1000.0+1.884;
//	turn_w =1.921;
}

void control_task(void)
{	
	time_tick++;
	
	
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)==1)
		flag_color_ready_to_change=1;
	if(flag_color_ready_to_change==1)
	{if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)==0)
		{state=!state;
			delay_ms(20);
		}
	}
	if(state==1)
	{buf_r=250;
	buf_b=0;
	}
	else
	{buf_r=0;
		buf_b=150;
	}

	my_ws2812_set_all_on();

		if(time_tick%1000==1)
		{
	//		Energy_state_Send(USART3_DMA_TX_BUF[0]);
		}
		RC_CtrlData.rc.s2=1;
		energe_mode=1;
		flag_R=1;
	//		if(RC_CtrlData.rc.ch0 == 1)
			
	if(1)		//总开关
	{	
		if(flag_R==1)
		{
				//	my_ws2812_set_all_on();
		}
		else 
		{
			//my_ws2812_set_all_off();
		}
	
		if(time_tick%1==0)
		{
			//		energe_turn_input();
					time_Small_energe++;
					speed_small=PI/3.0*RAD_TO_ANGLE*REDUCTION_RATIO/6;
			speed_big=(turn_a*sin(turn_w*time_Big_energe/1000)+ turn_b)*RAD_TO_ANGLE*REDUCTION_RATIO/6;
				if(flag_turn_way==0)
				{speed_small=-speed_small; 
				speed_big=-speed_big;}
				else
				{speed_small=speed_small;
				speed_big=speed_big;}
					Motor_620_Speed(speed_big,speed_small);  
	
					time_Big_energe++;
				
			/*		angle = (-turn_a/turn_w* cos(turn_w * time_Big_energe/1000) + turn_b*time_Big_energe/1000)*RAD_TO_ANGLE*REDUCTION_RATIO
				-(-turn_a/turn_w )*RAD_TO_ANGLE*REDUCTION_RATIO
				+Motor620_Encoder.angle_bias;
			*/
//			
//					Motor_620_Speed(speed_big,speed_small);                                              //REDUCTION_RATIO

		
		
		if(LED_mode == Waiting_hit)
		{
		circle_key_input();					//检测是否打到
		}
		
		if(time_tick%50==0)
		{
			
		
			
			switch(RC_CtrlData.rc.s1)		//切换大小能量机关
			{
				case RC_SW_UP:
					energe_mode = Small_energe;
					flag_R=1;
					break;
				case RC_SW_DOWN:
					energe_mode = Big_energe;
					flag_R=1;
					break;
				case RC_SW_MID:
					energe_mode = Energe_none;
				flag_R=1;
					break;
			}
		}
		if(time_tick%50==5)
		{
			Energy_RUN();
			send_time+=1;
			flag1=1;
		}		
		if(time_tick%150==0)
		{
				led_time1++;
				if(led_time1>8) led_time1 = 1;
		}	
	}
//	else if(RC_CtrlData.rc.s2 == RC_SW_DOWN)
//	{
//		flag_R=0;
//		if(time_tick%50==5)
//		{
//	//		Energy_off_RUN();
//			
//		}
//	}
//	else if(RC_CtrlData.rc.s2 == RC_SW_MID)
//	{
//		if(time_tick%50==5)
//		{
//	//		Energy_off_RUN();
//			if(RC_CtrlData.rc.ch0!=1024)//转向
//	{	flag_turn_way=1;
//	}
//	else
//	{flag_turn_way=0;
//	}

//		}
//	}
		
	
	if(time_tick%10000000==0)
		time_tick=0;
}

}
int rc_input = 0;
int last_rc_input = 0;
void energe_turn_input(void)
{
	rc_input = RC_CtrlData.rc.s1;
	if(rc_input != last_rc_input)
	{
		turn_flag = 1;
	}
	last_rc_input = rc_input;
	
	if(turn_flag == 1)
	{
		clean_energe_mode();
		turn_flag = 0;
		rand_big_energe();
	}
}

void clean_time(void)
{
	led_time1 = 1;
	last_led_time1 = 24 ;
	led_time2 = 1;
	last_led_time2 = 5 ;
	
}	


void control_task_Init(void)
{
	
}


