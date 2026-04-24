#include "control_task.h"
//本程序作为副板对23页进行控制仅使用tim5
//通信接受后板发送RC值并转给主板，并从主板接受led.mode
//2.27pa3的usart2接受完成；
//2.27can通信同步状态，或者不通信了
//2.28修符，改兑换站的灯
//2.29can通信同步状态，
int time_tick = 0;
int led_time1 = 1;
int all_finish_time[5]={0,0,0,0,0};
int last_led_time1 = 24 ;

int time1 =0;
int turn_flag = 0;
int energe_mode = 0;
int energe_leaf = 0;

double turn_a=1.045, turn_w=2, turn_b=1.045;
double angle = 0;
long long time_Big_energe=0;

int energe_leaf1_rand_flag = 0;
int energe_leaf2_rand_flag = 0;
int energe_leaf3_rand_flag = 0;
int energe_leaf4_rand_flag = 0;
int energe_leaf5_rand_flag = 0;
int energe_leaf_all_rand_flag = 0;
leaf LED[5]={0,0,0,0,0};
int current_leaf=0;
uint8_t n;
uint8_t m;
uint8_t flag_start=0;
uint8_t count_leaf=0;//完成数
uint8_t flag_all_finish=0;
uint8_t finish_flag=0;
uint8_t rand_finish=0;
void LED_task(void)
{		

if(time_tick%50==0)
	{   
        m=1;
        switch(LED[m].mode)
		{
            case Waiting_hit:		
                    LED_current(m);				//流水箭头对应数组填充
		
                    side_off(m);					//侧面关
                    LED_target(m);				//靶子图案		
//			LED_current_all_on(m);		//箭头流水全亮
//			side_on(m);
//			LED_circle(m);
                    break;
			case hit_finish:
                    LED_current_all_on(m);		//箭头流水全亮
                    side_off(m);					//侧面亮
                    LED_circle(m);				//靶子全亮		//流水			//检测
                    break;
			case LED_none:
					my_ws2812_set_all_off(m);
                    break;
			case all_finish:			
				LED_current_all_on(m);		//箭头流水全亮
					side_off(m);					//侧面亮
					LED_circle(m);	
				break;
		}	   
					transfer(m);
	}
	if(time_tick%50==1)
	{
        m=2;
	switch(LED[m].mode)
		{
	case Waiting_hit:		
					LED_current(m);				//流水箭头对应数组填充
		
					side_off(m);					//侧面关
					LED_target(m);				//靶子图案		
//			LED_current_all_on(m);		//箭头流水全亮
//			side_on(m);
//			LED_circle(m);
					break;
				case hit_finish:
					LED_current_all_on(m);		//箭头流水全亮
					side_off(m);					//侧面亮
					LED_circle(m);				//靶子全亮		//流水			//检测
				break;
			case LED_none:
										my_ws2812_set_all_off(m);
				break;
			case all_finish:			
				LED_current_all_on(m);		//箭头流水全亮
					side_off(m);					//侧面亮
					LED_circle(m);	
				break;
		}	   
				transfer(m);
	
	}
}
void control_task(void)
 {	
	time_tick++;
    LED_task();
	switch(RC_CtrlData.rc.s2)
	{	case 0:	//测试用
			flag_start=1;
		break;
		case 1:	
			flag_start=1;
		break;
		case 2:	
			flag_start=1;
		break;
		case 3:	
			flag_start=0;
		break;
	}
	
	CAN1_Send(14000,0,0,0,0);



	
	if(time_tick%10000000==0)
		time_tick=0;
}

int rc_input = 0;
int last_rc_input = 0;

void clean_time(void)
{
	led_time1 = 1;
	last_led_time1 = 24 ;

}	


void control_task_Init(void)
{
	
}
