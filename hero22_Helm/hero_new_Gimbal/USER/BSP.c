#include "public.h"

//extern Encoder V;

void BSP_Init(void){
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    #if EN_CAN1
        Can1_Init(CAN_BS1_9tq,CAN_BS2_4tq,3,CAN_Mode_Normal);
    #endif
    
    #if EN_CAN2
        Can2_Init(CAN_BS1_9tq,CAN_BS2_4tq,3,CAN_Mode_Normal);
    #endif
    
    #if EN_TIM6
        Tim6_Init();
    #endif
    
    #if EN_TIM2
        TIM2_Init();
    #endif
    
		TIM5_PWM_Init();
    #if EN_USART3
        Usart3_Init(921600);
    #endif
    
    #if EN_USART1
        Usart1_Init(100000);
    #endif
	
	#if EN_USART2
		Usart2_Init(115200);
	#endif
    
    #if EN_USART6
        usart6_init();//921600
    #endif
    
    #if EN_UART4
        uart4_init(1000000);
    #endif
    
    #if EN_UART5
        USART5_DMA_R_T_JUDGE_Init();
        //USART5_DMA_R_T_JUDGE_Init();
    #endif
    
    
    //GM6020_PID_task_Init(&V);
    //LK_5010_pitch_pid_Init();
    
    
    Gimbal_parameter_Init();
    _42mm_Shoot_PID_Init();
    C620_3508_PID_Init();//╣вел
    New_Speed_PID_Init(&chassis);
    //Power_Num_Init();
}

float convert_ecd_angle_to_0_2180(double ecd_angle,float _0_2pi_angle)
{
	_0_2pi_angle=fmod(ecd_angle,2*180.0);	
	if(_0_2pi_angle<0)
		 _0_2pi_angle+=2*180.0;

	return _0_2pi_angle;
}

float convert_ecd_angle_to__pi_pi(double ecd_angle,float __pi_pi_angle)
{
	float temp1;
	temp1 = convert_ecd_angle_to_0_2180(ecd_angle,temp1);
		if(temp1>180.0)
		{__pi_pi_angle=temp1-(2*180.0);}
		else
		{__pi_pi_angle=temp1;}
		
		return __pi_pi_angle;
}



