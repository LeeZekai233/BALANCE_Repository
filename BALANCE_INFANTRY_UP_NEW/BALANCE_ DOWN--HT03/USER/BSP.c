#include "BSP.h"

void BSP_Init(void)
{


NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2



usart1.Init(&usart1,100000,1,0,1);
usart2.Init(&usart2,115200,0,1,1);
usart3.Init(&usart3,115200,0,1,1);
usart4.Init(&usart4,115200,0,1,1);
usart5.Init(&usart5,115200,0,1,1);
usart6.Init(&usart6,921600,0,0,1);


	LED_Init();
#if EN_CAN1 
CAN1_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN初始化正常模式,波特率100Kbps   42M/（6+7+1）/30==1Mps
#endif
#if EN_CAN2
CAN2_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN初始化正常模式,波特率100Kbps   42M/（6+7+1）/30==1Mps
#endif
	
	#if EN_TIM1
		TIM1_Configuration();
	#endif
	#if EN_TIM2
		TIM2_Configuration();
	#endif
	#if EN_TIM3
		TIM3_Configuration();
	#endif
	#if EN_TIM4
		TIM4_Configuration();
	#endif
	#if EN_TIM5
		TIM5_Configuration();
	#endif
	#if EN_TIM6
		TIM6_Configuration();
	#endif
	#if EN_TIM7
		TIM7_Configuration();
	#endif
	#if EN_TIM8
		TIM8_Configuration();
	#endif
	
}
	

	
	




