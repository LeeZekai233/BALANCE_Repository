#include "BSP.h"
//串口2,5使能,can1,2,定时器356


void BSP_Init(void)
{


NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2


	//	
#if EN_USART1
	usart1_init(100000);	//初始化串口波特率为115200
#endif

#if EN_USART3
	usart3_init(921600);	//初始化串口波特率为115200
#endif
#if EN_UART4
	uart4_init(115200);	//初始化串口波特率为115200
#endif
#if EN_UART5
	uart5_init(115200);	//初始化串口波特率为115200
#endif
#if EN_USART6
	usart6_init();	//初始化串口波特率为115200
#endif



	Sucker_PC12_Init();
	Sucker_PA1_Init();
	Sucker_PC11_Init();
	Sucker_PB5_Init();
	Sucker_PB4_Init();
	Sucker_PB3_Init();
	#if EN_USART2
 USART2_Tx_Init(115200);	//初始化串口波特率为115200
#endif

#if EN_CAN1 

CAN1_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN初始化正常模式,波特率100Kbps   42M	/（6+7+1）/30==1Mps
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
	

	
	









