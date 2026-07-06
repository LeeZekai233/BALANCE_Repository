#include "main.h"

void Infantry_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(921600);
//    usart2_init(921600);
    USART3_Init(921600);
    USART6_Init(921600);
    UART4_Init(921600);
    USART5_DMA_R_T_JUDGE_Init();
    CAN1_Init( );
    CAN2_Init( );
    
    Control_Task_Init(&Chassis);//PID≥ı ºªØ
    
  //  TIM2_Init( );
    TIM6_Init( );
}


