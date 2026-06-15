#include "main.h"

void Infantry_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(100000);
    USART3_Init(921600);
    USART6_Init(921600);
    UART4_Init(115200);
    usart2_init(921600);
    CAN1_Init( );
    CAN2_Init( );
    
    Control_Task_Init();//PID≥ı ºªØ
    
  //  TIM2_Init( );
    TIM6_Init( );
}


