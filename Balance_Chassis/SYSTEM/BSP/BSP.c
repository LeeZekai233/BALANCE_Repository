#include "main.h"

void Infantry_Init(void)
{
    USART1_Init(100000);
    USART3_Init(115200);
    USART6_Init(115200);
    USART4_Init(115200);
    CAN1_Init( );
    CAN2_Init( );
    TIM2_Init( );
    TIM6_Init( );
}


