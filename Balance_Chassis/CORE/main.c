#include "main.h"

int main( )
{
        Control_Task_Init(&Chassis);
    Infantry_Init( );
   
	while (1)
	{
if (USART_GetFlagStatus(UART4, USART_FLAG_ORE) != RESET)
{
    (void) UART4->SR;
    (void) UART4->DR;
}
	}
}

