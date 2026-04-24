#include "main.h"
#include "Motor.h"
int speed = 0;

int main()
{
	
	BSP_Init();
rand_big_energe();
    
    // motor_run_init();
    
	
	while(1)
	{
	//GPIO_SetBits(GPIOC,GPIO_Pin_10);
//什么都不放烧录会导致芯片索锁死
		//Motor_620_out1(CAN1,0,100,0 ,0 );
        //Motor_Run();        
	}
}
