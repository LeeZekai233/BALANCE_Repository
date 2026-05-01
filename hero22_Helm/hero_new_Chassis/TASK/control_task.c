#include "public.h"
extern pid_t p1;//angle
extern pid_t p2;//speed
extern chassis_t 	chassis;
extern int16_t wheel_rpm[4];
uint32_t time_tick=0;
int heat_tick;

    
void control_task(void)
{
	//ÐÄÌø¼ì²â
	Peripheral_State_Judge(&Peripheral_State,0.005);
	if(Peripheral_State.Gimbal_CAN_Data.Link_State!=CONNECTED)
	{
		RC_CtrlData.inputmode=STOP;
	}
	
	

    Chassis_ModeSelect();
    Chassis_Encoder_Get();
	
    
    if(time_tick % 2 == 0)
    {
	}
    
		if(RC_CtrlData.inputmode==STOP)
		{
			for(int i=0;i<4;i++)
            {
                chassis.Helm_3508_Out[i]=0;
				chassis.Helm_6020_Out[i]=0;
            }
			
		}
    
    can_send();

    time_tick++;
    if(time_tick>=10000)
    {
        time_tick=0;
    }
		
  
}