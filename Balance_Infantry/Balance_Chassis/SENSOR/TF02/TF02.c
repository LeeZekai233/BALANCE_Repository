#include "main.h"

void TF02_Online_Handle(void)
{
    if((time_tick - Chassis.TF02_Middle.Heart_cnt) > 200 && Chassis.Control_Mode != CHASSIS_ANTI_FLY_SLOPE)
    {
        Chassis.TF02_Middle.Online_flag = 0;
    }
    else
    {
        Chassis.TF02_Middle.Online_flag = 1;
    }
    
    
    if(Chassis.TF02_Middle.Online_flag == 0)
    {
        Chassis.TF02_Middle.Distance_mm = 0;
    }
}


