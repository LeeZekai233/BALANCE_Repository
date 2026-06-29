#include "main.h"

void TF02_Online_Handle(void)
{
    if((time_tick - Chassis.TF02_Middle.Heart_cnt) > 500)
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


