#include "main.h"

void TF02_Online_Handle(TF02_t* TF02)
{
    if((time_tick - TF02->Heart_cnt) > 200)
    {
        TF02->Online_flag = 0;
    }
    else
    {
        TF02->Online_flag = 1;
    }
    
    
    if(TF02->Online_flag == 0)
    {
        TF02->Distance = 0;
    }
    
}


void TF02_Data_Handle(uint8_t* Data,TF02_t* TF02)
{
    if(Data[0] == 0x59 && Data[1] == 0x59)
    {
        TF02->Distance = (uint16_t)(Data[3] << 8) | Data[2];
        TF02->Strength = (uint16_t)(Data[5] << 8) | Data[4];
        TF02->Temperature = ( (uint16_t)(Data[7] << 8) | Data[6] )/8.0f - 256.0f;
        TF02->Heart_cnt = time_tick;
    }
    else
    {
        TF02->Distance = 0;
        TF02->Strength = 0;
        TF02->Temperature = 0;
    }
}
