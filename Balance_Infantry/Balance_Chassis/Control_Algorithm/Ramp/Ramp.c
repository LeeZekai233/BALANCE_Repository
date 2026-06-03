#include "main.h"



//* 斜坡函数 用于将当前值平滑地调整到参考值
float trackRamp(float current, float reference) 
	
{
    float delta = reference - current;

     if (fabs(delta) > MAX_DELTA) 
     {
        if (delta > 0) 
        {
            return current + MAX_DELTA;
        } 
        else 
        {
            return current - MAX_DELTA;
        }
    }
    return reference;
}




float trackRamp_leg(float max_delta_leg ,float current, float reference)
	
{
    float delta = reference - current;

    if (fabs(delta) > max_delta_leg) 
    {
        if (delta > 0) 
        {
          return current + max_delta_leg;
        } 
        else 
        {
            return current - max_delta_leg;
        }
    }
    return reference;
}


