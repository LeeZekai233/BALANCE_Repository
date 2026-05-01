#include "public.h"

sin_signal_t sin_signal;

void start_sin_cal(int frequent)
{
    if(sin_signal.start_flag == 1)
    {
        sin_signal.t+= 1.0f/frequent;
        sin_signal.output = sin_signal.A*sinf(sin_signal.W*sin_signal.t+sin_signal.PHI);
        
        int cnt = sin_signal.t*frequent;
        if(cnt%frequent==0)
        {
            sin_signal.seconds_point=1;
        }else
        {
            sin_signal.seconds_point=-1;
        }
    }else
    {
        sin_signal.t = 0;
    }
    
}


void sin_signal_Init(float A,float W,float phi)
{
    sin_signal.A = A;
    sin_signal.W = W;
    sin_signal.PHI = phi;
}

