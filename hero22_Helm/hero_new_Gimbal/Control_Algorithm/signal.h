#ifndef __SIGNAL_H
#define __SIGNAL_H
//#include <public.h>






typedef struct
{
    uint8_t start_flag;
    int seconds_point;
    float t;
    float output;
    float A;
    float W;
    float PHI;
}sin_signal_t;


void start_sin_cal(int frequent);
void sin_signal_Init(float A,float W,float phi);


extern sin_signal_t sin_signal;





#endif