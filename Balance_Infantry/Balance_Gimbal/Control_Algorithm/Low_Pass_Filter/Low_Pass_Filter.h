#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H
#include <stm32f4xx.h>


typedef struct _lpf_first_order
{
    float fc;       // cut-off frequency  截止频率
    float y_k1;     // last output        上一次滤波后的输出值
    float alpha;    // filter coefficient 滤波系数
    float ts;       // samping period     采样周期
    float u_k1;     // last input         上一次的原始输入信号
}Lpf1stObj;

float Lpf_1st_calcu(Lpf1stObj *filter, float u_k,float fc, float ts);


extern Lpf1stObj ACC_LPF;



#endif
