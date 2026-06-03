#ifndef __HIGH_PASS_FILTER_H
#define __HIGH_PASS_FILTER_H
#include <stm32f4xx.h>



typedef struct _hpf_first_order
{
    float fc;       // cut-off frequency  截止频率
    float y_k1;     // last output        上一次滤波后的输出值
    float alpha;    // filter coefficient 滤波系数
    float ts;       // samping period     采样周期
    float u_k1;     // last input         上一次的原始输入信号
}Hpf1stObj;

float hpf_1st_calcu(Hpf1stObj *filter, float u_k,float fc, float ts);





extern Hpf1stObj ACC_X_HIGHP;







#endif

