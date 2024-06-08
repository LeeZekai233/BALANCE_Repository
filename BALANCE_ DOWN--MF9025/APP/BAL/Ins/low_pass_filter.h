#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H
#include "public.h"

#ifndef __LPF
#define __LPF
typedef struct _lpf_first_order
{
    float fc;       // cut-off frequency
    float y_k1;     // last output
    float alpha;    // filter coefficient
    float ts;       // samping period
    float u_k1;     // last input
}Lpf1stObj;

#endif
float Lpf_1st_calcu(Lpf1stObj *filter, float u_k,float fc, float ts);



extern Lpf1stObj ACC_LPF;
extern Lpf1stObj TEST_LPF;
extern Lpf1stObj L_DDZW_LPF;
extern Lpf1stObj R_DDZW_LPF;




#endif
