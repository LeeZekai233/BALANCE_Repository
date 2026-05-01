#ifndef __LESO_H__
#define __LESO_H__

#include <stm32f4xx.h>


#define h 0.001


typedef struct
{
    float m0;
    float m1;
    float b;
    
    float l1;
    float l2;
    float l3;
    
    float b0;
    float w;
    float w0;
    
    float u;
    float y;
    float output;
    
    float z1;
    float z2;
    float z3;
    
    float y1;
    float y2;
    float y3;
    
    float phiE[3][3];
    float BE[3][2];
    float HE[3][3];
    float JE[3][2];
}gimbal_ESO_t;


void gimbal_ESO_param_Init(gimbal_ESO_t *eso,float C1,float C2,float tao,float J,float D,float b0,float w0,float z1_init,float z2_init,float z3_init);
void gimbal_ESO_cal(gimbal_ESO_t *eso,float u,float y);




#endif
