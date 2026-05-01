#include "public.h"


 
void gimbal_ESO_param_Init(gimbal_ESO_t *eso,float C1,float C2,float tao,float J,float D,float b0,float w0,float z1_init,float z2_init,float z3_init)
{
    eso->z1 = z1_init;
    eso->z2 = z2_init;
    eso->z3 = z3_init;
    
    eso->m0 = (-D/(J*tao));
    eso->m1 = (-(D*tao+J)/(J*tao));
    eso->b = ((C1*C2)/(J*tao));
    
    eso->w = w0;
    eso->w0 = exp(-eso->w*h);
    eso->b0 = b0;
    
    eso->l1 = (h*eso->m1-3*eso->w0+3);
    eso->l2 = ((3*eso->w0*eso->w0-3*eso->w0*h*eso->m1-6*eso->w0+h*h*eso->m1*eso->m1+eso->m0*h*h+3*h*eso->m1+3)/h);
    eso->l3 = (((1-eso->w0)*(1-eso->w0)*(1-eso->w0))/(h*h));
    
    eso->phiE[0][0] = 1-eso->l1;            eso->phiE[0][1] = h;           eso->phiE[0][2] = 0;
    eso->phiE[1][0] = eso->m0*h-eso->l2;    eso->phiE[1][1] = eso->m1*h+1; eso->phiE[1][2] = h;
    eso->phiE[2][0] = -eso->l3;             eso->phiE[2][1] = 0;           eso->phiE[2][2] = 1;
    
    eso->BE[0][0] = 0;        eso->BE[0][1] = eso->l1;
    eso->BE[1][0] = eso->b*h; eso->BE[1][1] = eso->l2;
    eso->BE[2][0] = 0;        eso->BE[2][1] = eso->l3;
    
//    eso->l1 = (-eso->m0*h*h+eso->m1*h-eso->w0*eso->w0*eso->w0+1)/(-eso->m0*h*h+eso->m1*h+1);
//    eso->l2 = (eso->w0*eso->w0*eso->w0+3*eso->m0*eso->w0*h*h-3*eso->w0*h*eso->m1-3*eso->w0-eso->m0*h*h*h*eso->m1+h*h*eso->m1*eso->m1-2*eso->m0*h*h+3*h*eso->m1+2)/(-eso->m0*h*h*h+eso->m1*h*h+h);
//    eso->l3 = ((1-eso->w0)*(1-eso->w0)*(1-eso->w0))/(h*h);
//    
//    eso->phiE[0][0] = 1-eso->l1-h*eso->l2;                                              eso->phiE[0][1] = h;           eso->phiE[0][2] = 0;
//    eso->phiE[1][0] = eso->m0*h-eso->l1*eso->m0*h-(eso->m1*h+1)*eso->l2-eso->l3*h;      eso->phiE[1][1] = eso->m1*h+1; eso->phiE[1][2] = h;
//    eso->phiE[2][0] = -eso->l3;                                                         eso->phiE[2][1] = 0;           eso->phiE[2][2] = 1;
//    
//    eso->BE[0][0] = 0;        eso->BE[0][1] = eso->l1+h*eso->l2;
//    eso->BE[1][0] = eso->b*h; eso->BE[1][1] = eso->m0*h*eso->l1+(eso->m1*h+1)*eso->l2+h*eso->l3;
//    eso->BE[2][0] = 0;        eso->BE[2][1] = eso->l3;
//    
//    eso->HE[0][0] = 1-eso->l1;   eso->HE[0][1] = 0;   eso->HE[0][2] = 0;
//    eso->HE[1][0] = -eso->l2;    eso->HE[1][1] = 1;   eso->HE[1][2] = 0;
//    eso->HE[2][0] = -eso->l3;    eso->HE[2][1] = 0;   eso->HE[2][2] = 1;
//    
//    eso->JE[0][0] = 0; eso->BE[0][1] = eso->l1;
//    eso->JE[1][0] = 0; eso->BE[1][1] = eso->l2;
//    eso->JE[2][0] = 0; eso->BE[2][1] = eso->l3;
}

void gimbal_ESO_cal(gimbal_ESO_t *eso,float u,float y)
{
    
    eso->u = u;
    eso->y = y;
    
    eso->w0 = exp(-eso->w*h);
    
    eso->l1 = (h*eso->m1-3*eso->w0+3);
    eso->l2 = ((3*eso->w0*eso->w0-3*eso->w0*h*eso->m1-6*eso->w0+h*h*eso->m1*eso->m1+eso->m0*h*h+3*h*eso->m1+3)/h);
    eso->l3 = (((1-eso->w0)*(1-eso->w0)*(1-eso->w0))/(h*h));
    
    eso->phiE[0][0] = 1-eso->l1;            eso->phiE[0][1] = h;           eso->phiE[0][2] = 0;
    eso->phiE[1][0] = eso->m0*h-eso->l2;    eso->phiE[1][1] = eso->m1*h+1; eso->phiE[1][2] = h;
    eso->phiE[2][0] = -eso->l3;             eso->phiE[2][1] = 0;           eso->phiE[2][2] = 1;
    
    eso->BE[0][0] = 0;        eso->BE[0][1] = eso->l1;
    eso->BE[1][0] = eso->b*h; eso->BE[1][1] = eso->l2;
    eso->BE[2][0] = 0;        eso->BE[2][1] = eso->l3;
    
    eso->z1 = eso->phiE[0][0]*eso->z1+eso->phiE[0][1]*eso->z2+eso->phiE[0][2]*eso->z3 + eso->BE[0][0]*eso->u+eso->BE[0][1]*eso->y;
    eso->z2 = eso->phiE[1][0]*eso->z1+eso->phiE[1][1]*eso->z2+eso->phiE[1][2]*eso->z3 + eso->BE[1][0]*eso->u+eso->BE[1][1]*eso->y;
    eso->z3 = eso->phiE[2][0]*eso->z1+eso->phiE[2][1]*eso->z2+eso->phiE[2][2]*eso->z3 + eso->BE[2][0]*eso->u+eso->BE[2][1]*eso->y;
    
//    eso->y1 = eso->HE[0][0]*eso->z1+eso->HE[0][1]*eso->z2+eso->HE[0][2]*eso->z3 + eso->JE[0][0]*eso->u+eso->JE[0][1]*eso->y;
//    eso->y2 = eso->HE[1][0]*eso->z1+eso->HE[1][1]*eso->z2+eso->HE[1][2]*eso->z3 + eso->JE[1][0]*eso->u+eso->JE[1][1]*eso->y;
//    eso->y3 = eso->HE[2][0]*eso->z1+eso->HE[2][1]*eso->z2+eso->HE[2][2]*eso->z3 + eso->JE[2][0]*eso->u+eso->JE[2][1]*eso->y;
    
    eso->output = eso->z3*(1.0/eso->b0);
}
