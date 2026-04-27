#ifndef __PID_H
#define __PID_H
#include <stm32f4xx.h>

/*********************************限幅*********************************/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
/*********************************限幅*********************************/
    
    
    
    


    
/********************************PID结构体******************************/
typedef enum 
{
    PID_POSITION = 0,     //位置式PID
    PID_DELTA    = 1,     //增量式PID
}PID_Mode_e;

typedef struct
{
    PID_Mode_e mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

    float input_max_err;    //最大输入误差
} PID_t;
/**********************************PID结构体***********************************/




/**********************************函数声明**********************************/
void PID_Init(PID_t *pid, PID_Mode_e mode,float kp,float ki,float kd, float max_out, float max_iout);
float PID_Calc(PID_t* pid, float get, float set);
void PID_Clear(PID_t *pid);
/**********************************函数声明**********************************/



#endif
