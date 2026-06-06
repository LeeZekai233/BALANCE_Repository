#ifndef __SHOOTER_TASK_H
#define __SHOOTER_TASK_H
#include "stm32f4xx.h"                  // Device header

#include "PID.h"
#include "DJI_Motor.h"
#include "LK.h"

#define    LEFT_FIRC_SPEED        2000
#define    RIGHT_FRIC_SPEED      -2000
/*********************************发射结构体**********************************/
typedef enum
{
    SHOOTER_RELAX = 0,//发射机构失能
    SINGLE_SHOOT  = 1,//单发
    BURST_FIRE    = 2,//连发
    STOP_FIRE     = 3,//停火
}Shooter_Mode_e;


typedef enum
{
    SHOOTER_NORMAL = 0       ,
	SHOOTER_TRAP   =  1      ,//卡弹
   // SHOOTER_TRAP_HANDLE  = 2   ,//卡弹处理
}Shooter_State_e;


typedef enum
{
    FSM_RELAX = 0,
    ACTIVATE  = 1,
    HANDLE    = 2,
}CF_FSM_State_e;//停火有限状态机，改善空程和双发用


typedef enum
{
    POKE_OFF = 0,
    POKE_ON  = 1,
}Poke_State_e;//拨盘状态，只用来看


typedef enum
{
    FRIC_OFF = 0,
    FRIC_ON  = 1,
}Fric_State_e;//摩擦轮状态，只用来看



typedef enum
{
    STOP = 0,//停机
    DETECT = 1,//检测
    CONFIRM = 2,//确认发射
}Heat_Detect_FSM_e;//热量检测状态机


typedef struct
{
    Heat_Detect_FSM_e Heat_Detect_FSM;//热量检测状态机
    
    uint16_t Now_Heat;//当前热量
//    uint16_t Warn_Heat;//热量预警阈值
    uint8_t Heat_CD;//冷却热量速度，裁判系统读
    uint16_t Residue_Heat;//剩余热量
    uint16_t Threshold_Heat;//热量门限阈值
    uint16_t Max_Heat;//裁判系统读最大热量
    
    uint16_t Heat_Detect_CNT;//热量检测计数
}Heat_Restrict_t;


typedef struct
{
    
    Shooter_Mode_e Shooter_Mode;//发射机构模式
    Shooter_Mode_e Last_Shooter_Mode;//上一次发射机构模式
    Shooter_State_e Shooter_State;//发射机构状态
    Shooter_State_e Last_Shooter_State;//上一次发射机构状态
//    CF_FSM_State_e CF_FSM_State;//停火有限状态机 //暂时不会使用
    Poke_State_e Poke_State;//拨盘模式
    Fric_State_e Fric_State;//摩擦轮状态
    
    Encoder_t Poke_Motor_Encoder;//拨盘电机编码器    LK4005
    Encoder_t Fric_Motor_Encoder[2];//摩擦轮电机编码器 M3508
    
    PID_t Poke_Speed_PID;//拨盘速度PID
    PID_t Poke_Angle_PID;//拨盘角度PID
    PID_t Fric_Speed_PID[2];//摩擦轮速度PID
    
    Heat_Restrict_t Heat_Restrict;//热量限制
    
    float Poke_Angle_Ref;//拨盘角度参考值，单位 °
    float Poke_Angle_Fdb;//拨盘角度反馈值，单位 °
    
    float Poke_Speed_Ref;//拨盘速度参考值，单位 °/s
    float Poke_Speed_Fdb;//拨盘速度反馈值，单位 °/s
    
    float Fric_Speed_Ref[2];//摩擦轮速度参考值,单位rpm
    float Fric_Speed_Fdb[2];//摩擦轮速度反馈值，单位rpm
    
    uint16_t Poke_Trap_CNT;//拨盘卡弹计数，拨盘电流过大增加计数
    uint16_t Poke_Trap_Handle_CNT;//拨盘卡弹处理计数，时间到之后回归到正常模式
    uint16_t Burst_Fire_Cnt;//把连发当成单发连点器用，可能可以降低延迟
    
//    uint16_t Fric_Fire_CNT;//停火有限状态机用，摩擦轮电流大增加计数  //不使用停火有限状态机
//    uint16_t FSM_Activate_CNT;//状态机计数，计数达到目标直接转换状态
    
    int16_t Poke_Motor_Set_Current;
    int16_t Fric_Motor_Ser_Current[2];
    
}Shooter_t;
/*********************************发射结构体**********************************/




/*********************************外部声明**********************************/
extern Shooter_t Shooter;
/*********************************外部声明**********************************/


void Shooter_Mode_Select(void);
void Shooter_Feedback_Update(void);
void Shooter_Reference_Update(void);
void Shooter_State_Update(void);
void Shooter_Task(void);


#endif

