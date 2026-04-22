#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include <stm32f4xx.h>
#include "PID.h"
#include "Generic_Encoder.h"
#include "Board_Connected_Task.h"
#include "USART3.h"



#define VAL_LIMIT(val, min, max)\
            if(val<=min)\
            {\
                val = min;\
            }\
            else if(val>=max)\
            {\
                val = max;\
            }\

            
            
#define WHEEL_R                        0.058
            
#define JM1_POLARITY                      1//右前电机极性   //老代码里的电机极性
#define JM2_POLARITY                     -1//左前电机极性
#define JM3_POLARITY                     -1//左后电机极性
#define JM4_POLARITY                      1//右后电机极性
            
#define LEFT_WHEEL_POLARITY              -1//左轮电机极性 //老车上的
#define RIGHT_WHEEL_POLARITY              1//右轮电机极性

#define JOINT_MAX_T                       34          //老代码里的限幅
#define WHEEL_MAX_T                       4.35 // 4.3

#define TIME_STEP                         2
            
            
typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  CHASSIS_INIT 			 = 2,
  MANUAL_FOLLOW_REMOTE   = 3,
  CHASSIS_STAND_MODE     = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,
  CHASSIS_DOWN_MODE      = 9,
  CHASSIS_SEPARATE 		 = 10,
  CHASSIS_AUTO_SUP       = 11,
  CHASSIS_SINGLE_LEG_HANDLE =12,
} Chassis_Mode_e;//底盘模式

typedef struct
{
	float theta;
	float dtheta;
	float ddtheta;
	float phi;
	float dphi;
	float x;
	float dx;
	float ddz;
	float wheel_dx;
	float Fm;
    float RPM;
	double K_error[2][6];
	double L0;
	double K[12];
	float k[2][6];
	float state_err[6];

	double lqrOutT;
	double lqrOutTp;
    
}LQR_System;//LQR参数

typedef struct
{
	float V_x;
	float Y_position;
	float V_y;
	float V_w;
	float Remote_Angle;
	float Remote_Speed;
	float Roll;
	float Pitch;
	float Leglength;
}Chassis_Ref_t;

typedef struct
{
	//float pos[2];//pos=[l0; phi0];
	//float spd[2];//spd[2]=[dl0; dphi0];
	float T_Set[2];//T[2]=[motor4;motor1];

	//支持力解算用计算变量
	float J[4];   //解雅可比矩阵的中间变量
	float j[2][2];//最终的雅可比矩阵
	float F_fdb;
	float Tp_fdb;

	float phi4;
	float phi1;
	float dphi4;
	float dphi1;

	float this_dl0;
	float last_dl0;

	float l0;
	float dl0;
	float ddl0;
	float phi0;
	float dphi0;
    
	float Leg_Length_Outer;
	float Leg_F;
	float ddzw;
	float Leg_FN;
	float Leg_Final_FN;

	uint8_t Wheel_State;

	PID_t Leg_Length_PID;//腿长PID
	
}Leg_State_t;//腿状态

typedef enum
{
    RELAX_STATE  = 0,//放松状态
    NORMOL_STATE = 1,//正常状态
    ROLL_STATE   = 2,//侧翻状态
    FLIP_STATE   = 3,//倒扣状态
}Init_State_e;

typedef struct
{
	Chassis_Mode_e Control_Mode;
	Chassis_Mode_e Last_Control_Mode;
    
	u8 jump_flag;//这些之后照我的习惯改成枚举
	u8 overstep_flag;
    Init_State_e Init_State;
	u8 rotate_flag;
	LQR_System balance_loop;
	Chassis_Ref_t Chassis_Ref;//实际参考值
	Chassis_Ref_t Chassis_Remote_Ref;//控输入的参考值
    
    CH040DATA_t Chassis_GYRO;
    
	float vw_limit_rate;
	
	Leg_State_t Left_Leg;       //左腿状态
	Leg_State_t Right_Leg;      //右腿状态
   // Leg_State_t Double_Leg;
	
	PID_t Leg_Harmonize_Pid_Inner;
	PID_t Leg_Harmonize_Pid_Outer;
	PID_t V_w_Pid;
	
	PID_t Roll_Pid_Angle;//roll角度pid
	PID_t Roll_Leg_F_Pid;
	PID_t Roll_leg_F_Rotate_Pid;
	
	PID_t Pid_Follow_Gimbal;
	
	
	PID_t Pid_Yaw_Dist;
	PID_t pid_chassis_side;
	
	PID_t Pid_Seperate_Gimbal;
	
	PID_t Init_Tp_Pid;
	PID_t Init_Tp_dphi0_Pid;
	PID_t Init_Tp_phi0_Pid;
	
	PID_t Over_Step_phi0_Left_Pid;
	PID_t Over_Step_phi0_Right_Pid;
	
	PID_t Init_phi0_pid_left;
	PID_t Init_phi0_pid_right;
	
	
    PID_t Init_dphi0_pid_left;
	PID_t Init_dphi0_pid_right;
	u16 Max_power_to_PM01;//好像没用过
	
	float Left_theta;
    float Right_theta;


	double yaw_encoder_ecd_angle;
	float yaw_angle_0_2pi;
	float yaw_angle__pi_pi;
	float normal_Y_erroffset;
	float remote_ref_vx;
	
	float predict_power;
	float Max_Speed;
	float Min_Speed;
	//tqouce
	double joint_T[4];
	double driving_T[2];
    
    Encoder_t Driving_Motor[2];
    Encoder_t Joint_Motor[4];
    
    float Balance_Tpgain;
    float Balance_Tpoutlandgain;
    
    uint8_t Jump_State;//跳跃状态
    
    USART_Chassis_Data_t USART_Chassis_Data;//串口传来的控制底盘数据
    
    float dphi0;//左右腿平均dphi0
    float phi0;//左右腿平均phi0
    float dtheta;//平均dtheta
    
    float Init_Tp;//初始化调整腿角度的力矩
    
    float Harmonize_Outer;//双腿协调外环
    float Harmonize_Inner;//双腿协调内环
    
}Balance_Chassis_t;//复制来的，有些没用




extern Balance_Chassis_t Chassis;



float Normalize_Angle_PI(float angle);
float Transform_Angle_0_2PI(float angle);
void Motor_Out_Limit(Balance_Chassis_t* Chassis);
void Motor_Torque_Set(Balance_Chassis_t* Chassis,float Joint_T_0,float Joint_T_1,float Joint_T_2,float Joint_T_3,float Driving_T_1,float Driving_T_2);
void Init_Tp_Calc(float Ref_Leglength,float Harmonize,float Init_Tp,Balance_Chassis_t* Chassis);
void Chassis_Param_Init(Balance_Chassis_t* Chassis);
void Chassis_State_Update(Balance_Chassis_t* Chassis);
void Chassis_Relax_Handle(Balance_Chassis_t* Chassis);
void Chassis_Init_State_Update(Balance_Chassis_t* Chassis);
void Chassis_Init_Handle(Balance_Chassis_t* Chassis);
void Balance_Task(Balance_Chassis_t* Chassis);
void Chassis_Control_Loop(Balance_Chassis_t* Chassis);

#endif
