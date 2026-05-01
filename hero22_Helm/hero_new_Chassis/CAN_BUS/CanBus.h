#ifndef __CANBUS_H
#define __CANBUS_H
//#include "public.h"

#define DM4310 1
#define MF3508 0

/**********************舵轮底盘电机id||麦轮底盘电机id**************************************/
#define GM1Encoder_MOTOR 0x205
#define GM2Encoder_MOTOR 0X206
#define GM3Encoder_MOTOR 0X207
#define GM4Encoder_MOTOR 0X208

#define CM1Encoder_MOTOR 0x201
#define CM2Encoder_MOTOR 0x202
#define CM3Encoder_MOTOR 0x203
#define CM4Encoder_MOTOR 0x204
/*************************云台电机id******************************/
#define GIMBAL_YAW_MOTOR 0x141
#define GIMBAL_YAW_MOTOR_ID 1
#define GIMBAL_PITCH_MOTOR 0x142
/****************************英雄小云台电机id***********************************/
#define SMALL_GIMBAL_MOTOR 0X00
#define SCOPE_MOTOR 0X00
/*********************************摩擦轮电机id**************************************/
#define LEFT_FRICTION 0X202
#define RIGHT_FRICTION 0X201
//哨兵
#define RIGHT_FRONT_FRICTION 0x201
#define LEFT_FRONT_FRICTION 0x202
#define LEFT_BEHIND_FRIICTION 0x203
#define RIGHT_BEHIND_FRICTION 0x204
/**********************************拨盘电机id**************************************/
#define DOWN_POKE 0x00   
//#define UP_POKE 0X143
//#define LEFT_POKE 0X00			//右一左二，只有一个用一
//#define RIGHT_POKE 0X00
#define POKE 0X02
/*********************************舵轮上下板通信id*********************************/
#define UP_CAN2_TO_DOWN_CAN1_1 0X407
#define UP_CAN2_TO_DOWN_CAN1_2 0X408
#define UP_CAN2_TO_DOWN_CAN1_3 0X409




#define  GMPitchEncoder_Offset 14922
#define  GMYawEncoder_Offset   602606



//测试用
//#define GM6020  0x

extern Helm_wheel_t Helm_chassis;
//extern Encoder_plus Pitch_Encoder;
//extern Encoder_plus yaw_Encoder;


void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);




										
//void can_bus_send_task(void);
//void can_bus_pitch_send_task(void);
//void _42mm_friction_fdb(void);
//void _42mm_poke_fdb(void);
//void _42mm_shoot_fdb(void);
//void All_Motor_fdb(void);

//extern float gravity_input;
//extern float gravity_angle;
//extern int poke_init_angle;
//extern uint8_t zero_flag;
//extern u8 first_in_flag;
//extern float lg;
#endif
