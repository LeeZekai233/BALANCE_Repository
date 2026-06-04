#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H
#include "stm32f4xx.h"                  // Device header


/**********************************宏定义*************************************/
#define         MIDDLE                          3
#define         UP                              1
#define         DOWN                            2
#define         RC_CH_VALUE_OFFSET              1024 
#define         LONG_PRESS_THRESHOLD            500

#define         RIGHT                           2
#define         CENTER                          1
#define         LEFT                            0
/**********************************宏定义*************************************/





/**********************************灰白都有***************************************/
typedef __packed struct
{
	int Cnt;//计数，用来判断长短按
	uint8_t  Original_Press_Flag;//原始短按标志位,只用来判断按键状态
    uint8_t  Short_Press_Flag;//短按标志位
    uint8_t  Long_Press_Flag;//长按标志位
    uint8_t  Last_Original_Press_Flag;//上一次的原始短按标志位，来检测上升下降沿去判断有没有重新按下，用来判断是否反转按键状态
    uint8_t  Toggle_Press_Flag;//按键状态反转标志位
}Key_Mouse_Action_t;//鼠标键盘长按短按动作

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
    uint8_t last_press_m;//M鼠标中键
	uint8_t press_l;
	uint8_t press_r;
    uint8_t press_m;
	float Mouse_W;//鼠标下的角速度
	Key_Mouse_Action_t Press_L_Action;
	Key_Mouse_Action_t Press_R_Action;
    Key_Mouse_Action_t Press_M_Action;
}Mouse_t;//鼠标相关

typedef	__packed struct
{
	uint16_t v;
	uint16_t last_v;
	float Key_V_Y;//按键下Y轴速度
	float Key_V_X;//按键下X轴速度
	Key_Mouse_Action_t Key_W_Action;
	Key_Mouse_Action_t Key_S_Action;
	Key_Mouse_Action_t Key_A_Action;
	Key_Mouse_Action_t Key_D_Action;
	Key_Mouse_Action_t Key_Q_Action;
	Key_Mouse_Action_t Key_E_Action;
	Key_Mouse_Action_t Key_R_Action;
	Key_Mouse_Action_t Key_F_Action;
	Key_Mouse_Action_t Key_G_Action;
	Key_Mouse_Action_t Key_Z_Action;	
	Key_Mouse_Action_t Key_X_Action;
	Key_Mouse_Action_t Key_C_Action;
	Key_Mouse_Action_t Key_V_Action;
	Key_Mouse_Action_t Key_B_Action;
	Key_Mouse_Action_t Key_SHIFT_Action;
	Key_Mouse_Action_t Key_CTRL_Action;
}Key_t;//键盘相关
/**********************************灰白都有***************************************/



/***********************************白控************************************/
typedef enum
{
	KEEP            = 0   ,
	UP_TO_MIDDLE    = 1   ,
	MIDDLE_TO_UP    = 2   ,
	MIDDLE_TO_DOWN  = 3   ,
	DOWN_TO_MIDDLE  = 4   ,
}Clicker_Action_e;//拨杆的动作


typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;//拨轮
	int8_t s1;
	int8_t s2;
	/**********Mine**********/
	uint8_t s1_last;
	uint8_t s2_last;
	Clicker_Action_e s1_Action;
	Clicker_Action_e s2_Action;
    uint8_t ch4_Up;
    uint8_t ch4_Down;
    Key_Mouse_Action_t ch4_Up_Action;
    Key_Mouse_Action_t ch4_Down_Action;
    
}DT7_Clicker_t;//遥控器相关



typedef __packed struct
{
	DT7_Clicker_t Remote_clicker;
	Mouse_t Remote_mouse;
	Key_t key;
    uint32_t heart_cnt;
    uint8_t online_flag;
}Remote_DT7_t;//总

/*********************************白控************************************/




/**********************************灰控************************************/
typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;//左0 中1 右2
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}remote_data_t;//灰控收数据用


typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	Key_Mouse_Action_t Pause_Action;
	int8_t Switch;//挡位切换
	uint8_t fn1;
	uint8_t fn2;
	/**********Mine**********/
    uint8_t trigger;
	uint8_t s1_last;
	uint8_t s2_last;
	uint8_t fn1_last;
	uint8_t fn2_last;
	Key_Mouse_Action_t fn2_Action;
	Key_Mouse_Action_t Trigger_Action;	
	Key_Mouse_Action_t fn1_Action;
    uint8_t ch4_Up;
    uint8_t ch4_Down;
    Key_Mouse_Action_t ch4_Up_Action;
    Key_Mouse_Action_t ch4_Down_Action;
}VTM_Clicker_t;


typedef __packed struct
{
	VTM_Clicker_t Remote_clicker;
	Mouse_t Remote_mouse;
	Key_t key;
    uint8_t online_flag;
    uint32_t heart_cnt;
}Remote_VTM_t;//总

/**********************************灰控************************************/


//typedef enum
//{
//	RELAX = 0,
//	REMOTE,
//	KEY_MOUSE,
//}Control_Mode_e;//控制模式切换





/********************************外部声明*************************************/
extern Remote_DT7_t Remote_DT7_data;
//extern Control_Mode_e Control_Mode;
extern Remote_VTM_t  Remote_VTM;
/********************************外部声明*************************************/



/********************************函数声明*************************************/
void DT7_Remote_Data_Dispose(uint8_t* RemoteData,Remote_DT7_t* Remote_data);
void VTM_Reomte_Data_Handle(uint8_t *pData,uint16_t rec_len,Remote_VTM_t* Remote_VTM);
void Key_Mouse_Action_Detect(Key_Mouse_Action_t* Key_Mouse_Action);
void Key_Mouse_State_Update(Key_t* Key, Mouse_t* Mouse);
//void Control_Mode_Select(Remote_DT7_t* Remote_data,Control_Mode_e* Control_Mode);
void VTM_Clicker_State_Update(Remote_VTM_t* Remote_data);
void Remote_Switch_Action_Detect(Remote_DT7_t* Remote_data);
void Remote_Online_Detect(Remote_DT7_t* Remote_DT7,Remote_VTM_t* Remote_VTM);
/********************************函数声明*************************************/





#endif
