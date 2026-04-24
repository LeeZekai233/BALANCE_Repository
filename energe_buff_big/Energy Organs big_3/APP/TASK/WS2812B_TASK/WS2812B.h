#ifndef _WS2812B_H
#define _WS2812B_H
#include "public.h"
//64 184 111
//定义灯珠数量 多发一个灯的 保证数据稳定
#define MY_WS2812_MAX_NUM  200
#define SIDE_START 0
#define CIRCLE_START 0

typedef enum 
{
	Energe_none=0,
	Small_energe,
	Big_energe
}energe_mode_type;

typedef enum
{
	LED_none=0,
	Waiting_hit,
	hit_finish
}LED_mode_type;

typedef enum
{
	LED_circle_none=0,
	LED_circle1,
	LED_circle2,
	LED_circle3
}LED_circle_mode_type;


#define WS28_SENDBUFF_SIZE	MY_WS2812_MAX_NUM*24

//低电平偏移，复位
#define MY_WS2812_RST_NUM  600

////数据位
//#define WS2812_SET        (75)  //1//可能灯不一样
//#define WS2812_RSET       (25)  //0
#define TIMING_ONE  8
#define TIMING_ZERO 3

extern uint16_t LED_BYTE_Buffer[WS28_SENDBUFF_SIZE];//问题在这

extern int buf_r;
extern int buf_g;
extern int buf_b;
extern uint8_t rgb0[][3];
extern uint8_t rgb1[106][3];
extern uint8_t rgb2[53][3];
extern uint8_t rgb3[53][3];

extern	uint16_t buffersize;
extern int Mode_LED;

/****************** 函 数 声 明 *****************************/


void Energy_RUN(void);
void Energy_off_RUN(void);
void clean_energe_mode(void);

void Input_Change_LED(void);
void Energy_WS2812_circle(void);
void my_ws2812_1_set(uint16_t num,uint8_t rv,uint8_t gv,uint8_t bv);
void my_ws2812_set_all_off(void);
void my_ws2812_set_all_on(void);

void LED_current_mode(int mode , int line);
void set_side_circle_on(void);
void side_off(void);
void side_on(void);
void LED_current(void);
void LED_current_all_on(void);
void LED_current_all_off(void);
void LED_circle(void);

void Enegy_WS2812_R(void);
void Enegy_WS2812_R_off(void);

/***************************************************************/

#endif

