#ifndef _WS2812B_H
#define _WS2812B_H
#include "public.h"
// 64 184 111
// 40 30 20 12 6 3
// 定义灯珠数量 多发一个灯的 保证数据稳定

#define MY_WS2812_MAX_NUM (64 * 5 + 184 + 111 + 57)
#define SIDE_START (64 * 5)
#define CIRCLE_START (64 * 5 + 184)

// #define MY_WS2812_MAX_NUM  64+184+111
// #define SIDE_START 64
// #define CIRCLE_START 64+184
#define WS28_SENDBUFF_SIZE (MY_WS2812_MAX_NUM * 24)
#define CIRCLE6_START CIRCLE_START
#define CIRCLE5_START (CIRCLE_START + 40)
#define CIRCLE4_START (CIRCLE_START + 40 + 30)
#define CIRCLE3_START (CIRCLE_START + 40 + 30 + 20)
#define CIRCLE2_START (CIRCLE_START + 40 + 30 + 20 + 12)
#define CIRCLE1_START (CIRCLE_START + 40 + 30 + 20 + 12 + 6)
#define CIRCLE2_RESTART (CIRCLE_START + 40 + 30 + 20 + 12 + 6 + 3)
typedef enum {
  energe_leaf1 = 0,
  energe_leaf2,
  energe_leaf3,
  energe_leaf4,
  energe_leaf5
} energe_leaf_type;

typedef enum { Energe_none = 0, Small_energe, Big_energe } energe_mode_type;

typedef enum {
  LED_none = 0,
  Waiting_hit,
  hit_finish,
  all_finish
} LED_mode_type;

typedef enum {
  LED_circle_none = 0,
  LED_circle1,
  LED_circle2,
  LED_circle3
} LED_circle_mode_type;

typedef enum {
  LED_leaf_none = 0,
  LED_leaf1,
  LED_leaf2,
  LED_leaf3,
  LED_leaf4,
  LED_leaf5
} Leaf_mode_type;

// 低电平偏移，复位
#define MY_WS2812_RST_NUM 600

////数据位
// #define WS2812_SET        (75)  //1//可能灯不一样
// #define WS2812_RSET       (25)  //0
#define TIMING_ONE 8
#define TIMING_ZERO 3

extern uint32_t LED_BYTE_Buffer1[WS28_SENDBUFF_SIZE];  // 600
extern uint16_t LED_BYTE_Buffer2[WS28_SENDBUFF_SIZE];
// extern uint16_t LED_BYTE_Buffer3[WS28_SENDBUFF_SIZE];
// extern uint16_t LED_BYTE_Buffer4[WS28_SENDBUFF_SIZE];
// extern uint16_t LED_BYTE_Buffer[WS28_SENDBUFF_SIZE]; // PWM ?????
// extern uint16_t LED_BYTE_Buffer5[WS28_SENDBUFF_SIZE];

extern uint16_t buffersize;
extern int Mode_LED;
extern int energe_leaf1_rand_flag;
extern int energe_leaf2_rand_flag;
extern int energe_leaf3_rand_flag;
extern int energe_leaf4_rand_flag;
extern int energe_leaf5_rand_flag;
extern int energe_leaf_all_rand_flag;
extern int current_leaf;
extern uint8_t n;

/****************** 函 数 声 明 *****************************/

void Energy_RUN(uint16_t leaf);
void Energy_off_RUN(void);
void Energy_on_RUN(void);

void Input_Change_LED(uint16_t leaf);
void Energy_WS2812_circle(uint16_t leaf);
void my_ws2812_1_set(uint16_t leaf,
                     uint16_t num,
                     uint8_t rv,
                     uint8_t gv,
                     uint8_t bv);
void my_ws2812_set_all_off(uint16_t leaf);
void my_ws2812_set_all_on(uint16_t leaf);

void LED_current_mode(uint16_t leaf, int mode, int line);
void set_side_circle_on(uint16_t leaf);
void side_off(uint16_t leaf);
void side_on(uint16_t leaf);
void LED_current(uint16_t leaf);
void LED_current_all_on(uint16_t leaf);
void LED_current_all_off(uint16_t leaf);
void LED_circle(uint16_t leaf);
void LED_target(uint16_t leaf);
void LED_target_allOff(uint16_t leaf);
void LED_current_all_off(uint16_t leaf);
void LED_currentAll(uint16_t leaf);
void LED_Strip(uint16_t leaf);

// void transfer1(void);
void transfer2(void);
void transfer3(void);
// void transfer4(void);
// void transfer5(void);
/***************************************************************/

#endif
