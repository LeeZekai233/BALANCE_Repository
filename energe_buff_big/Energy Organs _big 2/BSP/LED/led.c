#include "led.h"
#include "sys.h"

// 引用 control_task.c 定义的全局变量
extern int game_stage;
extern int bonus_window_timer;
extern uint8_t flag_first_hit;
extern leaf LED[5];  // 确保能访问LED数组

// 原有全局变量保留
int Flag_Input_Circle4;
extern int Flag_Input_Circle3;
extern int Flag_Input_Circle2;
extern int Flag_Input_Circle1;

int Flag_Input_1 = 0;
int Flag_Input_2 = 0;
int Flag_Input_3 = 0;
int Flag_Input_4 = 0;
int Flag_Input_5 = 0;
int Flag_Input_n = 0;
int Flag_Input_1_Last = 0;
int Flag_Input_2_Last = 0;
int Flag_Input_3_Last = 0;
int Flag_Input_4_Last = 0;
int Flag_Input_5_Last = 0;

int flag2_return = 0;
int flag3_return = 0;
int flag4_return = 0;
int flag5_return = 0;

int Flag_Input_PA7;
int Flag_Input_PA6;
int Flag_Input_PC4;
int Flag_Input_PA7_Last;
int Flag_Input_PA6_Last;
int Flag_Input_PC4_Last;

int LED_mode[5];
int LED_circle_mode[5];
int Leaf_mode[5];

int cnt_Circle_1;
int cnt_Circle_2;
int cnt_Circle_3;
int cnt_Circle_4;
uint16_t count_reset = 0;
int flag_rc1;
int flag_rc11;
uint16_t reset_time = 0;

// [新增] 击打保护计时器 (防抖/无敌时间)
static int hit_protection_timer[5] = {0};
#define PROTECTIONPERIOD 50
// 传感器读取与命中检测逻辑
void key_input(void) {
  // 1. 计时器倒数 (消抖保护)
  for (int i = 0; i < 5; i++) {
    if (hit_protection_timer[i] > 0)
      hit_protection_timer[i]--;
  }

  /* 2. 读取传感器电平状态 */
  // 注意：删除了原本的 if(LED.mode == Waiting_hit) 判断
  // 必须时刻读取传感器，否则无法检测“错误击打”

  Flag_Input_2 = (!Sucker_PB_4);  // 读取 LED[1] 的传感器
  Flag_Input_3 = (!Sucker_PB_3);  // 读取 LED[2] 的传感器

  /* 3. 命中判定（上升沿检测）- 仅处理 LED[1] 和 LED[2] */

  // ==================== 处理 LED[1] ====================
  if (Flag_Input_2_Last == 0 && Flag_Input_2 == 1) {
    // 情况 A: 正常击打
    if (LED[1].mode == Waiting_hit) {
      LED[1].mode = hit_finish;
      hit_protection_timer[1] = PROTECTIONPERIOD;  // 开启 200ms 保护期
      // count_reset = 0; // 副板可能无权清零计时，由主板逻辑判断是否重置
    }
    // 情况 B: 错误击打 (惩罚)
    // 只有当保护期结束，且击中了 不该打的东西 (已完成 或 已熄灭)
    else if ((LED[1].mode == LED_none || LED[1].mode == hit_finish)) {
      if (hit_protection_timer[1] == 0) {
        count_reset = 25000;  // 触发全局惩罚
      }
    }
  }

  // ==================== 处理 LED[2] ====================
  if (Flag_Input_3_Last == 0 && Flag_Input_3 == 1) {
    // 情况 A: 正常击打
    if (LED[2].mode == Waiting_hit) {
      LED[2].mode = hit_finish;
      hit_protection_timer[2] = 200;  // 开启 200ms 保护期
    }
    // 情况 B: 错误击打 (惩罚)
    else if ((LED[2].mode == LED_none || LED[2].mode == hit_finish)) {
      if (hit_protection_timer[2] == 0) {
        count_reset = 25000;  // 触发全局惩罚
      }
    }
  }

  /* 4. 更新历史状态 */
  Flag_Input_2_Last = Flag_Input_2;
  Flag_Input_3_Last = Flag_Input_3;
}

// LED IO 初始化
void LED_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2);  // 默认熄灭
}

void Sucker_PC11_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruecture;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitStruecture.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruecture.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruecture.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruecture.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruecture.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruecture);
  GPIO_ResetBits(GPIOC, GPIO_Pin_11);
}

void Sucker_PC12_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruecture;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitStruecture.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruecture.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStruecture.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruecture.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruecture.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruecture);
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}

void Sucker_PB5_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruecture;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStruecture.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruecture.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStruecture.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruecture.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruecture.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruecture);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void Sucker_PA1_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruecture;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStruecture.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruecture.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStruecture.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruecture.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruecture.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruecture);
}

void Sucker_PB4_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruecture;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStruecture.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruecture.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStruecture.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruecture.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruecture.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruecture);
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}

void Sucker_PB3_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruecture;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStruecture.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruecture.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStruecture.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruecture.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruecture.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruecture);
  GPIO_ResetBits(GPIOB, GPIO_Pin_3);
}