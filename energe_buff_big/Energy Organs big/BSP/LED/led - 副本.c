#include "led.h"
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// LED驱动代码与按键检测逻辑
//////////////////////////////////////////////////////////////////////////////////

// 引用 control_task.c 定义的全局变量
extern int game_stage;
extern int bonus_window_timer;
extern uint8_t flag_first_hit;
extern leaf LED[5];  // 确保能访问LED数组

// 原有全局变量保留
int Flag_Input_Circle4;
int Flag_Input_Circle3;
int Flag_Input_Circle2;
int Flag_Input_Circle1;

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

/**
 * @brief 核心裁判逻辑：处理击打、误打、连击及进度
@note 压感器input、遥控器调试
 */
// 定义在函数外部或作为静态变量
int hit_protection_timer[5] = {0, 0, 0, 0, 0};  // 击打保护计时器
#define PROTECTIONPERIOD 50                     // 保护期时长，单位ms
void key_input(void) {
  // ================= 1. 读取传感器状态 =================
  Flag_Input_1 = (Sucker_PB_5);
  Flag_Input_2 = (Sucker_PB_4);
  Flag_Input_3 = (Sucker_PB_3);
  Flag_Input_4 = (Sucker_PC_12);
  Flag_Input_5 = (Sucker_PC_11);
  Flag_Input_n = (Sucker_PA_1);

  // 遥控过符逻辑
  if (Flag_Input_n == 1)
    flag_rc1 = 1;
  if (Flag_Input_n == 0 && flag_rc1 == 1)
    flag_rc11 = 1;

  int Sensors_Current[5] = {Flag_Input_1, Flag_Input_2, Flag_Input_3,
                            Flag_Input_4, Flag_Input_5};
  int Sensors_Last[5] = {Flag_Input_1_Last, Flag_Input_2_Last,
                         Flag_Input_3_Last, Flag_Input_4_Last,
                         Flag_Input_5_Last};

  // ================= 2. 核心裁判逻辑 =================
  for (int i = 0; i < 5; i++) {
    // [新增] 计时器倒数：保护期内递减
    if (hit_protection_timer[i] > 0) {
      hit_protection_timer[i]--;
    }

    // 模拟遥控击打
    if (LED[i].mode == Waiting_hit && flag_rc11 == 1) {
      Sensors_Current[i] = 1;
      if (i == 4) {
        flag_rc1 = 0;
        flag_rc11 = 0;
      }
    }

    // 上升沿检测 (由 0 变 1)
    if (Sensors_Last[i] == 0 && Sensors_Current[i] == 1) {
      if (LED[i].mode == Waiting_hit) {
        // --- [A] 有效击打 ---
        LED[i].mode = hit_finish;
        count_reset = 0;

        // [新增] 开启200ms保护期 (假设函数1ms调用一次)
        // 只有过了这200ms，再次击打才会触发下面的惩罚
        hit_protection_timer[i] = PROTECTIONPERIOD;

        if (flag_first_hit == 0) {
          flag_first_hit = 1;
          game_stage++;
          bonus_window_timer = 0;
        } else {
          bonus_window_timer = 1000;
        }
      }
      // --- [B] 惩罚逻辑 (合并了灭灯误击 和 重复击打) ---
      else if (LED[i].mode == LED_none || LED[i].mode == hit_finish) {
        // [关键] 只有当保护期结束后，才进行惩罚判定
        if (hit_protection_timer[i] == 0) {
          game_stage = 0;      // 进度清零
          count_reset = 4000;  // 强制触发超时重置

          // 可选：为了防止惩罚连续触发，也可以给惩罚加一点CD
          hit_protection_timer[i] = PROTECTIONPERIOD*2;
        }
      }
    }
  }

  // 遥控清理
  if (flag_rc11 == 1) {
    flag_rc1 = 0;
    flag_rc11 = 0;
  }

  // ================= 3. 超时/手动重置判定 (保持不变) =================
  if (flag_first_hit == 0 && game_stage < 5) {
    int any_active = 0;
    for (int k = 0; k < 5; k++)
      if (LED[k].mode == Waiting_hit)
        any_active = 1;

    if (any_active) {
      count_reset++;
      if (count_reset >= 2500) {
        game_stage = 0;
        for (int k = 0; k < 5; k++)
          LED[k].mode = LED_none;
        count_reset = 0;
      }
    }
  }

  if (ready_to_reset == 1 && RC_CtrlData.rc.s2 == 3)
    count_reset = 20000;

  if (count_reset >= 4000) {
    for (int k = 0; k < 5; k++)
      LED[k].mode = LED_none;
    count_reset = 0;
    // [新增] 重置时也要清空保护计时器
    for (int k = 0; k < 5; k++)
      hit_protection_timer[k] = 0;
  }

  // ================= 4. 更新状态历史 =================
  Flag_Input_1_Last = Flag_Input_1;
  Flag_Input_2_Last = Flag_Input_2;
  Flag_Input_3_Last = Flag_Input_3;
  Flag_Input_4_Last = Flag_Input_4;
  Flag_Input_5_Last = Flag_Input_5;
}
// ================= 底层初始化函数 (保持原样) =================

void LED_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2);
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
