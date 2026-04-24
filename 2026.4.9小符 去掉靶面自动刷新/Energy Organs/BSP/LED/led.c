#include "led.h"
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// LED驱动代码与按键检测逻辑 (适配小符规则)
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
uint16_t count_reset = 0;  // 超时计数器
int flag_rc1;
int flag_rc11;
uint16_t reset_time = 0;

// 定义防抖/击打保护计时器
int hit_protection_timer[5] = {0, 0, 0, 0, 0};
#define PROTECTIONPERIOD 50  // 保护期时长ms，防止传感器误触

/**
 * @brief 核心裁判逻辑：处理击打、误打及重置
 */
void key_input(void) {
  // ================= 1. 读取传感器状态 =================
  Flag_Input_1 = (Sucker_PB_5);
  Flag_Input_2 = (Sucker_PB_4);
  Flag_Input_3 = (Sucker_PB_3);
  Flag_Input_4 = (Sucker_PC_12);
  Flag_Input_5 = (Sucker_PC_11);
  Flag_Input_n = (Sucker_PA_1);

  // 遥控器模拟击打逻辑 (调试用)
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
    // 计时器倒数：保护期内递减
    if (hit_protection_timer[i] > 0) {
      hit_protection_timer[i]--;
    }

    // 调试：模拟击打当前亮起的灯
    if (LED[i].mode == Waiting_hit && flag_rc11 == 1) {
      Sensors_Current[i] = 1;
      if (i == 4) {
        flag_rc1 = 0;
        flag_rc11 = 0;
      }
    }

    // 上升沿检测 (传感器由 0 变 1)
    if (Sensors_Last[i] == 0 && Sensors_Current[i] == 1) {
      // --- [情况A] 有效击打 (Hit Correct) ---
      if (LED[i].mode == Waiting_hit) {
        // 规则：击中被点亮的装甲，灯臂会被完全点亮
        LED[i].mode = all_finish;

        count_reset = 0;  // 成功击打，重置超时计数

        // 开启保护期，防止物理抖动造成二次误判
        hit_protection_timer[i] = PROTECTIONPERIOD;

        // 进度增加 (不需要双击/连击逻辑，因为小符是逐个激活)
        game_stage++;
      }

      // --- [情况B] 误击打 (Hit Wrong) ---
      // 规则：击中非随机点亮的装甲(熄灭或已激活)，视为失败 [cite: 70]
      else if (LED[i].mode == LED_none || LED[i].mode == all_finish) {
        // 只有当保护期结束后，才进行惩罚判定
        if (hit_protection_timer[i] == 0) {
          game_stage = 0;      // 进度清零
          count_reset = 4000;  // 设置大数值，强制触发下方的重置逻辑

          // 防止惩罚逻辑连续触发
          hit_protection_timer[i] = 100;
        }
      }
    }
  }

  // 遥控标志清理
  if (flag_rc11 == 1) {
    flag_rc1 = 0;
    flag_rc11 = 0;
  }

  // ================= 3. 超时重置判定 =================
  // 规则：2.5秒内未击中，恢复未激活状态 
  // 假设 control_task 1ms 调用一次，2500次即为2.5秒

  if (game_stage < 5) {  // 如果未通关
    int any_active = 0;
    // 检查场上是否有待击打的目标
    for (int k = 0; k < 5; k++)
      if (LED[k].mode == Waiting_hit)
        any_active = 1;

    if (any_active) {
      //count_reset++;
      // 超时阈值：2500ms
      if (count_reset >= 2500)//_
        {
        game_stage = 0;      // 进度清零
        count_reset = 4000;  // 触发重置
      }
    }
  }

  // 手动重置 (调试用)
  if (ready_to_reset == 1 && RC_CtrlData.rc.s2 == 3)
    count_reset = 20000;

  // 执行重置操作 (超时 或 误击触发)
  if (count_reset >= 4000)
		{
    game_stage = 0;
    for (int k = 0; k < 5; k++) {
      LED[k].mode = LED_none;       // 全部熄灭
      hit_protection_timer[k] = 0;  // 清空计时器
    }
    count_reset = 0;
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

// ... (保留Sucker_Init函数，无需修改) ...
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