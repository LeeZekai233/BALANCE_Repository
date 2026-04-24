#include "control_task.h"
#include <stdlib.h>
#include "led.h"      // 确保包含led头文件以访问LED数组
#include "ws2812b.h"  // 确保包含以访问底层

#define dsplT 5000
// ========== 全局变量定义 (Master) ==========
int game_stage = 0;           // 当前激活进度 (0-5)，表示已点亮的扇叶数量
int bonus_window_timer = 0;   // (小符模式下弃用，保留变量定义防报错)
uint8_t flag_first_hit = 0;   // (小符模式下弃用)
uint16_t reset_time_cnt = 0;  // 通关展示计时
int g1 = 0;
int time_tick = 0;
int time1 = 0;
int led_time1 = 1;
int all_finish_time[5]={0,0,0,0,0};
int last_led_time1 = 24 ;


int turn_flag = 0;
int energe_mode = 0;
int energe_leaf = 0;
long long time_Big_energe=0;


int energe_leaf_all_rand_flag = 0;
leaf LED[5]={0,0,0,0,0};
int current_leaf;
uint8_t n=1;
uint8_t m;
uint8_t flag_start=0;//                                                                                                                                          
uint8_t count_leaf=0;//完成数
uint8_t flag_all_finish=0;
uint8_t finish_flag=0;
uint8_t rand_finish=0;
uint8_t ready_to_reset=0;

// 0为未激活，1为激活待击打，2为完成击打
//->未激活=已经击打=0,...
uint8_t target_mode[5] = {0, 0, 0, 0, 0};  // 靶面控制


void led_display_set(uint8_t m) {
  switch (LED[m].mode) {
    case Waiting_hit:
      my_ws2812_set_all_off(m);

      LED_current(m); // 流水箭头对应数组填充
      // side_off(m);        // 侧面关
      target_mode[m] = 1;
      break;
    case hit_finish:
      my_ws2812_set_all_off(m);  // 箭头流水全亮->熄灭

      // side_off(m);
      //        // 靶子全亮->=none case

      LED_current_all_on(m);
      target_mode[m] = 2;
      break;
    case LED_none:
      my_ws2812_set_all_off(m);

      
      target_mode[m] = 0;
      break;
    case all_finish:
      my_ws2812_set_all_off(m);

      LED_current_all_on(m);  // 箭头流水全亮,改名
      //    side_off(m);            // 侧面亮
      //      LED_circle(m);
      target_mode[m] = 2;
      break;
  }
  transfer(m);
  if (time_tick % 50 == 0) {
    uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
                        target_mode[2], target_mode[3], target_mode[4]};
    USART2_SendArray(tx_buf, 6);  // to 继电器,控制靶面
  }  // 删掉这个会发送失败
}

// =========================================

void LED_task(void) {
  if (time_tick % 50 == 0) {
    m = 0;
    led_display_set(m);
  }

  if (time_tick % 50 == 20) {
    m = 3;
    led_display_set(m);
    //     uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
    //                     target_mode[2], target_mode[3], target_mode[4]};
    // USART2_SendArray(tx_buf, 6);
  }

  if (time_tick % 50 == 2) {
    m = 4;
    led_display_set(m);
    //     uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
    //                     target_mode[2], target_mode[3], target_mode[4]};
    // USART2_SendArray(tx_buf, 6);
  }
  if (time_tick % 50 == 40) {
    m = 1;
    switch (LED[m].mode) {
      case Waiting_hit:
        target_mode[m] = 1;
        break;
      case hit_finish:
        target_mode[m] = 2;
        break;
      case LED_none:
        target_mode[m] = 0;
        break;
      case all_finish:
        target_mode[m] = 2;
        break;
    }
    // 	    uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
    //                     target_mode[2], target_mode[3], target_mode[4]};
    // USART2_SendArray(tx_buf, 6);
  }
  if (time_tick % 50 == 42) {
    m = 2;
    switch (LED[m].mode) {
      case Waiting_hit:
        target_mode[m] = 1;
        break;
      case hit_finish:
        target_mode[m] = 2;
        break;
      case LED_none:
        target_mode[m] = 0;
        break;
      case all_finish:
        target_mode[m] = 2;
        break;
    }
    // 	    uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
    //                     target_mode[2], target_mode[3], target_mode[4]};
    // USART2_SendArray(tx_buf, 6);
  }

  if (time_tick % 50 == 46) {
    uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
                        target_mode[2], target_mode[3], target_mode[4]
                        };
    USART2_SendArray(tx_buf, 6);  // to 继电器,控制靶面
  }
}

void control_task(void) {
  time_tick++;
  RC_CtrlData.rc.s2 = 0;  // 假设s2为0时运行逻辑
  flag_start = 1;

  // 1. CAN通信：主板向副板发送数据
  CAN1_Send(LED[1].mode, LED[2].mode, n, game_stage, 0);

  if (flag_start == 1) {
    LED_task();
  }

  if (time_tick % 50 == 0) {
    time1++;
  }
  if (time_tick % 50 == 46) {
    uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
                        target_mode[2], target_mode[3], target_mode[4]
                        };
    USART2_SendArray(tx_buf, 6);  // to 继电器,控制靶面
  }
  if (RC_CtrlData.rc.s2 == 0)  // 总开关
  {
    if (flag_start == 1) {
      srand(time_tick);
    }

    // ==================== [通关判定逻辑] ====================
    if (game_stage >= 5) {
      // 强制所有扇叶进入全亮状态
      for (int i = 0; i < 5; i++) {
        LED[i].mode = all_finish;
      }

      reset_time_cnt++;

      // 保持5秒展示后重置
      if (reset_time_cnt > dsplT) {
        game_stage = 0;
        reset_time_cnt = 0;
        flag_first_hit = 0;
        bonus_window_timer = 0;
        // 全灭重置
        for (int i = 0; i < 5; i++)
          LED[i].mode = LED_none;
      }

      // 通关状态下只刷新灯效
      LED_task();

      if (time_tick % 35 == 0) {
        led_time1++;
        if (led_time1 > 8)
          led_time1 = 1;
      }
      return;
    }

    // ==================== [小符生成逻辑] ====================
    // 统计当前场上处于"等待击打"状态的灯
    int active_count = 0;
    for (int i = 0; i < 5; i++) {
      if (LED[i].mode == Waiting_hit)
        active_count++;
    }

    // 条件：场上无目标 且 游戏未结束 且 无随机锁定
    // 小符逻辑：一次只生成一个
    if (active_count == 0 && game_stage < 5 && flag_start == 1 &&
        rand_finish == 0) {
      int t1 = rand() % 5;

      // 必须随机到一个"未激活"的扇叶 (不能是已经 all_finish 的)
      // 防止重复点亮已完成的扇叶
      while (LED[t1].mode == all_finish) {
        t1 = rand() % 5;
      }

      g1 = t1;
      LED[t1].mode = Waiting_hit;  // 设定目标

      // 重置超时计数器 (小符要求2.5秒内击打)
      count_reset = 0;
    }

    // ==================== [裁判与输入检测] ====================

    // 调用 led.c 中的按键检测逻辑
    key_input();

    // LED 流水灯效计时更新
    if (time_tick % 35 == 0) {
      led_time1++;
      if (led_time1 > 8)
        led_time1 = 1;
    }

    if (time_tick % 10000000 == 0)
      time_tick = 0;
  }

}

void clean_time(void) {
  led_time1 = 1;
  last_led_time1 = 24;
}

void control_task_Init(void) {
  // 初始化代码
}