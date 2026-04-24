#include "control_task.h"
#include <stdlib.h>
#include "led.h"      // 确保包含led头文件以访问LED数组
#include "ws2812b.h"  // 确保包含以访问底层

// 本程序作为主板做逻辑处理和按键检测并控制三片扇叶
// transfer5,作为第五块板使用tim3的ch2-PA7  单独使用uint16_t 数组传输
// transfer1,4分别为leaf1/4，tim5的ch1和ch4 =分别为pa0和pa3共用uint32_t 数组
// 尝试解决PA7首个灯乱闪，未果2.25
// 1板can对2板通信
// 2.26完成超时检测
// 3.8 1的pa0，2的pa3，1的pa3，
// a0_pb5；a3_pc12;2a3_pb3;a7_pc11;2a0_PB4
// 6.28 1的PA2进行对外接f1的当前扇叶告知

// ========== 全局变量定义 (Master) ==========
#define dsplT 1000
int game_stage = 0;           // 当前激活进度 (0-5)
int bonus_window_timer = 0;   // 1秒连击窗口计时器
uint8_t flag_first_hit = 0;   // 标记：本轮是否已首击
uint16_t reset_time_cnt = 0;  // 通关展示计时
int g1 = 0;
int time_tick = 0;
int time1 = 0;
int led_time1 = 1;
int all_finish_time[5] = {0, 0, 0, 0, 0};
int last_led_time1 = 24;

int turn_flag = 0;
int energe_mode = 0;
int energe_leaf = 0;
long long time_Big_energe = 0;

int energe_leaf_all_rand_flag = 0;
leaf LED[5] = {0, 0, 0, 0, 0};
int current_leaf;
uint8_t n = 1;
uint8_t m;
uint8_t flag_start = 0;
uint8_t count_leaf = 0;  // 完成数
uint8_t flag_all_finish = 0;
uint8_t finish_flag = 0;
uint8_t rand_finish = 0;
uint8_t ready_to_reset = 0;

uint8_t target_mode[5] = {0, 0, 0, 0, 0};  // 靶面控制
// 0为未击打，1为待击打，2为完成
//->未激活=已经击打=0,...
void led_display_set(uint8_t m) {
  switch (LED[m].mode) {
    case Waiting_hit:
      my_ws2812_set_all_off(m);

      LED_currentAll(m);  // 流水箭头对应数组填充
      // side_off(m);        // 侧面关
      target_mode[m] = 1;
      break;
    case hit_finish:
      my_ws2812_set_all_off(m);  // 箭头流水全亮->熄灭

      // side_off(m);
      //        // 靶子全亮->=none case

      LED_Strip(m);
      target_mode[m] = 0;
      break;
    case LED_none:
      my_ws2812_set_all_off(m);

      LED_Strip(m);
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
        target_mode[m] = 0;
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
        target_mode[m] = 0;
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
                        target_mode[2], target_mode[3], target_mode[4],
                        target_mode[5]};
    USART2_SendArray(tx_buf, 6);  // to 继电器,控制靶面
  }
}

void control_task(void) {
  time_tick++;
  RC_CtrlData.rc.s2 = 0;
  flag_start = 1;
  // 1. CAN通信：主板向副板发送数据，包含当前 game_stage
  CAN1_Send(LED[1].mode, LED[2].mode, n, game_stage, 0);

  if (flag_start == 1) {
    LED_task();
  }

  if (time_tick % 50 == 0) {
    time1++;
  }

  if (RC_CtrlData.rc.s2 == 0)  // 总开关
  {
    if (flag_start == 1) {
      srand(time_tick);
    }

    // ==================== [新增] 通关判定逻辑 ====================
    if (game_stage >= 5 && flag_first_hit == 0) {
      // 强制所有扇叶进入全亮状态
      for (int i = 0; i < 5; i++) {
        LED[i].mode = all_finish;
        target_mode[i] = 2;
      }
      // 保持5秒展示 (假设 control_task 每 1ms 被调用一次)
      reset_time_cnt++;

      if (reset_time_cnt > dsplT) {
        game_stage = 0;
        reset_time_cnt = 0;
        flag_first_hit = 0;
        bonus_window_timer = 0;
        // 全灭重置
        for (int i = 0; i < 5; i++)
          LED[i].mode = LED_none;
      }

      // 通关状态下只刷新灯效和检测按键，不再生成新目标
      LED_task();

      if (time_tick % 35 == 0) {
        led_time1++;
        if (led_time1 > 8)
          led_time1 = 1;
      }
      // key_input();
      return;
    }

    // ==================== [修改] 双目标生成逻辑 ====================
    // 统计当前场上处于等待击打状态的灯
    int active_count = 0;
    for (int i = 0; i < 5; i++) {
      if (LED[i].mode == Waiting_hit)
        active_count++;
    }

    // 条件：场上无目标 且 不在连击窗口期 且 游戏已开始
    if (active_count == 0 && bonus_window_timer == 0 && flag_start == 1 &&
        rand_finish == 0) {
      int t1 = rand() % 5;
      int t2 = rand() % 5;

      // 确保生成的两个随机目标不重叠
      while (t1 == t2) {
        t2 = rand() % 5;
      }
      g1 = t1;
      LED[t1].mode = Waiting_hit;
      LED[t2].mode = Waiting_hit;

      flag_first_hit = 0;  // 重置首击标记
      count_reset = 0;     // 重置超时计数
    }

    // ==================== [新增] 连击窗口计时 ====================
    if (1) {
      if (flag_first_hit == 1) {
        bonus_window_timer++;
        // 1秒 (1000次tick) 结束后，无论是否击中第二个，都强行结束本轮
        if (bonus_window_timer >= 1000) {
          for (int i = 0; i < 5; i++) {
            if (LED[i].mode == Waiting_hit ||
                LED[i].mode == hit_finish)  //"!"refresh"!" state
              LED[i].mode = LED_none;
          }
          flag_first_hit = 0;
          bonus_window_timer = 0;
        }
      }

      if (flag_start == 0) {  // 之前就是这样,怪怪的
        LED_task();
      }

      key_input();  // 调用裁判/按键逻辑

      if (time_tick % 35 == 0) {
        led_time1++;
        if (led_time1 > 8)
          led_time1 = 1;
      }

      // if (time_tick % 50 == 0) {
      //   uint8_t tx_buf[] = {0x40,        LED[0].mode, LED[1].mode,
      //                       LED[2].mode, LED[3].mode, LED[4].mode};
      //   USART2_SendArray(tx_buf, 6);
      // }

      if (time_tick % 10000000 == 0)
        time_tick = 0;
    }
  }
}

void clean_time(void) {
  led_time1 = 1;
  last_led_time1 = 24;
}

void control_task_Init(void) {
  // 初始化代码
}
