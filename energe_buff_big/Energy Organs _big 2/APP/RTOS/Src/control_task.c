#include "control_task.h"
// 本程序作为副板对23叶进行控制仅使用tim5
// 通信接受后板发送RC值并转给主板，并从主板接受led.mode
// 2.27pa3的usart2接受完成；
// 2.27can通信同步状态，或者不通信了
// 2.28修符，改兑换站的灯
// 2.29can通信同步状态，

/********************** 变量定义 *********************/
// 副板也需要这个变量来显示进度条，但它的值来源于 CAN 接收
int game_stage = 0;

int time_tick = 0;
int led_time1 = 1;
int all_finish_time[5] = {0, 0, 0, 0, 0};
int last_led_time1 = 24;

int time1 = 0;
int turn_flag = 0;
int energe_mode = 0;
int energe_leaf = 0;

double turn_a = 1.045, turn_w = 2, turn_b = 1.045;
double angle = 0;
long long time_Big_energe = 0;

int energe_leaf1_rand_flag = 0;
int energe_leaf2_rand_flag = 0;
int energe_leaf3_rand_flag = 0;
int energe_leaf4_rand_flag = 0;
int energe_leaf5_rand_flag = 0;
int energe_leaf_all_rand_flag = 0;
leaf LED[5] = {0, 0, 0, 0, 0};
int current_leaf = 0;
uint8_t n;
uint8_t m;
uint8_t flag_start = 0;
uint8_t count_leaf = 0;  // 完成数
uint8_t flag_all_finish = 0;
uint8_t finish_flag = 0;
uint8_t rand_finish = 0;

uint8_t target_mode[6] = {0, 0, 0, 0, 0, 0}; 
/********************** 函数实现 *********************/

void LED_task(void) {
  // 分时处理不同叶片的显示，减少单次循环负载
  if (time_tick % 50 == 0) {
    m = 1;
    led_display_set(m);
  }

  if (time_tick % 50 == 1) {
    m = 2;
    led_display_set(m);
  }
}

void led_display_set(uint8_t m) {
  switch (LED[m].mode) {
    case Waiting_hit:
      my_ws2812_set_all_off(m);

      LED_currentAll(m);  // 流水箭头对应数组填充
      // side_off(m);        // 侧面关
      target_mode[m]= 1; 
      break;
    case hit_finish:
      my_ws2812_set_all_off(m);  // 箭头流水全亮->熄灭

      // side_off(m);
      //        // 靶子全亮->=none case

      LED_Strip(m);
      target_mode[m]= 0;
      break;
    case LED_none:
      my_ws2812_set_all_off(m);

      LED_Strip(m);
      target_mode[m]= 0;
      break;
    case all_finish:
      my_ws2812_set_all_off(m);

      LED_current_all_on(m);  // 箭头流水全亮,改名
      //    side_off(m);            // 侧面亮
      //      LED_circle(m);
      target_mode[m]= 2;
      break;
  }
  transfer(m);
//  if (time_tick % 50 == 0) {
//    uint8_t tx_buf[] = {0x40,           target_mode[0], target_mode[1],
//                        target_mode[2], target_mode[3], target_mode[4],
//                        target_mode[5]};
//    USART2_SendArray(tx_buf, 6);  // to 继电器,控制靶面
//  }
}

void control_task(void) {
  time_tick++;
  LED_task();  // 刷新灯效
  RC_CtrlData.rc.s2 = 0;
  // 1. 根据遥控器状态切换（保留原逻辑）
  switch (RC_CtrlData.rc.s2) {
    case 0:  // 测试用
      flag_start = 1;
      break;
    case 1:
      flag_start = 1;
      break;
    case 2:
      flag_start = 1;
      break;
    case 3:
      flag_start = 1;  //
      break;
  }

  // 2. CAN 发送：上报传感器数据或心跳
  CAN1_Send(14000, 0, 0, 0, 0);

  // 3. 辅助计时更新：控制流水箭头的移动速度
  if (time_tick % 35 == 0) {
    led_time1++;
    if (led_time1 > 8)
      led_time1 = 1;
  }

  if (time_tick % 10000000 == 0)
    time_tick = 0;
}

int rc_input = 0;
int last_rc_input = 0;

void clean_time(void) {
  led_time1 = 1;
  last_led_time1 = 24;
}

void control_task_Init(void) {
  // 初始化代码
}