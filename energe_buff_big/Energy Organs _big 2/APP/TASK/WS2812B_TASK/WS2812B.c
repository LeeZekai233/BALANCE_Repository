#include "ws2812b.h"

/****************** 结构体定义***********************/

/***************************************************/
int Flag_Input_Circle1 = 0;
int Flag_Input_Circle2 = 0;
int Flag_Input_Circle3 = 0;
/********************** 变量定义*********************/
// 副板的 game_stage 值应该由 CAN 接收中断更新
extern int game_stage;

int current_r = 254;  // 150
int side_r = 222;

int buf_r = 2;  // 幅亮度
int buf_g = 0;  // 幅亮度
int buf_b = 0;  // 幅亮度

int buf_r_off = 0;  // 幅亮度
int buf_g_off = 0;  // 幅亮度
int buf_b_off = 0;  // 幅亮度

int buf_r_edg = 2;  // 幅亮度
int buf_g_edg = 0;  // 幅亮度
int buf_b_edg = 0;  // 幅亮度

// uint16_t LED_BYTE_Buffer[WS28_SENDBUFF_SIZE]; // PWM ?????
uint32_t LED_BYTE_Buffer1[WS28_SENDBUFF_SIZE];  // 600
uint16_t LED_BYTE_Buffer2[WS28_SENDBUFF_SIZE];
// uint16_t LED_BYTE_Buffer3[WS28_SENDBUFF_SIZE];
// uint16_t LED_BYTE_Buffer4[WS28_SENDBUFF_SIZE];
// uint16_t LED_BYTE_Buffer5[WS28_SENDBUFF_SIZE];

int cnt = 0;

//	void transfer1(void)
//	{DMA_Cmd(DMA1_Stream2, DISABLE);
//		DMA_SetCurrDataCounter(DMA1_Stream2, WS28_SENDBUFF_SIZE);
//// load number of bytes to be transferred 	DMA_Cmd(DMA1_Stream2, ENABLE);
//// enable DMA channel 6 	TIM_Cmd(TIM5, ENABLE);
//// enable Timer 3 		TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable);
//	while(!DMA_GetFlagStatus(DMA1_Stream2,DMA_FLAG_TCIF2)) ; 	// wait
// until transfer complete
// TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable);
//	DMA_Cmd(DMA1_Stream2, DISABLE); 			// disable DMA
// channel 6 	DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2);
//// clear DMA1 Channel 6 transfer complete flag
//		}

void transfer2(void) {
  DMA_SetCurrDataCounter(
      DMA1_Stream4,
      WS28_SENDBUFF_SIZE);  // load number of bytes to be transferred
  DMA_Cmd(DMA1_Stream4, ENABLE);
  // enable Timer 3
  TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Enable);
  TIM_Cmd(TIM5, ENABLE);
  while (!DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4))
    ;                      // wait until transfer complete
  TIM_Cmd(TIM5, DISABLE);  // disable Timer 3
  TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Disable);

  DMA_Cmd(DMA1_Stream4, DISABLE);  // disable DMA channel 6
  DMA_ClearFlag(DMA1_Stream4,
                DMA_FLAG_TCIF4);  // clear DMA1 Channel 6 transfer complete flag
}

void transfer3(void) {
  DMA_SetCurrDataCounter(
      DMA1_Stream2,
      WS28_SENDBUFF_SIZE);        // load number of bytes to be transferred
  DMA_Cmd(DMA1_Stream2, ENABLE);  // enable DMA channel 6
  TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable);
  TIM_Cmd(TIM5, ENABLE);  // enable Timer 3
  while (!DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2))
    ;                              // wait until transfer complete
  DMA_Cmd(DMA1_Stream2, DISABLE);  // disable DMA channel 6
  TIM_Cmd(TIM5, DISABLE);          // disable Timer 3
  TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable);
  DMA_ClearFlag(DMA1_Stream2,
                DMA_FLAG_TCIF2);  // clear DMA1 Channel 6 transfer complete flag
}

//	void transfer4(void)
//	{DMA_SetCurrDataCounter(DMA1_Stream1, WS28_SENDBUFF_SIZE); 	// load
// number of bytes to be transferred 	DMA_Cmd(DMA1_Stream1, ENABLE);
//// enable DMA channel 6 	TIM_Cmd(TIM5, ENABLE);
//// enable Timer 3 		TIM_CCxCmd(TIM5, TIM_Channel_4, TIM_CCx_Enable);
//	while(!DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1)) ; 	// wait
// until transfer complete 	TIM_CCxCmd(TIM5, TIM_Channel_4,
// TIM_CCx_Disable); 		TIM_Cmd(TIM5, DISABLE); 	// disable Timer
// 3
//	DMA_Cmd(DMA1_Stream1, DISABLE); 			// disable DMA
// channel 6 	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);
//// clear DMA1 Channel 6 transfer complete flag
//		}
//	void transfer5(void)
//	{
//		DMA_SetCurrDataCounter(DMA1_Stream5, WS28_SENDBUFF_SIZE);
//// load number of bytes to be transferred 	DMA_Cmd(DMA1_Stream5, ENABLE);
//// enable DMA channel 6
// TIM_Cmd(TIM3, ENABLE); 						//
// enable Timer 3
//		TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);
//	while(!DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)) ; 	// wait
// until transfer complete
//
// TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable);
//	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
//	DMA_Cmd(DMA1_Stream5, DISABLE); 			// disable DMA
// channel 6 	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
//// clear DMA1 Channel 6 transfer complete flag
//		}
// dma尚未配置

void transfer(uint8_t leaf) {
  switch (leaf) {
    case 1:
      transfer2();
      break;
    case 2:
      transfer3();
      break;
  }
}

void my_ws2812_1_set(uint16_t leaf,
                     uint16_t num,
                     uint8_t rv,
                     uint8_t gv,
                     uint8_t bv) {
  uint32_t indexx = ((num - 1) * 24);
  switch (leaf) {
    case energe_leaf1:
    case energe_leaf2:
    case energe_leaf3:
    case energe_leaf4:
    case energe_leaf5:
      for (uint8_t i = 0; i < 8; i++) {
        // 填充数组
        LED_BYTE_Buffer1[indexx + i] =
            (gv << i) & (0x80) ? TIMING_ONE : TIMING_ZERO;
        LED_BYTE_Buffer1[indexx + i + 8] =
            (rv << i) & (0x80) ? TIMING_ONE : TIMING_ZERO;
        LED_BYTE_Buffer1[indexx + i + 16] =
            (bv << i) & (0x80) ? TIMING_ONE : TIMING_ZERO;
      }
      if (leaf == energe_leaf2)
        cnt++;  // 保留原代码在leaf2时的计数逻辑
      break;
  }
}

// 关闭所有
void my_ws2812_set_all_off(uint16_t leaf) {
  for (uint16_t i = 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
}

// 开启所有
void my_ws2812_set_all_on(uint16_t leaf) {
  for (uint16_t i = 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, buf_r, buf_g, buf_b);
  }
}

void Input_Change_LED(uint16_t leaf) {
  if (Flag_Input_Circle3 == 0 && Flag_Input_Circle2 == 0 &&
      Flag_Input_Circle1 == 0)
    LED_circle_mode[leaf] = 0;  // 0表示全灭
  else if (Flag_Input_Circle3 == 0 && Flag_Input_Circle2 == 0 &&
           Flag_Input_Circle1 == 1)
    LED_circle_mode[leaf] = 1;  // 1表示环数1亮
  else if (Flag_Input_Circle3 == 0 && Flag_Input_Circle2 == 1 &&
           Flag_Input_Circle1 == 0)
    LED_circle_mode[leaf] = 2;  // 2表示环数2亮
  else if (Flag_Input_Circle3 == 1 && Flag_Input_Circle2 == 0 &&
           Flag_Input_Circle1 == 0)
    LED_circle_mode[leaf] = 3;  // 3表示环数3亮
  else if (Flag_Input_Circle3 == 0 && Flag_Input_Circle2 == 1 &&
           Flag_Input_Circle1 == 1)
    LED_circle_mode[leaf] = 4;  // 4表示环数1、2亮
  else if (Flag_Input_Circle3 == 1 && Flag_Input_Circle2 == 0 &&
           Flag_Input_Circle1 == 1)
    LED_circle_mode[leaf] = 5;  // 5表示环数1、3亮
  else if (Flag_Input_Circle3 == 1 && Flag_Input_Circle2 == 1 &&
           Flag_Input_Circle1 == 0)
    LED_circle_mode[leaf] = 6;  // 6表示环数2、3亮
  else if (Flag_Input_Circle3 == 1 && Flag_Input_Circle2 == 1 &&
           Flag_Input_Circle1 == 1)
    LED_circle_mode[leaf] = 7;  // 7表示环数1、2、3亮
}

void Circle3_Open(uint16_t leaf) {
  for (int i = 1; i <= 12; i++)
    my_ws2812_1_set(leaf, CIRCLE_START + i, 222, 0, 0);
}
void Circle2_Open(uint16_t leaf) {
  for (int i = 13; i <= 20; i++)
    my_ws2812_1_set(leaf, CIRCLE_START + i, 222, 0, 0);
}
void Circle1_Open(uint16_t leaf) {
  for (int i = 21; i <= 24; i++)
    my_ws2812_1_set(leaf, CIRCLE_START + i, 222, 0, 0);
}
void Circle3_Close(uint16_t leaf) {
  for (int i = 1; i <= 12; i++)
    my_ws2812_1_set(leaf, CIRCLE_START + i, 0, 0, 0);
}
void Circle2_Close(uint16_t leaf) {
  for (int i = 13; i <= 20; i++)
    my_ws2812_1_set(leaf, CIRCLE_START + i, 0, 0, 0);
}
void Circle1_Close(uint16_t leaf) {
  for (int i = 21; i <= 24; i++)
    my_ws2812_1_set(leaf, CIRCLE_START + i, 0, 0, 0);
}

void LED_current_mode(uint16_t leaf, int mode, int line) {
  switch (mode) {
    case 1:
      my_ws2812_1_set(leaf, line * 8 + 1, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, 0, 0, 0);
      break;
    case 2:
      my_ws2812_1_set(leaf, line * 8 + 1, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, 0, 0, 0);
      break;
    case 3:
      my_ws2812_1_set(leaf, line * 8 + 1, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, 0, 0, 0);
      break;
    case 4:
      my_ws2812_1_set(leaf, line * 8 + 1, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, 0, 0, 0);
      break;
    case 5:
      my_ws2812_1_set(leaf, line * 8 + 1, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, 0, 0, 0);
      break;
    case 6:
      my_ws2812_1_set(leaf, line * 8 + 1, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, current_r, 0, 0);
      break;
    case 7:
      my_ws2812_1_set(leaf, line * 8 + 1, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, current_r, 0, 0);
      break;
    case 8:
      my_ws2812_1_set(leaf, line * 8 + 1, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 2, current_r, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 3, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 4, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 5, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 6, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 7, 0, 0, 0);
      my_ws2812_1_set(leaf, line * 8 + 8, current_r, 0, 0);
      break;
  }
}
void set_side_circle_on(uint16_t leaf) {
  for (uint16_t i = SIDE_START + 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, 222, 0, 0);
  }
}

void side_off(uint16_t leaf) {
  for (uint16_t i = SIDE_START + 1; i <= CIRCLE_START; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
}

void side_on(uint16_t leaf) {
  for (uint16_t i = SIDE_START + 1; i <= CIRCLE_START; i++) {
    my_ws2812_1_set(leaf, i, side_r, 0, 0);
  }
}
void LED_Strip(uint16_t leaf) {
  int mode;

  // j 代表灯臂的 5 个段 (0-4)
  for (int j = 0; j < 5; j++) {
    // 【关键逻辑】只有在当前激活进度内的段才显示
    // game_stage = 1 时，显示 j=0 的段 (1/5)
    // game_stage = 2 时，显示 j=0,1 的段 (2/5)
    if (j < game_stage) {
      for (int k = 0; k < 8; k++) {
        int line = j * 8 + k;
        for (int p = 1; p <= 8; p++) {
          my_ws2812_1_set(leaf, line * 8 + p, current_r, 0, 0);
        }
      }
    } else {
      // --- 未激活区域：强制熄灭 ---
      // 每个 j 段包含 8 行 (line)，每行 8 颗灯珠
      // 段 j 的行范围是: [j*8] 到 [j*8 + 7]
      for (int k = 0; k < 8; k++) {
        int line = j * 8 + k;
        // 熄灭该行对应的 8 颗灯珠 (索引 line*8+1 到 line*8+8)
        for (int p = 1; p <= 8; p++) {
          my_ws2812_1_set(leaf, line * 8 + p, 0, 0, 0);
        }
      }
    }
  }
}

// 已经被strip替换
void LED_current(uint16_t leaf) {
  int mode;

  // j 代表灯臂的 5 个段 (0-4)
  for (int j = 0; j < 5; j++) {
    // 【关键逻辑】只有在当前激活进度内的段才显示流水灯
    // game_stage = 1 时，显示 j=0 的段 (1/5)
    // game_stage = 2 时，显示 j=0,1 的段 (2/5)
    if (j < game_stage) {
      // --- 激活区域：正常播放流水灯特效 ---
      for (int i = 0; i < 4; i++) {
        mode = led_time1 - i;
        while (mode < 1)
          mode += 8;
        // 计算行号并显示
        LED_current_mode(leaf, mode, 3 - i + j * 8);
        LED_current_mode(leaf, mode, 4 + i + j * 8);
      }
    } else {
      // --- 未激活区域：强制熄灭 ---
      // 每个 j 段包含 8 行 (line)，每行 8 颗灯珠
      // 段 j 的行范围是: [j*8] 到 [j*8 + 7]
      for (int k = 0; k < 8; k++) {
        int line = j * 8 + k;
        // 熄灭该行对应的 8 颗灯珠 (索引 line*8+1 到 line*8+8)
        for (int p = 1; p <= 8; p++) {
          my_ws2812_1_set(leaf, line * 8 + p, 0, 0, 0);
        }
      }
    }
  }
}
void LED_current_all_on(uint16_t leaf) {
  for (int i = 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, current_r, 0, 0);
  }
}

void LED_current_all_off(uint16_t leaf) {
  for (int i = 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
}

void LED_circle(uint16_t leaf) {
  for (int i = CIRCLE_START + 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, 222, 0, 0);
  }
}

void LED_target(uint16_t leaf) {
  for (int i = CIRCLE6_START + 1; i <= CIRCLE5_START; i++)
    my_ws2812_1_set(leaf, i, 222, 0, 0);
  for (int i = CIRCLE5_START + 1; i <= CIRCLE4_START; i++)
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  for (int i = CIRCLE4_START + 1; i <= CIRCLE3_START; i++)
    my_ws2812_1_set(leaf, i, 222, 0, 0);
  for (int i = CIRCLE3_START + 1; i <= CIRCLE2_START; i++)
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  for (int i = CIRCLE2_START + 1; i <= CIRCLE1_START - 1; i++)
    my_ws2812_1_set(leaf, i, 222, 0, 0);
  for (int i = CIRCLE1_START; i <= CIRCLE2_RESTART; i++)
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  for (int i = CIRCLE2_RESTART + 1; i <= MY_WS2812_MAX_NUM; i++)
    my_ws2812_1_set(leaf, i, 254, 0, 0);
}

void Energy_WS2812_circle(uint16_t leaf) {
  // buffersize = (len*24);//+43;	// number of bytes needed is #LEDs * 24
  // bytes + 42 trailing bytes
  switch (LED_circle_mode[leaf]) {
    case LED_circle_none:
      Circle1_Close(leaf);
      Circle2_Close(leaf);
      Circle3_Close(leaf);
      break;
    case LED_circle1:
      Circle1_Open(leaf);
      Circle2_Close(leaf);
      Circle3_Close(leaf);
      break;
    case LED_circle2:
      Circle1_Close(leaf);
      Circle2_Open(leaf);
      Circle3_Close(leaf);
      break;
    case LED_circle3:
      Circle1_Close(leaf);
      Circle2_Close(leaf);
      Circle3_Open(leaf);
      break;
  }
}

void Energy_RUN(uint16_t leaf) {
  switch (energe_mode) {
    case Big_energe:
      switch (LED_mode[leaf])  // 小能量机关
      {
        case LED_none:
          my_ws2812_set_all_off(leaf);  // 全关
          break;
        case Waiting_hit:
          LED_current(leaf);  // 流水箭头
          side_off(leaf);     // 侧面关
          LED_circle(leaf);   // 靶子图案(进度条显示)
          break;
        case hit_finish:
          LED_current_all_on(leaf);  // 箭头流水全亮
          side_off(leaf);            // 侧面亮
          LED_circle(leaf);          // 靶子全亮
          break;
        case all_finish:
          all_finish_time[leaf]++;
          LED_current_all_on(leaf);  // 箭头流水全亮
          side_on(leaf);             // 侧面亮
          LED_circle(leaf);          // 靶子全亮
          if (all_finish_time[leaf] == 16) {
            all_finish_time[leaf] = 0;
            LED_mode[leaf] = LED_none;
          }
          break;
      }
      break;
    case Small_energe:
      switch (LED_mode[leaf])  // 大能量机关
      {
        case LED_none:
          my_ws2812_set_all_off(leaf);  // 全关
          break;
        case Waiting_hit:
          LED_current(leaf);  // 流水箭头
          side_off(leaf);     // 侧面关
          LED_circle(leaf);   // 靶子x图案(进度条显示)
          break;
        case hit_finish:
          LED_current_all_on(leaf);  // 箭头流水全亮
          side_off(leaf);            // 侧面亮
          LED_circle(leaf);          // 靶子全亮
          break;
        case all_finish:
          all_finish_time[leaf]++;
          LED_current_all_on(leaf);  // 箭头流水全亮
          side_on(leaf);             // 侧面亮
          LED_circle(leaf);          // 靶子全亮
          if (all_finish_time[leaf] == 16) {
            all_finish_time[leaf] = 0;
            LED_mode[leaf] = LED_none;
          }
          break;
      }
      break;
  }
  // transfer1();
}

void Energy_off_RUN(void) {
  for (int i = 0; i < 5; i++) {
    my_ws2812_set_all_off(i);
  }
  DMA_SetCurrDataCounter(
      DMA1_Stream4,
      WS28_SENDBUFF_SIZE);        // load number of bytes to be transferred
  DMA_Cmd(DMA1_Stream4, ENABLE);  // enable DMA channel 6
  TIM_Cmd(TIM3, ENABLE);          // enable Timer 3
  while (!DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4))
    ;                              // wait until transfer complete
  TIM_Cmd(TIM3, DISABLE);          // disable Timer 3
  DMA_Cmd(DMA1_Stream4, DISABLE);  // disable DMA channel 6
  DMA_ClearFlag(DMA1_Stream4,
                DMA_FLAG_TCIF4);  // clear DMA1 Channel 6 transfer complete flag
}

void Energy_on_RUN(void) {
  for (int i = 0; i < 5; i++) {
    my_ws2812_set_all_on(i);
  }
  DMA_SetCurrDataCounter(
      DMA1_Stream4,
      WS28_SENDBUFF_SIZE);        // load number of bytes to be transferred
  DMA_Cmd(DMA1_Stream4, ENABLE);  // enable DMA channel 6
  TIM_Cmd(TIM3, ENABLE);          // enable Timer 3
  while (!DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4))
    ;                              // wait until transfer complete
  TIM_Cmd(TIM3, DISABLE);          // disable Timer 3
  DMA_Cmd(DMA1_Stream4, DISABLE);  // disable DMA channel 6
  DMA_ClearFlag(DMA1_Stream4,
                DMA_FLAG_TCIF4);  // clear DMA1 Channel 6 transfer complete flag
}

void Energy_state_Send(uint8_t* pData) {
  CanTxMsg TX;
  TX.DLC = 0x08;
  TX.StdId = 0x300;
  TX.IDE = CAN_Id_Standard;
  TX.RTR = CAN_RTR_Data;
  for (int i = 0; i < 8; i++)
    TX.Data[i] = pData[i];
  CAN_Transmit(CAN1, &TX);

  TX.DLC = 0x08;
  TX.StdId = 0x301;
  TX.IDE = CAN_Id_Standard;
  TX.RTR = CAN_RTR_Data;
  for (int i = 0; i < 7; i++)
    TX.Data[i] = pData[8 + i];
  TX.Data[7] = 0;
  CAN_Transmit(CAN1, &TX);
  // while((CAN1->TSR&CAN_TSR_TME)==0);
}
void LED_target_allOff(uint16_t leaf) {
  for (int i = CIRCLE6_START + 1; i <= CIRCLE5_START; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
  for (int i = CIRCLE5_START + 1; i <= CIRCLE4_START; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
  for (int i = CIRCLE4_START + 1; i <= CIRCLE3_START; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
  for (int i = CIRCLE3_START + 1; i <= CIRCLE2_START; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
  for (int i = CIRCLE2_START + 1; i <= CIRCLE1_START - 1; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
  for (int i = CIRCLE1_START; i <= CIRCLE2_RESTART; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
  for (int i = CIRCLE2_RESTART + 1; i <= MY_WS2812_MAX_NUM; i++) {
    my_ws2812_1_set(leaf, i, 0, 0, 0);
  }
}
void LED_currentAll(uint16_t leaf) {
  int mode;

  for (int j = 0; j < 5; j++) {
    for (int i = 0; i < 4; i++) {
      mode = led_time1 - i;

      while (mode < 1)

        mode += 8;

      LED_current_mode(leaf, mode, 3 - i + j * 8);

      LED_current_mode(leaf, mode, 4 + i + j * 8);
    }
  }
}