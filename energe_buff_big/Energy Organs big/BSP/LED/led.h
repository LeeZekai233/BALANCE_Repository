#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "public.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED端口定义
#define LED0_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED1_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define LED0_OFF GPIO_SetBits(GPIOC,GPIO_Pin_1)
#define LED1_OFF GPIO_SetBits(GPIOC,GPIO_Pin_2)	 

//
#define Sucker_PB_5   	PBin(5)//?读取传感器输入电平?
#define Sucker_PB_4   	PBin(4)
#define Sucker_PB_3  		PBin(3)
#define Sucker_PC_12  	PCin(12)
#define Sucker_PC_11  	PCin(11)
#define Sucker_PA_1  		PAin(1)


extern int LED_mode[5];
extern int LED_circle_mode[5];
extern int Leaf_mode[5];

extern int Flag_Input_Circle4;
extern int Flag_Input_Circle3;
extern int Flag_Input_Circle2;
extern int Flag_Input_Circle1;

extern int Flag_Input_1  ;//高电平受击.低电平未受击
extern int Flag_Input_2  ;
extern int Flag_Input_3  ;
extern int Flag_Input_4  ;
extern int Flag_Input_5  ;

extern int flag2_return ;
extern int flag3_return ;
extern int flag4_return ;
extern int flag5_return ;
extern uint16_t count_reset;
extern uint16_t reset_time;
void key_input(void);//检测并改变LED受击状态
void circle_key_input(uint16_t leaf);//无
void Energy_Input(uint16_t leaf);//无
void Sucker_PA1_Init(void);
void Sucker_PC12_Init(void);//1芯片设置这五个口输出
void Sucker_PC11_Init(void);
void Sucker_PB5_Init(void);
void Sucker_PB4_Init(void);
void Sucker_PB3_Init(void);



void LED_Init(void);//初始化	PC1PC2	 				    
#endif
