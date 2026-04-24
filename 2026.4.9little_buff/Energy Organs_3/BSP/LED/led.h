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

#define Sucker_PC_4 PCin(4)	//输入
#define Sucker_PA_6 PAin(6)	//输入
#define Sucker_PA_7 PAin(7)	//输入
#define Sucker_PC_7 PCin(7)	//输入

extern int LED_mode;
extern int LED_circle_mode;
extern int Flag_Input_Circle4;
extern int Flag_Input_Circle3;
extern int Flag_Input_Circle2;
extern int Flag_Input_Circle1;
extern int circle_lock;


void circle_key_input(void);
void Energy_Input(void);

void Sucker_PC4_Init(void);  //输入
void Sucker_PC7_Init(void);  //输入
void Sucker_PA6_Init(void);  //输入
void Sucker_PA7_Init(void);  //输入
void Sucker_PC10_Init(void);
void Sucker_PC8_Init(void); 
void LED_Init(void);//初始化		 				    
#endif
