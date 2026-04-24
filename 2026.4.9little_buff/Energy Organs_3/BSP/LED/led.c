#include "led.h" 
#include "sys.h"
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
int Flag_Input_Circle4;
int Flag_Input_Circle3;
int Flag_Input_Circle2;
int Flag_Input_Circle1;
int Flag_Input_PA7;
int Flag_Input_PA6;
int Flag_Input_PC4;
int Flag_Input_PC7;
int Flag_Input_PA7_Last;
int Flag_Input_PA6_Last;
int Flag_Input_PC4_Last;
int Flag_Input_PC7_Last;
int LED_mode = LED_none;
int LED_circle_mode =0;
int cnt_Circle_1;
int cnt_Circle_2;
int cnt_Circle_3;
int cnt_Circle_4;
int circle_lock = 0;

void circle_key_input(void)
{
	Flag_Input_PC4=(!Sucker_PC_4);
	Flag_Input_PC7=(!Sucker_PC_7);
	Flag_Input_PA6=(!Sucker_PA_6);
	Flag_Input_PA7=(!Sucker_PA_7);
	
	if(circle_lock == 0 && LED_mode == 2)
	{
		if(Flag_Input_PC4_Last==0&&Flag_Input_PC4==1)	
		{
			LED_circle_mode = 1;
			circle_lock = 1;
			LED_mode = hit_finish;
			LED_current_all_off();
		}
		if(Flag_Input_PA6_Last==0&&Flag_Input_PA6==1)
		{
			LED_circle_mode = 2;
			circle_lock = 1;
			LED_mode = hit_finish;
			LED_current_all_off();
		}
		if(Flag_Input_PA7_Last==0&&Flag_Input_PA7==1)
		{
			LED_circle_mode = 3;
			circle_lock = 1;
			LED_mode = 3;
			LED_current_all_off();
		}
		if(Flag_Input_PC7_Last==0&&Flag_Input_PC7==1)
		{
			LED_circle_mode = 4;
			circle_lock = 1;
			LED_mode = 3;
			LED_current_all_off();
		}
	}
	Flag_Input_PC4_Last = Flag_Input_PC4;
	Flag_Input_PC7_Last = Flag_Input_PC7;
	Flag_Input_PA6_Last = Flag_Input_PA6;
	Flag_Input_PA7_Last = Flag_Input_PA7;
	
}	

void Energy_Input(void)
{	
	Flag_Input_PC4=(!Sucker_PC_4);
	Flag_Input_PC7=(!Sucker_PC_7);
	Flag_Input_PA6=(!Sucker_PA_6);
	Flag_Input_PA7=(!Sucker_PA_7);
	
	if(Flag_Input_PC4_Last==0&&Flag_Input_PC4==1)	Flag_Input_Circle1=1;
	if(Flag_Input_PA6_Last==0&&Flag_Input_PA6==1)	Flag_Input_Circle2=1;
	if(Flag_Input_PA7_Last==0&&Flag_Input_PA7==1)	Flag_Input_Circle3=1;
	if(Flag_Input_PC7_Last==0&&Flag_Input_PC7==1)	Flag_Input_Circle4=1;
	
	Flag_Input_PC4_Last = Flag_Input_PC4;
	Flag_Input_PC7_Last = Flag_Input_PC7;
	Flag_Input_PA6_Last = Flag_Input_PA6;
	Flag_Input_PA7_Last = Flag_Input_PA7;
	
	Input_Change_LED();
	
	//Test in 灯不亮
	if(Flag_Input_Circle1==1)
	{
		cnt_Circle_1++;
		if(cnt_Circle_1>500)Flag_Input_Circle1=0;
	}
	else 
	{cnt_Circle_1=0;}
	
	if(Flag_Input_Circle2==1)
	{
		cnt_Circle_2++;
		if(cnt_Circle_2>500)Flag_Input_Circle2=0;
	}
	else 
	{cnt_Circle_2=0;}
	
	if(Flag_Input_Circle3==1)
	{
		cnt_Circle_3++;
		if(cnt_Circle_3>500)Flag_Input_Circle3=0;
	}
	else 
	{cnt_Circle_3=0;}
	
	if(Flag_Input_Circle4==1)
	{
		cnt_Circle_4++;
		if(cnt_Circle_4>500)Flag_Input_Circle4=0;
	}
	else 
	{cnt_Circle_4=0;}
	
	
}


//初始化PF9和PF10为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	
	GPIO_SetBits(GPIOC,GPIO_Pin_1 | GPIO_Pin_2);//GPIOF9,F10设置高，灯灭

}



/********************************
*@Brief：   PC4初始化
*@Cal：     内部或外部
*@param:    无
*@Note:     无
*@RetVal:   无
********************************/
void Sucker_PC4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruecture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruecture);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_4);
}

void Sucker_PC10_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruecture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruecture);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_4);
}
/********************************
*@Brief：   PA6初始化
*@Cal：     内部或外部
*@param:    无
*@Note:     无
*@RetVal:   无
********************************/
void Sucker_PA6_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruecture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruecture);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}
void Sucker_PC9_Init(void)
{GPIO_InitTypeDef GPIO_InitStruecture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruecture);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);
}
/********************************
*@Brief：   PA7初始化
*@Cal：     内部或外部
*@param:    无
*@Note:     无
*@RetVal:   无
********************************/
void Sucker_PA7_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruecture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruecture);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
}


/********************************
*@Brief：   PC7初始化
*@Cal：     内部或外部
*@param:    无
*@Note:     无
*@RetVal:   无
********************************/
void Sucker_PC7_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruecture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	GPIO_InitStruecture.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruecture.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStruecture.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruecture.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruecture.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruecture);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);
}



