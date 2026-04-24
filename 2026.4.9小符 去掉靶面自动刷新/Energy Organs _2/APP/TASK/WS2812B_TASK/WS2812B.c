 #include "ws2812b.h"

/****************** 结构体定义***********************/


/***************************************************/

/********************** 变量定义*********************/
int current_r = 254;//150
int side_r = 222;

int buf_r=2; //幅亮度
int buf_g=0; //幅亮度
int buf_b=0; //幅亮度

int buf_r_off=0; //幅亮度
int buf_g_off=0; //幅亮度
int buf_b_off=0; //幅亮度

int buf_r_edg=2; //幅亮度
int buf_g_edg=0; //幅亮度
int buf_b_edg=0; //幅亮度
//uint16_t LED_BYTE_Buffer[WS28_SENDBUFF_SIZE]; // PWM ?????
uint32_t LED_BYTE_Buffer1[WS28_SENDBUFF_SIZE];//600
uint16_t LED_BYTE_Buffer2[WS28_SENDBUFF_SIZE];
//uint16_t LED_BYTE_Buffer3[WS28_SENDBUFF_SIZE];
//uint16_t LED_BYTE_Buffer4[WS28_SENDBUFF_SIZE];
//uint16_t LED_BYTE_Buffer5[WS28_SENDBUFF_SIZE];



  int cnt = 0 ;
//	void transfer1(void)
//	{DMA_Cmd(DMA1_Stream2, DISABLE);
//		DMA_SetCurrDataCounter(DMA1_Stream2, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
//	DMA_Cmd(DMA1_Stream2, ENABLE); 			// enable DMA channel 6
//	TIM_Cmd(TIM5, ENABLE); 						// enable Timer 3
//		TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable);
//	while(!DMA_GetFlagStatus(DMA1_Stream2,DMA_FLAG_TCIF2)) ; 	// wait until transfer complete
//TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable);
//	DMA_Cmd(DMA1_Stream2, DISABLE); 			// disable DMA channel 6
//	DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2); 				// clear DMA1 Channel 6 transfer complete flag
//		}
	void transfer2(void)
		{	DMA_SetCurrDataCounter(DMA1_Stream4, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
		DMA_Cmd(DMA1_Stream4, ENABLE);
 						// enable Timer 3
		TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Enable);
			TIM_Cmd(TIM5, ENABLE);
	while(!DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)) ; 	// wait until transfer complete
			TIM_Cmd(TIM5, DISABLE); 	// disable Timer 3
TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Disable);

	DMA_Cmd(DMA1_Stream4, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); 				// clear DMA1 Channel 6 transfer complete flag
		}
	void transfer3(void)
	{
		
	
		DMA_SetCurrDataCounter(DMA1_Stream2, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream2, ENABLE); 			// enable DMA channel 6
		TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable);
		TIM_Cmd(TIM5, ENABLE); 						// enable Timer 3	
	while(!DMA_GetFlagStatus(DMA1_Stream2,DMA_FLAG_TCIF2)) ; 	// wait until transfer complete
		DMA_Cmd(DMA1_Stream2, DISABLE); 			// disable DMA channel 6	
				TIM_Cmd(TIM5, DISABLE); 	// disable Timer 3
	TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable);
	DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2); 				// clear DMA1 Channel 6 transfer complete flag
		
		
	}
//	void transfer4(void)
//	{DMA_SetCurrDataCounter(DMA1_Stream1, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
//	DMA_Cmd(DMA1_Stream1, ENABLE); 			// enable DMA channel 6
//	TIM_Cmd(TIM5, ENABLE); 						// enable Timer 3
//		TIM_CCxCmd(TIM5, TIM_Channel_4, TIM_CCx_Enable);
//	while(!DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1)) ; 	// wait until transfer complete
//	TIM_CCxCmd(TIM5, TIM_Channel_4, TIM_CCx_Disable);
//		TIM_Cmd(TIM5, DISABLE); 	// disable Timer 3
//	DMA_Cmd(DMA1_Stream1, DISABLE); 			// disable DMA channel 6
//	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1); 				// clear DMA1 Channel 6 transfer complete flag
//		}
//	void transfer5(void)
//	{
//		DMA_SetCurrDataCounter(DMA1_Stream5, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
//	DMA_Cmd(DMA1_Stream5, ENABLE); 			// enable DMA channel 6
//TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
//		TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);
//	while(!DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)) ; 	// wait until transfer complete
//		
//TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable);
//	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
//	DMA_Cmd(DMA1_Stream5, DISABLE); 			// disable DMA channel 6
//	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5); 				// clear DMA1 Channel 6 transfer complete flag
//		}
	//dma尚未配置
		
		void transfer(uint8_t leaf)
	{switch(leaf)
		{
	
		case 1:
			transfer2();
			break;
		case 2:
			transfer3();
			break;

	}
	
	}
		void my_ws2812_1_set(uint16_t leaf ,uint16_t num,uint8_t rv,uint8_t gv,uint8_t bv)
{
		uint32_t indexx=((num-1)*24);
	switch(leaf)
	{
		case energe_leaf1:
			for (uint8_t i = 0;i < 8;i++)
			{
					//填充数组
					LED_BYTE_Buffer1[indexx+i]      = (gv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 8]  = (rv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 16] = (bv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
			}
			break;
		case energe_leaf2:
			for (uint8_t i = 0;i < 8;i++)
			{
					//填充数组
				cnt++;
					LED_BYTE_Buffer1[indexx+i]      = (gv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 8]  = (rv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 16] = (bv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
			}
			break;
		case energe_leaf3:
			for (uint8_t i = 0;i < 8;i++)
			{
					//填充数组
					LED_BYTE_Buffer1[indexx+i]      = (gv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 8]  = (rv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 16] = (bv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
			}
			break;
		case energe_leaf4:
			for (uint8_t i = 0;i < 8;i++)
			{
					//填充数组
					LED_BYTE_Buffer1[indexx+i]      = (gv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 8]  = (rv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 16] = (bv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
			}
			break;
		case energe_leaf5:
			for (uint8_t i = 0;i < 8;i++)
			{
					//填充数组
					LED_BYTE_Buffer1[indexx+i]      = (gv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 8]  = (rv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
					LED_BYTE_Buffer1[indexx+i + 16] = (bv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
			}
			break;
	}
}


//关闭所有
void my_ws2812_set_all_off(uint16_t leaf)
{
	for(uint16_t i=1;i<=MY_WS2812_MAX_NUM;i++)
	{
		 my_ws2812_1_set(leaf,i,0,0,0);
	}
}

//开启所有
void my_ws2812_set_all_on(uint16_t leaf)
{
	for(uint16_t i=1;i<=MY_WS2812_MAX_NUM;i++)
	{
	   my_ws2812_1_set(leaf,i,buf_r,buf_g,buf_b);
	}	
}


void Input_Change_LED(uint16_t leaf)
{
	if(Flag_Input_Circle3==0&&Flag_Input_Circle2==0&&Flag_Input_Circle1==0)
		LED_circle_mode[leaf] = 0;//0表示全灭
	else if(Flag_Input_Circle3==0&&Flag_Input_Circle2==0&&Flag_Input_Circle1==1)
		LED_circle_mode[leaf] = 1;//1表示环数1亮
	else if(Flag_Input_Circle3==0&&Flag_Input_Circle2==1&&Flag_Input_Circle1==0)
		LED_circle_mode[leaf] = 2;//2表示环数2亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==0&&Flag_Input_Circle1==0)
		LED_circle_mode[leaf] = 3;//3表示环数3亮
	else if(Flag_Input_Circle3==0&&Flag_Input_Circle2==1&&Flag_Input_Circle1==1)
		LED_circle_mode[leaf] = 4;//4表示环数1、2亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==0&&Flag_Input_Circle1==1)
		LED_circle_mode[leaf] = 5;//5表示环数1、3亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==1&&Flag_Input_Circle1==0)
		LED_circle_mode[leaf] = 6;//6表示环数2、3亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==1&&Flag_Input_Circle1==1)
		LED_circle_mode[leaf] = 7;//7表示环数1、2、3亮
}

void Circle3_Open(uint16_t leaf)
{
			my_ws2812_1_set(leaf ,CIRCLE_START+1  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+2  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+3  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+4  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+5  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+6  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+7  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+8  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+9  ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+10 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+11 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+12 ,222 ,0 ,0);
}
void Circle2_Open(uint16_t leaf)
{
			my_ws2812_1_set(leaf ,CIRCLE_START+13 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+14 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+15 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+16 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+17 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+18 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+19 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+20 ,222 ,0 ,0);
}
void Circle1_Open(uint16_t leaf)
{
			my_ws2812_1_set(leaf ,CIRCLE_START+21 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+22 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+23 ,222 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+24 ,222 ,0 ,0);
}                           
void Circle3_Close(uint16_t leaf)    
{
			my_ws2812_1_set(leaf ,CIRCLE_START+1  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+2  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+3  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+4  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+5  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+6  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+7  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+8  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+9  ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+10 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+11 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+12 ,0 ,0 ,0);
}
void Circle2_Close(uint16_t leaf)
{
			my_ws2812_1_set(leaf ,CIRCLE_START+13 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+14 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+15 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+16 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+17 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+18 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+19 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+20 ,0 ,0 ,0);
}
void Circle1_Close(uint16_t leaf)
{
			my_ws2812_1_set(leaf ,CIRCLE_START+21 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+22 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+23 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,CIRCLE_START+24 ,0 ,0 ,0);
}

void LED_current_mode(uint16_t leaf ,int mode , int line)
{
	switch (mode)
	{
		case 1:
			my_ws2812_1_set(leaf ,line*8+1 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,0 ,0 ,0);
			break;
		case 2:
			my_ws2812_1_set(leaf ,line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,current_r,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,current_r,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,0 ,0 ,0);
			break;
		case 3:
			my_ws2812_1_set(leaf ,line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,0 ,0 ,0);
			break;
		case 4:
			my_ws2812_1_set(leaf ,line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,0 ,0 ,0);
			break;
		case 5:
			my_ws2812_1_set(leaf ,line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,0 ,0 ,0);
			break;
		case 6:
			my_ws2812_1_set(leaf ,line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,current_r ,0 ,0);
			break;                          
		case 7:
			my_ws2812_1_set(leaf ,line*8+1 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,current_r ,0 ,0);
			break;
		case 8:
			my_ws2812_1_set(leaf ,line*8+1 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+2 ,current_r ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(leaf ,line*8+8 ,current_r ,0 ,0);
			break;
		
	}
}
void set_side_circle_on(uint16_t leaf)
{
	for(uint16_t i=SIDE_START+1;i<=MY_WS2812_MAX_NUM;i++)
	{
	   my_ws2812_1_set(leaf ,i,222,0,0);
	}	
}

void side_off(uint16_t leaf)
{
	for(uint16_t i=SIDE_START+1;i<=CIRCLE_START;i++)
	{
	   my_ws2812_1_set(leaf ,i,0,0,0);
	}	
}

void side_on(uint16_t leaf)
{
	for(uint16_t i=SIDE_START+1;i<=CIRCLE_START;i++)
	{
	   my_ws2812_1_set(leaf ,i,side_r,0,0);
	}	
}


void LED_current(uint16_t leaf)
{
	int mode;
	for(int j=0; j<5 ; j++)
	{
		for(int i=0; i<4 ;i++)
		{
			mode = led_time1-i;
			while(mode < 1) mode +=8;
			LED_current_mode( leaf,mode ,3-i+j*8);
			LED_current_mode( leaf,mode ,4+i+j*8);
		}
	}
	
}

void LED_current_all_on(uint16_t leaf )
{
	for(int i=1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(leaf ,i ,current_r ,0 ,0);
	}

}

void LED_current_all_off(uint16_t leaf)
{
	for(int i=1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(leaf ,i ,0 ,0 ,0);
	}
}

void LED_circle(uint16_t leaf)
{
	for(int i=CIRCLE_START+1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(leaf ,i ,222 ,0 ,0);
	}
}

void LED_target(uint16_t leaf)
{
			for(int i=CIRCLE6_START+1; i<=CIRCLE5_START; i++)
	{
		my_ws2812_1_set(leaf ,i ,222 ,0 ,0);
	}
	for(int i=CIRCLE5_START+1; i<=CIRCLE4_START; i++)
	{
		my_ws2812_1_set(leaf ,i ,0 ,0 ,0);
	}
	for(int i=CIRCLE4_START+1; i<=CIRCLE3_START; i++)
	{
		my_ws2812_1_set(leaf ,i ,222 ,0 ,0);
	}
	for(int i=CIRCLE3_START+1; i<=CIRCLE2_START; i++)
	{
		my_ws2812_1_set(leaf ,i ,0 ,0 ,0);
	}
	for(int i=CIRCLE2_START+1; i<=CIRCLE1_START-1; i++)
	{
		my_ws2812_1_set(leaf ,i ,222 ,0 ,0);
	}
	for(int i=CIRCLE1_START; i<=CIRCLE2_RESTART; i++)
	{
		my_ws2812_1_set(leaf ,i ,0 ,0 ,0);
	}
	for(int i=CIRCLE2_RESTART+1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(leaf ,i ,254 ,0 ,0);
	}
}

void Energy_WS2812_circle(uint16_t leaf)
{
//	buffersize = (len*24);//+43;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes


	switch(LED_circle_mode[leaf])
	{
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
				
//		case 4:
//			Circle1_Open();
//			Circle2_Open();
//			Circle3_Close();
//		break;
//				
//		case 5:
//			Circle1_Open();
//			Circle2_Close();
//			Circle3_Open();
//		break;
//				
//		case 6:
//			Circle1_Close();
//			Circle2_Open();
//			Circle3_Open();
//		break;
//				
//		case 7:
//			Circle1_Open();
//			Circle2_Open();
//			Circle3_Open();
//		break;
	}
	
}


 void Energy_RUN(uint16_t leaf)
{	switch(energe_mode)
	{
		case Big_energe:
			switch(LED_mode[leaf])					//小能量机关
			{
				case LED_none:
					my_ws2812_set_all_off(leaf);	//全关
					break;
				case Waiting_hit:
					LED_current(leaf);				//流水箭头
					side_off(leaf);					//侧面关
					LED_circle(leaf);				//靶子图案
					break;
				case hit_finish:
					LED_current_all_on(leaf);		//箭头流水全亮
					side_off(leaf);					//侧面亮
					LED_circle(leaf);				//靶子全亮
					break;
				case all_finish:
					all_finish_time[leaf]++;
				
					LED_current_all_on(leaf);		//箭头流水全亮
					side_on(leaf);					//侧面亮
					LED_circle(leaf);				//靶子全亮
				
					if(all_finish_time[leaf] == 16)
					{
							all_finish_time[leaf] = 0;
							LED_mode[leaf] = LED_none;
//							clean_energe_leaf_flag();
					}
					break;
			}
			break;
		case Small_energe :
			switch(LED_mode[leaf])   					//大能量机关
			{
				case LED_none:
					my_ws2812_set_all_off(leaf);	//全关
					break;
				case Waiting_hit:
					LED_current(leaf);				//流水箭头
					side_off(leaf);					//侧面关
					LED_circle(leaf);				//靶子x图案
					break;
				case hit_finish:
					LED_current_all_on(leaf);		//箭头流水全亮
					side_off(leaf);					//侧面亮
					LED_circle(leaf);				//靶子全亮
					break;
				case all_finish:
					all_finish_time[leaf]++;
				
					LED_current_all_on(leaf);		//箭头流水全亮
					side_on(leaf);					//侧面亮
					LED_circle(leaf);				//靶子全亮
				
					if(all_finish_time[leaf] == 16)
					{
							all_finish_time[leaf] = 0;
							LED_mode[leaf] = LED_none;
//							clean_energe_leaf_flag();
					}
					break;
			}
			break;
		}
	
//		transfer1();

		
}

void Energy_off_RUN(void)
{
	for(int i=0; i<5 ; i++)
	{
		my_ws2812_set_all_off(i);
	}
	
	DMA_SetCurrDataCounter(DMA1_Stream4, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream4, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)) ; 	// wait until transfer complete
	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
	DMA_Cmd(DMA1_Stream4, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); 				// clear DMA1 Channel 6 transfer complete flag
				
}

void Energy_on_RUN(void)
{
	for(int i=0; i<5 ; i++)
	{
		my_ws2812_set_all_on(i);
	}
	DMA_SetCurrDataCounter(DMA1_Stream4, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream4, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)) ; 	// wait until transfer complete
	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
	DMA_Cmd(DMA1_Stream4, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); 				// clear DMA1 Channel 6 transfer complete flag
		
}


void Energy_state_Send(uint8_t *pData)
{
	
	CanTxMsg TX;
	
	TX.DLC=0x08;
	TX.StdId=0x300;
	TX.IDE=CAN_Id_Standard;
	TX.RTR=CAN_RTR_Data;
	TX.Data[0]  =  pData[0];
	TX.Data[1]  =  pData[1];
	TX.Data[2]  =  pData[2];
	TX.Data[3]  =  pData[3];
	TX.Data[4]  =  pData[4];
	TX.Data[5]  =  pData[5];
	TX.Data[6]  =  pData[6];
	TX.Data[7]  =  pData[7];
	
	CAN_Transmit(CAN1,&TX);
	
	
	TX.DLC=0x08;
	TX.StdId=0x301;
	TX.IDE=CAN_Id_Standard;
	TX.RTR=CAN_RTR_Data;
	TX.Data[0]  =  pData[8];
	TX.Data[1]  =  pData[9];
	TX.Data[2]  =  pData[10];
	TX.Data[3]  =  pData[11];
	TX.Data[4]  =  pData[12];
	TX.Data[5]  =  pData[13];
	TX.Data[6]  =  pData[14];
	TX.Data[7]  =  0;
	
	CAN_Transmit(CAN1,&TX);
	//while((CAN1->TSR&CAN_TSR_TME)==0);
}


