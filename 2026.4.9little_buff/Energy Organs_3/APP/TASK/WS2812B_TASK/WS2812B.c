 #include "ws2812b.h"

/****************** 结构体定义***********************/


/***************************************************/

/********************** 变量定义*********************/

int buf_r=0; //幅亮度0/254/220
int buf_g=0 ; //幅亮度
int buf_b=150; //幅亮度0/250/150     范围和典型值

int buf_r_off=0; //幅亮度
int buf_g_off=0; //幅亮度
int buf_b_off=0; //幅亮度

int buf_r_edg=2; //幅亮度
int buf_g_edg=0; //幅亮度
int buf_b_edg=0; //幅亮度

uint16_t LED_BYTE_Buffer[WS28_SENDBUFF_SIZE];//600

uint16_t rgb_i,rgb_j; 	//无用
/***************************************************/
//RGB
uint8_t rgb0[][3] = {222,0,0};	//RGB//没用


/*，*
  * @brief  填充数组MY_WS2812_GRB_BUF_1中的一颗灯珠
  * @param  uint16_t num:第多少个灯珠
  * @param  uint8_t rv：红颜色亮度
  * @param  uint8_t gv：绿颜色亮度
  * @param  uint8_t bv：蓝颜色亮度
  */
void my_ws2812_1_set(uint16_t num,uint8_t rv,uint8_t gv,uint8_t bv)
{
//		uint32_t indexx=MY_WS2812_RST_NUM+(num*(3*8));
		uint32_t indexx=((num-1)*24);
		for (uint8_t i = 0;i < 8;i++)
		{
				//填充数组
				LED_BYTE_Buffer[indexx+i]      = (gv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
				LED_BYTE_Buffer[indexx+i + 8]  = (rv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
				LED_BYTE_Buffer[indexx+i + 16] = (bv << i) & (0x80)?TIMING_ONE:TIMING_ZERO;
		}
		
}

void clean_energe_mode(void)
{
	LED_mode = LED_none;
	circle_lock = 0;
	LED_circle_mode =LED_circle_none;
	turntable_count=0;
	time_Small_energe=0;
	time_Big_energe=0;
	Motor620_Encoder.round_cnt = 0;
	Motor620_Encoder.ecd_bias = Motor620_Encoder.raw_value; 
	pid_clear();
}

//关闭所有
void my_ws2812_set_all_off(void)
{
	for(uint16_t i=1;i<=MY_WS2812_MAX_NUM;i++)
	{
		 my_ws2812_1_set(i,0,0,0);
	}
}

//开启所有
void my_ws2812_set_all_on(void)
{
	for(uint16_t i=1;i<=MY_WS2812_MAX_NUM;i++)
	{
	   my_ws2812_1_set(i,buf_r,buf_g,buf_b);
	}	
}


void Input_Change_LED(void)
{
	if(Flag_Input_Circle3==0&&Flag_Input_Circle2==0&&Flag_Input_Circle1==0)
		LED_circle_mode = 0;//0表示全灭
	else if(Flag_Input_Circle3==0&&Flag_Input_Circle2==0&&Flag_Input_Circle1==1)
		LED_circle_mode = 1;//1表示环数1亮
	else if(Flag_Input_Circle3==0&&Flag_Input_Circle2==1&&Flag_Input_Circle1==0)
		LED_circle_mode = 2;//2表示环数2亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==0&&Flag_Input_Circle1==0)
		LED_circle_mode = 3;//3表示环数3亮
	else if(Flag_Input_Circle3==0&&Flag_Input_Circle2==1&&Flag_Input_Circle1==1)
		LED_circle_mode = 4;//4表示环数1、2亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==0&&Flag_Input_Circle1==1)
		LED_circle_mode = 5;//5表示环数1、3亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==1&&Flag_Input_Circle1==0)
		LED_circle_mode = 6;//6表示环数2、3亮
	else if(Flag_Input_Circle3==1&&Flag_Input_Circle2==1&&Flag_Input_Circle1==1)
		LED_circle_mode = 7;//7表示环数1、2、3亮
}

void Circle3_Open(void)
{
			my_ws2812_1_set(CIRCLE_START+1  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+2  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+3  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+4  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+5  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+6  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+7  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+8  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+9  ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+10 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+11 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+12 ,222 ,0 ,0);
}
void Circle2_Open(void)
{
			my_ws2812_1_set(CIRCLE_START+13 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+14 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+15 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+16 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+17 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+18 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+19 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+20 ,222 ,0 ,0);
}
void Circle1_Open(void)
{
			my_ws2812_1_set(CIRCLE_START+21 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+22 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+23 ,222 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+24 ,222 ,0 ,0);
}
void Circle3_Close(void)
{
			my_ws2812_1_set(CIRCLE_START+1  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+2  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+3  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+4  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+5  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+6  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+7  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+8  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+9  ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+10 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+11 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+12 ,0 ,0 ,0);
}
void Circle2_Close(void)
{
			my_ws2812_1_set(CIRCLE_START+13 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+14 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+15 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+16 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+17 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+18 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+19 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+20 ,0 ,0 ,0);
}
void Circle1_Close(void)
{
			my_ws2812_1_set(CIRCLE_START+21 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+22 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+23 ,0 ,0 ,0);
			my_ws2812_1_set(CIRCLE_START+24 ,0 ,0 ,0);
}

void LED_current_mode(int mode , int line)
{
	switch (mode)
	{
		case 1:
			my_ws2812_1_set(line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,0 ,0 ,0);
			break;
		case 2:
			my_ws2812_1_set(line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,0 ,0 ,0);
			break;
		case 3:
			my_ws2812_1_set(line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,0 ,0 ,0);
			break;
		case 4:
			my_ws2812_1_set(line*8+1 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,0 ,0 ,0);
			break;
		case 5:
			my_ws2812_1_set(line*8+1 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,222 ,0 ,0);
			break;
		case 6:
			my_ws2812_1_set(line*8+1 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,222 ,0 ,0);
			break;
		case 7:
			my_ws2812_1_set(line*8+1 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,222 ,0 ,0);
			break;
		case 8:
			my_ws2812_1_set(line*8+1 ,222 ,0 ,0);
			my_ws2812_1_set(line*8+2 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+3 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+4 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+5 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+6 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+7 ,0 ,0 ,0);
			my_ws2812_1_set(line*8+8 ,222 ,0 ,0);
			break;
	}
}

void set_side_circle_on(void)
{
	for(uint16_t i=SIDE_START+1;i<=MY_WS2812_MAX_NUM;i++)
	{
	   my_ws2812_1_set(i,222,0,0);
	}	
}

void side_off(void)
{
	for(uint16_t i=SIDE_START+1;i<=CIRCLE_START;i++)
	{
	   my_ws2812_1_set(i,0,0,0);
	}	
}

void side_on(void)
{
	for(uint16_t i=SIDE_START+1;i<=CIRCLE_START;i++)
	{
	   my_ws2812_1_set(i,222,0,0);
	}	
}


void LED_current(void)
{
	int mode;
	for(int i=0; i<8*5 ;i++)
	{
		mode = led_time1-i;
		while(mode < 1) mode +=8;
		LED_current_mode( mode ,i);
	}
}

void LED_current_all_on(void)
{
	for(int i=1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(i ,222 ,0 ,0);
	}
}

void LED_current_all_off(void)
{
	for(int i=1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(i ,0 ,0 ,0);
	}
}

void LED_circle(void)
{
	for(int i=CIRCLE_START+1; i<=MY_WS2812_MAX_NUM; i++)
	{
		my_ws2812_1_set(i ,222 ,0 ,0);
	}
}
void Enegy_WS2812_R(void)
{	for(uint8_t i=1;i<=64;i++)
	{
	my_ws2812_1_set(i,222,0,0);
	}
	DMA_SetCurrDataCounter(DMA1_Stream7, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream7, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)) ; 	// wait until transfer complete
	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
	DMA_Cmd(DMA1_Stream7, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 				// clear DMA1 Channel 6 transfer complete flag
}
//关闭R标
void Enegy_WS2812_R_off(void)
{	for(uint8_t i=0;i<64;i++)
	my_ws2812_1_set(i+1,0,0,0);
	DMA_SetCurrDataCounter(DMA1_Stream7, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream7, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)) ; 	// wait until transfer complete
	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
	DMA_Cmd(DMA1_Stream7, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 				// clear DMA1 Channel 6 transfer complete flag
}

void Energy_WS2812_circle(void)
{
//	buffersize = (len*24);//+43;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes


	switch(LED_circle_mode)
	{
		case LED_circle_none:
			Circle1_Close();
			Circle2_Close();
			Circle3_Close();
			break;
			
		case LED_circle1:
			Circle1_Open();
			Circle2_Close();
			Circle3_Close();
		break;
				
		case LED_circle2:
			Circle1_Close();
			Circle2_Open();
			Circle3_Close();
		break;
				
		case LED_circle3:
			Circle1_Close();
			Circle2_Close();
			Circle3_Open();
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

void Energy_RUN(void)
{
//	switch(energe_mode)
//	{
//		case Big_energe:
//			switch(LED_mode)					//小能量机关
//			{
//				case LED_none:
//					circle_lock=1;
//	//				my_ws2812_set_all_off();	//全关
//					break;
//				case Waiting_hit:
//					circle_lock=0;
////					LED_current();				//流水箭头
////					side_off();					//侧面关
////					LED_circle();				//靶子全亮  图案待写
//					break;
//				case hit_finish:
////					LED_current_all_on();		//箭头流水全亮
////					side_on();					//侧面亮
////					LED_circle();				//靶子全亮
//					break;
//			}
//			break;
//		case Small_energe :
//			switch(LED_mode)   					//大能量机关
//			{
//				case LED_none:
//					circle_lock=1;
//		//			my_ws2812_set_all_off();	//全关
//					break;
//				case Waiting_hit:
//					circle_lock=0;
////					LED_current();				//流水箭头
////					side_off();					//侧面关
////					LED_circle();				//靶子全亮  图案待写
//					break;
//				case hit_finish:
////					LED_current_all_on();		//箭头流水全亮
////					side_on();					//侧面亮
////					Energy_WS2812_circle();		//靶子保持单环
//					break;
//			}
//			break;
//		}
	
	DMA_SetCurrDataCounter(DMA1_Stream7, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream7, ENABLE); 			// enable DMA channel 6
//		TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable);
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)) ; 	// wait until transfer complete
DMA_Cmd(DMA1_Stream7, DISABLE); 			// disable DMA channel 6	
TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
	
	DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 				// clear DMA1 Channel 6 transfer complete flag
		
}

void Energy_off_RUN(void)
{
	my_ws2812_set_all_off();
	
	DMA_SetCurrDataCounter(DMA1_Stream7, WS28_SENDBUFF_SIZE); 	// load number of bytes to be transferred
	DMA_Cmd(DMA1_Stream7, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	while(!DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)) ; 	// wait until transfer complete
	TIM_Cmd(TIM3, DISABLE); 	// disable Timer 3
	DMA_Cmd(DMA1_Stream7, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 				// clear DMA1 Channel 6 transfer complete flag
		
}






