#include "main.h"

/**
  ******************************************************************************
  * @file    平衡步兵下层（HT03）
  * @author  Lee_ZEKAI
  * @version V3.1.1
  * @intron   1.改下板上下通信串口（usart3->usart5）
             2.画新电池UI
             3.改底盘跟随云台速度，侧身速度，黑子云台微调
  * @date    2-JULY-2024						 
 ===============================================================================
 **/
 
int main()
{

   BSP_Init();
	IWDG_Configuration();
	control_task_Init();
	 
	while(1)
	{
		

	}
}
