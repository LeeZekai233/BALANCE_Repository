#include "main.h"

/**
  ******************************************************************************
  * @file    平衡步兵下层（HT03）
  * @author  Lee_ZEKAI
  * @version V3.1.2
  * @intron   自动跳跃
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
