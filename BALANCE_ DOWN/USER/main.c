#include "main.h"

//调试记录：VMC解算无问题，常用腿长0.23
//调试记录：完成上下板通信的嵌入，腿长还未嵌入，原balance_mode_switch未删，底盘跟随云台没写，小陀螺没写
int main()
{

   BSP_Init();
	control_task_Init();
	 
	while(1)
	{
		

	}
}
