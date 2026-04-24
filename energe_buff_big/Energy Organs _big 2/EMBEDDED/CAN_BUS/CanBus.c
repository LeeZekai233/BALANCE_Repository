#include "CanBus.h"


/**
  ******************************************************************************
  * @file    CanBus.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件用于配置can总线的发送与接收任务，若设置模块id可去头文件设置
						 
	* @notice  有关can的模块的接收函数的使用请移步至can_bus.c文件中并在其中的接受
						 函数里选择模块id并调用解算函数，也可在can_bus.h文件中修改模块id
						 有关can的发送函数的使用也移步至can_bus.c文件，并在总发送任务函数中
						 配置要发送的函数，请务必将can_bus_send_task函数的调用放在controltask
						 中。
@verbatim
 ===============================================================================
 **/
 extern int game_stage;

uint16_t turntable_count=0;
void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
		case 0x310 ://遥控接受
			RC_CANrx1(msg->Data);
			break;
//		case 0x304:
//			CAN1_r1(msg->Data);
//			break;
//		case 0x200:
//			CAN1_r2(msg->Data);
//			break;
		case 0x301:
			CAN1_r3(msg->Data);
			break;
		default:
			break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {

    default:
        break;
    }
}








void can_bus_send_task(void)
{
	
		
}


void CAN1_r1(uint8_t *pData)
{
	if(pData == NULL)
	{
	return;
	}

	Flag_Input_4= ((int16_t)pData[1]);
	Flag_Input_5= ((int16_t)pData[3]);

}

void CAN1_r2(uint8_t *pData)
{
	if(pData == NULL)
	{
	return;
	}

	Flag_Input_3= ((int16_t)pData[1]);

}


void CAN1_r3(uint8_t *pData)
{
	if(pData == NULL)
	{
	return;
	}
//you know why?mode = 0,1,2 and no more!原来是懒得改？！
	LED[1].mode= ((int16_t)pData[1]);
	LED[2].mode=((int16_t)pData[3]);
	current_leaf=((int16_t)pData[5]);
game_stage=((int)pData[6]);
}

