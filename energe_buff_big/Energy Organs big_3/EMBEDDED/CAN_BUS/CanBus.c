#include "CanBus.h"


/**
  ******************************************************************************
  * @file    CanBus.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件用于配置can总线的发送与接收任务，若设置模块id可去头文件设置
						 
	* @    notice  有关can的模块的接收函数的使用请移步至can_bus.c文件中并在其中的接受
						 函数里选择模块id并调用解算函数，也可在can_bus.h文件中修改模块id
						 有关can的发送函数的使用也移步至can_bus.c文件，并在总发送任务函数中
						 配置要发送的函数，请务必将can_bus_send_task函数的调用放在controltask
						 中。
@verbatim
 ===============================================================================
 **/
 

uint16_t turntable_count=0;
void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
		case TURNTABLE_MOTOR:
			{
				turntable_count++;
				if(turntable_count<=5)  GetEncoderBias(&Motor620_Encoder ,msg);
				else EncoderProcess(&Motor620_Encoder ,msg);
				if(turntable_count > 10000)
				{
					turntable_count = 10000;
				}
			}break;
		case ENERGE_STM1:
			RC_CANrx1(msg->Data);
			break;
//		case ENERGE_STM2:
//			RC_CANrx2(msg->Data);
//			break;
		default:
			break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
        /* code */
        break;
			case TURNTABLE_MOTOR:
			{
				turntable_count++;
				if(turntable_count<=5)  GetEncoderBias(&Motor620_Encoder ,msg);
				else EncoderProcess(&Motor620_Encoder ,msg);
				if(turntable_count > 10000)
				{
					turntable_count = 10000;
				}
			}break;


    default:
        break;
    }
}


void Energy_state_Send(uint8_t *pData)
{
	
	CanTxMsg TX;
	
	TX.DLC=0x08;
	TX.StdId=0x310;
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
	
	
//	TX.DLC=0x08;
//	TX.StdId=0x302;
//	TX.IDE=CAN_Id_Standard;
//	TX.RTR=CAN_RTR_Data;
//	TX.Data[0]  =  pData[8];
//	TX.Data[1]  =  pData[9];
//	TX.Data[2]  =  pData[10];
//	TX.Data[3]  =  pData[11];
//	TX.Data[4]  =  pData[12];
//	TX.Data[5]  =  pData[13];
//	TX.Data[6]  =  pData[14];
//	TX.Data[7]  =  0;
//	
//	CAN_Transmit(CAN1,&TX);
//	//while((CAN1->TSR&CAN_TSR_TME)==0);
}


void can_bus_send_task(void)
{
	
	
}

