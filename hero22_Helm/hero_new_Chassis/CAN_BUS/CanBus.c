#include "public.h"
//extern Encoder_plus Pitch_Encoder;
//extern Encoder_plus yaw_Encoder;
int64_t raw_angle=0;
extern can_capacitance_message_t can_capacitance_message;
int16_t test_rpm0;

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
 

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    switch(msg->StdId)
    {
		case GM1Encoder_MOTOR:
			GM6020EncoderProcess(&Helm_chassis.Heading_Encoder[0],msg);
		break;
		case GM2Encoder_MOTOR:
			GM6020EncoderProcess(&Helm_chassis.Heading_Encoder[1],msg);
		break;
		case GM3Encoder_MOTOR:
			GM6020EncoderProcess(&Helm_chassis.Heading_Encoder[2],msg);
		break;
		case GM4Encoder_MOTOR:
			GM6020EncoderProcess(&Helm_chassis.Heading_Encoder[3],msg);
		break;
		
        case CM1Encoder_MOTOR:
            M3508orM2006EncoderTask(&Helm_chassis.Driving_Encoder[0],msg);
            break;
        case CM2Encoder_MOTOR:
            M3508orM2006EncoderTask(&Helm_chassis.Driving_Encoder[1],msg);
            break;
        case CM3Encoder_MOTOR:
            M3508orM2006EncoderTask(&Helm_chassis.Driving_Encoder[2],msg);
            break;
        case CM4Encoder_MOTOR:
            M3508orM2006EncoderTask(&Helm_chassis.Driving_Encoder[3],msg);
            break;
        default:
			
        break;
        
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    Can_SuperCap_message_Process(&can_capacitance_message,msg);
    switch(msg->StdId)
    {
        case G2C_ID:
			CAN2_Chassis_Recieve(&chassis,msg);
//			CAN2_Gimbal_Gryo_Transmit();
//			CAN2_Gimbal_Motor_Transmit();
			Equipment_Counter_Make_Zero(&Peripheral_State.Gimbal_CAN_Data);
		break;
        case 0x100:
			CAN2_SuperCap_Data(msg);
		break;
        default:
            break;
            
    }
}






