#include "public.h"
//extern Encoder_plus Pitch_Encoder;
//extern Encoder_plus yaw_Encoder;
extern Encoder Pitch_Encoder;
extern Encoder_plus yaw_Encoder;
extern LK_M_t LK_M_Gimbal_Yaw;
extern Encoder Poke_3508;
int64_t raw_angle=0;
extern friction_t general_friction;
extern can_capacitance_message_t can_capacitance_message;


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
        case C620_3508_rx_id1://左后大摩擦轮
            M3508orM2006EncoderTask(&general_friction.left_down_motor,msg);
            break;
        
        case C620_3508_rx_id2://下后大摩擦轮
            M3508orM2006EncoderTask(&general_friction.down_down_motor,msg);
            M3508orM2006EncoderTask(&test_down_1,msg);
            break;
        
        case C620_3508_rx_id3://右后大摩擦轮
            M3508orM2006EncoderTask(&general_friction.right_down_motor,msg);
            break;
        
        case C620_3508_rx_id4://左前小摩擦轮
            M3508orM2006EncoderTask(&general_friction.left_up_motor,msg);
            break;
        
        case C620_3508_rx_id5://下前小摩擦轮
            M3508orM2006EncoderTask(&general_friction.down_up_motor,msg);
            break;
        
        case C620_3508_rx_id6://右前小摩擦轮
            M3508orM2006EncoderTask(&general_friction.right_up_motor,msg);
            break;
            
        case PITCH_3508:
            M3508orM2006EncoderTask(&Pitch_Encoder,msg);
            break;
        case Scope_2006:
			M3508orM2006EncoderTask(&Scope_Encoder,msg);
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
        
        
        case C2G_ID_Gryo:
			CAN2_Chassis_Gyro_Recieve(&chassis,msg);
			Equipment_Counter_Make_Zero(&Peripheral_State.Chassis_CAN_Data_Gyro);
			
            break;
		case C2G_ID_Motor:
			CAN2_Chassis_Motor_Recieve(&chassis,msg);
			Equipment_Counter_Make_Zero(&Peripheral_State.Chassis_CAN_Data_Motor);
			break;
        
        case GIMBAL_YAW_MOTOR://LK_yaw_5010
            LK_task(&yaw_Encoder,msg,GMYawEncoder_Offset);
            LK_M_Data_Process(msg,GIMBAL_YAW_MOTOR_ID,&LK_M_Gimbal_Yaw);
            break;
        
        case 0x205:
            M3508orM2006EncoderTask(&Poke_3508,msg);
            break;
        
        default:
            break;
            
    }
}






