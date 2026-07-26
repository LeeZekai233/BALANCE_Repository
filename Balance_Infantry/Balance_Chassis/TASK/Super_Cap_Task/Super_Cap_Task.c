#include "main.h"

SuperCap_Send_t Super_Cap_Send;


uint8_t Capacitance_Message_Buf[100];
volatile can_capacitance_message_t can_capacitance_message;


/************************************* CAN 通信  ************************************************/
void Can_SuperCap_message_Process(volatile can_capacitance_message_t *v,CanRxMsg * msg)
{
    switch (msg->StdId)
    {
			
    case 0x123:
    {
        memcpy((uint8_t *)v,msg->Data,8);
    }
    break;
    case 0x124:
    {
        memcpy((uint8_t *)v+8,msg->Data,8);
    }
    break;
    case 0x125:
    {
        memcpy((uint8_t *)v+16,msg->Data,8);
    }
    break;
    case 0x126:
    {
        memcpy((uint8_t *)v+24,msg->Data,8);
    }
    break;
    case 0x127:
    {
        memcpy((uint8_t *)v+32,msg->Data,8);
    }
    break;
    case 0x128:
    {
        memcpy((uint8_t *)v+40,msg->Data,8);
    }
    break;


    default:
        break;
    }
}



void CAN_POWER_Control(CAN_TypeDef *CANx,SuperCap_Send_t *SC)
{
    if(judge_rece_mesg.game_robot_state.power_management_chassis_output==1 && Chassis.Driving_Motor[0].online_flag == 1 && Chassis.Driving_Motor[1].online_flag == 1)//上电并且左右轮子都在线
    {
          SC->Stop_Control_Flag=1;
    }
    else 
    {
         SC->Stop_Control_Flag=0;
    }
    SC->chassis_power_buffer=judge_rece_mesg.power_heat_data.buffer_energy;
    SC->chassis_power_limit =judge_rece_mesg.game_robot_state.chassis_power_limit;
    memcpy(&Capacitance_Message_Buf,(uint8_t *)SC,sizeof(SuperCap_Send_t));

    CanTxMsg tx_message;
    tx_message.StdId = 0x100;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    memcpy(tx_message.Data,(uint8_t *)SC,sizeof(SuperCap_Send_t));
    CAN_Transmit(CANx,&tx_message);
}

