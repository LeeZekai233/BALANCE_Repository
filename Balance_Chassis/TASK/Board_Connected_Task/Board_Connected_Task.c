#include "main.h"

//usart_gimbal_data_t usart_gimbal_data;
//usart_chassis_data_t usart_chassis_data;


//void gimbal_control_online_detective(void)//云台心跳检测
//{
//    if(abs(time_tick - gimbal_control_online_heart_cnt) > 500)
//    {
//        gimbal_control_state = 0;
//		
//    }else
//    {
//        gimbal_control_state = 1;
//    }
//	
//	 if(abs(time_tick - gimbal_control_online_heart_cnt) > 8000)
//    {
//        gimbal_control_state_longtime=0;		
//    }else
//    {
//        gimbal_control_state_longtime=1;
//    }
//}


void usart_gimbal_send(
					   uint16_t shooter_id1_17mm_cooling_heat,
	                   uint16_t shooter_barrel_heat_limit,
	                   uint16_t shooter_barrel_cooling_value,
	                   uint8_t  robot_level,
                       float    bullet_speed_x_hat,
					   float    bullet_speed,
                       uint8_t  power_management_chassis_output,
					   uint16_t current_HP,
					   uint8_t  robot_id,
					   uint8_t  allow_gimbal_init,
					   uint8_t  game_state,usart_gimbal_data_t* usart_gimbal_data)
{
    
    unsigned char tx_buff[GIMBAL_SEND_DATA_LENGTH];//要改
	usart_gimbal_data->shooter_id1_17mm_cooling_heat  = shooter_id1_17mm_cooling_heat;//存进结构体
	usart_gimbal_data->shooter_barrel_heat_limit      = shooter_barrel_heat_limit;
	usart_gimbal_data->shooter_barrel_cooling_value   = shooter_barrel_cooling_value;
	usart_gimbal_data->robot_level                    = robot_level;
	usart_gimbal_data->bullet_speed_x_hat             = bullet_speed_x_hat; 
	usart_gimbal_data->bullet_speed                   = bullet_speed;
	usart_gimbal_data->power_management_chassis_output= power_management_chassis_output;
	usart_gimbal_data->current_HP                     = current_HP;
	usart_gimbal_data->robot_id                       = robot_id;
	usart_gimbal_data->allow_gimbal_init              = allow_gimbal_init;
//	usart_gimbal_data.remain_heat                    = remain_heat;
	usart_gimbal_data->game_state                     = game_state;
    
    memcpy(&tx_buff[0],&usart_gimbal_data,sizeof(usart_gimbal_data));
    Append_CRC8_Check_Sum(&tx_buff[0],GIMBAL_SEND_DATA_LENGTH);//加入crc8
    Uart4SendBytesInfoProc(tx_buff,GIMBAL_SEND_DATA_LENGTH);//DMA发送
    
//	remain_heat = 0
    
}

//白控到虚拟双板通信的转换，用于调试
void Remote_DT7_To_USART_Chassis_Data(Remote_DT7_t* Remote,USART_Chassis_Data_t* USART_Chassis_Data)
{
    if(Remote->Remote_clicker.s1 == DOWN)
    {
        USART_Chassis_Data->Chassis_Mode = 0;
        
    }
    
    if(Remote->Remote_clicker.s1 == MIDDLE)
    {
         USART_Chassis_Data->Chassis_Mode = 2;
//        if(Remote->Remote_clicker.s2_Action == MIDDLE_TO_DOWN)
//        {
//            USART_Chassis_Data->Chassis_Mode = 7;
//        }
//        else if(Remote->Remote_clicker.s2 == MIDDLE)
//        {
//            USART_Chassis_Data->Chassis_Mode = 10;
//        }
    }
    
    USART_Chassis_Data->V_x = Remote->Remote_clicker.ch3 * 0.004;
    USART_Chassis_Data->Omega = Remote->Remote_clicker.ch2 * 0.004;
    if(Remote->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag == 0)
    {
        USART_Chassis_Data->Cmd_Leg_Length = 0.15f;
    }
    else if(Remote->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag == 1)
    {
        USART_Chassis_Data->Cmd_Leg_Length = 0.25f;
    }
}


