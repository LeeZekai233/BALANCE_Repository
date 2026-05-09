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
                       float remain_heat,
					   uint8_t  game_state,usart_gimbal_data_t* usart_gimbal_data)
{
    
    unsigned char tx_buff[GIMBAL_SEND_DATA_LENGTH];//要改78 78
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
	usart_gimbal_data->remain_heat                    = remain_heat;
	usart_gimbal_data->game_state                     = game_state;
    
    memcpy(&tx_buff[0],&usart_gimbal_data,sizeof(usart_gimbal_data));
    Append_CRC8_Check_Sum(&tx_buff[0],GIMBAL_SEND_DATA_LENGTH);//加入crc8
    memcpy(&UART4_DMA_TX_BUF,&tx_buff[0],26);
    
    
     DMA_Cmd(DMA1_Stream4, DISABLE);                                  //关闭DMA传输
     DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);    //清除标志位
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}              //确保DMA可以被设置
    DMA_SetCurrDataCounter(DMA1_Stream4,26);                   //数据传输量
     DMA_Cmd(DMA1_Stream4, ENABLE);                                   //开启DMA传输
//	remain_heat = 0
    
}





void usart_chassis_receive(uint8_t *DataAddress,USART_Chassis_Data_t* USART_Chassis_Data)
{
    USART_Chassis_Data->yaw_Encoder_ecd_angle = ((int32_t)(((DataAddress[11]<<24)|(DataAddress[12]<<16)|(DataAddress[13]<<8)|DataAddress[14])))/10000.0f;
	USART_Chassis_Data->if_follow_gim = DataAddress[0];
	USART_Chassis_Data->jump_cmd = DataAddress[1];
	USART_Chassis_Data->chassis_mode = DataAddress[2];
	USART_Chassis_Data->cmd_leg_length = ((int16_t)((DataAddress[3]<<8)|DataAddress[4]))/100.0;
	USART_Chassis_Data->vx = ((int16_t)((DataAddress[5]<<8)|DataAddress[6]))/100.0;
	USART_Chassis_Data->vy = ((int16_t)((DataAddress[7]<<8)|DataAddress[8]))/100.0;
	USART_Chassis_Data->rotate_speed = ((DataAddress[9]<<8)|DataAddress[10]);
	USART_Chassis_Data->ctrl_mode = DataAddress[15];
    USART_Chassis_Data->roll = ((int16_t)((DataAddress[16]<<8)|DataAddress[17]))/100.0;
	USART_Chassis_Data->overstep_cmd = DataAddress[18];
	USART_Chassis_Data->remote_online_flag=DataAddress[19];
	USART_Chassis_Data->fric_wheel_run  =DataAddress[20];
	USART_Chassis_Data->Rollover_posture_cmd =DataAddress[21];
	USART_Chassis_Data->low_speed_cmd =DataAddress[22];
	USART_Chassis_Data->UI_auto_aim_state =DataAddress[23];
	USART_Chassis_Data->gimbal_data_if_finish_Init=DataAddress[24];
    USART_Chassis_Data->leg_single_angle_handle_left = ((int16_t)((DataAddress[25]<<8)|DataAddress[26]))/100.0;
    USART_Chassis_Data->leg_single_angle_handle_right = ((int16_t)((DataAddress[27]<<8)|DataAddress[28]))/100.0;
	USART_Chassis_Data->fn_2_trigger_flag = DataAddress[29];//按fn2
	USART_Chassis_Data->lock_shoot_check = DataAddress[30];
	
 //   gimbal_control_online_heart_cnt=time_tick;
	

	
}


//白控到虚拟双板通信的转换，用于调试
//void Remote_DT7_To_USART_Chassis_Data(Remote_DT7_t* Remote,USART_Chassis_Data_t* USART_Chassis_Data)
//{
//    if(Remote->Remote_clicker.s1 == DOWN)
//    {
//        USART_Chassis_Data->Chassis_Mode = 0;
//        
//    }
//    
//    if(Remote->Remote_clicker.s1 == MIDDLE)
//    {
//         USART_Chassis_Data->Chassis_Mode = 3;
////        if(Remote->Remote_clicker.s2_Action == MIDDLE_TO_DOWN)
////        {
////            USART_Chassis_Data->Chassis_Mode = 7;
////        }
////        else if(Remote->Remote_clicker.s2 == MIDDLE)
////        {
////            USART_Chassis_Data->Chassis_Mode = 10;
////        }
//    }
//    
//    USART_Chassis_Data->V_x = Remote->Remote_clicker.ch3 * 0.004;
//    USART_Chassis_Data->Omega = Remote->Remote_clicker.ch2 * 0.004;
//}


