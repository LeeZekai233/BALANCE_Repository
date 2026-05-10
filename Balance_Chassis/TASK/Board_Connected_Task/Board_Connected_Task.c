#include "main.h"


uint32_t gimbal_control_online_heart_cnt;

uint8_t gimbal_control_online_detective(void)//云台心跳检测
{
    uint8_t gimbal_control_state;
    if(time_tick - gimbal_control_online_heart_cnt > 500)
    {
        gimbal_control_state = 0;
		
    }
    else
    {
        gimbal_control_state = 1;
    }
    return gimbal_control_state;
	
//	 if(abs(time_tick - gimbal_control_online_heart_cnt) > 8000)
//    {
//        gimbal_control_state_longtime=0;		
//    }else
//    {
//        gimbal_control_state_longtime=1;
//    }
}


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
					   uint8_t  game_state,USART_Gimbal_Data_t* USART_Gimbal_Data)
{
    
    unsigned char tx_buff[GIMBAL_SEND_DATA_LENGTH];//要改78 78
	USART_Gimbal_Data->shooter_id1_17mm_cooling_heat  = shooter_id1_17mm_cooling_heat;//存进结构体
	USART_Gimbal_Data->shooter_barrel_heat_limit      = shooter_barrel_heat_limit;
	USART_Gimbal_Data->shooter_barrel_cooling_value   = shooter_barrel_cooling_value;
	USART_Gimbal_Data->robot_level                    = robot_level;
	USART_Gimbal_Data->bullet_speed_x_hat             = bullet_speed_x_hat; 
	USART_Gimbal_Data->bullet_speed                   = bullet_speed;
	USART_Gimbal_Data->power_management_chassis_output= power_management_chassis_output;
	USART_Gimbal_Data->current_HP                     = current_HP;
	USART_Gimbal_Data->robot_id                       = robot_id;
	USART_Gimbal_Data->allow_gimbal_init              = allow_gimbal_init;
	USART_Gimbal_Data->remain_heat                    = remain_heat;
	USART_Gimbal_Data->game_state                     = game_state;
    
    
    memcpy(tx_buff,USART_Gimbal_Data,25);
    Append_CRC8_Check_Sum(&tx_buff[0],GIMBAL_SEND_DATA_LENGTH);//加入crc8
    memcpy(UART4_DMA_TX_BUF,tx_buff,26);
    
    
     DMA_Cmd(DMA1_Stream4, DISABLE);                                  //关闭DMA传输
     DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);    //清除标志位
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}              //确保DMA可以被设置
    DMA_SetCurrDataCounter(DMA1_Stream4,26);                   //数据传输量
     DMA_Cmd(DMA1_Stream4, ENABLE);                                   //开启DMA传输
//	remain_heat = 0
    
}





void usart_chassis_receive(uint8_t *DataAddress,USART_Chassis_Data_t* USART_Chassis_Data)
{
    memcpy(USART_Chassis_Data,DataAddress,sizeof(*USART_Chassis_Data));
//	USART_Chassis_Data->if_follow_gim = DataAddress[0];
//	USART_Chassis_Data->jump_cmd = DataAddress[1];
//	USART_Chassis_Data->Chassis_Mode = DataAddress[2];
//	USART_Chassis_Data->Cmd_Leg_Length = ((int16_t)((DataAddress[3]<<8)|DataAddress[4]));
//	USART_Chassis_Data->V_x = ((int16_t)((DataAddress[5]<<8)|DataAddress[6]));
//	USART_Chassis_Data->V_y = ((int16_t)((DataAddress[7]<<8)|DataAddress[8]));
//	USART_Chassis_Data->rotate_speed = ((DataAddress[9]<<8)|DataAddress[10]);
//    USART_Chassis_Data->Yaw_Encoder_Angle = ((int32_t)(((DataAddress[11]<<24)|(DataAddress[12]<<16)|(DataAddress[13]<<8)|DataAddress[14])));
//	USART_Chassis_Data->Control_Mode = DataAddress[15];
//    USART_Chassis_Data->roll = ((int16_t)((DataAddress[16]<<8)|DataAddress[17]))/100.0;
//	USART_Chassis_Data->overstep_cmd = DataAddress[18];
//	USART_Chassis_Data->remote_online_flag=DataAddress[19];
//	USART_Chassis_Data->fric_wheel_run  =DataAddress[20];
//	USART_Chassis_Data->Rollover_posture_cmd =DataAddress[21];
//	USART_Chassis_Data->low_speed_cmd =DataAddress[22];
//	USART_Chassis_Data->UI_auto_aim_state =DataAddress[23];
//	USART_Chassis_Data->gimbal_data_if_finish_Init=DataAddress[24];
//    USART_Chassis_Data->leg_single_angle_handle_left = ((int16_t)((DataAddress[25]<<8)|DataAddress[26]));
//    USART_Chassis_Data->leg_single_angle_handle_right = ((int16_t)((DataAddress[27]<<8)|DataAddress[28]));
//	USART_Chassis_Data->fn_2_trigger_flag = DataAddress[29];//按fn2
//	USART_Chassis_Data->lock_shoot_check = DataAddress[30];
	
 //   gimbal_control_online_heart_cnt=time_tick;
	
	
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
         USART_Chassis_Data->Chassis_Mode = 3;
        if(Remote->Remote_clicker.s2_Action == MIDDLE_TO_DOWN)
        {
            USART_Chassis_Data->Chassis_Mode = 7;
        }
        else if(Remote->Remote_clicker.s2 == MIDDLE)
        {
            USART_Chassis_Data->Chassis_Mode = 10;
        }
    }
    
    USART_Chassis_Data->V_x = Remote->Remote_clicker.ch3 * 0.004;
   // USART_Chassis_Data->Omega = Remote->Remote_clicker.ch2 * 0.004;
}


