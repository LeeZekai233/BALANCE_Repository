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
	
}


#define GIMBAL_PAYLOAD_LENGTH     25
#define GIMBAL_FRAME_LENGTH       26

#if (GIMBAL_SEND_DATA_LENGTH != GIMBAL_FRAME_LENGTH)
#error "GIMBAL_SEND_DATA_LENGTH must be 26: 25 bytes payload + 1 byte CRC8"
#endif


static void pack_u16_little_endian(uint8_t *buf, uint16_t data)
{
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
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
                       uint8_t  Gimbal_Init_Cmd,
                       float    remain_heat,
                       uint8_t  game_state,
                       USART_Gimbal_Data_t* USART_Gimbal_Data)
{
    uint8_t tx_buff[GIMBAL_FRAME_LENGTH];
    uint8_t index = 0;

    USART_Gimbal_Data->shooter_id1_17mm_cooling_heat   = shooter_id1_17mm_cooling_heat;
    USART_Gimbal_Data->shooter_barrel_heat_limit       = shooter_barrel_heat_limit;
    USART_Gimbal_Data->shooter_barrel_cooling_value    = shooter_barrel_cooling_value;
    USART_Gimbal_Data->robot_level                     = robot_level;
    USART_Gimbal_Data->bullet_speed_x_hat              = bullet_speed_x_hat;
    USART_Gimbal_Data->bullet_speed                    = bullet_speed;
    USART_Gimbal_Data->power_management_chassis_output = power_management_chassis_output;
    USART_Gimbal_Data->current_HP                      = current_HP;
    USART_Gimbal_Data->robot_id                        = robot_id;
    USART_Gimbal_Data->Gimbal_Init_Cmd                 = Gimbal_Init_Cmd;
    USART_Gimbal_Data->remain_heat                     = remain_heat;
    USART_Gimbal_Data->game_state                      = game_state;

    memset(tx_buff, 0, GIMBAL_FRAME_LENGTH);

    pack_u16_little_endian(&tx_buff[index], shooter_id1_17mm_cooling_heat);
    index += 2;

    pack_u16_little_endian(&tx_buff[index], shooter_barrel_heat_limit);
    index += 2;

    pack_u16_little_endian(&tx_buff[index], shooter_barrel_cooling_value);
    index += 2;

    tx_buff[index++] = robot_level;

    memcpy(&tx_buff[index], &bullet_speed_x_hat, 4);
    index += 4;

    memcpy(&tx_buff[index], &bullet_speed, 4);
    index += 4;

    tx_buff[index++] = power_management_chassis_output;

    pack_u16_little_endian(&tx_buff[index], current_HP);
    index += 2;

    tx_buff[index++] = robot_id;

    tx_buff[index++] = Gimbal_Init_Cmd;

    memcpy(&tx_buff[index], &remain_heat, 4);
    index += 4;

    tx_buff[index++] = game_state;

    /*
     * 此处必须正好打包 25 字节数据。
     * 第 26 字节留给 CRC8。
     */
    if (index != GIMBAL_PAYLOAD_LENGTH)
    {
        return;
    }

    Append_CRC8_Check_Sum(&tx_buff[0], GIMBAL_FRAME_LENGTH);

    /*
     * 发送端本地自检。
     * 如果这里都不过，说明 CRC 函数或长度定义有问题。
     * 正常情况下这里必须通过。
     */
    if (Verify_CRC8_Check_Sum(&tx_buff[0], GIMBAL_FRAME_LENGTH) == 0)
    {
        return;
    }

    memcpy(UART4_DMA_TX_BUF, tx_buff, GIMBAL_FRAME_LENGTH);

    /*
     * 如果上一帧还没发完，不要强行 DISABLE。
     * 原代码这里直接关闭 DMA，可能截断上一帧，导致对方收到半帧 + 新帧，CRC 必错。
     */
    if (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE)
    {
        return;
    }

    DMA_Cmd(DMA1_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}

    DMA_ClearFlag(DMA1_Stream4,
                  DMA_FLAG_FEIF4 |
                  DMA_FLAG_DMEIF4 |
                  DMA_FLAG_TEIF4 |
                  DMA_FLAG_HTIF4 |
                  DMA_FLAG_TCIF4);

    DMA1_Stream4->M0AR = (uint32_t)&UART4_DMA_TX_BUF[0];
    DMA_SetCurrDataCounter(DMA1_Stream4, GIMBAL_FRAME_LENGTH);

    DMA_Cmd(DMA1_Stream4, ENABLE);
}


void usart_chassis_receive(uint8_t *DataAddress,USART_Chassis_Data_t* USART_Chassis_Data)
{
    memcpy(USART_Chassis_Data,DataAddress,sizeof(*USART_Chassis_Data));
    gimbal_control_online_heart_cnt = time_tick ;
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
         USART_Chassis_Data->Chassis_Mode = 1;
//        if(Remote->Remote_clicker.s2_Action == MIDDLE_TO_DOWN)
//        {
//            USART_Chassis_Data->Chassis_Mode = 7;
//        }
//        else if(Remote->Remote_clicker.s2 == MIDDLE)
//        {
//            USART_Chassis_Data->Chassis_Mode = 10;
        
    }
    
    USART_Chassis_Data->V_y = Remote->Remote_clicker.ch3 * 0.004;
   // USART_Chassis_Data->Omega = Remote->Remote_clicker.ch2 * 0.004;
}


