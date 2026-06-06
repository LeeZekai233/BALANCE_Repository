#include "main.h"


USART_Gimbal_Data_t USART_Gimbal_Data;
USART_Chassis_Data_t USART_Chassis_Data;


void USART_Gimbal_Receive(uint8_t *DataAddress,USART_Gimbal_Data_t* USART_Gimbal_Data)
{
     memcpy(USART_Gimbal_Data,DataAddress,sizeof(*USART_Gimbal_Data));
}




/**
 * @brief  发送底盘数据（云台 → 底盘）
 * @param  data  指向待发送结构体的指针
 */
#define CHASSIS_SEND_DATA_LENGTH  43   /* 有效负载字节数 */
#define CHASSIS_FRAME_LENGTH      44   /* 负载 + 1 字节 CRC8 */

/**
 * @brief  
 * @param  
 * @param  USART_Chassis_Data     输出：将被填充为完整结构体（外部可能需使用）
 */
#define CHASSIS_SEND_DATA_LENGTH  43
#define CHASSIS_FRAME_LENGTH      44

void USART_Chassis_Send(const USART_Chassis_Data_t *data)
{
    uint8_t tx_buff[CHASSIS_FRAME_LENGTH];
    uint8_t index = 0;

    if (data == NULL) {
        return;
    }

    memset(tx_buff, 0, CHASSIS_FRAME_LENGTH);

    /* ---- 按结构体顺序逐个打包 ---- */
    tx_buff[index++] = data->if_follow_gim;
    tx_buff[index++] = data->jump_cmd;
    tx_buff[index++] = data->overstep_cmd;
    tx_buff[index++] = data->Chassis_Mode;

    /* float 成员：先用局部变量接收，再 memcpy */
    {
        float tmp = data->Yaw_Encoder_Angle;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    {
        float tmp = data->Cmd_Leg_Length;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    {
        float tmp = data->V_x;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    {
        float tmp = data->V_y;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    {
        float tmp = data->roll;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    /* int16_t rotate_speed 手动小端打包（本来就没问题） */
    {
        int16_t tmp = data->rotate_speed;
        tx_buff[index++] = (uint8_t)(tmp & 0xFF);
        tx_buff[index++] = (uint8_t)((tmp >> 8) & 0xFF);
    }

    tx_buff[index++] = data->Control_Mode;
    tx_buff[index++] = data->remote_online_flag;
    tx_buff[index++] = data->fric_wheel_run;
    tx_buff[index++] = data->Rollover_posture_cmd;
    tx_buff[index++] = data->low_speed_cmd;
    tx_buff[index++] = data->UI_auto_aim_state;
    tx_buff[index++] = data->Gimbal_Init_Finish_Flag;

    {
        float tmp = data->leg_single_angle_handle_left;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    {
        float tmp = data->leg_single_angle_handle_right;
        memcpy(&tx_buff[index], &tmp, 4);
    }
    index += 4;

    tx_buff[index++] = data->fn_2_trigger_flag;
    tx_buff[index++] = data->lock_shoot_check;

    /* 严格校验打包长度 */
    if (index != CHASSIS_SEND_DATA_LENGTH) {
        return;
    }

    /* 追加 CRC8 校验 */
    Append_CRC8_Check_Sum(tx_buff, CHASSIS_FRAME_LENGTH);

    /* 本地自检 */
    if (Verify_CRC8_Check_Sum(tx_buff, CHASSIS_FRAME_LENGTH) == 0) {
        return;
    }

    /* 拷贝到 DMA 缓冲区并发送 */
    memcpy(UART4_DMA_TX_BUF, tx_buff, CHASSIS_FRAME_LENGTH);

    if (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {
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

    DMA_SetCurrDataCounter(DMA1_Stream4, CHASSIS_FRAME_LENGTH);
    DMA_Cmd(DMA1_Stream4, ENABLE);
}
