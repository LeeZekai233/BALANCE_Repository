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
 * @brief  发送底盘数据（云台 → 底盘），保留“先装填结构体”的逻辑
 * @param  ...                    各字段值（与结构体成员一一对应）
 * @param  USART_Chassis_Data     输出：将被填充为完整结构体（外部可能需使用）
 */
void usart_chassis_send(
    uint8_t  if_follow_gim,
    uint8_t  jump_cmd,
    uint8_t  overstep_cmd,
    uint8_t  Chassis_Mode,
    float    Yaw_Encoder_Angle,
    float    Cmd_Leg_Length,
    float    V_x,
    float    V_y,
    float    roll,
    int16_t  rotate_speed,
    uint8_t  Control_Mode,
    uint8_t  remote_online_flag,
    uint8_t  fric_wheel_run,
    uint8_t  Rollover_posture_cmd,
    uint8_t  low_speed_cmd,
    uint8_t  UI_auto_aim_state,
    uint8_t  Gimbal_Init_Finish_Flag,
    float    leg_single_angle_handle_left,
    float    leg_single_angle_handle_right,
    uint8_t  fn_2_trigger_flag,
    uint8_t  lock_shoot_check,
    USART_Chassis_Data_t *USART_Chassis_Data)
{
    uint8_t tx_buff[CHASSIS_FRAME_LENGTH];
    uint8_t index = 0;

    /* ---------- 1. 装填结构体（原封不动保留） ---------- */
    USART_Chassis_Data->if_follow_gim               = if_follow_gim;
    USART_Chassis_Data->jump_cmd                    = jump_cmd;
    USART_Chassis_Data->overstep_cmd                = overstep_cmd;
    USART_Chassis_Data->Chassis_Mode                = Chassis_Mode;
    USART_Chassis_Data->Yaw_Encoder_Angle           = Yaw_Encoder_Angle;
    USART_Chassis_Data->Cmd_Leg_Length              = Cmd_Leg_Length;
    USART_Chassis_Data->V_x                         = V_x;
    USART_Chassis_Data->V_y                         = V_y;
    USART_Chassis_Data->roll                        = roll;
    USART_Chassis_Data->rotate_speed                = rotate_speed;
    USART_Chassis_Data->Control_Mode                = Control_Mode;
    USART_Chassis_Data->remote_online_flag          = remote_online_flag;
    USART_Chassis_Data->fric_wheel_run              = fric_wheel_run;
    USART_Chassis_Data->Rollover_posture_cmd        = Rollover_posture_cmd;
    USART_Chassis_Data->low_speed_cmd               = low_speed_cmd;
    USART_Chassis_Data->UI_auto_aim_state           = UI_auto_aim_state;
    USART_Chassis_Data->Gimbal_Init_Finish_Flag     = Gimbal_Init_Finish_Flag;
    USART_Chassis_Data->leg_single_angle_handle_left  = leg_single_angle_handle_left;
    USART_Chassis_Data->leg_single_angle_handle_right = leg_single_angle_handle_right;
    USART_Chassis_Data->fn_2_trigger_flag           = fn_2_trigger_flag;
    USART_Chassis_Data->lock_shoot_check            = lock_shoot_check;

    /* ---------- 2. 打包发送（直接使用函数参数，与原始写法一致） ---------- */
    memset(tx_buff, 0, CHASSIS_FRAME_LENGTH);

    tx_buff[index++] = if_follow_gim;
    tx_buff[index++] = jump_cmd;
    tx_buff[index++] = overstep_cmd;
    tx_buff[index++] = Chassis_Mode;

    memcpy(&tx_buff[index], &Yaw_Encoder_Angle, 4);
    index += 4;

    memcpy(&tx_buff[index], &Cmd_Leg_Length, 4);
    index += 4;

    memcpy(&tx_buff[index], &V_x, 4);
    index += 4;

    memcpy(&tx_buff[index], &V_y, 4);
    index += 4;

    memcpy(&tx_buff[index], &roll, 4);
    index += 4;

    /* int16_t 手动小端打包 */
    tx_buff[index++] = (uint8_t)(rotate_speed & 0xFF);
    tx_buff[index++] = (uint8_t)((rotate_speed >> 8) & 0xFF);

    tx_buff[index++] = Control_Mode;
    tx_buff[index++] = remote_online_flag;
    tx_buff[index++] = fric_wheel_run;
    tx_buff[index++] = Rollover_posture_cmd;
    tx_buff[index++] = low_speed_cmd;
    tx_buff[index++] = UI_auto_aim_state;
    tx_buff[index++] = Gimbal_Init_Finish_Flag;

    memcpy(&tx_buff[index], &leg_single_angle_handle_left, 4);
    index += 4;

    memcpy(&tx_buff[index], &leg_single_angle_handle_right, 4);
    index += 4;

    tx_buff[index++] = fn_2_trigger_flag;
    tx_buff[index++] = lock_shoot_check;

    /* 长度校验 */
    if (index != CHASSIS_SEND_DATA_LENGTH) {
        return;
    }

    /* CRC8 并发送 */
    Append_CRC8_Check_Sum(tx_buff, CHASSIS_FRAME_LENGTH);
    if (Verify_CRC8_Check_Sum(tx_buff, CHASSIS_FRAME_LENGTH) == 0) {
        return;
    }

    memcpy(UART4_DMA_TX_BUF, tx_buff, CHASSIS_FRAME_LENGTH);

    if (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {
        return;
    }

    DMA_Cmd(DMA1_Stream4, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}

    DMA_ClearFlag(DMA1_Stream4,
                  DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 |
                  DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TCIF4);

    DMA_SetCurrDataCounter(DMA1_Stream4, CHASSIS_FRAME_LENGTH);
    DMA_Cmd(DMA1_Stream4, ENABLE);
}
