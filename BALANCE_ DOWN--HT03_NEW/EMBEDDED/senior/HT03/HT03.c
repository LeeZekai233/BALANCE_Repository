#include "HT03.h"




void CAN_HT03_SendControlPara(CAN_TypeDef *CANx,uint32_t id,float f_p, float f_v, float f_kp, float f_kd, float f_t)
{
    CanTxMsg txmsg;
    float real_T = f_t;
    uint16_t p, v, kp, kd, t;
    
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(real_T,      T_MIN,  T_MAX,  12);
    
    txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
    
    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    txmsg.Data[0] = p>>8;
	txmsg.Data[1] = p&0xFF;
	txmsg.Data[2] = v>>4;
	txmsg.Data[3] = ((v&0xF)<<4)|(kp>>8);
	txmsg.Data[4] = kp&0xFF;
	txmsg.Data[5] = kd>>4;
	txmsg.Data[6] = ((kd&0xF)<<4)|(t>>8);
	txmsg.Data[7] = t&0xff;
    
    CAN_Transmit(CANx,&txmsg);
}

void HT03_START(CAN_TypeDef *CANx,uint32_t id)
{
    CanTxMsg txmsg;
    txmsg.StdId = id;
	txmsg.DLC = 0x08;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.RTR = CAN_RTR_Data;
    
    txmsg.Data[0] = 0xFF;
    txmsg.Data[1] = 0xFF;
    txmsg.Data[2] = 0xFF;
    txmsg.Data[3] = 0xFF;
    txmsg.Data[4] = 0xFF;
    txmsg.Data[5] = 0xFF;
    txmsg.Data[6] = 0xFF;
    txmsg.Data[7] = 0xFC;
    
    CAN_Transmit(CANx,&txmsg);
}



    


void HT03_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{

    uint16_t vel_value;
	uint16_t pos_value;
	uint16_t cur_value;
    
    v->cal_data.heart_cnt = time_tick;
    v->cal_data.last_raw_angle_val = v->cal_data.raw_angle_val;
    
    pos_value = ((msg->Data[1] << 8) | msg->Data[2]);
    v->cal_data.raw_angle_val = uint_to_float(pos_value, P_MIN, P_MAX, 16);
    v->cal_data.angle_diff = v->cal_data.raw_angle_val - v->cal_data.last_raw_angle_val;
    if(v->cal_data.angle_diff < P_MIN)
    {
        v->cal_data.round_cnt++;
    }
    else if(v->cal_data.angle_diff > P_MAX)
    {
        v->cal_data.round_cnt--;
    }
    v->angle = v->cal_data.round_cnt*191.0 + v->cal_data.raw_angle_val + P_MAX;
    
    vel_value =  (msg->Data[3]<<4)|(msg->Data[4]>>4);
    v->gyro = uint_to_float(vel_value, V_MIN, V_MAX, 12);
    
    cur_value =  (((msg->Data[4]&0x0F))<<8)|(msg->Data[5]); //收到的电流信息
    v->Torque = uint_to_float(cur_value, T_MIN, T_MAX, 12)*MOTOR_Reduction_ratio;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval 
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
