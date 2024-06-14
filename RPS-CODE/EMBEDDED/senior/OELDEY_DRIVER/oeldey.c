#include "oeldey.h"

/**
  ******************************************************************************
  * @file    oeldey.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���д����ż����������ͨ�ŵ���ط�������յĺ�����
							value��λrpm�����õ���ż�����Ϊ7��value*7Ϊ����
							��ת��
						 
@verbatim
 ===============================================================================
 **/
 
void Set_OIDelec_speed(CAN_TypeDef *CANx, int16_t id, uint8_t cmd, int32_t value)
{
	  int32_t ecd_value = value*7;
    CanTxMsg tx_message;
    tx_message.StdId = id;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x05;
    tx_message.Data[0] = cmd;
    tx_message.Data[1] = (uint8_t)(ecd_value >> 24);
    tx_message.Data[2] = (uint8_t)(ecd_value >> 16);
    tx_message.Data[3] = (uint8_t)(ecd_value >> 8);
    tx_message.Data[4] = (uint8_t)ecd_value;
    
    CAN_Transmit(CANx,&tx_message);
}

void Set_OIDelec_heart(CAN_TypeDef *CANx, int16_t id, uint8_t cmd)
{
    CanTxMsg tx_message;
    tx_message.StdId = id;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x01;
    tx_message.Data[0] = cmd;
    
    
    CAN_Transmit(CANx,&tx_message);
}

void OIDelec_EncoderTask(volatile Encoder *v, CanRxMsg * msg)
{
	v->filter_rate = (msg->Data[2]<<24)|(msg->Data[3]<<16)|(msg->Data[4]<<8)|(msg->Data[5]);
}


