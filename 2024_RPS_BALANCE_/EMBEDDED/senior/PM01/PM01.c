#include "PM01.h"


/**
  ******************************************************************************
  * @file    LK_TECH.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���дLK_TECH���ͺŵ���Ľ���,��ڲ�������ͨ��
						 �������ṹ�壬can���ߵļ�����can�ṹ�壬��������ʼֵ
						 �趨.������������senior�ļ�
						 
@verbatim
 ===============================================================================
 **/
 
/******************************capacitance_define*************************************/
volatile capacitance_message_t capacitance_message;

void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x610:
	{
		v->mode = (msg->Data[0] << 8) | msg->Data[1];
		v->err_fdb = (msg->Data[2] << 8) | msg->Data[3];
	}
	break;
	case 0x611:
	{
		v->in_power = (msg->Data[0] << 8) | msg->Data[1];
		v->in_v = (msg->Data[2] << 8) | msg->Data[3];
		v->in_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x612:
	{
		v->out_power = (msg->Data[0] << 8) | msg->Data[1];
		v->out_v = (msg->Data[2] << 8) | msg->Data[3];
		v->out_i = (msg->Data[4] << 8) | msg->Data[5];
        v->cap_voltage_filte = v->out_v/100.0f;
	}
	break;
	case 0x613:
	{
		v->tempureture=(msg->Data[0]<<8)|msg->Data[1];
		v->time=(msg->Data[2]<<8)|msg->Data[3];
        v->this_time=(msg->Data[4]<<8)|msg->Data[5];
	}break;

	default:
		break;
	}
}



/**********************�������ݿ��ư�command**************************/

void PM01_command_set(CAN_TypeDef *CANx ,uint16_t data,uint32_t StdId) //���ò���ʹ������֡�����óɹ��������ã�����ʧ�ܷ��� 0x00 00
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (data >> 8);
    tx_message.Data[1] = data;
    tx_message.Data[2] = (0 >> 8);
    tx_message.Data[3] = 0;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

void PM01_data_read(CAN_TypeDef *CANx ,uint32_t StdId)//��ȡ���ݲ���Զ��֡���ʣ�ģ�鷴������������֡
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Remote;//CAN_RTR_Data;
    tx_message.DLC = 0x06;
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;

    CAN_Transmit(CANx,&tx_message);
}

void power_data_read_handle(CAN_TypeDef *CANx)
{
    PM01_data_read(CANx,0x610); //��ȡģ��״̬
    PM01_data_read(CANx,0x611); //��ȡ�����ѹ�����
    PM01_data_read(CANx,0x612); //��ȡ������ʣ���ѹ������
    PM01_data_read(CANx,0x613); //�¶ȣ��ۼ�����ʱ���뱾��ʱ��
}

void power_data_set_handle(CAN_TypeDef *CANx,u16 Max_Power)
{
    PM01_command_set(CANx,2, 0x600); //������ư忪��
    PM01_command_set(CANx,Max_Power * 100, 0x601); //������빦������
    PM01_command_set(CANx,2500, 0x602); //��������ѹ����
    PM01_command_set(CANx,7 * 100, 0x603); //��������������
}
