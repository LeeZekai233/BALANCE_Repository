#include "USART_chassis_transmit.h"

usart_chassis_data_t usart_chassis_data;
u8 databuff[100];

void usart_chassis_send(
												u8 if_follow_gim,
										u8 jump_cmd,
										u8 chassis_mode,
										float yaw_encoder_angle,
										int16_t cmd_leg_length,
										int16_t x,
										int16_t y,
										int16_t rotate_speed,
										int16_t chassis_power,
										uint16_t chassis_power_buffer,
										u8 chassis_power_limit)
{
	 int32_t data = (int32_t)(yaw_encoder_angle*10000);
	 databuff[0] = if_follow_gim;
	 databuff[1] = jump_cmd;
	 databuff[2] = chassis_mode;
	 databuff[3] = (uint8_t)((cmd_leg_length) >> 8);
	 databuff[4] = (uint8_t)(cmd_leg_length);
	 databuff[5] = (uint8_t)((x) >> 8);
	 databuff[6] = (uint8_t)(x);
	 databuff[7] = (uint8_t)((y) >> 8);
	 databuff[8] = (uint8_t)(y);
	 databuff[9] = (uint8_t)((rotate_speed) >> 8);
	 databuff[10] = (uint8_t)(rotate_speed);
	 databuff[11] = (uint8_t)((chassis_power) >> 8);
	 databuff[12] = (uint8_t)(chassis_power);
	 databuff[13] = (uint8_t)((chassis_power_buffer) >> 8);
	 databuff[14] = (uint8_t)(chassis_power_buffer);
	 databuff[15] = (uint8_t)(data >> 24);
	 databuff[16] = (uint8_t)(data >> 16);
	 databuff[17] = (uint8_t)(data >> 8);
	 databuff[18] = (uint8_t)(data);
	databuff[19] = (uint8_t)(chassis_power_limit);
	Uart3SendBytesInfoProc(databuff,20);
}


void usart_chassis_receive(uint8_t *DataAddress)
{
	usart_chassis_data.yaw_Encoder_ecd_angle = ((int32_t)(((DataAddress[15]<<24)|(DataAddress[16]<<16)|(DataAddress[17]<<8)|DataAddress[18])))/10000.0f;
	usart_chassis_data.if_follow_gim = DataAddress[0];
	usart_chassis_data.jump_cmd = DataAddress[1];
	usart_chassis_data.chassis_mode = DataAddress[2];
	usart_chassis_data.cmd_leg_length = ((DataAddress[3]<<8)|DataAddress[4]);
	usart_chassis_data.x = ((DataAddress[5]<<8)|DataAddress[6]);
	usart_chassis_data.y = ((DataAddress[7]<<8)|DataAddress[8]);
	usart_chassis_data.rotate_speed = ((DataAddress[9]<<8)|DataAddress[10]);
	usart_chassis_data.chassis_power = ((DataAddress[11]<<8)|DataAddress[12]);
	usart_chassis_data.chassis_power_buffer = ((DataAddress[13]<<8)|DataAddress[14]);
	usart_chassis_data.chassis_power_limit = DataAddress[19];
}
