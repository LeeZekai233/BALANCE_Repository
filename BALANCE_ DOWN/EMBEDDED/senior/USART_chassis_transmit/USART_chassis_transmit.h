#ifndef __USART_chassis_transmit_H
#define __USART_chassis_transmit_H
#include "public.h"


typedef struct
{
	u8 if_follow_gim;
	u8 jump_cmd;
	u8 chassis_mode;
	float yaw_Encoder_ecd_angle;
	int16_t cmd_leg_length;
	int16_t x;
	int16_t y;
	int16_t rotate_speed;
	int16_t chassis_power;
	uint16_t chassis_power_buffer;
	u8 chassis_power_limit;
} usart_chassis_data_t;



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
										u8 chassis_power_limit);
void usart_chassis_receive(uint8_t *DataAddress);
										
extern usart_chassis_data_t usart_chassis_data;
										
#endif
