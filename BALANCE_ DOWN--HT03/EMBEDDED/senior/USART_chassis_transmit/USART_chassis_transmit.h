#ifndef __USART_chassis_transmit_H
#define __USART_chassis_transmit_H
#include "public.h"


typedef struct
{
	u8 if_follow_gim;
	u8 jump_cmd;
	u8 chassis_mode;
	float yaw_Encoder_ecd_angle;
	float cmd_leg_length;
	float x;
	float y;
	int16_t rotate_speed;
    float roll;
	float chassis_power;
	uint16_t chassis_power_buffer;
	u8 chassis_power_limit;
	u8 ctrl_mode;
} usart_chassis_data_t;

typedef struct
{
	float cap_v;
} usart_gimbal_data_t;
void usart_chassis_send(
												u8 if_follow_gim,
										u8 jump_cmd,
										u8 chassis_mode,
										float yaw_encoder_angle,
										float cmd_leg_length,
										float x,
										float y,
										int16_t rotate_speed,
                                        float roll,
										float chassis_power,
										uint16_t chassis_power_buffer,
										u8 chassis_power_limit,
										u8 ctrl_mode);
void usart_chassis_receive(uint8_t *DataAddress);
void usart_gimbal_send(float cap_v,float input_V);
void usart_gimbal_receive(uint8_t *DataAddress);										
extern usart_chassis_data_t usart_chassis_data;
extern usart_gimbal_data_t usart_gimbal_data;										
#endif
