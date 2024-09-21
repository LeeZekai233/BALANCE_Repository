#include "USART_chassis_transmit.h"

usart_chassis_data_t usart_chassis_data;
usart_gimbal_data_t usart_gimbal_data;
u8 databuff[100];

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
										u8 ctrl_mode)
{
	float yaw_0_2pi;
	yaw_0_2pi=fmod(-1*yaw_encoder_angle*ANGLE_TO_RAD,2*PI);	
	if(yaw_0_2pi<0)
		 yaw_0_2pi+=2*PI;
	
	 int32_t data = (int32_t)(yaw_0_2pi*10000);
    int16_t cp= chassis_power*100;
    int16_t r = roll*100;
    int16_t vx = x*100;
    int16_t vy = y*100;
    int16_t lg = cmd_leg_length*100;
	 databuff[0] = (uint8_t)if_follow_gim;
	 databuff[1] = (uint8_t)jump_cmd;
	 databuff[2] = (uint8_t)chassis_mode;
	 databuff[3] = (uint8_t)((lg) >> 8);
	 databuff[4] = (uint8_t)(lg);
	 databuff[5] = (uint8_t)((vx) >> 8);
	 databuff[6] = (uint8_t)(vx);
	 databuff[7] = (uint8_t)((vy) >> 8);
	 databuff[8] = (uint8_t)(vy);
	 databuff[9] = (uint8_t)((rotate_speed) >> 8);
	 databuff[10] = (uint8_t)(rotate_speed);
	 databuff[11] = (uint8_t)((cp) >> 8);
	 databuff[12] = (uint8_t)(cp);
	 databuff[13] = (uint8_t)((chassis_power_buffer) >> 8);
	 databuff[14] = (uint8_t)(chassis_power_buffer);
	 databuff[15] = (uint8_t)(data >> 24);
	 databuff[16] = (uint8_t)(data >> 16);
	 databuff[17] = (uint8_t)(data >> 8);
	 databuff[18] = (uint8_t)(data);
	databuff[19] = (uint8_t)(chassis_power_limit);
	databuff[20] = (uint8_t)(ctrl_mode);
    databuff[21] = (uint8_t)((r) >> 8);
	 databuff[22] = (uint8_t)(r);
     
	
	usart3.Send_bytes(&usart3,databuff,23);
}



void usart_chassis_receive(uint8_t *DataAddress)
{
	usart_chassis_data.yaw_Encoder_ecd_angle = ((int32_t)(((DataAddress[15]<<24)|(DataAddress[16]<<16)|(DataAddress[17]<<8)|DataAddress[18])))/10000.0f;
	usart_chassis_data.if_follow_gim = DataAddress[0];
	usart_chassis_data.jump_cmd = DataAddress[1];
	usart_chassis_data.chassis_mode = DataAddress[2];
	usart_chassis_data.cmd_leg_length = ((int16_t)((DataAddress[3]<<8)|DataAddress[4]))/100.0;
	usart_chassis_data.x = ((int16_t)((DataAddress[5]<<8)|DataAddress[6]))/100.0;
	usart_chassis_data.y = ((int16_t)((DataAddress[7]<<8)|DataAddress[8]))/100.0;
	usart_chassis_data.rotate_speed = ((DataAddress[9]<<8)|DataAddress[10]);
	usart_chassis_data.chassis_power = ((int16_t)((DataAddress[11]<<8)|DataAddress[12]))/100.0;
	usart_chassis_data.chassis_power_buffer = ((DataAddress[13]<<8)|DataAddress[14]);
	usart_chassis_data.chassis_power_limit = DataAddress[19];
	usart_chassis_data.ctrl_mode = DataAddress[20];
    usart_chassis_data.roll = ((int16_t)((DataAddress[21]<<8)|DataAddress[22]))/100.0;
}

void usart_gimbal_send(float cap_v,float input_V,float phi4,float phi1,float phi0,float L0,uint8_t jump_flag)
{
	usart_gimbal_data.cap_v = cap_v;
    usart_gimbal_data.input_V = input_V;
    usart_gimbal_data.phi4 = phi4;
    usart_gimbal_data.phi1 = phi1;
    usart_gimbal_data.phi0 = phi0;
    usart_gimbal_data.L0 = L0;
    usart_gimbal_data.jump_flag = jump_flag;
    
	usart5.Send_bytes(&usart5,(uint8_t *)&usart_gimbal_data,sizeof(usart_gimbal_data_t));
}

void usart_gimbal_receive(usart_gimbal_data_t *data,uint8_t *DataAddress)
{
	memcpy((uint8_t *)data,DataAddress,sizeof(usart_gimbal_data_t));
}