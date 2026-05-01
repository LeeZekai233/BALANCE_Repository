#include "Radar.h"

/**
  ******************************************************************************
  * @file    Radar.c
  * @author  HDW
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了与视觉通信的内容，包含吊射数据的接收及发送
						 
@verbatim
 ===============================================================================
 **/
 
/******************************************auto_snipe_define***************************************/


Auto_Snipe_t My_Auto_Snipe;
New_Auto_Snipe_Aim_t New_Auto_Snipe_Aim;
New_Auto_Snipe_Aim_Send_t New_Auto_Snipe_Aim_Send;


u16 SNIPE_AUTO_CRC;
void Radar_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Snipe_t *Auto_Shoot)
{
	
	New_Auto_Snipe_Aim_t New_Auto_Aim_Medium;
	memcpy(&New_Auto_Aim_Medium,&address[0],sizeof(New_Auto_Aim_Medium));
	
	if(New_Auto_Aim_Medium.Header!=0xbe)
	 return;
	SNIPE_AUTO_CRC=Verify_CRC16_Check_Sum(address,length);	//length不减1 ，自瞄要减1
	if(!Verify_CRC16_Check_Sum(address,length))
		return;
	
	memcpy(&New_Auto_Snipe_Aim,&address[0],sizeof(New_Auto_Snipe_Aim));
	
	/**************************↓自瞄模式下的位置识别↓***************************/
	float Auto_Aim_Yaw_Angle_Medium=New_Auto_Snipe_Aim.Yaw_Angle;
	float Auto_Aim_Pitch_Angle_Medium=New_Auto_Snipe_Aim.Pitch_Angle;
	//单片机没法存储bull型变量，当视觉数据为bull型变量时，不进行数据处理
	if(Auto_Aim_Pitch_Angle_Medium==New_Auto_Snipe_Aim.Pitch_Angle&&Auto_Aim_Yaw_Angle_Medium==New_Auto_Snipe_Aim.Yaw_Angle)
	{
		if(Auto_Aim_Pitch_Angle_Medium!=0&&Auto_Aim_Pitch_Angle_Medium!=0)
		{

			Auto_Shoot->Auto_Aim.Yaw_Angle = New_Auto_Snipe_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle = New_Auto_Snipe_Aim.Pitch_Angle;

		}
		else
		{
			{

			}

		}
	}                             
		
}


float snipe_yaw_angle__pi_pi;
void Send_Radar(float Yaw, float Pitch, float Roll, int id, float ammo_speed, uint8_t mode, u8* data)
{
	   
	snipe_yaw_angle__pi_pi = convert_ecd_angle_to__pi_pi(Yaw,snipe_yaw_angle__pi_pi);
	New_Auto_Snipe_Aim_Send.Pitch=Pitch;
	New_Auto_Snipe_Aim_Send.Yaw=snipe_yaw_angle__pi_pi;

	
	if(ammo_speed < 10)
    {
        New_Auto_Snipe_Aim_Send.Shoot_Speed=16.4;
    }
	else
    {
       New_Auto_Snipe_Aim_Send.Shoot_Speed=ammo_speed; 
    }
	if(chassis.Vx!=0||chassis.Vy!=0)
	{
		New_Auto_Snipe_Aim_Send.Move_Flag=1;
	}
	data[0]=0xA5;
	memcpy(&data[1],&New_Auto_Snipe_Aim_Send,sizeof(New_Auto_Snipe_Aim_Send));
	Append_CRC16_Check_Sum(&data[0],sizeof(New_Auto_Snipe_Aim_Send)+3);
    Uart2SendBytesInfoProc(data,sizeof(New_Auto_Snipe_Aim_Send)+3);
}
