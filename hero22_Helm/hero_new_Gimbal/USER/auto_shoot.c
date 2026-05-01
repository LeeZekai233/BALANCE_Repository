#include "public.h"

Auto_Shoot_t My_Auto_Shoot;
New_Auto_Aim_t New_Auto_Aim;
New_Auto_Aim_Send_t New_Auto_Aim_Send;

u16 AUTO_CRC;
void Vision_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Shoot_t *Auto_Shoot)
{
	
	New_Auto_Aim_t New_Auto_Aim_Medium;
	memcpy(&New_Auto_Aim_Medium,&address[0],sizeof(New_Auto_Aim_Medium));
	
	if(New_Auto_Aim_Medium.Header!=0xbe)
	 return;
	AUTO_CRC=Verify_CRC16_Check_Sum(address,length);	//length不减1 ，自瞄要减1
	memcpy(&New_Auto_Aim,&address[0],sizeof(New_Auto_Aim));
	
	/**************************↓自瞄模式下的位置识别↓***************************/
	float Auto_Aim_Yaw_Angle_Medium=New_Auto_Aim.Yaw_Angle;
	float Auto_Aim_Pitch_Angle_Medium=New_Auto_Aim.Pitch_Angle;
	//单片机没法存储bull型变量，当视觉数据为bull型变量时，不进行数据处理
	if(Auto_Aim_Pitch_Angle_Medium==New_Auto_Aim.Pitch_Angle&&Auto_Aim_Yaw_Angle_Medium==New_Auto_Aim.Yaw_Angle)
	{
		if(Auto_Aim_Pitch_Angle_Medium!=0&&Auto_Aim_Pitch_Angle_Medium!=0)
		{
			Auto_Shoot->Auto_Aim.Yaw_Angle_Last = Auto_Shoot->Auto_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle_Last = Auto_Shoot->Auto_Aim.Pitch_Angle;
			Auto_Shoot->Auto_Aim.Yaw_Angle = New_Auto_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle = New_Auto_Aim.Pitch_Angle;
			Auto_Shoot->Auto_Aim.Flag_Get_Target = 1;
			Auto_Shoot->Auto_Aim.enable_shoot = New_Auto_Aim.enable_shoot;
			
			Auto_Shoot->Auto_Aim.Lost_Cnt=0;
		}
		else
		{
			{
				Auto_Shoot->Auto_Aim.Flag_Get_Target = 0;
//				Auto_Shoot->Auto_Aim.Yaw_Angle = gimbal_gyro.yaw_Angle;
//				Auto_Shoot->Auto_Aim.Pitch_Angle = gimbal_gyro.pitch_Angle;
				
				Auto_Shoot->Buff.Flag_Get_Target = 0;
//				Auto_Shoot->Buff.Yaw_Angle = gimbal_gyro.yaw_Angle;
//				Auto_Shoot->Buff.Pitch_Angle = gimbal_gyro.pitch_Angle;
			}
			Auto_Shoot->Auto_Aim.enable_shoot = 0;
		}
	}
}


float yaw_angle__pi_pi;
void send_protocol_New(float Yaw, float Pitch, float Roll, int id, float ammo_speed, uint8_t mode, u8* data)
{
	
	yaw_angle__pi_pi = convert_ecd_angle_to__pi_pi(Yaw,yaw_angle__pi_pi);
	New_Auto_Aim_Send.Pitch=Pitch;
	New_Auto_Aim_Send.Roll=Roll;
	New_Auto_Aim_Send.Yaw=yaw_angle__pi_pi;
	if(id>100)
		New_Auto_Aim_Send.Current_Color=1;
	else
		New_Auto_Aim_Send.Current_Color=0;
	
	
	New_Auto_Aim_Send.mode=0;
	
	if(ammo_speed < 10)
    {
        New_Auto_Aim_Send.Shoot_Speed=11.7;
    }else
    {
       New_Auto_Aim_Send.Shoot_Speed=ammo_speed; 
    }
	
	New_Auto_Aim_Send.game_state = judge_rece_mesg.game_state.game_progress;
	//New_Auto_Aim_Send.poke_state = 1;
    New_Auto_Aim_Send.shoot_freqency= 1;
    
	data[0]=0xbe;
    //New_Auto_Aim_Send.header=0xBE;
    
	memcpy(&data[1],&New_Auto_Aim_Send,sizeof(New_Auto_Aim_Send));
	Append_CRC16_Check_Sum(&data[0],sizeof(New_Auto_Aim_Send)+3);
    //data[52]=0xED;
    Uart4SendBytesInfoProc(data,sizeof(New_Auto_Aim_Send)+3);
    
    
//    New_Auto_Aim_Send.header=0xBE;
//    memcpy(&data[0],&New_Auto_Aim_Send,sizeof(New_Auto_Aim_Send));
//	Append_CRC16_Check_Sum(&data[0],sizeof(New_Auto_Aim_Send)+3);
//    Uart4SendBytesInfoProc(data,sizeof(New_Auto_Aim_Send)+3);
}

