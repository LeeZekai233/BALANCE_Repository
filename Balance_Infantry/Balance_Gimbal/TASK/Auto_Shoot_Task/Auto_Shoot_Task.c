#include "main.h"

/******************************************auto_shoot_define***************************************/

Auto_Shoot_t My_Auto_Shoot;

New_Auto_Aim_t New_Auto_Aim;

New_Auto_Aim_Send_t New_Auto_Aim_Send;

New_Auto_Aim_t New_Auto_Aim_Medium;
/**********************************************auto_shoot_handle*****************************************/
u16 AUTO_CRC;

void Vision_Process_General_Message_New(unsigned char* address, unsigned int length, Auto_Shoot_t *Auto_Shoot)
{
    Auto_Shoot->heart_cnt = time_tick ;
	
	memcpy(&New_Auto_Aim_Medium,&address[0],sizeof(New_Auto_Aim_t));
	
	if(New_Auto_Aim_Medium.Header!=0xbe)
	 return;
	AUTO_CRC=Verify_CRC16_Check_Sum(address,length-1);		
	if(!AUTO_CRC)				//crc校验
		return;
	
	memcpy(&New_Auto_Aim,&address[0],sizeof(New_Auto_Aim_t));
	
	/**************************↓自瞄模式下的位置识别↓***************************/
	float Auto_Aim_Yaw_Angle_Medium=New_Auto_Aim.Yaw_Angle;
	float Auto_Aim_Pitch_Angle_Medium=New_Auto_Aim.Pitch_Angle;
	//单片机没法存储bull型变量，当视觉数据为bull型变量时，不进行数据处理
	if((Auto_Aim_Pitch_Angle_Medium==New_Auto_Aim.Pitch_Angle&&Auto_Aim_Yaw_Angle_Medium==New_Auto_Aim.Yaw_Angle)
		&&!(Gimbal.Gimbal_Mode==GIMBAL_BIG_BUFF||Gimbal.Gimbal_Mode==GIMBAL_SMALL_BUFF||Gimbal.Gimbal_Mode==GIMBAL_AUTO_BIG_BUFF||Gimbal.Gimbal_Mode==GIMBAL_AUTO_SMALL_BUFF))
	{
		//用于检测视觉是否接收到串口数据
		Auto_Shoot->Auto_Aim.Link_State=New_Auto_Aim.Link_State;
		if(!(Auto_Aim_Pitch_Angle_Medium==0&&Auto_Aim_Pitch_Angle_Medium==0))
		{
			Auto_Shoot->Auto_Aim.Yaw_Angle_Last = Auto_Shoot->Auto_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle_Last = Auto_Shoot->Auto_Aim.Pitch_Angle;
			Auto_Shoot->Auto_Aim.Yaw_Angle = New_Auto_Aim.Yaw_Angle;
			Auto_Shoot->Auto_Aim.Pitch_Angle = New_Auto_Aim.Pitch_Angle;
			
			
			if(Auto_Shoot->Auto_Aim.Pitch_Angle<36&&
				Auto_Shoot->Auto_Aim.Pitch_Angle>-16)		//到云台范围内
			{
				Auto_Shoot->Auto_Aim.Enable_Shoot=New_Auto_Aim.Enable_Shoot;
				Auto_Shoot->Auto_Aim.Flag_Get_Target=1;
			}
			else
			{
				Auto_Shoot->Auto_Aim.Yaw_Angle=Auto_Shoot->Auto_Aim.Yaw_Angle_Last;
				Auto_Shoot->Auto_Aim.Pitch_Angle=Auto_Shoot->Auto_Aim.Pitch_Angle_Last;
				Auto_Shoot->Auto_Aim.Enable_Shoot=0;
				Auto_Shoot->Auto_Aim.Flag_Get_Target=0;
			}
			
			
			Auto_Shoot->Auto_Aim.Lost_Cnt=0;
		}
		
		else
		{
			if(New_Auto_Aim.Link_State == 2)//防止出现同时为0
			{
				Auto_Shoot->Auto_Aim.Yaw_Angle_Last = Auto_Shoot->Auto_Aim.Yaw_Angle;
				Auto_Shoot->Auto_Aim.Pitch_Angle_Last = Auto_Shoot->Auto_Aim.Pitch_Angle;
				Auto_Shoot->Auto_Aim.Yaw_Angle = New_Auto_Aim.Yaw_Angle;
				Auto_Shoot->Auto_Aim.Pitch_Angle = New_Auto_Aim.Pitch_Angle;
				
				
				if(Auto_Shoot->Auto_Aim.Pitch_Angle<36&&
					Auto_Shoot->Auto_Aim.Pitch_Angle>-16)		//到云台范围内
				{
					Auto_Shoot->Auto_Aim.Enable_Shoot=New_Auto_Aim.Enable_Shoot;
					Auto_Shoot->Auto_Aim.Flag_Get_Target=1;
				}
				else
				{
					Auto_Shoot->Auto_Aim.Yaw_Angle=Auto_Shoot->Auto_Aim.Yaw_Angle_Last;
					Auto_Shoot->Auto_Aim.Pitch_Angle=Auto_Shoot->Auto_Aim.Pitch_Angle_Last;
					Auto_Shoot->Auto_Aim.Enable_Shoot=0;
					Auto_Shoot->Auto_Aim.Flag_Get_Target=0;
				}
			
			
				Auto_Shoot->Auto_Aim.Lost_Cnt=0;
			}

			else
			{
				Auto_Shoot->Auto_Aim.Flag_Get_Target = 0;
				Auto_Shoot->Auto_Aim.Yaw_Angle = 0;
				Auto_Shoot->Auto_Aim.Pitch_Angle = 0;
				Auto_Shoot->Auto_Aim.Enable_Shoot=0;
			}
				
		}
	}
	/**************************↑自瞄模式下的位置识别↑***************************/
	
	
	/**************************↓符模式下的位置识别↓***************************/
		float Buff_X_Yaw_Angle_Medium = New_Auto_Aim.Yaw_Angle;
		float Buff_Y_Pitch_Angle_Medium = New_Auto_Aim.Pitch_Angle;
		
		
		if(Buff_X_Yaw_Angle_Medium==New_Auto_Aim.Yaw_Angle&&Buff_Y_Pitch_Angle_Medium==New_Auto_Aim.Pitch_Angle)
		{
			if(!(Buff_X_Yaw_Angle_Medium==0&&Buff_Y_Pitch_Angle_Medium==0)&&(Gimbal.Gimbal_Mode==GIMBAL_BIG_BUFF||Gimbal.Gimbal_Mode==GIMBAL_SMALL_BUFF||Gimbal.Gimbal_Mode==GIMBAL_AUTO_BIG_BUFF||Gimbal.Gimbal_Mode==GIMBAL_AUTO_SMALL_BUFF))
			{
				Auto_Shoot->Buff.Lost_Cnt=0;
				
				
				Auto_Shoot->Buff.Yaw_Angle_Last = Auto_Shoot->Auto_Aim.Yaw_Angle;
				Auto_Shoot->Buff.Pitch_Angle_Last = Auto_Shoot->Auto_Aim.Pitch_Angle;
				Auto_Shoot->Buff.Yaw_Angle = New_Auto_Aim.Yaw_Angle;
				Auto_Shoot->Buff.Pitch_Angle = New_Auto_Aim.Pitch_Angle;
				My_Auto_Shoot.Buff.Last_Shoot_flag = My_Auto_Shoot.Buff.Shoot_flag;
				My_Auto_Shoot.Buff.Shoot_flag = New_Auto_Aim.Enable_Shoot;
                My_Auto_Shoot.Buff.Buff_Shoot_Delay = New_Auto_Aim.Buff_Shoot_Delay ;


				
				if(Auto_Shoot->Buff.Pitch_Angle<36&&
					Auto_Shoot->Buff.Pitch_Angle>-16)		//到云台范围内
				{
					My_Auto_Shoot.Buff.Enable_Shoot=New_Auto_Aim.Enable_Shoot;
					Auto_Shoot->Buff.Flag_Get_Target=1;
				}
				else
				{
					Auto_Shoot->Buff.Enable_Shoot=0;
					Auto_Shoot->Buff.Flag_Get_Target=0;
					
				}
			}
			else
			{
				if(Auto_Shoot->Buff.Lost_Cnt<20)
				{
					Auto_Shoot->Buff.Lost_Cnt++;
					
				}
				else
				{
					Auto_Shoot->Buff.Flag_Get_Target=0;
					My_Auto_Shoot.Buff.Yaw_Angle =0;
					My_Auto_Shoot.Buff.Pitch_Angle = 0;
					My_Auto_Shoot.Buff.Enable_Shoot=0;
					My_Auto_Shoot.Buff.Last_Shoot_flag=My_Auto_Shoot.Buff.Shoot_flag;
				}
			}
		}
	/**************************↑符模式下的位置识别↑***************************/
	
	
}



void send_protocol_New(float Yaw, float Pitch, float Roll,float Speed,u8 ID, u8* data)
{
	New_Auto_Aim_Send.Pitch=Pitch;
	New_Auto_Aim_Send.Roll=Roll;
	New_Auto_Aim_Send.Yaw=Yaw;
	
	if(New_Auto_Aim_Send.Yaw>180)New_Auto_Aim_Send.Yaw-=360;
	if(New_Auto_Aim_Send.Yaw<-180)New_Auto_Aim_Send.Yaw+=360;
	
	

	New_Auto_Aim_Send.Shoot_Speed = Shooter.Bullet_Speed;
	if(ID>100)
		New_Auto_Aim_Send.Current_Color=1;
	else
		New_Auto_Aim_Send.Current_Color=0;
	
	
	if(Gimbal.Gimbal_Mode==GIMBAL_BIG_BUFF || Gimbal.Gimbal_Mode==GIMBAL_AUTO_BIG_BUFF)
		New_Auto_Aim_Send.Mode=2;
	else if(Gimbal.Gimbal_Mode==GIMBAL_SMALL_BUFF || Gimbal.Gimbal_Mode==GIMBAL_AUTO_SMALL_BUFF)
		New_Auto_Aim_Send.Mode=1;
	else
		New_Auto_Aim_Send.Mode=0;
	
		New_Auto_Aim_Send.Game_State=USART_Gimbal_Data.game_state;
	
		New_Auto_Aim_Send.Poke_State=Shooter.Poke_State;
	
		New_Auto_Aim_Send.Shoot_Frequency=Shooter.Shoot_Frequency;
	

	data[0]=0xbe;
	memcpy(&data[1],&New_Auto_Aim_Send,sizeof(New_Auto_Aim_Send));
	Append_CRC16_Check_Sum(&data[0],sizeof(New_Auto_Aim_Send)+3);
	data[52]=0xed;
    
    
    DMA_Cmd(DMA1_Stream6, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}

    DMA1->LIFCR = DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6;

    DMA_SetCurrDataCounter(DMA1_Stream6, sizeof(New_Auto_Aim_Send) + 3);
    DMA_Cmd(DMA1_Stream6, ENABLE);


}



void Auto_Shoot_Online_Detect(Auto_Shoot_t* Auto_Shoot)
{
    if(time_tick - Auto_Shoot->heart_cnt > 500)
    {
        Auto_Shoot->Online_Flag = 0;
    }
    else
    {
        Auto_Shoot->Online_Flag = 1;
    }
}

