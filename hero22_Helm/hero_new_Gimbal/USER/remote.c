#include "public.h"
RC_Ctl_t RC_CtrlData;
extern chassis_t chassis;
extern _42mm_shoot_t _42mm_shoot;
uint8_t remote_lost_flag=0;
uint32_t follow_cnt;

uint8_t Auto_Shoot_Fire;

S1_Position_e S1_Positon;

void RemoteDataPrcess(uint8_t *pData)
{
	u8 static presss_l_last;
	if (pData == NULL)
	{
		remote_lost_flag = 1;
		return;
	}
    
    presss_l_last = RC_CtrlData.mouse.press_l;
	RC_CtrlData.mouse.last_press_l = presss_l_last;
    
	// 遥控器部分
	remote_lost_flag = 0;	//信号正常接收，丢失标志位清除
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
						  ((int16_t)pData[4] << 10)) &
						 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;
	RC_CtrlData.rc.ch4 = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003); // 模式切换
    
    RC_CtrlData.mouse.x=((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y=((int16_t)pData[8] | ((int16_t)pData[9] << 8));
    RC_CtrlData.mouse.z=((int16_t)pData[10] | ((int16_t)pData[11] << 8));
    RC_CtrlData.mouse.press_l=pData[12];
    RC_CtrlData.mouse.press_r=pData[13];
    RC_CtrlData.key.v=((int16_t)pData[14] | ((int16_t)pData[15] << 8));
     
    GetRemoteSwitchAction(&RC_CtrlData);
    keyborad_process(&RC_CtrlData);
    
    
}

void SetInputMode(RC_Ctl_t *remote)
{
    if(remote->rc.s2==1)
    {
        remote->inputmode=REMOTE_INPUT;
    }
    else if(remote->rc.s2==3)
    {
        remote->inputmode=KEY_MOUSE_INPUT;
    }
    else if(remote->rc.s2==2)
    {
        remote->inputmode=STOP;
    }
}

uint8_t test_flag;
uint8_t trigger1_flag, trigger2_flag;
uint16_t trigger1_cnt, trigger2_cnt;

void GetRemoteSwitchAction(RC_Ctl_t *remote)
{
    u8 static trigger_last;
    remote->RemoteSwitch.last_switch_value = remote->RemoteSwitch.switch_value_raw;
    remote->RemoteSwitch.switch_value_raw = remote->rc.s1;
	remote->RemoteSwitch.switch_value = remote->RemoteSwitch.last_switch_value - remote->RemoteSwitch.switch_value_raw;
	
	if(remote->RemoteSwitch.switch_value == 1)
	{
		remote->RemoteSwitch.s3to2 = 1;
		
	}
	if(remote->RemoteSwitch.switch_value == 2)
	{
		remote->RemoteSwitch.s3to1 = 1;
	}
    
    if((remote->rc.ch4 - 1024) < 200 && (remote->rc.ch4 - 1024) > -200)
	{
		remote->RemoteSwitch.trigger = 0;

	}
	else if( (remote->rc.ch4 - 1024) > 200)
	{
		remote->RemoteSwitch.trigger = 2;

	}
	else if(remote->rc.ch4 - 1024 < -200)
	{
		
		remote->RemoteSwitch.trigger = 1;
	}
	
	if(remote->RemoteSwitch.trigger_last - remote->RemoteSwitch.trigger == -1)
	{
		trigger1_flag = 1;
	}
	else if(remote->RemoteSwitch.trigger_last - remote->RemoteSwitch.trigger == -2)
	{
		trigger2_cnt++;
		if(trigger2_cnt % 2 == 0)
			trigger2_flag = 0;
		else
			trigger2_flag = 1;
	}
	else
	{
		if(trigger1_cnt<100&&trigger1_flag==1)
			trigger1_cnt++;
		else
		{
			trigger1_flag = 0;
			trigger1_cnt=0;
		}
	}

	
	trigger_last = remote->RemoteSwitch.trigger;
	remote->RemoteSwitch.trigger_last = trigger_last;
    
    
    
}


u8 remote_flagW1, remote_flagW2 = 0;
u8 remote_flagA1, remote_flagA2 = 0;
u8 remote_flagS1, remote_flagS2 = 0;
u8 remote_flagD1, remote_flagD2 = 0;
u8 remote_flagSHIFT1, remote_flagSHIFT2 = 0;
u8 remote_flagCTRL1, remote_flagCTRL2 = 0;
u8 remote_flagQ1, remote_flagQ2 = 0;
u8 remote_flagE1, remote_flagE2 = 0;
u8 remote_flagR1, remote_flagR2 = 0;
u8 remote_flagF1, remote_flagF2 = 0;
u8 remote_flagG1, remote_flagG2 = 0;
u8 remote_flagZ1, remote_flagZ2 = 0;
u8 remote_flagX1, remote_flagX2 = 0;
u8 remote_flagC1, remote_flagC2 = 0;
u8 remote_flagV1, remote_flagV2 = 0;
u8 remote_flagB1, remote_flagB2 = 0;

void updateKeyFlag(uint16_t key,RC_Ctl_t *remote, uint8_t *flag) 
{
    *flag = (remote->key.v & key) ? 1 : 0;
}

void keyborad_process(RC_Ctl_t *remote) 
{

    updateKeyFlag(KEY_W, remote,&(remote->Key_Flag.Key_W_Flag));
    updateKeyFlag(KEY_A, remote,&(remote->Key_Flag.Key_A_Flag));
    updateKeyFlag(KEY_S, remote,&(remote->Key_Flag.Key_S_Flag));
    updateKeyFlag(KEY_D, remote,&(remote->Key_Flag.Key_D_Flag));
    updateKeyFlag(KEY_SHIFT, remote,&(remote->Key_Flag.Key_SHIFT_Flag));
    updateKeyFlag(KEY_CTRL, remote,&(remote->Key_Flag.Key_CTRL_Flag));
    updateKeyFlag(KEY_Q, remote,&(remote->Key_Flag.Key_Q_Flag));
    updateKeyFlag(KEY_E, remote,&(remote->Key_Flag.Key_E_Flag));
    updateKeyFlag(KEY_R, remote,&(remote->Key_Flag.Key_R_Flag));
    updateKeyFlag(KEY_F, remote,&(remote->Key_Flag.Key_F_Flag));
    updateKeyFlag(KEY_G, remote,&(remote->Key_Flag.Key_G_Flag));
    updateKeyFlag(KEY_Z, remote,&(remote->Key_Flag.Key_Z_Flag));
    updateKeyFlag(KEY_X, remote,&(remote->Key_Flag.Key_X_Flag));
    updateKeyFlag(KEY_C, remote,&(remote->Key_Flag.Key_C_Flag));
    updateKeyFlag(KEY_V, remote,&(remote->Key_Flag.Key_V_Flag));
    updateKeyFlag(KEY_B, remote,&(remote->Key_Flag.Key_B_Flag));

	remote->Key_Flag.Key_W_TFlag = T_Key_procces(remote->Key_Flag.Key_W_Flag,&remote_flagW1,&remote_flagW2);
	remote->Key_Flag.Key_A_TFlag = T_Key_procces(remote->Key_Flag.Key_A_Flag,&remote_flagA1,&remote_flagA2);
	remote->Key_Flag.Key_S_TFlag = T_Key_procces(remote->Key_Flag.Key_S_Flag,&remote_flagS1,&remote_flagS2);
	remote->Key_Flag.Key_D_TFlag = T_Key_procces(remote->Key_Flag.Key_D_Flag,&remote_flagD1,&remote_flagD2);
	remote->Key_Flag.Key_SHIFT_TFlag = T_Key_procces(remote->Key_Flag.Key_SHIFT_Flag,&remote_flagSHIFT1,&remote_flagSHIFT2);
	remote->Key_Flag.Key_CTRL_TFlag = T_Key_procces(remote->Key_Flag.Key_CTRL_Flag,&remote_flagCTRL1,&remote_flagCTRL2);
	remote->Key_Flag.Key_Q_TFlag = T_Key_procces(remote->Key_Flag.Key_Q_Flag,&remote_flagQ1,&remote_flagQ2);
	remote->Key_Flag.Key_E_TFlag = T_Key_procces(remote->Key_Flag.Key_E_Flag,&remote_flagE1,&remote_flagE2);
	remote->Key_Flag.Key_R_TFlag = T_Key_procces(remote->Key_Flag.Key_R_Flag,&remote_flagR1,&remote_flagR2);
	remote->Key_Flag.Key_F_TFlag = T_Key_procces(remote->Key_Flag.Key_F_Flag,&remote_flagF1,&remote_flagF2);
	remote->Key_Flag.Key_G_TFlag = T_Key_procces(remote->Key_Flag.Key_G_Flag,&remote_flagG1,&remote_flagG2);
	remote->Key_Flag.Key_Z_TFlag = T_Key_procces(remote->Key_Flag.Key_Z_Flag,&remote_flagZ1,&remote_flagZ2);
	remote->Key_Flag.Key_X_TFlag = T_Key_procces(remote->Key_Flag.Key_X_Flag,&remote_flagX1,&remote_flagX2);
	remote->Key_Flag.Key_C_TFlag = T_Key_procces(remote->Key_Flag.Key_C_Flag,&remote_flagC1,&remote_flagC2);
	remote->Key_Flag.Key_V_TFlag = T_Key_procces(remote->Key_Flag.Key_V_Flag,&remote_flagV1,&remote_flagV2);
	remote->Key_Flag.Key_B_TFlag = T_Key_procces(remote->Key_Flag.Key_B_Flag,&remote_flagB1,&remote_flagB2);


}

uint8_t T_Key_procces(u8 flag,u8 *a,u8 *i)
{
	
	if (flag)
	{
	  if ( *a == 0)
	  {
		  *a = 1;
		  *i = *i+1;
	  }
	}
	else
	{
	  *a = 0;
	}
	if(*i%2==0)
	{
		return 0;
	}else
	{
		return 1;
	}
}

int snipe_cnt,manual_snipe_cnt,follow_cnt1;
uint8_t init_cnt;
float aaa=0;
void Remote_Input(void)
{
    if((gimbal_data.ctrl_mode==GIMBAL_FOLLOW_ZGYRO && chassis.ctrl_mode==AUTO_FOLLOW_GIMBAL)||
		(gimbal_data.ctrl_mode==GIMBAL_FOLLOW_ZGYRO && chassis.ctrl_mode==MANUAL_SEPARATE_GIMBAL)||
		gimbal_data.ctrl_mode==GIMBAL_FOLLOW_ZGYRO && (chassis.ctrl_mode==CHASSIS_ROTATE||chassis.ctrl_mode==CHASSIS_REVERSE_ROTATE))
    {
        follow_cnt1++;
		
		gimbal_data.gim_ref_and_fdb.yaw_angle_ref -= ( RC_CtrlData.rc.ch2 - 1024 ) * GIMBAL_NORMAL_YAW_SENSITIVITY;
		gimbal_data.gim_ref_and_fdb.pit_angle_ref += ( RC_CtrlData.rc.ch3 - 1024 ) * GIMBAL_NORMAL_PITCH_SENSITIVITY;
        
        //实际为用遥控器输入Vx,Vy值
        chassis.ChassisSpeed_Ref.forward_back_ref = ( RC_CtrlData.rc.ch1 - 1024 ) * X_CHASSIS_AUTO_FOLLOW_GIMBAL_SENSITIVITY;//1684-1024=660 * 0.002对速度映射到0-1.32m/s
		chassis.ChassisSpeed_Ref.left_right_ref = ( RC_CtrlData.rc.ch0 - 1024 ) * Y_CHASSIS_AUTO_FOLLOW_GIMBAL_SENSITIVITY;
        //chassis.ChassisSpeed_Ref.rotate_ref=(RC_CtrlData.rc.ch4-1024)*W_CHASSIS_AUTO_FOLLOW_GIMBAL_SENSITIVITY;
    }
    
    
    if(gimbal_data.ctrl_mode==GIMBAL_SNIPE || gimbal_data.ctrl_mode==GIMBAL_RADAR_ASSISTANT_SNIPE)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref -= ( RC_CtrlData.rc.ch2 - 1024 ) * GIMBAL_SNIPE_YAW_SENSITIVITY;//0.00004
		gimbal_data.gim_ref_and_fdb.pit_angle_ref += ( RC_CtrlData.rc.ch3 - 1024 ) * GIMBAL_SNIPE_PITCH_SENSITIVITY;//0.00004
        
//        gimbal_data.gim_ref_and_fdb.yaw_angle_ref -= 0;
//		gimbal_data.gim_ref_and_fdb.pit_angle_ref += 0;
    }
    

}

















