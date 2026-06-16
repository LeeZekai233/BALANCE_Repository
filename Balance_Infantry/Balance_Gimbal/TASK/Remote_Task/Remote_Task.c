#include "main.h"


Remote_DT7_t Remote_DT7_data={0};
//Control_Mode_e Control_Mode;
Remote_VTM_t  Remote_VTM={0};

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
int32_t verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
//        return false;
		  return 0;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}


/**
  *@Brief： DT7遥控器数据处理函数
  *@Cal：   内部或外部  
  *@param:  遥控的结构体
  *@Note:   放在对应串口空闲中断中确保信息的实时性
  *@RetVal: 无
  */

void DT7_Remote_Data_Dispose(uint8_t* RemoteData,Remote_DT7_t* Remote_data)
{
    if(RemoteData == NULL)
    {
      return;
    }
	Remote_data->Remote_clicker.ch0 = ((int16_t)RemoteData[0] | ((int16_t)RemoteData[1] << 8)) & 0x07FF;
	Remote_data->Remote_clicker.ch1 = (((int16_t)RemoteData[1] >> 3) | ((int16_t)RemoteData[2] << 5)) & 0x07FF;
	Remote_data->Remote_clicker.ch2 = (((int16_t)RemoteData[2] >> 6) | ((int16_t)RemoteData[3] << 2) |((int16_t)RemoteData[4] << 10)) & 0x07FF;
	Remote_data->Remote_clicker.ch3 = (((int16_t)RemoteData[4] >> 1) | ((int16_t)RemoteData[5]<<7)) & 0x07FF;
	Remote_data->Remote_clicker.ch4 = ((int16_t)RemoteData[16]>>0 | ((int16_t)RemoteData[17] << 8)) & 0x07FF;
	Remote_data->Remote_clicker.s1 = ((RemoteData[5] >> 4) & 0x000C) >> 2;
	Remote_data->Remote_clicker.s2 = ((RemoteData[5] >> 4) & 0x0003);//模式切换
		
	Remote_data->Remote_mouse.x = ((int16_t)RemoteData[6]) | ((int16_t)RemoteData[7] << 8);
	Remote_data->Remote_mouse.y = ((int16_t)RemoteData[8]) | ((int16_t)RemoteData[9] << 8);
	Remote_data->Remote_mouse.z = ((int16_t)RemoteData[10]) | ((int16_t)RemoteData[11] << 8);
	Remote_data->Remote_mouse.Press_L_Action.Original_Press_Flag = RemoteData[12];
	Remote_data->Remote_mouse.Press_R_Action.Original_Press_Flag = RemoteData[13];
    
	Remote_data->key.Key_W_Action.Original_Press_Flag = (RemoteData[14]) & 0x01;
    Remote_data->key.Key_S_Action.Original_Press_Flag = (RemoteData[14]>>1) & 0x01;
    Remote_data->key.Key_A_Action.Original_Press_Flag = (RemoteData[14]>>2) & 0x01;
    Remote_data->key.Key_D_Action.Original_Press_Flag = (RemoteData[14]>>3) & 0x01;
    Remote_data->key.Key_SHIFT_Action.Original_Press_Flag = (RemoteData[14]>>4) & 0x01;
    Remote_data->key.Key_CTRL_Action.Original_Press_Flag = (RemoteData[14]>>5) & 0x01;
    Remote_data->key.Key_Q_Action.Original_Press_Flag = (RemoteData[14]>>6) & 0x01;
    Remote_data->key.Key_E_Action.Original_Press_Flag = (RemoteData[14]>>7) & 0x01;
    
    Remote_data->key.Key_R_Action.Original_Press_Flag = (RemoteData[15]) & 0x01;//手册疏于维护，实际以代码为准
    Remote_data->key.Key_F_Action.Original_Press_Flag = (RemoteData[15]>>1) & 0x01;
    Remote_data->key.Key_G_Action.Original_Press_Flag = (RemoteData[15]>>2) & 0x01;
    Remote_data->key.Key_Z_Action.Original_Press_Flag = (RemoteData[15]>>3) & 0x01;
    Remote_data->key.Key_X_Action.Original_Press_Flag = (RemoteData[15]>>4) & 0x01;
    Remote_data->key.Key_C_Action.Original_Press_Flag = (RemoteData[15]>>5) & 0x01;
    Remote_data->key.Key_V_Action.Original_Press_Flag = (RemoteData[15]>>6) & 0x01;
    Remote_data->key.Key_B_Action.Original_Press_Flag = (RemoteData[15]>>7) & 0x01;
    
    
    Remote_data->Remote_clicker.ch0-=RC_CH_VALUE_OFFSET;//减去偏移，归化到-660——660
    Remote_data->Remote_clicker.ch1-=RC_CH_VALUE_OFFSET;
    Remote_data->Remote_clicker.ch2-=RC_CH_VALUE_OFFSET;
    Remote_data->Remote_clicker.ch3-=RC_CH_VALUE_OFFSET;
    Remote_data->Remote_clicker.ch4-=RC_CH_VALUE_OFFSET;
    
    Remote_data->heart_cnt = time_tick;
}




remote_data_t vtm_remote_data;
u16 Remote_CRC16;
u16 CRC_Check;
void VTM_Reomte_Data_Handle(uint8_t *pData,u16 rec_len,Remote_VTM_t* Remote_VTM)
{
	u16 deal_cnt = 0;
	u16 Frame_length = 0;
	
	Frame_length = 21;		//一帧数据总长度为21
	
//	while(rec_len > deal_cnt)		//这个while好像没啥实际用处
	{
		vtm_remote_data.sof_1 = pData[deal_cnt];
		vtm_remote_data.sof_2 = pData[1];
		
		CRC_Check= verify_crc16_check_sum(pData,Frame_length);
		Remote_CRC16=Get_CRC16_Check_Sum(pData,Frame_length-2,crc16_init);
		
		if(vtm_remote_data.sof_1==0xa9&&vtm_remote_data.sof_2==0x53)
		{
			if(verify_crc16_check_sum(&pData[deal_cnt],Frame_length))
			{

				memcpy(&vtm_remote_data,pData,sizeof(remote_data_t));
				
				Remote_VTM->Remote_clicker.ch0 = vtm_remote_data.ch_0;
				Remote_VTM->Remote_clicker.ch1 = vtm_remote_data.ch_1;
				Remote_VTM->Remote_clicker.ch2 = vtm_remote_data.ch_3;
				Remote_VTM->Remote_clicker.ch3 = vtm_remote_data.ch_2; 
				Remote_VTM->Remote_clicker.ch4 = vtm_remote_data.wheel;
				Remote_VTM->Remote_clicker.Pause_Action.Original_Press_Flag = vtm_remote_data.pause;
                Remote_VTM->Remote_clicker.Switch = vtm_remote_data.mode_sw;        
                Remote_VTM->Remote_clicker.fn1_Action.Original_Press_Flag = vtm_remote_data.fn_1;
                Remote_VTM->Remote_clicker.fn2_Action.Original_Press_Flag = vtm_remote_data.fn_2;
                Remote_VTM->Remote_clicker.Trigger_Action.Original_Press_Flag = vtm_remote_data.trigger;
                
				Remote_VTM->Remote_mouse.x = vtm_remote_data.mouse_x;
				Remote_VTM->Remote_mouse.y = vtm_remote_data.mouse_y;
				Remote_VTM->Remote_mouse.z = vtm_remote_data.mouse_z;
				Remote_VTM->Remote_mouse.Press_L_Action.Original_Press_Flag = vtm_remote_data.mouse_left;
				Remote_VTM->Remote_mouse.Press_R_Action.Original_Press_Flag = vtm_remote_data.mouse_right;
		
				Remote_VTM->key.v = (pData[18]<<8) | pData[17];
				
                Remote_VTM->Remote_clicker.ch0 -= RC_CH_VALUE_OFFSET;
                Remote_VTM->Remote_clicker.ch1 -= RC_CH_VALUE_OFFSET;
                Remote_VTM->Remote_clicker.ch2 -= RC_CH_VALUE_OFFSET;
                Remote_VTM->Remote_clicker.ch3 -= RC_CH_VALUE_OFFSET;
                Remote_VTM->Remote_clicker.ch4 -= RC_CH_VALUE_OFFSET;
                
                Remote_VTM->key.Key_W_Action.Original_Press_Flag = (Remote_VTM->key.v)&0x0001;
                Remote_VTM->key.Key_S_Action.Original_Press_Flag = (Remote_VTM->key.v>>1)&0x0001;
                Remote_VTM->key.Key_A_Action.Original_Press_Flag = (Remote_VTM->key.v>>2)&0x0001;
                Remote_VTM->key.Key_D_Action.Original_Press_Flag = (Remote_VTM->key.v>>3)&0x0001;
                Remote_VTM->key.Key_SHIFT_Action.Original_Press_Flag = (Remote_VTM->key.v>>4)&0x0001;
                Remote_VTM->key.Key_CTRL_Action.Original_Press_Flag = (Remote_VTM->key.v>>5)&0x0001;
                Remote_VTM->key.Key_Q_Action.Original_Press_Flag = (Remote_VTM->key.v>>6)&0x0001;
                Remote_VTM->key.Key_E_Action.Original_Press_Flag = (Remote_VTM->key.v>>7)&0x0001;
                Remote_VTM->key.Key_R_Action.Original_Press_Flag = (Remote_VTM->key.v>>8)&0x0001;
                Remote_VTM->key.Key_F_Action.Original_Press_Flag = (Remote_VTM->key.v>>9)&0x0001;
                Remote_VTM->key.Key_G_Action.Original_Press_Flag = (Remote_VTM->key.v>>10)&0x0001;
                Remote_VTM->key.Key_Z_Action.Original_Press_Flag = (Remote_VTM->key.v>>11)&0x0001;
                Remote_VTM->key.Key_X_Action.Original_Press_Flag = (Remote_VTM->key.v>>12)&0x0001;
                Remote_VTM->key.Key_C_Action.Original_Press_Flag = (Remote_VTM->key.v>>13)&0x0001;
                Remote_VTM->key.Key_V_Action.Original_Press_Flag = (Remote_VTM->key.v>>14)&0x0001;
                Remote_VTM->key.Key_B_Action.Original_Press_Flag = (Remote_VTM->key.v>>15)&0x0001;
                Remote_VTM->heart_cnt = time_tick;
			}
		}
	}
}


/**
  * @brief  按键状态检测函数，用于判断按键状态
  * @param  Key_Mouse_Action_t 按键结构体
  * @retval 无返回值
  * @note   无
  */
void Key_Mouse_Action_Detect(Key_Mouse_Action_t* Key_Mouse_Action)
{
    uint8_t diff;//检测上升沿
    diff=Key_Mouse_Action->Original_Press_Flag - Key_Mouse_Action->Last_Original_Press_Flag ;
    Key_Mouse_Action->Last_Original_Press_Flag = Key_Mouse_Action->Original_Press_Flag ;
    if(Key_Mouse_Action->Original_Press_Flag == 1)                  //当按下时，开始计数
    {
        Key_Mouse_Action->Cnt++ ;
       // Key_Mouse_Action->Short_Press_Flag = 1;//这里短按逻辑有问题，应该达成按下一次只置一次1的效果，这里一直置1，会打出很多发弹
    }
    else
    {
        Key_Mouse_Action->Cnt = 0 ;                              //当松手时，清零长按计数和长按标志位
        Key_Mouse_Action->Long_Press_Flag = 0 ;
    }
    
    if(Key_Mouse_Action->Cnt >= LONG_PRESS_THRESHOLD)
    {
        Key_Mouse_Action->Cnt = LONG_PRESS_THRESHOLD;
        Key_Mouse_Action->Long_Press_Flag = 1 ;              //长按标志位置1时，短按标志位置0
        Key_Mouse_Action->Short_Press_Flag = 0;
    }
    
    
    if(diff == 1)                                          //当按下按键，检测到一次上升沿时，翻转一次标志位
    {
        Key_Mouse_Action->Short_Press_Flag = 1;
        
        if(Key_Mouse_Action->Toggle_Press_Flag == 0)
            Key_Mouse_Action->Toggle_Press_Flag = 1 ;
        else
            Key_Mouse_Action->Toggle_Press_Flag = 0 ;
    }
    else
    {
        Key_Mouse_Action->Short_Press_Flag = 0;             //检测不到上升沿时，短按标志清零
    }
    
    
}




/**
  * @brief  更新所有按键的长短按状态
  * @retval 无返回值
  * @note   无
  */
void Key_Mouse_State_Update(Key_t* Key, Mouse_t* Mouse)
{
    Key_Mouse_Action_Detect(&Mouse->Press_L_Action);//每个按键都调用一次检测函数
    Key_Mouse_Action_Detect(&Mouse->Press_R_Action);
    Key_Mouse_Action_Detect(&Mouse->Press_M_Action);
    
    Key_Mouse_Action_Detect(&Key->Key_W_Action);
    Key_Mouse_Action_Detect(&Key->Key_S_Action);
    Key_Mouse_Action_Detect(&Key->Key_A_Action);
    Key_Mouse_Action_Detect(&Key->Key_D_Action);
    Key_Mouse_Action_Detect(&Key->Key_SHIFT_Action);
    Key_Mouse_Action_Detect(&Key->Key_CTRL_Action);
    Key_Mouse_Action_Detect(&Key->Key_Q_Action);
    Key_Mouse_Action_Detect(&Key->Key_E_Action);
    
    Key_Mouse_Action_Detect(&Key->Key_R_Action);
    Key_Mouse_Action_Detect(&Key->Key_F_Action);
    Key_Mouse_Action_Detect(&Key->Key_G_Action);
    Key_Mouse_Action_Detect(&Key->Key_Z_Action);
    Key_Mouse_Action_Detect(&Key->Key_X_Action);
    Key_Mouse_Action_Detect(&Key->Key_C_Action);
    Key_Mouse_Action_Detect(&Key->Key_V_Action);
    Key_Mouse_Action_Detect(&Key->Key_B_Action);
}

void VTM_Clicker_State_Update(Remote_VTM_t* Remote_data)
{
    //按键状态更新
    Key_Mouse_Action_Detect(&Remote_data->Remote_clicker.fn1_Action);
    Key_Mouse_Action_Detect(&Remote_data->Remote_clicker.fn2_Action);
    Key_Mouse_Action_Detect(&Remote_data->Remote_clicker.Pause_Action);
    Key_Mouse_Action_Detect(&Remote_data->Remote_clicker.Trigger_Action);
    
    
     //拨轮向上或向下拨一次，再回正，视为一次动作，执行按键状态翻转
    if(Remote_data->Remote_clicker.ch4 == 660) Remote_data->Remote_clicker.ch4_Up = 1 ;
    else if(Remote_data->Remote_clicker.ch4 == -660) Remote_data->Remote_clicker.ch4_Down = 1 ;
    
    if(Remote_data->Remote_clicker.ch4 == 0 & Remote_data->Remote_clicker.ch4_Up == 1)
    {
        Remote_data->Remote_clicker.ch4_Up = 0;
        if(Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag == 1) Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag = 0;
        else Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag = 1;
    }
    
    if(Remote_data->Remote_clicker.ch4 == 0 & Remote_data->Remote_clicker.ch4_Down == 1)
    {
        Remote_data->Remote_clicker.ch4_Down = 0;
        if(Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag == 1) Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag = 0;
        else Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag = 1;
    }
    
    //向上或向下一直拨不回正，视为长按
    if(Remote_data->Remote_clicker.ch4 == 660) 
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Short_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt ++;
    }
    else
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Short_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt = 0;
    }
    
    if(Remote_data->Remote_clicker.ch4 == -660)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Short_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt ++;
    }
    else
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Short_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt = 0;
    }
    
    if(Remote_data->Remote_clicker.ch4_Down_Action.Cnt >= LONG_PRESS_THRESHOLD)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt = LONG_PRESS_THRESHOLD;//长按时置长按标志位，清零短按，翻转标志位
        Remote_data->Remote_clicker.ch4_Down_Action.Long_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Down = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Short_Press_Flag = 0;
        
    }
    else if(Remote_data->Remote_clicker.ch4_Up_Action.Cnt >=LONG_PRESS_THRESHOLD)
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt = LONG_PRESS_THRESHOLD;
        Remote_data->Remote_clicker.ch4_Up_Action.Long_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Up = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Short_Press_Flag = 0;
    }
    
    if(Remote_data->Remote_clicker.ch4 == 0)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Long_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Long_Press_Flag = 0;
    }
   
}


/**
  * @brief  选择控制模式
  * @param  Remote_DT7_t   控结构体
  * @param  Control_Mode_e 控制模式结构体
  * @retval 无返回值
  * @note   无
  */
//void Control_Mode_Select(Remote_DT7_t* Remote_data,Control_Mode_e* Control_Mode)
//{
//    switch(Remote_data->Remote_clicker.s1)
//    {
//        case UP:
//            *Control_Mode = KEY_MOUSE;
//            break;
//        case MIDDLE:
//            *Control_Mode = REMOTE;
//            break;
//        case DOWN:
//            *Control_Mode = RELAX;
//            break;
//        default :
//            *Control_Mode = RELAX;
//            break;
//    }
//}



/**
  * @brief  拨杆拨轮动作检测
  * @param  Remote_DT7_t   控结构体
  * @retval 无返回值
  * @note   无
  */
int8_t s2_Diff;
int8_t s1_Diff;
void Remote_Switch_Action_Detect(Remote_DT7_t* Remote_data)
{
    s2_Diff = Remote_data->Remote_clicker.s2 - Remote_data->Remote_clicker.s2_last;
    switch (s2_Diff)
    {
        case 0:                                                                     //根据手册，拨杆在中间为3，在上是1，在下是2，作差可判断拨杆动作
            Remote_data->Remote_clicker.s2_Action = KEEP;
            break;
        case 1:
            Remote_data->Remote_clicker.s2_Action = DOWN_TO_MIDDLE;
            break;
        case -2:
            Remote_data->Remote_clicker.s2_Action = MIDDLE_TO_UP;
            break;
        case 2:
            Remote_data->Remote_clicker.s2_Action = UP_TO_MIDDLE;
            break;
        case -1:
            Remote_data->Remote_clicker.s2_Action = MIDDLE_TO_DOWN;
            break;
        default :
            break;
    }
    
    
    s1_Diff = Remote_data->Remote_clicker.s1 - Remote_data->Remote_clicker.s1_last;
    switch (s1_Diff)
    {
        case 0:                                                                     //根据手册，拨杆在中间为3，在上是1，在下是2，作差可判断拨杆动作
            Remote_data->Remote_clicker.s1_Action = KEEP;
            break;
        case 1:
            Remote_data->Remote_clicker.s1_Action = DOWN_TO_MIDDLE;
            break;
        case -2:
            Remote_data->Remote_clicker.s1_Action = MIDDLE_TO_UP;
            break;
        case 2:
            Remote_data->Remote_clicker.s1_Action = UP_TO_MIDDLE;
            break;
        case -1:
            Remote_data->Remote_clicker.s1_Action = MIDDLE_TO_DOWN;
            break;
        default :
            break;
    }
    Remote_data->Remote_clicker.s2_last = Remote_data->Remote_clicker.s2  ;//保存本次数据作为下一次的上一次
    Remote_data->Remote_clicker.s1_last = Remote_data->Remote_clicker.s1  ;
    
    
    //拨轮向上或向下拨一次，再回正，视为一次动作，执行按键状态翻转
    if(Remote_data->Remote_clicker.ch4 == 660) Remote_data->Remote_clicker.ch4_Up = 1 ;
    else if(Remote_data->Remote_clicker.ch4 == -660) Remote_data->Remote_clicker.ch4_Down = 1 ;
    
    if(Remote_data->Remote_clicker.ch4 == 0 & Remote_data->Remote_clicker.ch4_Down == 1)
    {
        Remote_data->Remote_clicker.ch4_Down = 0;
        if(Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag == 1) Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag = 0;
        else Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag = 1;
    }
    
    if(Remote_data->Remote_clicker.ch4 == 0 & Remote_data->Remote_clicker.ch4_Up == 1)
    {
        Remote_data->Remote_clicker.ch4_Up = 0;
        if(Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag == 1) Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag = 0;
        else Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag = 1;
    }
    
    //向上或向下一直拨不回正，视为长按
    if(Remote_data->Remote_clicker.ch4 >= 650) 
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Original_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt ++;
    }
    else
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Original_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt = 0;
    }
    
    if(Remote_data->Remote_clicker.ch4 <= -650)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Original_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt ++;
    }
    else
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Original_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt = 0;
    }
    
    
    if(Remote_data->Remote_clicker.ch4_Down_Action.Original_Press_Flag == 1 && Remote_data->Remote_clicker.ch4_Down_Action.Last_Original_Press_Flag == 0)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Short_Press_Flag = 1;
    }
    else
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Short_Press_Flag = 0;
    }
    
    
    if(Remote_data->Remote_clicker.ch4_Up_Action.Original_Press_Flag == 1 && Remote_data->Remote_clicker.ch4_Up_Action.Last_Original_Press_Flag == 0)
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Short_Press_Flag = 1;
    }
    else
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Short_Press_Flag = 0;
    }
    
    
    if(Remote_data->Remote_clicker.ch4_Down_Action.Cnt >= LONG_PRESS_THRESHOLD)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt = LONG_PRESS_THRESHOLD;//长按时置长按标志位，清零短按，翻转标志位
        Remote_data->Remote_clicker.ch4_Down_Action.Long_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Down = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Toggle_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Short_Press_Flag = 0;
        
    }
    else if(Remote_data->Remote_clicker.ch4_Up_Action.Cnt >=LONG_PRESS_THRESHOLD)
    {
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt = LONG_PRESS_THRESHOLD;
        Remote_data->Remote_clicker.ch4_Up_Action.Long_Press_Flag = 1;
        Remote_data->Remote_clicker.ch4_Up = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Toggle_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Short_Press_Flag = 0;
    }
    
    if(Remote_data->Remote_clicker.ch4 == 0)
    {
        Remote_data->Remote_clicker.ch4_Down_Action.Cnt = 0;
        Remote_data->Remote_clicker.ch4_Down_Action.Long_Press_Flag = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Cnt = 0;
        Remote_data->Remote_clicker.ch4_Up_Action.Long_Press_Flag = 0;
    }
   
    Remote_data->Remote_clicker.ch4_Down_Action.Last_Original_Press_Flag = Remote_data->Remote_clicker.ch4_Down_Action.Original_Press_Flag ;
    Remote_data->Remote_clicker.ch4_Up_Action.Last_Original_Press_Flag = Remote_data->Remote_clicker.ch4_Up_Action.Original_Press_Flag ;
}



void Remote_Online_Detect(Remote_DT7_t* Remote_DT7,Remote_VTM_t* Remote_VTM)
{
    if(time_tick - Remote_DT7->heart_cnt >= 500)
    {
        Remote_DT7->online_flag = 0 ;
    }
    else
    {
        Remote_DT7->online_flag = 1 ;
    }
    
    if(time_tick - Remote_VTM->heart_cnt >= 500)
    {
        Remote_VTM->online_flag = 0;
    }
    else
    {
        Remote_VTM->online_flag = 1;
    }
}




