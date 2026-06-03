#include "main.h"

Remote_DT7_t Remote_DT7_data={0};
Control_Mode_e Control_Mode;


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
	Remote_data->Remote_mouse.Press_L_Action.Original_Short_Press_Flag = RemoteData[12];
	Remote_data->Remote_mouse.Press_R_Action.Original_Short_Press_Flag = RemoteData[13];
    
	Remote_data->key.Key_W_Action.Original_Short_Press_Flag = (RemoteData[14]) & 0x01;
    Remote_data->key.Key_S_Action.Original_Short_Press_Flag = (RemoteData[14]>>1) & 0x01;
    Remote_data->key.Key_A_Action.Original_Short_Press_Flag = (RemoteData[14]>>2) & 0x01;
    Remote_data->key.Key_D_Action.Original_Short_Press_Flag = (RemoteData[14]>>3) & 0x01;
    Remote_data->key.Key_SHIFT_Action.Original_Short_Press_Flag = (RemoteData[14]>>4) & 0x01;
    Remote_data->key.Key_CTRL_Action.Original_Short_Press_Flag = (RemoteData[14]>>5) & 0x01;
    Remote_data->key.Key_Q_Action.Original_Short_Press_Flag = (RemoteData[14]>>6) & 0x01;
    Remote_data->key.Key_E_Action.Original_Short_Press_Flag = (RemoteData[14]>>7) & 0x01;
    
    Remote_data->key.Key_R_Action.Original_Short_Press_Flag = (RemoteData[15]) & 0x01;//手册疏于维护，实际以代码为准
    Remote_data->key.Key_F_Action.Original_Short_Press_Flag = (RemoteData[15]>>1) & 0x01;
    Remote_data->key.Key_G_Action.Original_Short_Press_Flag = (RemoteData[15]>>2) & 0x01;
    Remote_data->key.Key_Z_Action.Original_Short_Press_Flag = (RemoteData[15]>>3) & 0x01;
    Remote_data->key.Key_X_Action.Original_Short_Press_Flag = (RemoteData[15]>>4) & 0x01;
    Remote_data->key.Key_C_Action.Original_Short_Press_Flag = (RemoteData[15]>>5) & 0x01;
    Remote_data->key.Key_V_Action.Original_Short_Press_Flag = (RemoteData[15]>>6) & 0x01;
    Remote_data->key.Key_B_Action.Original_Short_Press_Flag = (RemoteData[15]>>7) & 0x01;
    
    
    Remote_data->Remote_clicker.ch0-=RC_CH_VALUE_OFFSET;//减去偏移，归化到-660——660
    Remote_data->Remote_clicker.ch1-=RC_CH_VALUE_OFFSET;
    Remote_data->Remote_clicker.ch2-=RC_CH_VALUE_OFFSET;
    Remote_data->Remote_clicker.ch3-=RC_CH_VALUE_OFFSET;
    Remote_data->Remote_clicker.ch4-=RC_CH_VALUE_OFFSET;
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
    diff=Key_Mouse_Action->Original_Short_Press_Flag - Key_Mouse_Action->Last_Original_Short_Press_Flag ;
    Key_Mouse_Action->Last_Original_Short_Press_Flag = Key_Mouse_Action->Original_Short_Press_Flag ;
    if(Key_Mouse_Action->Original_Short_Press_Flag == 1)                  //当按下时，开始计数
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
}




/**
  * @brief  更新所有按键的长短按状态
  * @param  Remote_DT7_t 控结构体
  * @retval 无返回值
  * @note   无
  */
void Key_Mouse_State_Update(Remote_DT7_t* Remote_data)
{
    Key_Mouse_Action_Detect(&Remote_data->Remote_mouse.Press_L_Action);//每个按键都调用一次检测函数
    Key_Mouse_Action_Detect(&Remote_data->Remote_mouse.Press_R_Action);
    
    Key_Mouse_Action_Detect(&Remote_data->key.Key_W_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_S_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_A_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_D_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_SHIFT_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_CTRL_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_Q_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_E_Action);
    
    Key_Mouse_Action_Detect(&Remote_data->key.Key_R_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_F_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_G_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_Z_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_X_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_C_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_V_Action);
    Key_Mouse_Action_Detect(&Remote_data->key.Key_B_Action);
}




/**
  * @brief  选择控制模式
  * @param  Remote_DT7_t   控结构体
  * @param  Control_Mode_e 控制模式结构体
  * @retval 无返回值
  * @note   无
  */
void Control_Mode_Select(Remote_DT7_t* Remote_data,Control_Mode_e* Control_Mode)
{
    switch(Remote_data->Remote_clicker.s1)
    {
        case UP:
            *Control_Mode = KEY_MOUSE;
            break;
        case MIDDLE:
            *Control_Mode = REMOTE;
            break;
        case DOWN:
            *Control_Mode = RELAX;
            break;
        default :
            *Control_Mode = RELAX;
            break;
    }
}



/**
  * @brief  拨杆拨轮动作检测
  * @param  Remote_DT7_t   控结构体
  * @retval 无返回值
  * @note   无
  */
int8_t s2_Diff;
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
    
    
    //拨轮向上或向下拨一次，再回正，视为一次动作，执行按键状态翻转
    if(Remote_data->Remote_clicker.ch4 == 660) Remote_data->Remote_clicker.Trigger_Up = 1 ;
    else if(Remote_data->Remote_clicker.ch4 == -660) Remote_data->Remote_clicker.Trigger_Down = 1 ;
    
    if(Remote_data->Remote_clicker.ch4 == 0 & Remote_data->Remote_clicker.Trigger_Up == 1)
    {
        Remote_data->Remote_clicker.Trigger_Up = 0;
        if(Remote_data->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag == 1) Remote_data->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag = 0;
        else Remote_data->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag = 1;
    }
    
    if(Remote_data->Remote_clicker.ch4 == 0 & Remote_data->Remote_clicker.Trigger_Down == 1)
    {
        Remote_data->Remote_clicker.Trigger_Down = 0;
        if(Remote_data->Remote_clicker.Trigger_Down_Action.Toggle_Press_Flag == 1) Remote_data->Remote_clicker.Trigger_Down_Action.Toggle_Press_Flag = 0;
        else Remote_data->Remote_clicker.Trigger_Down_Action.Toggle_Press_Flag = 1;
    }
    
    //向上或向下一直拨不回正，视为长按
    if(Remote_data->Remote_clicker.ch4 == 660) 
    {
        Remote_data->Remote_clicker.Trigger_Up_Action.Short_Press_Flag = 1;
        Remote_data->Remote_clicker.Trigger_Up_Action.Cnt ++;
    }
    else if(Remote_data->Remote_clicker.ch4 == -660)
    {
        Remote_data->Remote_clicker.Trigger_Down_Action.Short_Press_Flag = 1;
        Remote_data->Remote_clicker.Trigger_Down_Action.Cnt ++;
    }
    
    if(Remote_data->Remote_clicker.Trigger_Up_Action.Cnt >= LONG_PRESS_THRESHOLD)
    {
        Remote_data->Remote_clicker.Trigger_Up_Action.Cnt = LONG_PRESS_THRESHOLD;//长按时置长按标志位，清零短按，翻转标志位
        Remote_data->Remote_clicker.Trigger_Up_Action.Long_Press_Flag = 1;
        Remote_data->Remote_clicker.Trigger_Up = 0;
        Remote_data->Remote_clicker.Trigger_Up_Action.Toggle_Press_Flag = 0;
        Remote_data->Remote_clicker.Trigger_Up_Action.Short_Press_Flag = 0;
        
    }
    else if(Remote_data->Remote_clicker.Trigger_Down_Action.Cnt >=LONG_PRESS_THRESHOLD)
    {
        Remote_data->Remote_clicker.Trigger_Down_Action.Cnt = LONG_PRESS_THRESHOLD;
        Remote_data->Remote_clicker.Trigger_Down_Action.Long_Press_Flag = 1;
        Remote_data->Remote_clicker.Trigger_Down = 0;
        Remote_data->Remote_clicker.Trigger_Down_Action.Toggle_Press_Flag = 0;
        Remote_data->Remote_clicker.Trigger_Down_Action.Short_Press_Flag = 0;
    }
    
    if(Remote_data->Remote_clicker.ch4 == 0)
    {
        Remote_data->Remote_clicker.Trigger_Up_Action.Cnt = 0;
        Remote_data->Remote_clicker.Trigger_Up_Action.Long_Press_Flag = 0;
        Remote_data->Remote_clicker.Trigger_Down_Action.Cnt = 0;
        Remote_data->Remote_clicker.Trigger_Down_Action.Long_Press_Flag = 0;
    }
    Remote_data->Remote_clicker.s2 = Remote_data->Remote_clicker.s2_last;//保存本次数据作为下一次的上一次
}


