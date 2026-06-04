#include "main.h"

uint32_t time_tick;


void Control_Task(void)
{
    time_tick++;
    Remote_Online_Detect(&Remote_DT7_data,&Remote_VTM);
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)
    {
        Key_Mouse_State_Update(&Remote_DT7_data.key,&Remote_DT7_data.Remote_mouse);
        Remote_Switch_Action_Detect(&Remote_DT7_data);
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)
    {
        Key_Mouse_State_Update(&Remote_VTM.key,&Remote_VTM.Remote_mouse);
        VTM_Clicker_State_Update(&Remote_VTM);
    }
}




void Chassis_Mode_Select(void)
{
    static uint16_t rorate_reserve_cnt = 0;//反转小陀螺状态用
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控，这里理解为白控时不用键鼠
    {
        if(Remote_DT7_data.Remote_clicker.s1 == DOWN)
        {
            USART_Chassis_Data.Chassis_Mode = 0;
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE || Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            
            if(Remote_DT7_data.Remote_clicker.s2 == DOWN)//跟随遥控
            {
                USART_Chassis_Data.Chassis_Mode = 1;
            }
            else if(Remote_DT7_data.Remote_clicker.s2 == MIDDLE)//小陀螺
            {
                if(Remote_DT7_data.Remote_clicker.s2_Action == DOWN_TO_MIDDLE)
                {
                    rorate_reserve_cnt ++;
                }
                
                if(rorate_reserve_cnt%2 == 0)
                {
                    USART_Chassis_Data.Chassis_Mode = 5;
                }
                else if(rorate_reserve_cnt%2 == 1)
                {
                    USART_Chassis_Data.Chassis_Mode = 6;
                }
            }
            else if(Remote_DT7_data.Remote_clicker.s2 == UP)//打弹用
            {
                USART_Chassis_Data.Chassis_Mode = 1;
            }
        }
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//使用灰控
    {
        if(Remote_VTM.Remote_clicker.Switch == RIGHT)
        {
            USART_Chassis_Data.Chassis_Mode = 0;
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)
        {
            USART_Chassis_Data.Chassis_Mode = 1;
        }
        else if(Remote_VTM.Remote_clicker.Switch == LEFT)
        {
            USART_Chassis_Data.Chassis_Mode = 1;
        }
    }
    else//如果灰控和白控都在或都不在
    {
        USART_Chassis_Data.Chassis_Mode = 0;
    }
}



void Chassis_Reference_Update(void)
{
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE || Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            USART_Chassis_Data.V_y = Remote_DT7_data.Remote_clicker.ch2/660*2.5f;
            USART_Chassis_Data.V_x = Remote_DT7_data.Remote_clicker.ch3/660*2.5f;
        }
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//用灰控
    {
        if(Remote_VTM.Remote_clicker.Switch == LEFT)
        {
            USART_Chassis_Data.V_y = Remote_VTM.Remote_clicker.ch2/660*2.2f;
            USART_Chassis_Data.V_x = Remote_VTM.Remote_clicker.ch3/660*2.2f;
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)
        {
            if(Remote_VTM.key.Key_SHIFT_Action.Original_Press_Flag == 1)
            {
                USART_Chassis_Data.V_y = (Remote_VTM.key.Key_W_Action.Original_Press_Flag - Remote_VTM.key.Key_S_Action.Original_Press_Flag)*2.5f;
                USART_Chassis_Data.V_x = (Remote_VTM.key.Key_D_Action.Original_Press_Flag - Remote_VTM.key.Key_A_Action.Original_Press_Flag)*2.5f;
            }
            else
            {
                USART_Chassis_Data.V_y = (Remote_VTM.key.Key_W_Action.Original_Press_Flag - Remote_VTM.key.Key_S_Action.Original_Press_Flag)*2.2f;
                USART_Chassis_Data.V_x = (Remote_VTM.key.Key_D_Action.Original_Press_Flag - Remote_VTM.key.Key_A_Action.Original_Press_Flag)*2.2f;
            }
        }
    }
    else
    {
        USART_Chassis_Data.V_y = 0;
        USART_Chassis_Data.V_x = 0;
    }
}


