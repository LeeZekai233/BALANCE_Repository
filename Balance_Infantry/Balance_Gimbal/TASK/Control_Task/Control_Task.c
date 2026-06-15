#include "main.h"

uint32_t time_tick;


void Control_Task(void)
{
    time_tick++;
    Remote_Online_Detect(&Remote_DT7_data,&Remote_VTM);
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控
    {
        Key_Mouse_State_Update(&Remote_DT7_data.key,&Remote_DT7_data.Remote_mouse);
        Remote_Switch_Action_Detect(&Remote_DT7_data);
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//使用灰控
    {
        Key_Mouse_State_Update(&Remote_VTM.key,&Remote_VTM.Remote_mouse);
        VTM_Clicker_State_Update(&Remote_VTM);
    }
    
    Chassis_Task();
    Gimbal_Task();
    Shooter_Task();
    
    if(time_tick%2 == 0)
    {
        CAN2_Send_Task(Gimbal.Yaw_Motor_Set_T,Shooter.Poke_Motor_Set_Speed);
    }
    
    if(time_tick %2 == 1)
    {
        CAN1_Send_Task(Gimbal.Pitch_Motor_Set_Current, Shooter.Fric_Motor_Ser_Current[0],Shooter.Fric_Motor_Ser_Current[1]);
    }
    
    if(time_tick%5 == 0)
    {
        USART_Chassis_Send(&USART_Chassis_Data);
    }
}



void Control_Task_Init(void)
{
    PID_Init(&Gimbal.Pitch_Motor_Angle_PID,PID_POSITION,40,0,0,10000,0);
    PID_Init(&Gimbal.Pitch_Motor_Speed_PID,PID_POSITION,50,0,0,20000,0);
    PID_Init(&Gimbal.Yaw_Motor_Angle_PID,PID_POSITION,20,0,0,10000,0);
    PID_Init(&Gimbal.Yaw_Motor_Speed_PID,PID_POSITION,0.025,0.0003,0,10,1);
    PID_Init(&Shooter.Poke_Angle_PID,PID_POSITION,140,0,2000,20000,0);
    PID_Init(&Shooter.Poke_Speed_PID,PID_POSITION,0.04,0.0015,0,2048,512);
    PID_Init(&Shooter.Fric_Speed_PID[0],PID_POSITION,3.5,0,0,15000,5000);
    PID_Init(&Shooter.Fric_Speed_PID[1],PID_POSITION,3.5,0,0,15000,5000);
    PID_Init(&Gimbal.Yaw_Motor_Init_Speed_PID,PID_POSITION,0.6,0.008,0,10,4);
    PID_Init(&Gimbal.Yaw_Motor_Init_Angle_PID,PID_POSITION,40,0,0,100,0);
}


void Chassis_Mode_Select(void)
{
    static uint16_t rorate_reserve_cnt = 0;//反转小陀螺状态用
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控，这里理解为白控时不用键鼠
    {
        if(Remote_DT7_data.Remote_clicker.s1 == DOWN)
        {
            USART_Chassis_Data.Chassis_Mode = CHASSIS_RELAX;
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE || Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            if(Remote_DT7_data.Remote_clicker.s2 == MIDDLE)//跟随遥控
            {
                USART_Chassis_Data.Chassis_Mode = MANUAL_FOLLOW_REMOTE;
            }
            else if(Remote_DT7_data.Remote_clicker.s2 == DOWN)//小陀螺
            {
                if(Remote_DT7_data.Remote_clicker.s2_Action == MIDDLE_TO_DOWN)
                {
                    rorate_reserve_cnt ++;
                }
                
                if(rorate_reserve_cnt%2 == 0)
                {
                    USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
                }
                else if(rorate_reserve_cnt%2 == 1)
                {
                    USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
                }
            }
            else if(Remote_DT7_data.Remote_clicker.s2 == UP)//打弹用
            {
                USART_Chassis_Data.Chassis_Mode = MANUAL_FOLLOW_REMOTE;
            }
        }
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//使用灰控
    {
        if(Remote_VTM.Remote_clicker.Switch == RIGHT)//关控
        {
            USART_Chassis_Data.Chassis_Mode = CHASSIS_RELAX;
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)//使用键鼠
        {
           // USART_Chassis_Data.Chassis_Mode = 1;
            if(Remote_VTM.key.Key_B_Action.Short_Press_Flag == 1)//按一次，换一次小陀螺方向
            {
                rorate_reserve_cnt ++;
            }
            
            if(Remote_VTM.key.Key_B_Action.Short_Press_Flag == 1 && USART_Chassis_Data.Chassis_Mode == MANUAL_FOLLOW_REMOTE)//按B切入小陀螺
            {
                if(rorate_reserve_cnt%2 == 0)
                {
                    USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
                }
                else if(rorate_reserve_cnt%2 == 1)
                {
                    USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
                }
            }
            
            
      //      if()
            
            
            
            
            if(USART_Gimbal_Data.current_HP == 0)//死了一定Relax
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_RELAX;
            }
        }
        else if(Remote_VTM.Remote_clicker.Switch == LEFT)//使用遥控
        {
           // USART_Chassis_Data.Chassis_Mode = 1;
            if(Remote_VTM.Remote_clicker.Trigger_Action.Short_Press_Flag == 1)//按一次，换一次小陀螺方向
            {
                rorate_reserve_cnt ++;
            }
            
            if(Remote_VTM.Remote_clicker.Trigger_Action.Toggle_Press_Flag == 1)
            {
                if(rorate_reserve_cnt%2 == 0)
                {
                    USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
                }
                else if(rorate_reserve_cnt%2 == 1)
                {
                    USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
                }
            }
            else if(Remote_VTM.Remote_clicker.Trigger_Action.Toggle_Press_Flag == 0)
            {
                USART_Chassis_Data.Chassis_Mode = MANUAL_FOLLOW_REMOTE;
            }
        }
    }
    else//如果灰控和白控都在或都不在
    {
        USART_Chassis_Data.Chassis_Mode = CHASSIS_RELAX;
    }
}



void Chassis_Reference_Update(void)
{
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE || Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            USART_Chassis_Data.V_y = Remote_DT7_data.Remote_clicker.ch1/660.0f*2.5f;
          //  USART_Chassis_Data.V_x = Remote_DT7_data.Remote_clicker.ch2/660.0f*2.5f;//用控时不给Vx
            if(Remote_DT7_data.Remote_clicker.s2 == MIDDLE || Remote_DT7_data.Remote_clicker.s2 == DOWN)
            {
                if(Remote_DT7_data.Remote_clicker.ch4 == 0)
                {
                    USART_Chassis_Data.Cmd_Leg_Length = 1;
                }
                else if(Remote_DT7_data.Remote_clicker.ch4 >= 640)
                {
                    USART_Chassis_Data.Cmd_Leg_Length = 2;
                }
                else if(Remote_DT7_data.Remote_clicker.ch4 <= -640)
                {
                    USART_Chassis_Data.Cmd_Leg_Length = 3;
                }
            }
        }
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//用灰控
    {
        if(Remote_VTM.Remote_clicker.Switch == LEFT)
        {
            USART_Chassis_Data.V_y = Remote_VTM.Remote_clicker.ch1/660*2.2f;
          //  USART_Chassis_Data.V_x = Remote_VTM.Remote_clicker.ch2/660*2.2f;
            if(Remote_VTM.Remote_clicker.ch4 == 0)
            {
                USART_Chassis_Data.Cmd_Leg_Length = 1;
            }
            else if(Remote_VTM.Remote_clicker.ch4 >= 640)
            {
                USART_Chassis_Data.Cmd_Leg_Length = 2;
            }
            else if(Remote_VTM.Remote_clicker.ch4 <= -640)
            {
                USART_Chassis_Data.Cmd_Leg_Length = 3;
            }
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)
        {
            if(Remote_VTM.key.Key_SHIFT_Action.Original_Press_Flag == 1)//底盘速度给定
            {
                USART_Chassis_Data.V_y = (Remote_VTM.key.Key_W_Action.Original_Press_Flag - Remote_VTM.key.Key_S_Action.Original_Press_Flag)*2.5f;
                USART_Chassis_Data.V_x = (Remote_VTM.key.Key_D_Action.Original_Press_Flag - Remote_VTM.key.Key_A_Action.Original_Press_Flag)*2.5f;
            }
            else
            {
                USART_Chassis_Data.V_y = (Remote_VTM.key.Key_W_Action.Original_Press_Flag - Remote_VTM.key.Key_S_Action.Original_Press_Flag)*2.2f;
                USART_Chassis_Data.V_x = (Remote_VTM.key.Key_D_Action.Original_Press_Flag - Remote_VTM.key.Key_A_Action.Original_Press_Flag)*2.2f;
            }
            
            if(Remote_VTM.key.Key_Z_Action.Long_Press_Flag == 1)//长按Z中腿长
            {
                USART_Chassis_Data.Cmd_Leg_Length = 2;
            }
            else if(Remote_VTM.key.Key_CTRL_Action.Long_Press_Flag == 1)//长按CTRL高腿长
            {
                USART_Chassis_Data.Cmd_Leg_Length = 3;
            }
            else
            {
                USART_Chassis_Data.Cmd_Leg_Length = 1;
            }
        }
    }
    else//灰控白控都不在
    {
        USART_Chassis_Data.V_y = 0;
        USART_Chassis_Data.V_x = 0;
        USART_Chassis_Data.Cmd_Leg_Length = 1;
    }
}



void Chassis_Task(void)
{
    Chassis_Mode_Select();
    Chassis_Reference_Update();
}
