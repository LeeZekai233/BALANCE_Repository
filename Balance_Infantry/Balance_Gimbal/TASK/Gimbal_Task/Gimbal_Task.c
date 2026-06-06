#include "main.h"

Gimbal_t Gimbal = {0};

/**
************************************************************************************************************************
* @Name     : Normalize_Angle_PI
* @brief    : 归化角度-PI ——PI
* @param	: float angle
* @retval   : float
* @Note     : 调整角度至-PI~PI，并且舍弃非正常数据
************************************************************************************************************************
**/
float Normalize_Angle_PI(float angle)
{
    // 如果是无穷大或非数字，直接返回 0 或指定值
    if (isinf(angle) || isnan(angle))
    {
        return 0.0f; // 默认返回 0，可根据需要修改
    }

    // 使用 fmod 将角度归一化到 [-2PI, 2PI]
    angle = fmod(angle, 2 * PI);

    // 调整到 [-PI, PI] 范围
    if (angle > PI)
    {
        angle -= 2 * PI;
    }
    else if (angle < -PI)
    {
        angle += 2 * PI;
    }
    return angle;
}



void Gimbal_Mode_Select(void)
{
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == DOWN)
        {
            Gimbal.Remote_Gimbal_Mode = GIMBAL_RELAX ;
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE)
        {
            Gimbal.Remote_Gimbal_Mode =  GIMBAL_REMOTE;
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            Gimbal.Remote_Gimbal_Mode =  GIMBAL_REMOTE;
        }
       
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//使用灰控
    {
        if(Remote_VTM.Remote_clicker.Switch == RIGHT)
        {
            Gimbal.Remote_Gimbal_Mode = GIMBAL_RELAX ;
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)
        {
            Gimbal.Remote_Gimbal_Mode = GIMBAL_KEY_MOUSE  ;
        }
        else if(Remote_VTM.Remote_clicker.Switch == LEFT)
        { 
            Gimbal.Remote_Gimbal_Mode = GIMBAL_REMOTE;
        }
    }
    else//其他状态
    {
        Gimbal.Remote_Gimbal_Mode = GIMBAL_RELAX ;
    }
    
    
    
    if(USART_Gimbal_Data.Gimbal_Init_Cmd != 1 || Gimbal.Remote_Gimbal_Mode == GIMBAL_RELAX)
    {
        Gimbal.Gimbal_Mode = GIMBAL_RELAX ;
    }
    else
    {
        if(Gimbal.Remote_Gimbal_Mode != GIMBAL_RELAX && Gimbal.Init_Finish_Flag != 1)
        {
            Gimbal.Gimbal_Mode = GIMBAL_INIT ;
        }
        else if(Gimbal.Remote_Gimbal_Mode != GIMBAL_RELAX && Gimbal.Init_Finish_Flag == 1)
        {
            Gimbal.Gimbal_Mode = Gimbal.Remote_Gimbal_Mode;
        }
    }
    
}



void Gimbal_Feedback_Update(void)
{
    //反馈值更新
    Gimbal.Pitch_Speed_Fdb = Gimbal.CH040_Data.Pitch_Gyro_Omega ;
    Gimbal.Pitch_Angle_Fdb = Gimbal.CH040_Data.Pitch_Angle ;
    if(Gimbal.Gimbal_Mode != GIMBAL_INIT)
    {
        Gimbal.Yaw_Angle_Fdb = Gimbal.CH040_Data.Yaw_Multi_Angle ;
        Gimbal.Yaw_Speed_Fdb = Gimbal.CH040_Data.Yaw_Gyro_Omega ;
    }
    else if(Gimbal.Gimbal_Mode == GIMBAL_INIT)
    {
        Gimbal.Yaw_Angle_Fdb = Gimbal.Yaw_Motor_Encoder.Angle_Rad_fdb ;
        Gimbal.Yaw_Speed_Fdb = Gimbal.Yaw_Motor_Encoder.Omega_Rad_fdb ;
    }
    
    USART_Chassis_Data.Yaw_Encoder_Angle = Gimbal.Yaw_Motor_Encoder.Angle_Rad_fdb ;//发给底盘
    USART_Chassis_Data.Gimbal_Init_Finish_Flag = Gimbal.Init_Finish_Flag ;
}


void Gimbal_Reference_Update(void)
{
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控
    {
        
        switch (Gimbal.Gimbal_Mode)
        {
            case GIMBAL_REMOTE :
                {
                    Gimbal.Yaw_Angle_Ref += Remote_DT7_data.Remote_clicker.ch1 * 0.005f;
                    Gimbal.Pitch_Angle_Ref += Remote_DT7_data.Remote_clicker.ch0 * 0.005f;
                }
                break;
            default :
                break;
       }
    }
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//使用灰控
    {    
        switch (Gimbal.Gimbal_Mode)
        {
            case GIMBAL_KEY_MOUSE :
            {
                Gimbal.Yaw_Angle_Ref += Remote_VTM.Remote_mouse.y * 0.005f;
                Gimbal.Pitch_Angle_Ref += Remote_VTM.Remote_mouse.x * 0.005f;
            }
            break;
            case GIMBAL_REMOTE :
            {
                Gimbal.Yaw_Angle_Ref += Remote_DT7_data.Remote_clicker.ch1 * 0.005f;
                Gimbal.Pitch_Angle_Ref += Remote_DT7_data.Remote_clicker.ch0 * 0.005f;
            }
            break;
            default :
            break;
        }
        
    }
}



void Gimbal_Relax_Handle(void)
{
    Gimbal.Init_Finish_Flag = 0;
    Gimbal.Pitch_Motor_Set_Current = 0;
    Gimbal.Pitch_Angle_Ref = 0;
    Gimbal.Yaw_Angle_Ref = 0;
    Gimbal.Yaw_Speed_Ref = 0;
}

void Gimbal_Init_Handle(void)
{
    if(Gimbal.Yaw_Angle_Fdb <= -90 || Gimbal.Yaw_Angle_Fdb >=90)
    {
        Gimbal.Yaw_Angle_Ref = -180;
    }
    else
    {
        Gimbal.Yaw_Angle_Ref = 0;
    }
    Gimbal.Pitch_Angle_Ref = 0;
    
    if(fabs(Gimbal.Yaw_Angle_Ref - Gimbal.Yaw_Angle_Fdb) <= 5)
    {
        Gimbal.Pitch_Angle_Ref = Gimbal.CH040_Data.Pitch_Angle ;
        Gimbal.Yaw_Angle_Ref = Gimbal.CH040_Data.Yaw_Angle ;
        Gimbal.Init_Finish_Flag = 1;
    }
    
    Gimbal.Pitch_Speed_Ref = PID_Calc(&Gimbal.Pitch_Motor_Angle_PID ,Gimbal.Pitch_Angle_Ref ,Gimbal.Pitch_Angle_Fdb );
    Gimbal.Pitch_Motor_Set_Current = PID_Calc(&Gimbal.Pitch_Motor_Speed_PID, Gimbal.Pitch_Speed_Ref, Gimbal.Pitch_Speed_Fdb);
    Gimbal.Yaw_Speed_Ref = PID_Calc(&Gimbal.Yaw_Motor_Angle_PID, Gimbal.Yaw_Angle_Ref , Gimbal.Yaw_Angle_Fdb );
}



void Gimbal_Remote_Handle(void)
{
    if(Gimbal.Gimbal_Mode_Last == GIMBAL_INIT && Gimbal.Gimbal_Mode != GIMBAL_INIT)//防跳变
    {
        Gimbal.Pitch_Angle_Ref = Gimbal.Pitch_Angle_Fdb ;
        Gimbal.Yaw_Angle_Ref = Gimbal.Yaw_Angle_Fdb ;
    }
    Gimbal.Pitch_Speed_Ref = PID_Calc(&Gimbal.Pitch_Motor_Angle_PID ,Gimbal.Pitch_Angle_Ref ,Gimbal.Pitch_Angle_Fdb );
    Gimbal.Pitch_Motor_Set_Current = PID_Calc(&Gimbal.Pitch_Motor_Speed_PID, Gimbal.Pitch_Speed_Ref, Gimbal.Pitch_Speed_Fdb);
    Gimbal.Yaw_Speed_Ref = PID_Calc(&Gimbal.Yaw_Motor_Angle_PID, Gimbal.Yaw_Angle_Ref , Gimbal.Yaw_Angle_Fdb );
}




void Gimbal_Control_Loop(void)
{
    switch (Gimbal.Gimbal_Mode)
    {
        case GIMBAL_RELAX :
            Gimbal_Relax_Handle( );
            break;
        case GIMBAL_INIT :
            Gimbal_Init_Handle( );
            break;
        case GIMBAL_REMOTE :
            Gimbal_Remote_Handle( );
            break;
        case GIMBAL_KEY_MOUSE :
            Gimbal_Remote_Handle( );
            break;
        default :
            break;
    }
    Gimbal.Gimbal_Mode_Last = Gimbal.Gimbal_Mode ;
}




void Gimbal_Task(void)
{
    Gimbal_Mode_Select();
    Gimbal_Reference_Update();
    Gimbal_Feedback_Update();
    Gimbal_Control_Loop();
}

