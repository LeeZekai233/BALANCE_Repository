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



void Gimbal_Referance_Update(void)
{
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == DOWN)
        {
            Gimbal.Remote_Gimbal_Mode = GIMBAL_RELAX ;
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE)
        {
            Gimbal.Remote_Gimbal_Mode =  GIMBAL_FALLWO_GYRO;
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            Gimbal.Remote_Gimbal_Mode =  GIMBAL_FALLWO_GYRO;
        }
        
        
        if(Gimbal.Gimbal_Mode == GIMBAL_FALLWO_GYRO)//云台初始化完成后，开始更新参考值，白控状态下不用键鼠
        {
            Gimbal.Yaw_Angle_Ref += Remote_DT7_data.Remote_clicker.ch1 * 0.005f;
            Gimbal.Pitch_Angle_Ref += Remote_DT7_data.Remote_clicker.ch0 * 0.005f;
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
            Gimbal.Remote_Gimbal_Mode = GIMBAL_FALLWO_GYRO ;
        }
        else if(Remote_VTM.Remote_clicker.Switch == LEFT)
        { 
            Gimbal.Remote_Gimbal_Mode = GIMBAL_FALLWO_GYRO ;
        }
        
        if(Remote_VTM.Remote_clicker.Switch == CENTER && Gimbal.Gimbal_Mode == GIMBAL_FALLWO_GYRO)//键鼠
        {
            Gimbal.Yaw_Angle_Ref += Remote_VTM.Remote_mouse.y * 0.005f;
            Gimbal.Pitch_Angle_Ref += Remote_VTM.Remote_mouse.x * 0.005f;
        }
        else if(Remote_VTM.Remote_clicker.Switch == LEFT && Gimbal.Gimbal_Mode == GIMBAL_FALLWO_GYRO)//遥控
        {
            Gimbal.Yaw_Angle_Ref += Remote_VTM.Remote_clicker.ch1 * 0.005f;
            Gimbal.Pitch_Angle_Ref += Remote_VTM.Remote_clicker.ch0 * 0.005f;
        }
    }
    else//其他状态
    {
        Gimbal.Remote_Gimbal_Mode = GIMBAL_RELAX ;
    }
}



void Gimbal_State_Update(void)
{
    //反馈值更新
    Gimbal.Pitch_Speed_Fdb = Gimbal.CH040_Data.Pitch_Gyro_Omega ;
    Gimbal.Pitch_Angle_Fdb = Gimbal.CH040_Data.Pitch_Angle ;
    Gimbal.Yaw_Angle_Ref = Gimbal.CH040_Data.Yaw_Multi_Angle ;
    Gimbal.Yaw_Speed_Fdb = Gimbal.CH040_Data.Yaw_Gyro_Omega ;
    
    Gimbal.Yaw_Angle_To_Chassis = Gimbal.Yaw_Motor_Encoder.Angle_Rad_fdb ;
}



