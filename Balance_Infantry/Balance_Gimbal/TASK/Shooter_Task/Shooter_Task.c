#include "main.h"

Shooter_t Shooter;


void Shooter_Mode_Select(void)
{
    Shooter.Last_Shooter_Mode = Shooter.Shooter_Mode ;
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE || Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            if(Remote_DT7_data.Remote_clicker.s2 == UP)
            {
                if(Remote_DT7_data.Remote_clicker.ch4_Down_Action.Toggle_Press_Flag == 1  && /*云台没初始化完不允许打弹*/Gimbal.Gimbal_Mode != GIMBAL_RELAX && Gimbal.Gimbal_Mode != GIMBAL_INIT)//热量限制可以在这里添加
                {
                    if(Remote_DT7_data.Remote_clicker.ch4_Up_Action.Long_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)//摩擦轮开启时，长按，切入连发
                    {
                        Shooter.Shooter_Mode = BURST_FIRE;
                    }
                    else if(Remote_DT7_data.Remote_clicker.ch4_Up_Action.Short_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)//摩擦轮开启时，短按，切入单发
                    {
                        Shooter.Shooter_Mode = SINGLE_SHOOT;
                    }
                    else
                    {
                        if((fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) < 5 && Shooter.Last_Shooter_Mode != SHOOTER_RELAX) || Shooter.Last_Shooter_Mode == SHOOTER_RELAX) //单发或连发，拨盘转到位置， 或者上一次是失能， 切入停火模式
                        {
                            Shooter.Shooter_Mode = STOP_FIRE ;
                        }
                    }
                }
                else
                {
                    Shooter.Shooter_Mode = SHOOTER_RELAX;
                }
            }
        }
        else if(Remote_DT7_data.Remote_clicker.s1 == DOWN)
        {
            Shooter.Shooter_Mode = SHOOTER_RELAX ;
        }
    }
    
    else if(Remote_DT7_data.online_flag == 0 && Remote_VTM.online_flag == 1)//使用灰控
    {
        if(Remote_VTM.Remote_clicker.Switch == LEFT)//使用控进行控制
        {
            if(Remote_VTM.Remote_clicker.Trigger_Action.Toggle_Press_Flag == 1)//按一次Trigger后，拨轮控制打弹
            {
                if(Remote_VTM.Remote_clicker.ch4_Down_Action.Toggle_Press_Flag == 1 && /*云台没初始化完不允许打弹*/Gimbal.Gimbal_Mode != GIMBAL_RELAX && Gimbal.Gimbal_Mode != GIMBAL_INIT)
                {
                    if(Remote_VTM.Remote_clicker.ch4_Up_Action.Short_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                    {
                        Shooter.Shooter_Mode = SINGLE_SHOOT;
                    }
                    else if(Remote_VTM.Remote_clicker.ch4_Up_Action.Long_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                    {
                        Shooter.Shooter_Mode = BURST_FIRE;
                    }
                    else
                    {
                        if((fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) < 5 && Shooter.Last_Shooter_Mode != SHOOTER_RELAX) || Shooter.Last_Shooter_Mode == SHOOTER_RELAX) //单发或连发，拨盘转到位置， 或者上一次是失能， 切入停火模式
                        {
                            Shooter.Shooter_Mode = STOP_FIRE ;
                        }
                    }
                }
            }
            else//再按一次Trigger,退出后发射机构失能
            {
                Shooter.Shooter_Mode = SHOOTER_RELAX;
            }
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)//使用键鼠控制
        {
            if(Remote_VTM.key.Key_C_Action.Toggle_Press_Flag == 1)
            {
                if(Remote_VTM.key.Key_V_Action.Toggle_Press_Flag == 1 && Remote_VTM.Remote_mouse.Press_L_Action.Short_Press_Flag == 1 && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                {
                    Shooter.Shooter_Mode = SINGLE_SHOOT;
                }
                else if(Remote_VTM.key.Key_V_Action.Toggle_Press_Flag == 0 && Remote_VTM.Remote_mouse.Press_L_Action.Original_Press_Flag == 1 && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                {
                    Shooter.Shooter_Mode = BURST_FIRE;
                }
                else
                {
                    if((fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) < 5 && Shooter.Last_Shooter_Mode != SHOOTER_RELAX) || Shooter.Last_Shooter_Mode == SHOOTER_RELAX) //单发或连发，拨盘转到位置， 或者上一次是失能， 切入停火模式
                    {
                        Shooter.Shooter_Mode = STOP_FIRE ;
                    }
                }
            }
            else if(Remote_VTM.key.Key_C_Action.Toggle_Press_Flag == 0)
            {
                Shooter.Shooter_Mode = SHOOTER_RELAX;
            }
        }
        else if(Remote_VTM.Remote_clicker.Switch == RIGHT)//关控
        {
            Shooter.Shooter_Mode = SHOOTER_RELAX ;
        }
    }
}




void Shooter_Feedback_Update(void)
{
    Shooter.Fric_Speed_Fdb[0] = Fric_M3508[0].rate_rpm ;
    Shooter.Fric_Speed_Fdb[1] = Fric_M3508[1].rate_rpm ;
    Shooter.Poke_Angle_Fdb = Shooter.Poke_Motor_Encoder.Angle_Deg_Total_fdb;
    Shooter.Poke_Speed_Fdb = Shooter.Poke_Motor_Encoder.Omega_Deg_fdb ;
    
    Shooter.Heat_Restrict.Heat_Cooling_Value = USART_Gimbal_Data.shooter_barrel_cooling_value ;//热量限制用
    Shooter.Heat_Restrict.Shooter_Heat_meas = USART_Gimbal_Data.shooter_id1_17mm_cooling_heat ;
    Shooter.Heat_Restrict.Heat_Limit = Shooter.Heat_Restrict.Heat_Limit ;
}



void Shooter_State_Update(void)
{
    Shooter.Last_Shooter_State = Shooter.Shooter_State;
    if((fabs(Shooter.Fric_Speed_Fdb[0]) + fabs(Shooter.Fric_Speed_Fdb[1])) /2 >= 1800)
    {
        Shooter.Fric_State = FRIC_ON ;
    }
    else
    {
        Shooter.Fric_State = FRIC_OFF ;
    }
    
    
    switch (Shooter.Shooter_State)
    {
        case SHOOTER_NORMAL :
            if(fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) > 5.0f)//步兵一颗弹36度
            {
                Shooter.Poke_Trap_CNT ++;
            }
            else
            {
                Shooter.Poke_Trap_CNT = 0;
            }
            
            if(Shooter.Poke_Trap_CNT == 2000)
            {
                Shooter.Shooter_State = SHOOTER_TRAP ;
                Shooter.Poke_Trap_CNT = 0;
            }
            break;
        case SHOOTER_TRAP :
            Shooter.Poke_Trap_Handle_CNT ++;
            if(Shooter.Poke_Trap_Handle_CNT == 2000)
            {
                Shooter.Shooter_State = SHOOTER_NORMAL ;
                Shooter.Poke_Trap_Handle_CNT = 0;
            }
            break;
    }
}


void Shoot_Frequency_Select(void)
{
    if(Gimbal.Gimbal_Mode == GIMBAL_AUTO_AIM || Gimbal.Gimbal_Mode == GIMBAL_SENTRY)
    {
        Shooter.Shoot_Frequency = 12;
    }
    else
    {
        Shooter.Shoot_Frequency = 10;
    }
    
    if(Shooter.Heat_Restrict.Remain_Bullets < 3)
    {
        Shooter.Shoot_Frequency = 0;
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets < 4)
    {
        Shooter.Shoot_Frequency = 2;
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets < 5)
    {
        Shooter.Shoot_Frequency = 4;
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets < 6)
    {
        if(Shooter.Shoot_Frequency > 7)
        {
            Shooter.Shoot_Frequency = 5;
        }
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets < 7)
    {
        if(Shooter.Shoot_Frequency > 7)
        {
            Shooter.Shoot_Frequency = 6;
        }
    }
    
}





void Heat_Restrict(void)
{
    //更新剩余发弹量，在离线计算的热量和裁判系统读的热量选一个更保守的
    if(Shooter.Heat_Restrict.Remain_Bullets_hat > 0)
    {
        Shooter.Heat_Restrict.Remain_Bullets_hat -= Shooter.Heat_Restrict.Heat_Cooling_Value/1000.0f;
    }
    Shooter.Heat_Restrict.Remain_Bullets_hat = (Shooter.Heat_Restrict.Heat_Limit - Shooter.Heat_Restrict.Shooter_Heat_hat)/10.0f;
    Shooter.Heat_Restrict.Remain_Bullets_meas = (Shooter.Heat_Restrict.Heat_Limit - Shooter.Heat_Restrict.Shooter_Heat_meas)/10.0f;
    if(Shooter.Heat_Restrict.Remain_Bullets_hat > Shooter.Heat_Restrict.Remain_Bullets_meas)
    {
        Shooter.Heat_Restrict.Remain_Bullets = Shooter.Heat_Restrict.Remain_Bullets_meas;
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets_meas > Shooter.Heat_Restrict.Remain_Bullets_hat)
    {
        Shooter.Heat_Restrict.Remain_Bullets = Shooter.Heat_Restrict.Remain_Bullets_hat;
    }
    
    
    if(Shooter.Heat_Restrict.Remain_Bullets <= 4)
    {
        Shooter.Heat_Restrict.Fire_Permission = SHOOT_DENIED ;
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets > 4)
    {
        Shooter.Heat_Restrict.Fire_Permission = SHOOT_ALLOWED ;
    }
}






void Shooter_Reference_Update(void)
{
    Shooter.Last_Poke_Angle_Ref = Shooter.Poke_Angle_Ref ;
    
    switch (Shooter.Shooter_State)
    {
        case SHOOTER_NORMAL :
            if(Shooter.Shooter_Mode == STOP_FIRE)
            {
                Shooter.Fric_Speed_Ref[0] = LEFT_FIRC_SPEED;
                Shooter.Fric_Speed_Ref[1] = RIGHT_FRIC_SPEED;
                if(Shooter.Last_Shooter_Mode == SHOOTER_RELAX)
                {
                    Shooter.Poke_Angle_Ref = Shooter.Poke_Angle_Fdb; //上一次是失能模式，则这一次以反馈值为参考值
                }
                else if(Shooter.Last_Shooter_Mode == SINGLE_SHOOT || Shooter.Last_Shooter_Mode == BURST_FIRE)
                {
                    Shooter.Poke_Angle_Ref = Shooter.Poke_Angle_Ref;//上一次是单发或连发，则参考值不变
                }
                Shooter.Burst_Fire_Cnt = 0;
            }
            else if(Shooter.Shooter_Mode == SINGLE_SHOOT)
            {
                Shooter.Fric_Speed_Ref[0] = LEFT_FIRC_SPEED;
                Shooter.Fric_Speed_Ref[1] = RIGHT_FRIC_SPEED;
                if(Shooter.Last_Shooter_Mode == STOP_FIRE)
                {
                    Shooter.Poke_Angle_Ref += 36;
                    Shooter.Heat_Restrict.Shooter_Heat_hat += 10;
                }
            }
            else if(Shooter.Shooter_Mode == BURST_FIRE)
            {
                Shooter.Burst_Fire_Cnt ++;
                Shooter.Fric_Speed_Ref[0] = LEFT_FIRC_SPEED;
                Shooter.Fric_Speed_Ref[1] = RIGHT_FRIC_SPEED;
                if(Shooter.Last_Shooter_Mode == STOP_FIRE)
                {
                    Shooter.Poke_Angle_Ref += 36;
                    Shooter.Heat_Restrict.Shooter_Heat_hat += 10;
                }
                
                if(Shooter.Burst_Fire_Cnt == (1.0f/Shooter.Shoot_Frequency/1000.0f))
                {
                    Shooter.Poke_Angle_Ref += 36;
                    Shooter.Heat_Restrict.Shooter_Heat_hat += 10;
                    Shooter.Burst_Fire_Cnt = 0;
                }
            }
            else if(Shooter.Shooter_Mode == SHOOTER_RELAX)
            {
                Shooter.Fric_Speed_Ref[0] = Shooter.Fric_Speed_Fdb[0];
                Shooter.Fric_Speed_Ref[1] = Shooter.Fric_Speed_Fdb[1];
                Shooter.Poke_Angle_Ref = Shooter.Poke_Angle_Fdb ;
                Shooter.Poke_Speed_Ref = Shooter.Poke_Speed_Fdb ;
            }
            
//            
//            if(Shooter.Last_Shooter_State == SHOOTER_TRAP)
//            {
//                Shooter.Poke_Angle_Ref += 5;
//            }                
            break;
        case SHOOTER_TRAP :
            Shooter.Fric_Speed_Ref[0] = LEFT_FIRC_SPEED;
            Shooter.Fric_Speed_Ref[1] = RIGHT_FRIC_SPEED;
            if(Shooter.Last_Shooter_State != SHOOTER_TRAP)//卡弹处理
            {
                Shooter.Poke_Angle_Ref = Shooter.Poke_Angle_Fdb - 20;
            }
            break;
    }
}




void Shooter_Control_Loop(void)
{
    switch (Shooter.Shooter_Mode)
    {
        case SHOOTER_RELAX :
            Shooter.Fric_Motor_Ser_Current[0] = 0;
            Shooter.Fric_Motor_Ser_Current[1] = 0;
            Shooter.Poke_Motor_Set_Current = 0;
            break;
        case BURST_FIRE :
            Shooter.Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter.Fric_Speed_PID[0], Shooter.Fric_Speed_Fdb[0], Shooter.Fric_Speed_Ref[0]);
            Shooter.Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter.Fric_Speed_PID[1], Shooter.Fric_Speed_Fdb[1], Shooter.Fric_Speed_Ref[1]);
            Shooter.Poke_Speed_Ref = PID_Calc(&Shooter.Poke_Angle_PID, Shooter.Poke_Angle_Fdb, Shooter.Poke_Angle_Ref);
            Shooter.Poke_Motor_Set_Current = PID_Calc(&Shooter.Poke_Speed_PID, Shooter.Poke_Speed_Fdb, Shooter.Poke_Speed_Ref);
            break;
        case SINGLE_SHOOT :
            Shooter.Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter.Fric_Speed_PID[0], Shooter.Fric_Speed_Fdb[0], Shooter.Fric_Speed_Ref[0]);
            Shooter.Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter.Fric_Speed_PID[1], Shooter.Fric_Speed_Fdb[1], Shooter.Fric_Speed_Ref[1]);
            Shooter.Poke_Speed_Ref = PID_Calc(&Shooter.Poke_Angle_PID, Shooter.Poke_Angle_Fdb, Shooter.Poke_Angle_Ref);
            Shooter.Poke_Motor_Set_Current = PID_Calc(&Shooter.Poke_Speed_PID, Shooter.Poke_Speed_Fdb, Shooter.Poke_Speed_Ref);
            break;
        case STOP_FIRE :
            Shooter.Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter.Fric_Speed_PID[0], Shooter.Fric_Speed_Fdb[0], Shooter.Fric_Speed_Ref[0]);
            Shooter.Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter.Fric_Speed_PID[1], Shooter.Fric_Speed_Fdb[1], Shooter.Fric_Speed_Ref[1]);
            Shooter.Poke_Speed_Ref = PID_Calc(&Shooter.Poke_Angle_PID, Shooter.Poke_Angle_Fdb, Shooter.Poke_Angle_Ref);
            Shooter.Poke_Motor_Set_Current = PID_Calc(&Shooter.Poke_Speed_PID, Shooter.Poke_Speed_Fdb, Shooter.Poke_Speed_Ref);
            break;
        default :
            break;
    }
}


void Shooter_Debug(void)
{ 
    Shooter.Poke_Angle_Fdb = Shooter.Poke_Motor_Encoder.Angle_Deg_Total_fdb;
    Shooter.Poke_Speed_Fdb = Shooter.Poke_Motor_Encoder.Omega_Deg_fdb;
    Shooter.Poke_Angle_Ref +=Remote_DT7_data.Remote_clicker.ch0 * 0.001;
    Shooter.Poke_Speed_Ref = PID_Calc(&Shooter.Poke_Angle_PID ,Shooter.Poke_Angle_Fdb ,Shooter.Poke_Angle_Ref );
    Shooter.Poke_Motor_Set_Current = PID_Calc(&Shooter.Poke_Speed_PID, Shooter.Poke_Speed_Fdb, Shooter.Poke_Speed_Ref);
}


void Shooter_Task(void)
{
    Shooter_Mode_Select();
    Heat_Restrict();
    Shooter_Feedback_Update();
    Shooter_State_Update();
    Shoot_Frequency_Select();
    Shooter_Reference_Update();
    Shooter_Control_Loop();
}





