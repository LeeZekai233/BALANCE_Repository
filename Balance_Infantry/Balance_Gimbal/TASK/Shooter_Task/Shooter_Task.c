#include "main.h"

Shooter_t Shooter;


void Shooter_Mode_Select(void)
{
    Shooter.Last_Shooter_Mode = Shooter.Shooter_Mode ;
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//使用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE)
        {
            if(Remote_DT7_data.Remote_clicker.s2 == UP)
            {
                if(Remote_DT7_data.Remote_clicker.ch4_Down_Action.Toggle_Press_Flag == 1)//热量限制可以在这里添加
                {
                    if(Remote_DT7_data.Remote_clicker.ch4_Up_Action.Long_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON)
                    {
                        Shooter.Shooter_Mode = BURST_FIRE;
                    }
                    else if(Remote_DT7_data.Remote_clicker.ch4_Up_Action.Short_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON)
                    {
                        Shooter.Shooter_Mode = SINGLE_SHOOT;
                    }
                    else
                    {
                        if((fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) < 5 && Shooter.Last_Shooter_Mode != SHOOTER_RELAX) || Shooter.Last_Shooter_Mode == SHOOTER_RELAX)
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
        //暂时先不考虑灰控了
    }
}

void Shooter_Feedback_Update(void)
{
    Shooter.Fric_Speed_Fdb[0] = Fric_M3508[0].rate_rpm ;
    Shooter.Fric_Speed_Fdb[1] = Fric_M3508[1].rate_rpm ;
    Shooter.Poke_Angle_Fdb = Shooter.Poke_Motor_Encoder.Angle_Deg_Total_fdb;
    Shooter.Poke_Speed_Fdb = Shooter.Poke_Motor_Encoder.Omega_Deg_fdb ;
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
    
//    if(fabs(Shooter.Poke_Speed_Fdb) >= 1)//随便给的数
//    {
//        Shooter.Poke_State = POKE_ON; 
//    }
//    else
//    {
//        Shooter.Poke_State = POKE_OFF;
//    }
        
    
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



void Shooter_Reference_Update(void)
{
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
                }
 //               else if()
//                {
//                    Shooter.Poke_Angle_Ref = Shooter.Poke_Angle_Fdb;
//                }
            }
            else if(Shooter.Shooter_Mode == BURST_FIRE)
            {
                Shooter.Burst_Fire_Cnt ++;
                Shooter.Fric_Speed_Ref[0] = LEFT_FIRC_SPEED;
                Shooter.Fric_Speed_Ref[1] = RIGHT_FRIC_SPEED;
                if(Shooter.Last_Shooter_Mode == STOP_FIRE)
                {
                    Shooter.Poke_Angle_Ref += 36;
                }
                
                if(Shooter.Burst_Fire_Cnt % 20 == 0)
                {
                    Shooter.Poke_Angle_Ref += 36;
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
            if(Shooter.Last_Shooter_State != SHOOTER_TRAP)
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
//            Shooter.Poke_Angle_Ref  +=Remote_DT7_data.Remote_clicker.ch0 * 0.001;
//            Shooter.Poke_Angle_Fdb = Shooter.Poke_Motor_Encoder.Angle_Deg_Total_fdb;
//            Shooter.Poke_Speed_Ref = PID_Calc(&Shooter.Poke_Angle_PID ,Shooter.Poke_Angle_Fdb ,Shooter.Poke_Angle_Ref );
//            Shooter.Poke_Speed_Fdb = Shooter.Poke_Motor_Encoder.Omega_Deg_fdb;
//            Shooter.Poke_Motor_Set_Current = PID_Calc(&Shooter.Poke_Speed_PID, Shooter.Poke_Speed_Fdb, Shooter.Poke_Speed_Ref);
            break;
    }
}




void Shooter_Task(void)
{
    Shooter_Mode_Select();
    Shooter_Feedback_Update();
    Shooter_State_Update();
    Shooter_Reference_Update();
    Shooter_Control_Loop();
}





