#include "main.h"

Shooter_t Shooter;


/**
************************************************************************************************************************
* @Name     : Shooter_Mode_Select
* @brief    : 发射模式选择
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
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
                        Shooter.Burst_Fire_Cnt ++;
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
            if(Remote_VTM.Remote_clicker.fn1_Action.Short_Press_Flag == 1)
            {
                Shooter.Shooter_Enable_Flag = 1;
            }
            
            if(Remote_VTM.Remote_clicker.fn1_Action.Long_Press_Flag == 1)
            {
                Shooter.Shooter_Enable_Flag = 0;
            }
            
            if(Shooter.Shooter_Enable_Flag == 1)
            {
                if(/*云台没初始化完不允许打弹*/Gimbal.Gimbal_Mode != GIMBAL_RELAX && Gimbal.Gimbal_Mode != GIMBAL_INIT)
                {
                    if(Remote_VTM.Remote_clicker.ch4_Up_Action.Short_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                    {
                        Shooter.Shooter_Mode = SINGLE_SHOOT;
                    }
                    else if(Remote_VTM.Remote_clicker.ch4_Up_Action.Long_Press_Flag == 1 && Shooter.Fric_State == FRIC_ON && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                    {
                        Shooter.Shooter_Mode = BURST_FIRE;
                        Shooter.Burst_Fire_Cnt ++;
                    }
                    else
                    {
                        if((fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) < 10 && Shooter.Last_Shooter_Mode != SHOOTER_RELAX) || Shooter.Last_Shooter_Mode == SHOOTER_RELAX) //单发或连发，拨盘转到位置， 或者上一次是失能， 切入停火模式
                        {
                            Shooter.Shooter_Mode = STOP_FIRE ;
                        }
                    }
                }
            }
            else//再按一次fn1,退出后发射机构失能
            {
                Shooter.Shooter_Mode = SHOOTER_RELAX;
            }
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)//使用键鼠控制
        {
            if(Remote_VTM.key.Key_C_Action.Short_Press_Flag == 1)//短按C开发射，长按C关发射
            {
                Shooter.Shooter_Enable_Flag = 1;
            }
            
            if(Remote_VTM.key.Key_C_Action.Long_Press_Flag == 1)
            {
                Shooter.Shooter_Enable_Flag = 0;
            }
            
            if(Remote_VTM.key.Key_V_Action.Short_Press_Flag == 1)//短按V切换单发连发
            {
                if(Shooter.Shooter_Mode_Switch_Flag == 0)
                {
                    Shooter.Shooter_Mode_Switch_Flag = 1 ;
                }
                else if(Shooter.Shooter_Mode_Switch_Flag == 1)
                {
                    Shooter.Shooter_Mode_Switch_Flag = 0 ;
                }
            }
            
            if(Shooter.Shooter_Enable_Flag == 1)//按C进入停火模式
            {
                if(Shooter.Shooter_Mode_Switch_Flag == 1 && Remote_VTM.Remote_mouse.Press_L_Action.Short_Press_Flag == 1 && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                {
                    Shooter.Shooter_Mode = SINGLE_SHOOT;
                }
                else if(Shooter.Shooter_Mode_Switch_Flag == 0 && Remote_VTM.Remote_mouse.Press_L_Action.Original_Press_Flag == 1 && Shooter.Heat_Restrict.Fire_Permission == SHOOT_ALLOWED)
                {
                    Shooter.Shooter_Mode = BURST_FIRE;
                    Shooter.Burst_Fire_Cnt ++;
                }
                else
                {
                    if((fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) < 5 && Shooter.Last_Shooter_Mode != SHOOTER_RELAX) || Shooter.Last_Shooter_Mode == SHOOTER_RELAX) //单发或连发，拨盘转到位置， 或者上一次是失能， 切入停火模式
                    {
                        Shooter.Shooter_Mode = STOP_FIRE ;
                        Shooter.Burst_Fire_Cnt = 0;
                    }
                }
            }
            else if(Shooter.Shooter_Enable_Flag == 0)
            {
                Shooter.Shooter_Mode = SHOOTER_RELAX;
            }
        }
        else if(Remote_VTM.Remote_clicker.Switch == RIGHT)//关控
        {
            Shooter.Shooter_Mode = SHOOTER_RELAX ;
            Shooter.Shooter_Enable_Flag = 0;
        }
        
        if(USART_Gimbal_Data.current_HP == 0)//死了失能发射
        {
            Shooter.Shooter_Mode = SHOOTER_RELAX ;
            Shooter.Shooter_Enable_Flag = 0;
        }
    }
    else//灰控白控都在或都不在
    {
        Shooter.Shooter_Mode = SHOOTER_RELAX ;
        Shooter.Shooter_Enable_Flag = 0;
    }
}


/**
************************************************************************************************************************
* @Name     : Shooter_Feedback_Update
* @brief    : 发射反馈值更新
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
float temp_speed;
void Shooter_Feedback_Update(void)
{
    Shooter.Fric_Speed_Fdb[0] = Fric_M3508[0].rate_rpm ;
    Shooter.Fric_Speed_Fdb[1] = Fric_M3508[1].rate_rpm ;
    Shooter.Poke_Angle_Fdb = Shooter.Poke_Motor_Encoder.Angle_Deg_Total_fdb;
    Shooter.Poke_Speed_Fdb = Shooter.Poke_Motor_Encoder.Omega_Deg_fdb ;
    temp_speed =  -Shooter.Fric_Speed_Fdb[1];
    
    Shooter.Heat_Restrict.Heat_Cooling_Value = USART_Gimbal_Data.shooter_barrel_cooling_value ;//热量限制用
    Shooter.Heat_Restrict.Shooter_Heat_meas = USART_Gimbal_Data.shooter_id1_17mm_cooling_heat ;
    Shooter.Heat_Restrict.Heat_Limit = USART_Gimbal_Data.shooter_barrel_heat_limit ;
    
   
}


/**
************************************************************************************************************************
* @Name     : Shooter_Bullet_Speed_Self_Adaptation
* @brief    : 自适应弹速
* @param	: void
* @retval   : void
* @Note     : 根据反馈的弹速灵活调整摩擦轮转速，暂时未调用
************************************************************************************************************************
**/
float Shooter_Bullet_Speed_Self_Adaptation(float Bullet_Speed)
{
	float static Bullet_Speed_Error;
	Bullet_Speed_Error=(BULLET_SPEED_TARGET-Bullet_Speed);
	
	return Bullet_Speed_Error*BULLET_SPEED_SELF_ADAPTATION_K;
}



/**
************************************************************************************************************************
* @Name     : Shooter_State_Update
* @brief    : 发射状态更新
* @param	: void
* @retval   : void
* @Note     : 检测正常和卡弹，摩擦轮和拨盘的开启与关闭
************************************************************************************************************************
**/
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
    
   if(fabs(Shooter.Poke_Speed_Fdb) >= 5)
   {
       Shooter.Poke_State = POKE_ON;
   }
   else
   {
       Shooter.Poke_State = POKE_OFF;
   }
   
    switch (Shooter.Shooter_State)
    {
        case SHOOTER_NORMAL :
            if(fabs(Shooter.Poke_Angle_Ref - Shooter.Poke_Angle_Fdb) > 30.0f)//步兵一颗弹36度
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



/**
************************************************************************************************************************
* @Name     : Shoot_Frequency_Select
* @brief    : 弹频选择
* @param	: void
* @retval   : void
* @Note     : 弹频选择
************************************************************************************************************************
**/
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




/**
************************************************************************************************************************
* @Name     : Heat_Restrict
* @brief    : 热量限制
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Heat_Restrict(void)
{
    //更新剩余发弹量，在离线计算的热量和裁判系统读的热量选一个更保守的
    if(Shooter.Heat_Restrict.Shooter_Heat_hat > 0)
    {
        Shooter.Heat_Restrict.Shooter_Heat_hat -= Shooter.Heat_Restrict.Heat_Cooling_Value/1000.0f;
    }
    Shooter.Heat_Restrict.Remain_Bullets_hat = (Shooter.Heat_Restrict.Heat_Limit - Shooter.Heat_Restrict.Shooter_Heat_hat)/10.0f;
    Shooter.Heat_Restrict.Remain_Bullets_meas = (Shooter.Heat_Restrict.Heat_Limit - Shooter.Heat_Restrict.Shooter_Heat_meas)/10.0f;
    if(Shooter.Heat_Restrict.Remain_Bullets_hat >= Shooter.Heat_Restrict.Remain_Bullets_meas)
    {
        Shooter.Heat_Restrict.Remain_Bullets = Shooter.Heat_Restrict.Remain_Bullets_meas;
    }
    else if(Shooter.Heat_Restrict.Remain_Bullets_meas >= Shooter.Heat_Restrict.Remain_Bullets_hat)
    {
        Shooter.Heat_Restrict.Remain_Bullets = Shooter.Heat_Restrict.Remain_Bullets_hat;
    }
    
    if(Shooter.Heat_Restrict.Remain_Bullets <= 4)
    {
        Shooter.Heat_Restrict.Fire_Permission = SHOOT_DENIED ;
    }
    
    if(Shooter.Heat_Restrict.Remain_Bullets > 4)
    {
        Shooter.Heat_Restrict.Fire_Permission = SHOOT_ALLOWED ;
    }
}





/**
************************************************************************************************************************
* @Name     : Shooter_Reference_Update
* @brief    : 发射机构参考值更新
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
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
                if(Shooter.Last_Shooter_Mode == SHOOTER_RELAX)/*对正角度*/
                {                                                       
                    Shooter.Poke_Angle_Ref = Shooter.Poke_Angle_Fdb - ((int32_t)Shooter.Poke_Angle_Fdb%36/*整数部分*/) - 
                       (Shooter.Poke_Angle_Fdb - (int32_t)Shooter.Poke_Angle_Fdb/*小数部分*/); //上一次是失能模式，对正角度
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
                Shooter.Fric_Speed_Ref[0] = LEFT_FIRC_SPEED;
                Shooter.Fric_Speed_Ref[1] = RIGHT_FRIC_SPEED;
                if(Shooter.Last_Shooter_Mode == STOP_FIRE)
                {
                    Shooter.Poke_Angle_Ref += 36;
                    Shooter.Heat_Restrict.Shooter_Heat_hat += 10;
                }
                
                if(Shooter.Burst_Fire_Cnt >= (1000.0f/Shooter.Shoot_Frequency))
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
            
            if(Shooter.Last_Shooter_State == SHOOTER_TRAP)
            {
                Shooter.Poke_Angle_Ref += 20;
            }
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

/**
************************************************************************************************************************
* @Name     : Shoot_Detect
* @brief    : 射击检测
* @param	: void
* @retval   : void
* @Note     : 使用停火有限状态机检测打弹，打出弹后停止射击，未启用
************************************************************************************************************************
**/
void Shoot_Detect(void)
{
    static uint16_t FSM_cnt;
    switch (Shooter.CF_FSM_State)
    {
        case FSM_RELAX :
            if(Shooter.Last_Shooter_Mode == SHOOTER_RELAX && Shooter.Shooter_Mode == SINGLE_SHOOT)
            {
                Shooter.CF_FSM_State = FSM_ACTIVATE ;
            }
            break;
        case FSM_ACTIVATE :
            FSM_cnt ++;
            if(FSM_cnt == 1000)//计数到了没检测到弹也转换状态
            {
                Shooter.CF_FSM_State = FSM_RELAX;
                FSM_cnt = 0;
            }
            
            if((fabs(Shooter.Fric_Speed_Fdb[1] - RIGHT_FRIC_SPEED) +  fabs(Shooter.Fric_Speed_Fdb[0] - LEFT_FIRC_SPEED))/2.0f > 1000)
            {
                Shooter.CF_FSM_State = FSM_HANDLE ;
                FSM_cnt = 0;
            }
            break;
        case FSM_HANDLE :
            Shooter.CF_FSM_State = FSM_RELAX ;
            break;
    }
}


/**
************************************************************************************************************************
* @Name     : Shooter_Relax_Handle
* @brief    : 发射失能处理
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Shooter_Relax_Handle(void)
{
    Shooter.Fric_Motor_Ser_Current[0] = 0;
    Shooter.Fric_Motor_Ser_Current[1] = 0;
    Shooter.Poke_Motor_Set_Speed = 0;
    Shooter.Shooter_Enable_Flag = 0;
}



/**
************************************************************************************************************************
* @Name     : Shooter_Remote_Handle
* @brief    : 发射控制处理
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Shooter_Remote_Handle(void)
{
    Shooter.Fric_Motor_Ser_Current[0] = PID_Calc(&Shooter.Fric_Speed_PID[0], Shooter.Fric_Speed_Fdb[0], Shooter.Fric_Speed_Ref[0]);
    Shooter.Fric_Motor_Ser_Current[1] = PID_Calc(&Shooter.Fric_Speed_PID[1], Shooter.Fric_Speed_Fdb[1], Shooter.Fric_Speed_Ref[1]);
    Shooter.Poke_Motor_Set_Speed = PID_Calc(&Shooter.Poke_Angle_PID, Shooter.Poke_Angle_Fdb, Shooter.Poke_Angle_Ref);
}

/**
************************************************************************************************************************
* @Name     : Shooter_Control_Loop
* @brief    : 发射控制循环
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Shooter_Control_Loop(void)
{
    switch (Shooter.Shooter_Mode)
    {
        case SHOOTER_RELAX :
            Shooter_Relax_Handle( );
            break;
        case BURST_FIRE :
            Shooter_Remote_Handle( );
            break;
        case SINGLE_SHOOT :
            Shooter_Remote_Handle( );
            break;
        case STOP_FIRE :
            Shooter_Remote_Handle( );
            break;
        default :
            break;
    }
}

/**
************************************************************************************************************************
* @Name     : Shooter_Debug
* @brief    : 射击调试
* @param	: void
* @retval   : void
* @Note     : 最初调拨盘PID时候用，未启用
************************************************************************************************************************
**/
void Shooter_Debug(void)
{ 
    Shooter.Poke_Angle_Fdb = Shooter.Poke_Motor_Encoder.Angle_Deg_Total_fdb;
    Shooter.Poke_Speed_Fdb = Shooter.Poke_Motor_Encoder.Omega_Deg_fdb;
    Shooter.Poke_Angle_Ref +=Remote_DT7_data.Remote_clicker.ch0 * 0.001;
//    Shooter.Poke_Speed_Ref = Remote_DT7_data.Remote_clicker.ch0 * 50.0f;
    Shooter.Poke_Motor_Set_Speed = PID_Calc(&Shooter.Poke_Speed_PID, Shooter.Poke_Speed_Fdb, Shooter.Poke_Speed_Ref);
}


/**
************************************************************************************************************************
* @Name     : Shooter_Task
* @brief    : 发射任务
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Shooter_Task(void)
{
    Shooter_Feedback_Update();//反馈值更新
    Shooter_State_Update();//状态更新 正常 或 卡弹
    Heat_Restrict();//热量限制
    Shoot_Frequency_Select();//弹频选择
    Shooter_Mode_Select();//模式选择
    Shooter_Reference_Update();//参考值给定
    Shooter_Control_Loop();//控制环
}





