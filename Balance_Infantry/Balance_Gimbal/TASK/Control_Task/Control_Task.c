#include "main.h"

uint32_t time_tick;

/**
************************************************************************************************************************
* @Name     : Control_Task
* @brief    : 控制任务
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Control_Task(void)
{
    time_tick++;
    Remote_Online_Detect(&Remote_DT7_data,&Remote_VTM);
    Auto_Shoot_Online_Detect(&My_Auto_Shoot);
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
    
 
    Chassis_Task( );
    Gimbal_Task( );
    Shooter_Task( );
  
    
    if(time_tick%2 == 0)
    {
        CAN2_Send_Task(Gimbal.Yaw_Motor_Set_T,Shooter.Poke_Motor_Set_Speed);
        USART_Chassis_Send(&USART_Chassis_Data);
    }

    if(time_tick %2 == 1)
    {
        CAN1_Send_Task(Gimbal.Pitch_Motor_Set_Current, Shooter.Fric_Motor_Ser_Current[0],Shooter.Fric_Motor_Ser_Current[1]);
    }
    
    if(time_tick %2 == 1)
    {   
        send_protocol_New(Gimbal.Yaw_Angle_Fdb,Gimbal.Pitch_Angle_Fdb,
        Gimbal.CH040_Data.Roll_Angle,Shooter.Shooter_Speed_Kalman.X_hat,USART_Gimbal_Data.robot_id,USART2_DMA_TX_BUF);
        
    }
        
  

    
    if(time_tick%1000 == 5)
    {
        if(My_Auto_Shoot.Online_Flag == 0)//掉线视觉全部清零
        {
            My_Auto_Shoot.Auto_Aim.Flag_Get_Target = 0;
			My_Auto_Shoot.Auto_Aim.Yaw_Angle = 0;
			My_Auto_Shoot.Auto_Aim.Pitch_Angle = 0;
			My_Auto_Shoot.Auto_Aim.Enable_Shoot=0;
        }
    }
    
}


/**
************************************************************************************************************************
* @Name     : Control_Task_Init
* @brief    : PID参数初始化
* @param	: void
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Control_Task_Init(void)
{
    PID_Init(&Gimbal.Pitch_Motor_Angle_PID,PID_POSITION,40,0,0,10000,0);
    PID_Init(&Gimbal.Pitch_Motor_Speed_PID,PID_POSITION,100,0.5,0,20000,10000);
    
    PID_Init(&Gimbal.Yaw_Motor_Angle_PID,PID_POSITION,15,0,0,10000,0);//15 0 0 100000 0
    PID_Init(&Gimbal.Yaw_Motor_Speed_PID,PID_POSITION,0.015,0.0003,0,10,5);
    
//    PID_Init(&Gimbal.Yaw_Motor_Init_Speed_PID,PID_POSITION,0.5,0.007,0,10,4);8,0.02,0,100,50
 //   PID_Init(&Gimbal.Yaw_Motor_Angle_PID,PID_POSITION,0.05,0,0,10,4);
    
    PID_Init(&Shooter.Poke_Angle_PID,PID_POSITION,140,0,2000,20000,0);
    PID_Init(&Shooter.Poke_Speed_PID,PID_POSITION,0.04,0.0015,0,2048,512);
    
    PID_Init(&Shooter.Fric_Speed_PID[0],PID_POSITION,3.8,0,0,15000,5000);
    PID_Init(&Shooter.Fric_Speed_PID[1],PID_POSITION,3.8,0,0,15000,5000);
    
    PID_Init(&Gimbal.Yaw_Motor_Init_Speed_PID,PID_POSITION,0.5,0.007,0,10,4);
    PID_Init(&Gimbal.Yaw_Motor_Init_Angle_PID,PID_POSITION,40,0,0,100,0);
    
//    PID_Init(&Gimbal.Yaw_Motor_Init_Speed_PID,PID_POSITION,0.5,0.007,0,10,4);
//    PID_Init(&Gimbal.Yaw_Motor_Init_Angle_PID,PID_POSITION,10,0.02,0,100,50);
    
    
    PID_Init(&Gimbal.Auto_Shoot_Pitch_Angle_PID,PID_POSITION,40,0,0,10000,0);
    PID_Init(&Gimbal.Auto_Shoot_Pitch_Speed_PID,PID_POSITION,100,0.5,0,20000,10000);
    
    PID_Init(&Gimbal.Auto_Shoot_Yaw_Angle_PID,PID_POSITION,25,0,0,10000,0);
    PID_Init(&Gimbal.Auto_Shoot_Yaw_Speed_PID,PID_POSITION,0.015,0.0003,0,10,5);
}



/**
************************************************************************************************************************
* @Name     : Chassis_Mode_Select
* @brief    : 底盘模式更新
* @param	: void
* @retval   : void
* @Note     : 白控灰控都有，白控功能最不全，灰控遥控功能少一点，键鼠最全
************************************************************************************************************************
**/
void Chassis_Mode_Select(void)
{
    static uint16_t rorate_reserve_cnt = 0;//反转小陀螺状态用
    static uint8_t rotate_mode_switch_flag = 0;//切换模式，小陀螺
    static uint8_t jump_up_mode_flag = 0;//切换模式，跳上台阶
    static uint8_t anti_fly_slope_mode_flag = 0;//切换模式，反飞
    static uint8_t jump_down_mode_flag = 0;//切换模式，跳下台阶
    
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
            rotate_mode_switch_flag = 0;
            jump_up_mode_flag = 0;
            anti_fly_slope_mode_flag = 0;
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)//使用键鼠
        {
            if(Remote_VTM.key.Key_B_Action.Short_Press_Flag == 1)//短按B转陀螺
            {
                if(rotate_mode_switch_flag == 0)
                {
                    rotate_mode_switch_flag = 1;
                    rorate_reserve_cnt++;
                }
                else if(rotate_mode_switch_flag == 1)
                {
                    rotate_mode_switch_flag = 0;
                }
            }
            
            if(Remote_VTM.key.Key_E_Action.Short_Press_Flag == 1)//短按E跳上台阶
            {
                jump_up_mode_flag = 1;
            }
            else if(Remote_VTM.key.Key_E_Action.Long_Press_Flag == 1)//长按E取消
            {
                jump_up_mode_flag = 0;
            }
            else if(Remote_VTM.key.Key_E_Action.Original_Press_Flag == 0 && USART_Gimbal_Data.Jump_Finish_Flag == 1)//跳完清标志位
            {
                jump_up_mode_flag = 0;
            }
            
            
            if(Remote_VTM.key.Key_F_Action.Short_Press_Flag == 1)//短按F反飞坡
            {
                anti_fly_slope_mode_flag = 1;
            }
            else if(Remote_VTM.key.Key_F_Action.Long_Press_Flag == 1)
            {
                anti_fly_slope_mode_flag = 0;
            }
            else if(Remote_VTM.key.Key_F_Action.Original_Press_Flag == 0 && USART_Gimbal_Data.Jump_Finish_Flag == 1)
            {
                anti_fly_slope_mode_flag = 0;
            }
            
            if(fabs(Remote_VTM.Remote_mouse.z) != 0)
            {
                anti_fly_slope_mode_flag = 0;
                jump_up_mode_flag = 0;
                jump_down_mode_flag = 0;
                rotate_mode_switch_flag = 0;
            }
            
//            if(Remote_VTM.key.Key_F_Action.Short_Press_Flag == 1)//双击G跳下台阶
//            {
//                jump_down_mode_flag ++
//            }
//            
//            if(jump_down_mode_flag == 1)//滚轮取消
//            {
//                if(Remote_VTM.Remote_mouse.z < 0)
//                {
//                    jump_down_mode_flag = 0;
//                }
//            }
//            
//            if(Remote_VTM.key.Key_F_Action.Original_Press_Flag == 0 && USART_Gimbal_Data.remain_heat == 1)
//            {
//                jump_down_mode_flag = 0;
//            }
//            
            
            if(rotate_mode_switch_flag == 1)//按B小陀螺
            {
                if(rorate_reserve_cnt%2 == 0)
                {
                    if(Remote_VTM.key.Key_SHIFT_Action.Original_Press_Flag == 1)
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE_VAR_SPEED;
                    }
                    else
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
                    }
                }
                else if(rorate_reserve_cnt%2 == 1)
                {
                    if(Remote_VTM.key.Key_SHIFT_Action.Original_Press_Flag == 1)
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE_VAR_SPEED;
                    }
                    else
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
                    }
                }
            }
            else if(jump_up_mode_flag == 1)
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_JUMP_UP ;
            }
            else if(jump_down_mode_flag == 1)
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_JUMP_DOWN ;
            }
            else if(anti_fly_slope_mode_flag == 1)
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_FLY_SLOPE ;
            }
            else
            {
                USART_Chassis_Data.Chassis_Mode = MANUAL_FOLLOW_REMOTE;
            }
            
            if(USART_Gimbal_Data.current_HP == 0)//死了一定RELAX
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_RELAX;
                rotate_mode_switch_flag = 0;
                jump_up_mode_flag = 0;
                jump_down_mode_flag = 0;
                anti_fly_slope_mode_flag = 0;
            }
            USART_Chassis_Data.fn_2_trigger_flag = Remote_VTM.Remote_clicker.fn2_Action.Toggle_Press_Flag ;
        }
        else if(Remote_VTM.Remote_clicker.Switch == LEFT)//使用遥控
        {
            if(Remote_VTM.Remote_clicker.Trigger_Action.Short_Press_Flag == 1)//短按trigger转陀螺
            {
                if(rotate_mode_switch_flag == 0)
                {
                    rotate_mode_switch_flag = 1;
                    rorate_reserve_cnt++;
                }
                else if(rotate_mode_switch_flag == 1)
                {
                    rotate_mode_switch_flag = 0;
                }
            }
            
            if(Remote_VTM.Remote_clicker.Pause_Action.Short_Press_Flag  == 1)
            {
                jump_up_mode_flag = 1;
            }
            else if(Remote_VTM.Remote_clicker.Pause_Action.Long_Press_Flag == 1)//长按Pause取消
            {
                jump_up_mode_flag = 0;
            }
            else if(Remote_VTM.Remote_clicker.Pause_Action.Short_Press_Flag == 0 && USART_Gimbal_Data.Jump_Finish_Flag == 1)//跳完清标志位
            {
                jump_up_mode_flag = 0;
            }
            
            
            if(rotate_mode_switch_flag == 1)//小陀螺
            {
                if(rorate_reserve_cnt%2 == 0)
                {
                    if(Remote_VTM.Remote_clicker.ch2 == 660)
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE_VAR_SPEED ;
                    }
                    else
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_CLOCKWISE_ROTATE;
                    }
                }
                else if(rorate_reserve_cnt%2 == 1)
                {
                    if(Remote_VTM.Remote_clicker.ch2 == 660)
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE_VAR_SPEED ;
                    }
                    else
                    {
                        USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_CLOCKWISE_ROTATE;
                    }
                }
            }
            else if(jump_up_mode_flag == 1)//跳上台阶
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_JUMP_UP ;
            }
            else if(anti_fly_slope_mode_flag == 1)
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_ANTI_FLY_SLOPE ;
            }
            else if(jump_down_mode_flag == 1)
            {
                USART_Chassis_Data.Chassis_Mode = CHASSIS_JUMP_DOWN ;
            }
//            else if(Shooter.Shooter_Mode != SHOOTER_RELAX)
//            {
//                USART_Chassis_Data.Chassis_Mode = CHASSIS_SIT_DOWN;
//            }
            else
            {
                USART_Chassis_Data.Chassis_Mode = MANUAL_FOLLOW_REMOTE;
            }
            USART_Chassis_Data.fn_2_trigger_flag = 0;
        }
    }
    else//如果灰控和白控都在或都不在
    {
        USART_Chassis_Data.Chassis_Mode = CHASSIS_RELAX;
        rotate_mode_switch_flag = 0;
        USART_Chassis_Data.fn_2_trigger_flag = 0;
        rorate_reserve_cnt = 0;
        jump_up_mode_flag = 0;
        anti_fly_slope_mode_flag = 0;
    }
}


/**
************************************************************************************************************************
* @Name     : Chassis_Reference_Update
* @brief    : 底盘参考值更新
* @param	: void
* @retval   : void
* @Note     : 白控灰控都有，白控功能最不全，灰控遥控功能少一点，键鼠最全
************************************************************************************************************************
**/
void Chassis_Reference_Update(void)
{
    if(Remote_DT7_data.online_flag == 1 && Remote_VTM.online_flag == 0)//用白控
    {
        if(Remote_DT7_data.Remote_clicker.s1 == MIDDLE || Remote_DT7_data.Remote_clicker.s1 == UP)
        {
            USART_Chassis_Data.V_y = Remote_DT7_data.Remote_clicker.ch1/660.0f*2.5f;
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
        if(Remote_VTM.Remote_clicker.Switch == LEFT)//遥控
        {
            USART_Chassis_Data.V_y = Remote_VTM.Remote_clicker.ch1/660.0f*2.5f;
            if(Shooter.Shooter_Mode == SHOOTER_RELAX)//用控时拨轮同时控制打弹和变腿长，所以不打弹时可变腿长
            {
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
        }
        else if(Remote_VTM.Remote_clicker.Switch == CENTER)//键鼠
        {
            if(Remote_VTM.key.Key_SHIFT_Action.Original_Press_Flag == 1)//底盘速度给定
            {
                USART_Chassis_Data.V_y = (Remote_VTM.key.Key_W_Action.Original_Press_Flag - Remote_VTM.key.Key_S_Action.Original_Press_Flag)*2.5f;//只给V_y,按A D侧45°
                USART_Chassis_Data.V_x = 0.0f;
            }
            else
            {
                USART_Chassis_Data.V_y = (Remote_VTM.key.Key_W_Action.Original_Press_Flag - Remote_VTM.key.Key_S_Action.Original_Press_Flag)*2.2f;
                USART_Chassis_Data.V_x = 0.0f;
            }
           
            
            if(Remote_VTM.key.Key_Z_Action.Original_Press_Flag  == 1)//长按Z中腿长
            {
                USART_Chassis_Data.Cmd_Leg_Length = 2;
            }
            else if(Remote_VTM.key.Key_CTRL_Action.Original_Press_Flag == 1)//长按CTRL高腿长
            {
                USART_Chassis_Data.Cmd_Leg_Length = 3;
            }
            else//什么不按低腿长
            {
                USART_Chassis_Data.Cmd_Leg_Length = 1;
            }
            
            
            if(Remote_VTM.Remote_clicker.fn2_Action.Toggle_Press_Flag == 1)
            {
                USART_Chassis_Data.leg_single_angle_handle_left = Remote_VTM.Remote_clicker.ch3 * 0.0015f;
                USART_Chassis_Data.leg_single_angle_handle_right = Remote_VTM.Remote_clicker.ch1 * 0.0015f;
            }
            else
            {
                USART_Chassis_Data.leg_single_angle_handle_left = 0;
                USART_Chassis_Data.leg_single_angle_handle_right = 0;
            }
        }
    }
    else//灰控白控都不在
    {
        USART_Chassis_Data.V_y = 0;
        USART_Chassis_Data.V_x = 0;
        USART_Chassis_Data.Cmd_Leg_Length = 1;
        USART_Chassis_Data.leg_single_angle_handle_left = 0;
        USART_Chassis_Data.leg_single_angle_handle_right = 0;
    }
}


/**
************************************************************************************************************************
* @Name     : Chassis_Task
* @brief    : 底盘任务
* @param	: void
* @retval   : void
* @Note     : 底盘模式选择与底盘参考值更新
************************************************************************************************************************
**/
void Chassis_Task(void)
{
    Chassis_Mode_Select();
    Chassis_Reference_Update();
}
