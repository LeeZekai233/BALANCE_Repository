#include "main.h"
uint32_t time_tick = 0;



void Contorl_Task(Balance_Chassis_t* Chassis)
{
    time_tick++;
    
    //驱动轮在线检测
    Motor_Online_Detective(&Chassis->Driving_Motor[0]);
    Motor_Online_Detective(&Chassis->Driving_Motor[1]);
    
    
    //测距检测
    TF02_Online_Handle(&Chassis->TF02);
    vl53l4cx_Online_flag_Get(&Chassis->vl53l4cx_Left,time_tick);
    vl53l4cx_Online_flag_Get(&Chassis->vl53l4cx_Middle,time_tick);
    vl53l4cx_Online_flag_Get(&Chassis->vl53l4cx_Right,time_tick);
    Chassis->Distance_mm = Get_Max_Distance(Chassis->vl53l4cx_Left.Distance_mm,Chassis->vl53l4cx_Right.Distance_mm);
    
    
    //里程和加速度的更新
    if(fabs(Chassis->Chassis_Ref.V_w) > 0.8f)
    {
        Mileage_kalman_filter_calc(&Mileage_kalman_filter, 
        ((LEFT_WHEEL_POLARITY*Chassis->Driving_Motor[0].Angle_Rad_Total_fdb + RIGHT_WHEEL_POLARITY*Chassis->Driving_Motor[1].Angle_Rad_Total_fdb)/2.0f)*WHEEL_R,
        ((LEFT_WHEEL_POLARITY*Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY*Chassis->Driving_Motor[1].Omega_Rad_fdb)/2.0f)*WHEEL_R,
        0);
    }
    else
    {
        Mileage_kalman_filter_calc(&Mileage_kalman_filter, 
        ((LEFT_WHEEL_POLARITY*Chassis->Driving_Motor[0].Angle_Rad_Total_fdb + RIGHT_WHEEL_POLARITY*Chassis->Driving_Motor[1].Angle_Rad_Total_fdb)/2.0f)*WHEEL_R,
        ((LEFT_WHEEL_POLARITY*Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY*Chassis->Driving_Motor[1].Omega_Rad_fdb)/2.0f)*WHEEL_R,
        Chassis->Chassis_GYRO.Y_Acc);
    }
    
    Chassis->Left_Acc = Chassis->Left_Acc * 0.60f + difference_left_calc(Chassis->Driving_Motor[0].Omega_Rad_fdb,0.001)*0.40f;
    Chassis->Right_Acc = Chassis->Right_Acc * 0.60f + difference_left_calc(Chassis->Driving_Motor[1].Omega_Rad_fdb,0.001)*0.40f;
    
    
    //底盘控制
    if(time_tick%2==0)
    {
        Chassis_Task(Chassis);
        CAN2_Send_Task_1(Chassis->joint_T[0],Chassis->joint_T[3]);
    }
    
    if(time_tick%2==1)
    {
        CAN2_Send_Task_2(Chassis->joint_T[1],Chassis->joint_T[2]);
        CAN1_Send_Task(Chassis->driving_T[0],Chassis->driving_T[1]);
    }

    if(time_tick%10 == 0)
    {
        CAN_POWER_Control(CAN1,&Super_Cap_Send);
    }
    if(time_tick%2==0)
    {
        usart_gimbal_send
        (judge_rece_mesg.power_heat_data.shooter_17mm_barrel_heat,
        judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit,
        judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value,
        judge_rece_mesg.game_robot_state.robot_level,
        0,//不用bullet_speed_x_hat
        judge_rece_mesg.shoot_data.initial_speed,
        judge_rece_mesg.game_robot_state.power_management_chassis_output,
        judge_rece_mesg.game_robot_state.current_HP,
        judge_rece_mesg.game_robot_state.robot_id,
        Chassis->Gimbal_Init_Cmd,
        Chassis->Jump_Finish_Flag,
        judge_rece_mesg.game_state.game_progress,
        &Chassis->USART_Gimbal_Data);
    }
    
    if(time_tick % 100 == 0)
    {
        Client_Send_Handle();
    }
}



void Control_Task_Init(Balance_Chassis_t* Chassis)
{
    Chassis_Param_Init(Chassis);
}


