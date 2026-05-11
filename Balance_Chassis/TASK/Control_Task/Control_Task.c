#include "main.h"
uint32_t time_tick = 0;



void Contorl_Task(Balance_Chassis_t* Chassis)
{
    time_tick++;
        Chassis->Left_Leg.spring_FN = Get_Left_GasSpring_FN(Chassis->Left_Leg.l0);
    Chassis->Right_Leg.spring_FN = Get_Right_GasSpring_FN(Chassis->Right_Leg.l0);
    
//    //控动作判断
    Remote_Switch_Action_Detect(&Remote_DT7_data);
    Key_Mouse_State_Update(&Remote_DT7_data);
    Remote_DT7_To_USART_Chassis_Data(&Remote_DT7_data,&Chassis->USART_Chassis_Data);
    
    //驱动轮在线检测
    Motor_Online_Detective(&Chassis->Driving_Motor[0]);
    Motor_Online_Detective(&Chassis->Driving_Motor[1]);
    
    
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
//    Chassis->Left_Acc = Chassis->Left_Acc * 0.60f + difference_left_calc(Chassis->Driving_Motor[0].Omega_Rad_fdb,0.001)*0.40f;
//    Chassis->Right_Acc = Chassis->Right_Acc * 0.60f + difference_left_calc(Chassis->Driving_Motor[1].Omega_Rad_fdb,0.001)*0.40f;
//    
//    
    //底盘控制
    if(time_tick%2==0)
    {
        Chassis_Task(Chassis);
        CAN1_Send_Task_1(Chassis->joint_T[0],Chassis->joint_T[3]);
    }
    
    if(time_tick%2==1)
    {
       CAN1_Send_Task_2(Chassis->joint_T[1],Chassis->joint_T[2]);
    
       CAN2_Send_Task(Chassis->driving_T[0],Chassis->driving_T[1]);
    }
    
    if(time_tick%5==0)
    {
        usart_gimbal_send(0,0,0,0,0,0,0,0,0,0,0,0,&Chassis->USART_Gimbal_Data);
    }
}



void Control_Task_Init(Balance_Chassis_t* Chassis)
{
    Chassis_Param_Init(Chassis);
}


