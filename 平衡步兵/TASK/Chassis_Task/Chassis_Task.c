#include "main.h"

Balance_Chassis_t Chassis;


//调整角度至-PI~PI，并且舍弃非正常数据
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



float Transform_Angle_0_2PI(float angle)
{
    float new_angle=fmod(angle+2*PI,2*PI);
    {
        return (new_angle<0)?new_angle+2*PI:new_angle;
    }
}



void Chassis_Param_Init(Balance_Chassis_t* Chassis)
{
    memset(Chassis,0,sizeof(*Chassis));//清零底盘结构体
    Chassis->Chassis_Remote_Ref.Leglength = 0.25;
    
    //初始化力矩
    PID_Init(&Chassis->Init_Tp_Pid,PID_POSITION,250,0,10,500,200);
    
    //左腿腿长
    PID_Init(&Chassis->Left_Leg.leglengthpid_inner,PID_POSITION,90,0,50,20000,20000);
    PID_Init(&Chassis->Left_Leg.leglengthpid_outer,PID_POSITION,30,0,0,3000,20000);
    
    //右腿腿长
    PID_Init(&Chassis->Right_Leg.leglengthpid_inner,PID_POSITION,90,0,50,20000,20000);
    PID_Init(&Chassis->Right_Leg.leglengthpid_outer,PID_POSITION,30,0,0,3000,20000);
    
    //双腿协调
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner,PID_POSITION,9.3,0,1.0,35,3);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer,PID_POSITION,35,0,0.8,50,3);
    
    //roll平衡
    PID_Init(&Chassis->Roll_Pid_Angle,PID_POSITION,0.008,0,0.01,1,1);
    
    //小陀螺roll平衡
//    PID_Init(&Chassis->Roll_leg_F_Rotate_Pid,PID_POSITION,2,0,1,20,10);
    
    //普通模式
    PID_Init(&Chassis->Roll_Leg_F_Pid,PID_POSITION,35,0.01,10,600,10);
    
    PID_Init(&Chassis->V_w_Pid,PID_POSITION,3.5,0,0,5,5);
    
    //底盘跟随云台pid
    PID_Init(&Chassis->Pid_Follow_Gimbal,PID_POSITION,8,0,1,3,200);
    
    //先写这些，其他的再说
}


void Chassis_State_Update(Balance_Chassis_t* Chassis)
{
    /**************这里缺少加速度的计算以及打滑检测******************/
    
    
    /****************************************************************/
    leg_spd(Chassis->Joint_Motor[0].Angular_Vel_fdb , Chassis->Joint_Motor[3].Angular_Vel_fdb , 
    Chassis->Driving_Motor[0].Single_Angle_fdb , Chassis->Joint_Motor[3].Single_Angle_fdb , 
    &Chassis->Right_Leg);//求得右腿速度
    
    leg_spd(Chassis->Joint_Motor[1].Angular_Vel_fdb , Chassis->Joint_Motor[2].Angular_Vel_fdb , 
    Chassis->Driving_Motor[1].Single_Angle_fdb , Chassis->Joint_Motor[2].Single_Angle_fdb , 
    &Chassis->Left_Leg);//求得左腿速度
    
    leg_pos(Chassis->Driving_Motor[0].Single_Angle_fdb , Chassis->Joint_Motor[3].Single_Angle_fdb , &Chassis->Right_Leg.l0 , &Chassis->Right_Leg.phi0);//求得右腿位置
    leg_pos(Chassis->Driving_Motor[1].Single_Angle_fdb , Chassis->Joint_Motor[2].Single_Angle_fdb , &Chassis->Left_Leg.l0 , &Chassis->Left_Leg.phi0);//求得左腿位置
    
    Chassis->Chassis_Remote_Ref.V_x = Chassis->USART_Chassis_Data.V_x ;//暂时只有一个速度和角速度
    Chassis->Chassis_Remote_Ref.V_w = Chassis->USART_Chassis_Data.Omega ;
    
    //模式切换判断
    if( (Chassis->Control_Mode != CHASSIS_INIT && Chassis->Control_Mode != CHASSIS_STAND_MODE) || (Chassis->USART_Chassis_Data.Chassis_Mode == 0) )//正常进行切换
    {
       Chassis->Control_Mode = (Chassis_Mode_e)Chassis->USART_Chassis_Data.Chassis_Mode;
    }
    
    if(judge_rece_mesg.game_robot_state.power_management_chassis_output==0||judge_rece_mesg.game_robot_state.current_HP==0)
    {
        Chassis->Control_Mode = CHASSIS_RELAX ;
    }
    
    if(Chassis->Last_Control_Mode == CHASSIS_RELAX && Chassis->Control_Mode != CHASSIS_RELAX)//空闲之后必衔接初始化
    {
        Chassis->Control_Mode = CHASSIS_INIT ;
    }
    
    if( ( (Chassis->Control_Mode == CHASSIS_ROTATE||Chassis->Control_Mode == MANUAL_FOLLOW_REMOTE) && (fabs(Chassis->Chassis_GYRO.Pitch_Angle)>15) ) )//抬头太多进初始化，之后还要改的
    {
        Chassis->Control_Mode = CHASSIS_INIT ;
    }
     
    
    Chassis->Last_Control_Mode = Chassis->Control_Mode;
}



void Chassis_Relax_Handle(Balance_Chassis_t* Chassis)
{   
    //置零关节输出
    Chassis->joint_T[0] = 0;
    Chassis->joint_T[1] = 0;
    Chassis->joint_T[2] = 0;
    Chassis->joint_T[3] = 0;
    Chassis->driving_T[0] = 0;
    Chassis->driving_T[0] = 0;
    
    //置零Tp
    Chassis->Balance_Tpgain = 0;
    Chassis->Balance_Tpoutlandgain = 0;
    
    Chassis->Chassis_Ref.Pitch = 0;
    //Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x;
    Chassis->Roll_Pid_Angle.Iout = 0;
    Chassis->Chassis_Ref.Roll = 0;
    
    Chassis->Init_State = (Init_State_e)0;
    Chassis->rotate_flag = 0;//后续会改成枚举 7878
    
    Chassis->Left_Leg.leg_FN = 100;
    Chassis->Right_Leg.leg_FN = 100;
}

void Chassis_Init_State_Update(Balance_Chassis_t* Chassis)
{
    //
    //这里还要清零跳跃相关，暂时不跳
    //
    
    PID_Init(&Chassis->Init_Tp_Pid,PID_POSITION,180,0,10,500,200);
    
    PID_Init(&Chassis->Left_Leg.leglengthpid_inner,PID_POSITION,85,0,0,20000,20000);
    PID_Init(&Chassis->Left_Leg.leglengthpid_outer,PID_POSITION,25,0,0,20000,20000);
    
    PID_Init(&Chassis->Right_Leg.leglengthpid_inner,PID_POSITION,85,0,0,20000,20000);
    PID_Init(&Chassis->Right_Leg.leglengthpid_outer,PID_POSITION,25,0,0,20000,20000);
    
    Chassis->Chassis_Ref.V_y = 0;
    Chassis->Chassis_Ref.V_x = 0;
    Chassis->Chassis_Ref.V_w = 0;
    
//    float Left_Leg_phi1  = Normalize_Angle_PI(Chassis->Left_Leg.phi1);
//    float Right_Leg_phi1 = Normalize_Angle_PI(Chassis->Left_Leg.phi1);
//    float phi0_0_To_2PI_Left = Transform_Angle_0_2PI(Chassis->Left_Leg.phi0 )
    Chassis->balance_loop.theta = ((((Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f) - 1.57f) - Chassis->Chassis_GYRO.Pitch_Angle * PI /180.0f);
    Chassis->balance_loop.L0 = (Chassis->Left_Leg.l0 + Chassis->Right_Leg.l0)/2.0f;
    //对dphi0出现NUN的情况进行的处理
    if(isnan(Chassis->Left_Leg.dphi0 - Chassis->Right_Leg.dphi0))
    {
        Chassis->Right_Leg.dphi0 = 0.0f;
        Chassis->Left_Leg.dphi0 = 0.0f;
    }
    
    
    
    
}

void Balance_Task(Balance_Chassis_t* Chassis)
{
    
}
void Chassis_Control_Loop(Balance_Chassis_t* Chassis)
{
    switch (Chassis->Control_Mode)
    {
        CHASSIS_RELAX :
            Chassis_Relax_Handle(Chassis);
            break;
        CHASSIS_INIT :
            break;
        default:
            break;
            
    }
}










