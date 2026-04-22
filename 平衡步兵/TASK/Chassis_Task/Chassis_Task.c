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


//力矩限幅
void Motor_Out_Limit(Balance_Chassis_t* Chassis)
{
    VAL_LIMIT(Chassis->joint_T[1],-JOINT_MAX_T,JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[2], -JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->driving_T[0], -WHEEL_MAX_T, WHEEL_MAX_T);

    VAL_LIMIT(Chassis->joint_T[0], -JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[3], -JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->driving_T[1], -WHEEL_MAX_T, WHEEL_MAX_T);
}


void Motor_Torque_Set(Balance_Chassis_t* Chassis,float Joint_T_0,float Joint_T_1,float Joint_T_2,float Joint_T_3,float Driving_T_1,float Driving_T_2)
{
    //左
    Chassis->joint_T[1] = Joint_T_1;//前  //老车注释，不一定对
    Chassis->joint_T[2] = Joint_T_2;
    Chassis->driving_T[0] = Driving_T_1;
    //右
    Chassis->joint_T[0] = Joint_T_0;//前
    Chassis->joint_T[3] = Joint_T_3;
    Chassis->driving_T[1] = Driving_T_2;
}



void Init_Tp_Calc(float Ref_Leglength,float Harmonize,float Init_Tp,Balance_Chassis_t* Chassis)
{
    Chassis->Chassis_Ref.Leglength  = Ref_Leglength; // 期望腿长
    // 腿部竖直力F的计算
    Chassis->Left_Leg.Leg_F = PID_Calc(&Chassis->Left_Leg.Leg_Length_PID, Chassis->Left_Leg.l0, Chassis->Chassis_Ref.Leglength);
    //还得加入气弹簧拟合7878
    Chassis->Right_Leg.Leg_F = PID_Calc(&Chassis->Right_Leg.Leg_Length_PID, Chassis->Right_Leg.l0, Chassis->Chassis_Ref.Leglength);

    leg_conv(Chassis->Left_Leg.Leg_F, Init_Tp - Harmonize, Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);//正负号不知道对不对
    leg_conv(Chassis->Right_Leg.Leg_F, Init_Tp + Harmonize, Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);

}



void Chassis_Param_Init(Balance_Chassis_t* Chassis)
{
    memset(Chassis,0,sizeof(*Chassis));//清零底盘结构体
    Chassis->Chassis_Remote_Ref.Leglength = 0.25;
    
    //初始化力矩
    PID_Init(&Chassis->Init_Tp_Pid,PID_POSITION,250,0,10,500,200);
    
    //左腿腿长
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,90,0,50,20000,20000);
    
    //右腿腿长
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,90,0,50,20000,20000);
    
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
    //底盘各数据获取
    
//    leg_spd(Chassis->Joint_Motor[0].Angular_Vel_fdb , Chassis->Joint_Motor[3].Angular_Vel_fdb , 
//    Chassis->Driving_Motor[0].Single_Angle_fdb , Chassis->Joint_Motor[3].Single_Angle_fdb , 
//    &Chassis->Right_Leg);//求得右腿速度
//    
//    leg_spd(Chassis->Joint_Motor[1].Angular_Vel_fdb , Chassis->Joint_Motor[2].Angular_Vel_fdb , 
//    Chassis->Driving_Motor[1].Single_Angle_fdb , Chassis->Joint_Motor[2].Single_Angle_fdb , 
//    &Chassis->Left_Leg);//求得左腿速度
//    
//    leg_pos(Chassis->Driving_Motor[0].Single_Angle_fdb , Chassis->Joint_Motor[3].Single_Angle_fdb , &Chassis->Right_Leg.l0 , &Chassis->Right_Leg.phi0);//求得右腿位置
//    leg_pos(Chassis->Driving_Motor[1].Single_Angle_fdb , Chassis->Joint_Motor[2].Single_Angle_fdb , &Chassis->Left_Leg.l0 , &Chassis->Left_Leg.phi0);//求得左腿位置
    VMC_Data_Get(&Chassis->Right_Leg,Chassis->Joint_Motor[3].Speed_fdb,Chassis->Joint_Motor[0].Speed_fdb,
    Chassis->Joint_Motor[3].Single_Angle_fdb*PI/180.0f,Chassis->Joint_Motor[0].Single_Angle_fdb*PI/180.0f);//求得右腿状态
    VMC_Data_Get(&Chassis->Left_Leg,Chassis->Joint_Motor[2].Speed_fdb,Chassis->Joint_Motor[1].Speed_fdb,
    Chassis->Joint_Motor[2].Single_Angle_fdb*PI/180.0f,Chassis->Joint_Motor[1].Single_Angle_fdb*PI/180.0f);//求得左腿状态 //极性和角度没调7878解算之后再改
    //对dphi0出现NUN的情况进行的处理
    if(isnan(Chassis->Left_Leg.dphi0 - Chassis->Right_Leg.dphi0))
    {
        Chassis->Right_Leg.dphi0 = 0.0f;
        Chassis->Left_Leg.dphi0 = 0.0f;
    }
    
    Chassis->dphi0 = (Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.phi0)/2.0f;
    Chassis->phi0 = (Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f;
    
    Chassis->dtheta = ((Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.dphi0)/2.0f - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*PI/180.0f);
    if(isnan(Chassis->dtheta) || isinf(Chassis->dtheta))
    {
        Chassis->dphi0 = 0.0f;
    }
    
    
    
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
     
    //控制量获取
    if(Chassis->Control_Mode != CHASSIS_INIT)//非Init控制
    {
        //
        Chassis->Chassis_Remote_Ref.V_y = Chassis->USART_Chassis_Data.V_y ;
        Chassis->Chassis_Remote_Ref.V_w = Chassis->USART_Chassis_Data.Omega ;
//        Chassis->Chassis_Remote_Ref.Roll = Chassis->USART_Chassis_Data.Roll ;
//        Chassis->Chassis_Remote_Ref.V_x = Chassis->USART_Chassis_Data.V_x ;
        //速度限幅
        VAL_LIMIT(Chassis->Chassis_Remote_Ref.V_y ,Chassis->Min_Speed ,Chassis->Max_Speed);
        VAL_LIMIT(Chassis->Chassis_Remote_Ref.V_x ,-1.2f,1.2f);
        
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
    
    Chassis->Left_Leg.Leg_FN = 100;
    Chassis->Right_Leg.Leg_FN = 100;
}





void Chassis_Init_State_Update(Balance_Chassis_t* Chassis)
{
    //
    //这里还要清零跳跃相关，暂时不跳
    //
    
    PID_Init(&Chassis->Init_Tp_Pid,PID_POSITION,180,0,10,500,200);
    
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,85,0,0,20000,20000);

    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,85,0,0,20000,20000);

    
    Chassis->Chassis_Ref.V_y = 0;
    Chassis->Chassis_Ref.V_x = 0;
    Chassis->Chassis_Ref.V_w = 0;
    
//    float Left_Leg_phi1  = Normalize_Angle_PI(Chassis->Left_Leg.phi1);//先不使用老代码的局部变量
//    float Right_Leg_phi1 = Normalize_Angle_PI(Chassis->Left_Leg.phi1);
//    float phi0_0_To_2PI_Left = Transform_Angle_0_2PI(Chassis->Left_Leg.phi0 )
 

    if((Chassis->Left_Leg.phi1<1.1f && Chassis->Left_Leg.phi1>-2.5f && Chassis->Left_Leg.l0 > 0.28f && fabs(Chassis->Chassis_GYRO.Pitch_Angle)<30 && fabs(Chassis->Chassis_GYRO.Roll_Angle)<95) ||
       (Chassis->Right_Leg.phi1<1.1f && Chassis->Right_Leg.phi1>-2.5f && Chassis->Right_Leg.l0 >0.28f && fabs(Chassis->Chassis_GYRO.Pitch_Angle)<30 && fabs(Chassis->Chassis_GYRO.Roll_Angle)<95) )
    {//老代码里的侧翻
        Chassis->Init_State = RELAX_STATE;//先复刻老代码，其他姿势全relax
    }
    else if((fabs(Chassis->Chassis_GYRO.Pitch_Angle) > 45 && fabs(Chassis->Chassis_GYRO.Pitch_Angle)<100) ||fabs(Chassis->Chassis_GYRO.Roll_Angle)>95)//老代码里的倒翻
    {
        Chassis->Init_State = RELAX_STATE;
    }
    else
    {
        Chassis->Init_State = NORMOL_STATE;
    }
}






void Chassis_Init_Handle(Balance_Chassis_t* Chassis)
{
    switch (Chassis->Init_State) //这里注意老代码里phi0减过1.57，我这里没减过
    {
        case NORMOL_STATE:
            if(fabs(Chassis->Left_Leg.l0 - Chassis->Right_Leg.l0) > 0.08f)//两条腿一长一短，表示一条在车下，一条在车外
            {
                if(Chassis->Left_Leg.l0 > Chassis->Right_Leg.l0)//右腿在车下,先调整姿势，抽出右腿
                {
                    Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_Pid , Chassis->Right_Leg.phi0 , Chassis->Left_Leg.phi0);
                    Init_Tp_Calc(Chassis->Left_Leg.l0,0,Chassis->Init_Tp,Chassis);
                    Motor_Torque_Set(Chassis,Chassis->Right_Leg.T_Set[0],0,0,Chassis->Right_Leg.T_Set[1],0,0);//转矩赋值都还没调极性7878
                    Motor_Out_Limit(Chassis);
                }
                else if(Chassis->Right_Leg.l0 > Chassis->Left_Leg.l0)//左腿在车下，先调整姿势，抽出左腿
                {
                    Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_Pid , Chassis->Left_Leg.phi0 , Chassis->Right_Leg.phi0);
                    Init_Tp_Calc(Chassis->Right_Leg.l0,0,Chassis->Init_Tp,Chassis);
                    Motor_Torque_Set(Chassis,0,Chassis->Left_Leg.T_Set[0],Chassis->Left_Leg.T_Set[1],0,0,0);
                    Motor_Out_Limit(Chassis);
                }
            }
            else if((fabs(Chassis->phi0-1.57f) >= 3 * PI/180) && fabs(Chassis->Right_Leg.l0-Chassis->Left_Leg.l0)<0.08) //腿摆角偏离竖直方向 且 双腿腿长差距小  正常姿势初始化
            {
                Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_Pid,Chassis->phi0,1.57f);
                Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer , (Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0),0);//不知道这个正负号对不对7878
                Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner ,(Chassis->Right_Leg.dphi0 - Chassis->Right_Leg.dphi0),Chassis->Harmonize_Outer);
                Init_Tp_Calc(0.14f,Chassis->Harmonize_Inner,Chassis->Init_Tp,Chassis);
                Motor_Torque_Set(Chassis,Chassis->Right_Leg.T_Set[0],Chassis->Left_Leg.T_Set[0],Chassis->Left_Leg.T_Set[1],Chassis->Right_Leg.T_Set[1],0,0);
                Motor_Out_Limit(Chassis);
            }
            else
            {
                Chassis->Control_Mode = CHASSIS_STAND_MODE;
            }
            break;
        case RELAX_STATE:
            Chassis_Relax_Handle(Chassis);//Chassis_Relax_Handle本是用于底盘控制模式，这里用应该能防止其他姿势的初始化疯车，后续删
            break;
        default :
            break;
            
    }
}



void Balance_Task(Balance_Chassis_t* Chassis)
{
    //balance_loop数据获取
    if(Chassis->Control_Mode == CHASSIS_ROTATE)//小陀螺补偿phi0
    {
        Chassis->balance_loop.phi = (Chassis->Chassis_GYRO.Pitch_Angle+0.5f)*PI/180.0f;
    }
    else
    {
        Chassis->balance_loop.phi = Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f;
    }
    
    Chassis->balance_loop.dphi = Chassis->Chassis_GYRO.Pitch_Gyro_Omega*PI/180.0f;
    Chassis->balance_loop.x = ((LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Multi_Angle_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Multi_Angle_fdb)/2.0f) * WHEEL_R;
    //Chassis->balance_loop.dx = (LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Speed_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Speed_fdb) * WHEEL_R ;//1.没加减速比，2.最后是要卡尔曼滤波的 7878
    Chassis->balance_loop.theta = ((Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f - 1.57f)-Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f;
    Chassis->balance_loop.dtheta = ((Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.dphi0)/2.0f - Chassis->Chassis_GYRO.Pitch_Gyro_Omega * PI/180.0f);
    
    Chassis->Left_theta = Chassis->Left_Leg.phi0 - 1.57f - Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f;
    Chassis->Right_theta = Chassis->Right_Leg.phi0 - 1.57f -Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f;
}



void Chassis_Control_Loop(Balance_Chassis_t* Chassis)
{
    switch (Chassis->Control_Mode)
    {
        CHASSIS_RELAX :
            Chassis_Relax_Handle(Chassis);
            break;
        CHASSIS_INIT :
            Chassis_Init_State_Update(Chassis);
            Chassis_Init_Handle(Chassis);
            break;
        CHASSIS_STAND_MODE :
            
        default:
            break;
            
    }
}










