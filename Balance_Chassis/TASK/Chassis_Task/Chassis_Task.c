#include "main.h"


Balance_Chassis_t Chassis;
float temp_tp;
uint8_t leglength_cmd_temp;
uint8_t control_mode_temp;

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



void Motor_Online_Detective(Encoder_t *Encoder)
{
    if((time_tick - Encoder->heart_cnt) > 100)
    {
        Encoder->online_flag = 0;
    }
    else
    {
         Encoder->online_flag = 1;
    }
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




/*********************支持力解算*******************///改过，之前theta用的是车平均theta，这里我换成单腿，不知道行不行
//定义矩阵
mat Jacobian,
    JacobianT,
    JacobinT_inv,
    mat_F,
    mat_T;

static float  Jacobian_data[4];
static float  JacobianT_data[4];
static float  JacobinT_inv_data[4];
static float  mat_F_data[2];
static float  mat_T_data[2];

void FN_calculate(CH040DATA_t* Chassis_GYRO, Leg_State_t* Leg_State, Lpf1stObj *ft,float MT1_torque,float MT4_torque)
{
    static float  last_dtheta;
    float costheta = arm_cos_f32((Leg_State->phi0 - 1.57f) - Chassis_GYRO->Pitch_Angle*DEG_TO_RAD);
    float sintheta = arm_sin_f32((Leg_State->phi0 - 1.57f) - Chassis_GYRO->Pitch_Angle*DEG_TO_RAD);

    Leg_State->ddtheta = (Leg_State->dtheta - last_dtheta) / ((TIME_STEP * 0.001));//ddzw的计算   差分
    float ddz = arm_cos_f32(Chassis_GYRO->Z_Acc * Chassis_GYRO->Pitch_Angle*DEG_TO_RAD);//机体加速度 ddz
    float ddzw = ddz - Leg_State->ddl0 * costheta + \
                  2 * Leg_State->dl0 * Leg_State->dtheta * sintheta + \
                    Leg_State->l0 * Leg_State->ddtheta * sintheta + \
                    Leg_State->l0 * (Leg_State->dtheta * Leg_State->dtheta) * costheta;
    Leg_State->ddzw = Lpf_1st_calcu(ft,ddzw,5,0.002);// 计算一阶低通滤波器的输出值，并返回
    //P和Tp的计算
    mat_init(&Jacobian,2,2,(float *)Jacobian_data);
    mat_init(&JacobianT,2,2,(float *)JacobianT_data);
    mat_init(&JacobinT_inv,2,2,(float *)JacobinT_inv_data);
    mat_init(&mat_F,2,1,(float *)mat_F_data);
    mat_init(&mat_T,2,1,(float *)mat_T_data);

    Jacobian_data[0] = Leg_State->j[0][0];
    Jacobian_data[1] = Leg_State->j[0][1];
    Jacobian_data[2] = Leg_State->j[1][0];
    Jacobian_data[3] = Leg_State->j[1][1];

    mat_T_data[0] = MT1_torque;
    mat_T_data[1] = MT4_torque;

    //求得VMC逆转换矩阵
    // FTp = (J')\[MT1;MT4];
    mat_trans(&Jacobian,&JacobianT);//求得J的转置
    mat_inv(&JacobianT,&JacobinT_inv);//求得J的逆   //其实是求得J的转置的逆
    mat_mult(&JacobinT_inv,&mat_T,&mat_F);//求得J的逆和T的乘积

    Leg_State->F_fdb = mat_F.pData[0];
    Leg_State->Tp_fdb = mat_F.pData[1];

    float P = Leg_State->F_fdb*costheta + (Leg_State->Tp_fdb*sintheta)/Leg_State->l0;
    //支持力的计算
    Leg_State->Leg_FN = WHEEL_MASS * Leg_State->ddzw + P + WHEEL_MASS * 9.81;
    
    last_dtheta = Leg_State->dtheta;
}


/**
************************************************************************************************************************
* @Name     : wheel_state_estimate
* @brief    : 底盘离地检测函数
* @param		: leg
* @retval   : wheel_state
* @Note     :
************************************************************************************************************************
**/

uint8_t Wheel_State_Estimate(Leg_State_t *Leg_State)
{
    if (Leg_State->Leg_FN < 20) // 如果支持力小于20N 离地 轮子状态为0  leg->leg_FN < 20
    {

        Leg_State->Wheel_State = 0;
        return 0;
    }
    else // 未离地 轮子状态为1
    {
        Leg_State->Wheel_State = 1;
        return 1;
    }
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
    PID_Init(&Chassis->Init_Tp_Pid,PID_POSITION,0,0,0,500,200);
    
    //左腿腿长
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,800,0,10000,20000,20000);
    
    //右腿腿长
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,800,0,10000,20000,20000);
    
    //双腿协调
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner,PID_POSITION,9.3f,0,1,35,3);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer,PID_POSITION,35,0,1.8f,50,3);
//    
    
//    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner,PID_POSITION,6.3,0,0.7f,35,3);
//    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer,PID_POSITION,25,0,1.8f,50,3);
////    
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




void Chassis_Referance_Update(Balance_Chassis_t* Chassis)
{
    //速度，角度参考值更新
    float V_y;
    float V_x;
    float Temp_Angle;
    Chassis->Yaw_Angle_0_To_2PI = Chassis->USART_Chassis_Data.Yaw_Encoder_Angle;
    
    
    //云台角度劣弧优化
    if(Chassis->Yaw_Angle_0_To_2PI > PI)
    {
        Chassis->Yaw_Angle__PI_To_PI = Chassis->Yaw_Angle_0_To_2PI - 2*PI;
    }
    else
    {
        Chassis->Yaw_Angle__PI_To_PI = Chassis->Yaw_Angle_0_To_2PI ;
    }
    
    V_x = Chassis->Chassis_Remote_Ref.V_x ;
    V_y = Chassis->Chassis_Remote_Ref.V_y ;
    
    
    //参考速度和角度的更新
    if(V_x == 0 && V_y == 0)  //如果都为0，设置底盘速度和角度为0
    {
        Chassis->Chassis_Ref.Remote_Angle = 0;
        Chassis->Chassis_Ref.Remote_Speed = 0;
    }
    else
    {
        Chassis->Chassis_Ref.Remote_Speed = sqrtf(V_x*V_x + V_y*V_y);
        Temp_Angle = atan2f(V_y,V_x) - 1.57f;  //这里先抄老代码
        if(Temp_Angle < -PI)
        {
            Chassis->Chassis_Ref.Remote_Angle = Temp_Angle + 2*PI;
        }
        else
        {
            Chassis->Chassis_Ref.Remote_Angle = Temp_Angle;
        }
    }
}


void Chassis_State_Update(Balance_Chassis_t* Chassis)
{
    /**************这里缺少加速度的计算以及打滑检测******************/
    
    
    /****************************************************************/
    //底盘各数据获取
    VMC_Data_Get(&Chassis->Right_Leg,Chassis->Joint_Motor[3].Angle_Rad_fdb*JM4_POSITION_POLARITY + PI,Chassis->Joint_Motor[3].Omega_Rad_fdb*JM4_POSITION_POLARITY,
    Chassis->Joint_Motor[0].Angle_Rad_fdb*JM1_POSITION_POLARITY + PI,Chassis->Joint_Motor[0].Omega_Rad_fdb*JM1_POSITION_POLARITY);//求得右腿状态
    VMC_Data_Get(&Chassis->Left_Leg,Chassis->Joint_Motor[2].Angle_Rad_fdb*JM3_POSITION_POLARITY + PI,Chassis->Joint_Motor[2].Omega_Rad_fdb*JM3_POSITION_POLARITY,
    Chassis->Joint_Motor[1].Angle_Rad_fdb*JM2_POSITION_POLARITY + PI,Chassis->Joint_Motor[1].Omega_Rad_fdb*JM2_POSITION_POLARITY);//求得左腿状态
    if(leglength_cmd_temp == 0)
    {
        Chassis->Chassis_Remote_Ref.Leglength = 0.15;
    }
    else if(leglength_cmd_temp == 1)
    {
        Chassis->Chassis_Remote_Ref.Leglength = 0.25;
    }
    
    Leglength_Change(Chassis);
    if(control_mode_temp == 0)
    {
        Chassis->Control_Mode = CHASSIS_RELAX ;
    }
    else if(control_mode_temp == 1)
    {
       Chassis->Control_Mode = CHASSIS_INIT ;
    }
    Chassis->Control_Mode = Chassis->USART_Chassis_Data.Chassis_Mode;
    
    Chassis->Left_Leg.dtheta = Chassis->Left_Leg.dphi0  - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD;
    Chassis->Right_Leg.dtheta = Chassis->Right_Leg.dphi0  - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD;
    Chassis->Left_Leg.theta = Chassis->Left_Leg.phi0  - Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD;
    Chassis->Right_Leg.theta = Chassis->Right_Leg.phi0 - Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD;
    
    
    //对dphi0出现NUN的情况进行的处理
    if(isnan(Chassis->Left_Leg.dphi0 - Chassis->Right_Leg.dphi0))
    {
        Chassis->Right_Leg.dphi0 = 0.0f;
        Chassis->Left_Leg.dphi0 = 0.0f;
    }
    
    Chassis->dphi0 = (Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.phi0)/2.0f;
    Chassis->phi0 = (Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f;
    
    Chassis->dtheta = ((Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.dphi0)/2.0f - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD);
    if(isnan(Chassis->dtheta) || isinf(Chassis->dtheta))
    {
        Chassis->dphi0 = 0.0f;
    }
    
//    
//    //模式切换判断
//    if((Chassis->Driving_Motor[0].online_flag == 1) | (Chassis->Driving_Motor[1].online_flag == 1))
//    {
//        if( (Chassis->Control_Mode != CHASSIS_INIT && Chassis->Control_Mode != CHASSIS_STAND_MODE) || (Chassis->USART_Chassis_Data.Chassis_Mode == 0) )//正常进行切换
//        {
//           Chassis->Control_Mode = (Chassis_Mode_e)Chassis->USART_Chassis_Data.Chassis_Mode;
//        }
//        
//        if(judge_rece_mesg.game_robot_state.power_management_chassis_output==0||judge_rece_mesg.game_robot_state.current_HP==0)
//        {
//            Chassis->Control_Mode = CHASSIS_RELAX ;
//        }
//        
//        if(Chassis->Last_Control_Mode == CHASSIS_RELAX && Chassis->Control_Mode != CHASSIS_RELAX)//空闲之后必衔接初始化
//        {
//            Chassis->Control_Mode = CHASSIS_INIT ;
//        }
//        
//        if( ( (Chassis->Control_Mode == CHASSIS_ROTATE||Chassis->Control_Mode == MANUAL_FOLLOW_REMOTE) && (fabs(Chassis->Chassis_GYRO.Pitch_Angle)>15) ) )//抬头太多进初始化，之后还要改的
//        {
//            Chassis->Control_Mode = CHASSIS_INIT ;
//        }
//    }
//    else
//    {
//         Chassis->Control_Mode = CHASSIS_RELAX ;
//    }
//     
//    
//    
//    //遥控数据获取
////    if(Chassis->Control_Mode != CHASSIS_INIT)
////    {
////        
//        Chassis->Chassis_Remote_Ref.V_y = Chassis->USART_Chassis_Data.V_y ;
//        Chassis->Chassis_Remote_Ref.V_w = Chassis->USART_Chassis_Data.Omega ;//7878
//        Chassis->Chassis_Remote_Ref.Roll = Chassis->USART_Chassis_Data.Roll ;
//        Chassis->Chassis_Remote_Ref.V_x = Chassis->USART_Chassis_Data.V_x ;
//        //速度限幅
//        VAL_LIMIT(Chassis->Chassis_Remote_Ref.V_y ,Chassis->Min_Speed ,Chassis->Max_Speed);
//        VAL_LIMIT(Chassis->Chassis_Remote_Ref.V_x ,-1.2f,1.2f);
////    }
//    Chassis_Referance_Update(Chassis);
//    
//    
//    
//    Chassis->Last_Control_Mode = Chassis->Control_Mode;
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
    
    PID_Init(&Chassis->Init_Tp_Pid,PID_POSITION,25,0,0,500,200);
    
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,150,0,0,20000,20000);

    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,150,0,0,20000,20000);

    
    Chassis->Chassis_Ref.V_y = 0;
    Chassis->Chassis_Ref.V_x = 0;
    Chassis->Chassis_Ref.V_w = 0;
    
//    float Left_Leg_phi1  = Normalize_Angle_PI(Chassis->Left_Leg.phi1);//先不使用老代码的局部变量
//    float Right_Leg_phi1 = Normalize_Angle_PI(Chassis->Left_Leg.phi1);
//    float phi0_0_To_2PI_Left = Transform_Angle_0_2PI(Chassis->Left_Leg.phi0 )
 

//    if((Chassis->Left_Leg.phi1<1.1f && Chassis->Left_Leg.phi1>-2.5f && Chassis->Left_Leg.l0 > 0.28f && fabs(Chassis->Chassis_GYRO.Pitch_Angle)<30 && fabs(Chassis->Chassis_GYRO.Roll_Angle)<95) ||
//       (Chassis->Right_Leg.phi1<1.1f && Chassis->Right_Leg.phi1>-2.5f && Chassis->Right_Leg.l0 >0.28f && fabs(Chassis->Chassis_GYRO.Pitch_Angle)<30 && fabs(Chassis->Chassis_GYRO.Roll_Angle)<95) )
//    {//老代码里的侧翻
//        Chassis->Init_State = RELAX_STATE;//先复刻老代码，其他姿势全relax
//    }
//    else if((fabs(Chassis->Chassis_GYRO.Pitch_Angle) > 45 && fabs(Chassis->Chassis_GYRO.Pitch_Angle)<100) ||fabs(Chassis->Chassis_GYRO.Roll_Angle)>95)//老代码里的倒翻
//    {
//        Chassis->Init_State = RELAX_STATE;
//    }
//    else
//    {
//        Chassis->Init_State = NORMOL_STATE;
//    }
Chassis->Init_State = NORMOL_STATE;
}






void Chassis_Init_Handle(Balance_Chassis_t* Chassis)
{
    switch (Chassis->Init_State)
    {
        case NORMOL_STATE:
        {
            if(fabs(Chassis->Left_Leg.l0 - Chassis->Right_Leg.l0) > 0.08f)//两条腿一长一短，表示一条在车下，一条在车外
            {
                if(Chassis->Left_Leg.l0 > Chassis->Right_Leg.l0)//右腿在车下,先调整姿势，抽出右腿
                {
                    Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_Pid , Chassis->Right_Leg.phi0 , Chassis->Left_Leg.phi0);
                    Init_Tp_Calc(Chassis->Left_Leg.l0,0,Chassis->Init_Tp,Chassis);
                    Motor_Torque_Set(Chassis,Chassis->Right_Leg.T_Set[0]*JM1_POSITION_POLARITY,0,0,Chassis->Right_Leg.T_Set[1]*JM4_POSITION_POLARITY,0,0);//转矩赋值都还没调极性7878
                    Motor_Out_Limit(Chassis);
                }
                else if(Chassis->Right_Leg.l0 > Chassis->Left_Leg.l0)//左腿在车下，先调整姿势，抽出左腿
                {
                    Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_Pid , Chassis->Left_Leg.phi0 , Chassis->Right_Leg.phi0);
                    Init_Tp_Calc(Chassis->Right_Leg.l0,0,Chassis->Init_Tp,Chassis);
                    Motor_Torque_Set(Chassis,0,Chassis->Left_Leg.T_Set[0]*JM2_POSITION_POLARITY,Chassis->Left_Leg.T_Set[1]*JM4_POSITION_POLARITY,0,0,0);
                    Motor_Out_Limit(Chassis);
                }
            }
            else if((fabs(Chassis->phi0) >= 3 * PI/180) && fabs(Chassis->Right_Leg.l0-Chassis->Left_Leg.l0)<0.08) //腿摆角偏离竖直方向 且 双腿腿长差距小  正常姿势初始化
            {
                Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_Pid,Chassis->phi0,0.0f);
                Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer , (Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0),0);//不知道这个正负号对不对7878
                Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner ,(Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0),Chassis->Harmonize_Outer);
                Init_Tp_Calc(0.14f,Chassis->Harmonize_Inner/2,Chassis->Init_Tp,Chassis);
               // Init_Tp_Calc(0.14f,0,0,Chassis);
                Motor_Torque_Set(Chassis,Chassis->Right_Leg.T_Set[0]*JM1_POSITION_POLARITY, Chassis->Left_Leg.T_Set[0]*JM2_POSITION_POLARITY, Chassis->Left_Leg.T_Set[1]*JM3_POSITION_POLARITY, Chassis->Right_Leg.T_Set[1]*JM4_POSITION_POLARITY, 0, 0);
                Motor_Out_Limit(Chassis);
            }
            else
            {
                Chassis->Control_Mode = CHASSIS_STAND_MODE;
            }
        }
            break;
        case RELAX_STATE:
        {
            Chassis_Relax_Handle(Chassis);//Chassis_Relax_Handle本是用于底盘控制模式，这里用应该能防止其他姿势的初始化疯车，后续删
        }
            break;
        default :
            break;
            
    }
}




void Chassis_Standup_Handle(Balance_Chassis_t* Chassis)
{
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner, PID_POSITION, 9.3f, 0.0f, 1.0f, 35.0f, 3.0f);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer, PID_POSITION, 35.0f, 0.0f, 0.8f, 50.0f, 3.0f);
    
    Chassis->Chassis_Ref.Leglength = 0.14f;
    Chassis->Chassis_Ref.V_y = 0;
    Chassis->Chassis_Ref.V_x = 0;
    Chassis->Chassis_Ref.V_w = 0;
    Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x;
    if(fabs(Chassis->balance_loop.state_err[4]) < 8*DEG_TO_RAD)
    {
        Chassis->Control_Mode = (Chassis_Mode_e)Chassis->USART_Chassis_Data.Chassis_Mode;
    }
}
    



void Chassis_Fallow_Gimbal_Handle(Balance_Chassis_t* Chassis)
{
    PID_Init(&Chassis->Pid_Follow_Gimbal, PID_POSITION, 8, 0, 1, 6, 200);
    
    float Target_Angle;
    float Target_Speed;
    
    //转向的优化 先抄老代码
    if(fabs(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI) < PI/2)
    {
        Target_Angle = Chassis->Chassis_Ref.Remote_Angle ;
        Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = Chassis->USART_Chassis_Data.Roll ;
    }
    else if(Chassis->Yaw_Angle__PI_To_PI - Chassis->Chassis_Ref.Remote_Angle > 3*PI/2)
    {
        Target_Angle = Chassis->Chassis_Ref.Remote_Angle - 2*PI;
        Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = Chassis->USART_Chassis_Data.Roll;
    }
    else if(Chassis->Yaw_Angle__PI_To_PI - Chassis->Chassis_Ref.Remote_Angle > 3*PI/2)
    {
        Target_Angle = Chassis->Chassis_Ref.Remote_Angle - 2*PI;
        Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = Chassis->USART_Chassis_Data.Roll;
    }
    else if(Chassis->Yaw_Angle__PI_To_PI - Chassis->Chassis_Ref.Remote_Angle < 0)
    {
        Target_Angle = Chassis->Chassis_Ref.Remote_Angle - PI;
        Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = Chassis->USART_Chassis_Data.Roll;
    }
    else if(Chassis->Yaw_Angle__PI_To_PI - Chassis->Chassis_Ref.Remote_Angle < 0)
    {
        Target_Angle = Chassis->Chassis_Ref.Remote_Angle + PI;
        Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = Chassis->USART_Chassis_Data.Roll;
    }
    
    Chassis->Chassis_Ref.V_y = trackRamp(Chassis->Chassis_Ref.V_y,Target_Speed);
    Chassis->Chassis_Ref.V_w = -PID_Calc(&Chassis->Pid_Follow_Gimbal,Chassis->Yaw_Angle__PI_To_PI,Target_Angle);
}





void Leglength_Change(Balance_Chassis_t* Chassis)//调试版
{
    Chassis->Chassis_Ref.Leglength = trackRamp_leg(0.001,Chassis->Chassis_Ref.Leglength,Chassis->Chassis_Remote_Ref.Leglength);
}





void Balance_Task(Balance_Chassis_t* Chassis)
{
    //balance_loop数据获取
    if(Chassis->Control_Mode == CHASSIS_ROTATE)//小陀螺补偿phi0
    {
        Chassis->balance_loop.phi = (Chassis->Chassis_GYRO.Pitch_Angle+0.5f)*DEG_TO_RAD;
    }
    else
    {
        Chassis->balance_loop.phi = Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD;
    }
    Chassis->balance_loop.dphi = Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD;
    Chassis->balance_loop.x = ((LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Angle_Deg_Total_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Angle_Deg_Total_fdb)/2.0f) * WHEEL_R * M3508_ENCODER_TO_WHEEL ;
    Chassis->balance_loop.dx = (LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Omega_Rad_fdb) * WHEEL_R ;//1.没加减速比，2.最后是要卡尔曼滤波的 7878
    Chassis->balance_loop.theta = ((Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f - 1.57f)-Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD;
    Chassis->balance_loop.dtheta = ((Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.dphi0)/2.0f - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD);
    
    //机体重力加速度
    Chassis->balance_loop.ddz = Chassis->Chassis_GYRO.Z_Acc * arm_cos_f32(Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD);
    //底盘轮子平均线速度变化
    Chassis->balance_loop.wheel_dx = ((LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Omega_Rad_fdb)/2.0f) * WHEEL_R * M3508_ENCODER_TO_WHEEL;
    //底盘轮子平均转速
    Chassis->balance_loop.RPM = (LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Omega_Rad_fdb)/2.0f;
    //腿长平均值
    Chassis->balance_loop.L0 = (Chassis->Left_Leg.l0 + Chassis->Right_Leg.l0)/2.0f;
    //不用陀螺仪的向心力
    Chassis->balance_loop.Fm = Chassis->Chassis_Ref.V_w*Chassis->Chassis_Ref.V_y * BODY_MASS;
    //先不写7878
  //  Chassis->balance_loop.Current_Fm = 
    
    
    //支持力计算
    FN_calculate(&Chassis->Chassis_GYRO,&Chassis->Left_Leg,&Chassis->L_DDZW_LPF,Chassis->Joint_Motor[1].Torque*JM2_POSITION_POLARITY,Chassis->Joint_Motor[2].Torque*JM3_POSITION_POLARITY);//没调极性7878
    FN_calculate(&Chassis->Chassis_GYRO,&Chassis->Right_Leg,&Chassis->R_DDZW_LPF,Chassis->Joint_Motor[0].Torque*JM1_POSITION_POLARITY,Chassis->Joint_Motor[3].Torque*JM4_POSITION_POLARITY);
    
    
    //LQR增益获取
    lqr_k(Chassis->balance_loop.L0,Chassis->balance_loop.K);
    for(uint8_t i = 0; i < 6; i++)
    {
        for(uint8_t j = 0; j < 2; j++)
        {
            Chassis->balance_loop.k[j][i] = Chassis->balance_loop.K[i * 2 + j];
        }
    }
    
    //
    //这里有跳跃相关处理7878
    //
    
    //误差计算
    Chassis->balance_loop.state_err[0] = 0 - Chassis->balance_loop.theta;
    Chassis->balance_loop.state_err[1] = 0 - Chassis->balance_loop.dtheta;
    Chassis->balance_loop.state_err[2] = 0;//这里先认为位移没误差7878
    Chassis->balance_loop.state_err[3] = Chassis->Chassis_Ref.V_y - Chassis->balance_loop.dx;//dx还没算 7878
    Chassis->balance_loop.state_err[4] = 0 - Chassis->balance_loop.phi;//这里参考角度先给0 7878
    Chassis->balance_loop.state_err[5] = 0 - Chassis->balance_loop.dphi;
    
    
    
    //
    //还没添加其他处理
    //
    
    Chassis->balance_loop.K_error[0][0] = Chassis->balance_loop.k[0][0] * Chassis->balance_loop.state_err[0];        
    Chassis->balance_loop.K_error[0][1] = Chassis->balance_loop.k[0][1] * Chassis->balance_loop.state_err[1];
    Chassis->balance_loop.K_error[0][2] = Chassis->balance_loop.k[0][2] * Chassis->balance_loop.state_err[2];
    Chassis->balance_loop.K_error[0][3] = Chassis->balance_loop.k[0][3] * Chassis->balance_loop.state_err[3];
    Chassis->balance_loop.K_error[0][4] = Chassis->balance_loop.k[0][4] * Chassis->balance_loop.state_err[4];
    Chassis->balance_loop.K_error[0][5] = Chassis->balance_loop.k[0][5] * Chassis->balance_loop.state_err[5];

    Chassis->balance_loop.K_error[1][0] = Chassis->balance_loop.k[1][0] * Chassis->balance_loop.state_err[0];
    Chassis->balance_loop.K_error[1][1] = Chassis->balance_loop.k[1][1] * Chassis->balance_loop.state_err[1];
    Chassis->balance_loop.K_error[1][2] = Chassis->balance_loop.k[1][2] * Chassis->balance_loop.state_err[2];
    Chassis->balance_loop.K_error[1][3] = Chassis->balance_loop.k[1][3] * Chassis->balance_loop.state_err[3];
    Chassis->balance_loop.K_error[1][4] = Chassis->balance_loop.k[1][4] * Chassis->balance_loop.state_err[4];
    Chassis->balance_loop.K_error[1][5] = Chassis->balance_loop.k[1][5] * Chassis->balance_loop.state_err[5];
    
    
    //
    //处理打滑,没加7878
    //
    
    
    //触地增益计算
    Chassis->Balance_Tgain = Chassis->balance_loop.K_error[0][0] + 
                             Chassis->balance_loop.K_error[0][1] + 
                             Chassis->balance_loop.K_error[0][2] + 
                             Chassis->balance_loop.K_error[0][3] + 
                             Chassis->balance_loop.K_error[0][4] + 
                             Chassis->balance_loop.K_error[0][5] ;
                             
    Chassis->Balance_Tpgain = Chassis->balance_loop.K_error[1][0] + 
                              Chassis->balance_loop.K_error[1][1] + 
                              Chassis->balance_loop.K_error[1][2] + 
                              Chassis->balance_loop.K_error[1][3] + 
                              Chassis->balance_loop.K_error[1][4] + 
                              Chassis->balance_loop.K_error[1][5] ;
                              
     
    //离地增益计算
    Chassis->Balance_Toutlandgain = 0.0f;
    Chassis->Balance_Tpoutlandgain = Chassis->balance_loop.K_error[1][0] + Chassis->balance_loop.K_error[1][1];
    
    
    //双腿协调PID
    Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, (Chassis->Left_Leg.phi0 - Chassis->Right_Leg.phi0), 0.0f);
    Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Left_Leg.dphi0 - Chassis->Right_Leg.dphi0), Chassis->Harmonize_Outer);
    
    
    //转向PID
//    Chassis->V_w_Torque = PID_Calc(&Chassis->V_w_Pid, Chassis->Chassis_GYRO.Yaw_Gyro_Omega*DEG_TO_RAD, Chassis->Chassis_Ref.V_w);
//    Chassis->vw_limit_rate = 1.0f;
//    VAL_LIMIT(Chassis->V_w_Torque,-3.5,3.5);

    
    //roll平衡PID
    Chassis->Roll_Balance_Leglength = PID_Calc(&Chassis->Roll_Pid_Angle,Chassis->Chassis_GYRO.Roll_Angle, 0);//可能之后能peek，之后再说
    
    //腿部竖直力F的计算
    Chassis->Left_Leg.Leg_F = PID_Calc(&Chassis->Left_Leg.Leg_Length_PID,Chassis->Left_Leg.l0,Chassis->Chassis_Ref.Leglength + Chassis->Roll_Balance_Leglength) + BODY_MASS/2*9.8f;
    Chassis->Right_Leg.Leg_F = PID_Calc(&Chassis->Right_Leg.Leg_Length_PID,Chassis->Right_Leg.l0,Chassis->Chassis_Ref.Leglength - Chassis->Roll_Balance_Leglength) + BODY_MASS/2*9.8f;
    
    
    //设置左腿关节扭矩
    if(Wheel_State_Estimate(&Chassis->Left_Leg))
    {
        leg_conv(Chassis->Left_Leg.Leg_F, (Chassis->Balance_Tpgain-Chassis->Harmonize_Inner)/2.0f, //7878
        Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
        
        Chassis->joint_T[1] = JM2_POSITION_POLARITY * Chassis->Left_Leg.T_Set[0];//极性7878
        Chassis->joint_T[2] = JM3_POSITION_POLARITY * Chassis->Left_Leg.T_Set[1];
        
        Chassis->driving_T[0] = (Chassis->Balance_Tgain/2.0f) * LEFT_WHEEL_POLARITY;//还得叠加转向
    }
    
    
    //设置右腿关节扭矩
    if(Wheel_State_Estimate(&Chassis->Right_Leg))
    {
        leg_conv(Chassis->Right_Leg.Leg_F,(Chassis->Balance_Tpgain-Chassis->Harmonize_Inner)/2.0f, 
        Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
        
        Chassis->joint_T[0] = JM1_POSITION_POLARITY * Chassis->Right_Leg.T_Set[0];
        Chassis->joint_T[3] = JM4_POSITION_POLARITY * Chassis->Right_Leg.T_Set[1];
        
        Chassis->driving_T[1] = (Chassis->Balance_Tgain/2.0f) * RIGHT_WHEEL_POLARITY;
    }
    
    
    //力矩限幅
    VAL_LIMIT(Chassis->joint_T[1],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[2],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[0],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[3],-JOINT_MAX_T, JOINT_MAX_T);
    
    VAL_LIMIT(Chassis->driving_T[0],-WHEEL_MAX_T,WHEEL_MAX_T);
    VAL_LIMIT(Chassis->driving_T[1],-WHEEL_MAX_T,WHEEL_MAX_T);
}




void Chassis_Control_Loop(Balance_Chassis_t* Chassis)
{
    switch (Chassis->Control_Mode)
    {
        case CHASSIS_RELAX :
        {
            Chassis_Relax_Handle(Chassis);
        }
        break;
        case CHASSIS_INIT :
        {
            Chassis_Init_State_Update(Chassis);
            Chassis_Init_Handle(Chassis);
        }
        break;
//        CHASSIS_STAND_MODE :
//            Chassis_Standup_Handle(Chassis);
//            Balance_Task(Chassis);
//            break;
//        CHASSIS_SEPARATE :
//            Balance_Task(Chassis);
//            break;
//        MANUAL_FOLLOW_REMOTE :
//            Chassis_Fallow_Gimbal_Handle(Chassis);
//            Leglength_Change(Chassis);
//            Balance_Task(Chassis);
        case CHASSIS_TEXT :
        {
        Chassis->Left_Leg.Leg_F = PID_Calc(&Chassis->Left_Leg.Leg_Length_PID,Chassis->Left_Leg.l0,Chassis->Chassis_Ref.Leglength) ;
        Chassis->Right_Leg.Leg_F = PID_Calc(&Chassis->Right_Leg.Leg_Length_PID,Chassis->Right_Leg.l0,Chassis->Chassis_Ref.Leglength ) ;
        Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, ( Chassis->Right_Leg.phi0-Chassis->Left_Leg.phi0  ), 0.0f);
        Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, ( Chassis->Right_Leg.dphi0-Chassis->Left_Leg.dphi0  ), Chassis->Harmonize_Outer);;
        //设置左腿关节扭矩
    if(Wheel_State_Estimate(&Chassis->Left_Leg))
    {
        leg_conv(Chassis->Left_Leg.Leg_F, -Chassis->Harmonize_Outer, //7878
        Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
        
        Chassis->joint_T[1] = (JM2_POSITION_POLARITY * Chassis->Left_Leg.T_Set[0]);//极性7878
        Chassis->joint_T[2] = (JM3_POSITION_POLARITY * Chassis->Left_Leg.T_Set[1]);
        
        Chassis->driving_T[0] = (Chassis->Balance_Tgain/2.0f) * LEFT_WHEEL_POLARITY;//还得叠加转向
    }
    
    
    //设置右腿关节扭矩
    if(Wheel_State_Estimate(&Chassis->Right_Leg))
    {
        leg_conv(Chassis->Right_Leg.Leg_F,Chassis->Harmonize_Outer, 
        Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
        
        Chassis->joint_T[0] = JM1_POSITION_POLARITY * Chassis->Right_Leg.T_Set[0];
        Chassis->joint_T[3] = JM4_POSITION_POLARITY * Chassis->Right_Leg.T_Set[1];
        
   //     Chassis->driving_T[1] = (Chassis->Balance_Tgain/2.0f) * RIGHT_WHEEL_POLARITY;
    }
        }
    break;
        default:
            break;
    }
}



void Chassis_Task(Balance_Chassis_t* Chassis)
{
    Chassis_State_Update(Chassis);
    Chassis_Control_Loop(Chassis);
}





