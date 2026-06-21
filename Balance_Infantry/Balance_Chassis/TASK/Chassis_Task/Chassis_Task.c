#include "main.h"

Balance_Chassis_t Chassis;

//调试时的临时变量
float temp_tp;
uint8_t leglength_cmd_temp;
uint8_t control_mode_temp;



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



/**
************************************************************************************************************************
* @Name     : Motor_Online_Detective
* @brief    : 电机在线检测
* @param	: Encoder_t *Encoder
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
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



/**
************************************************************************************************************************
* @Name     : Transform_Angle_0_2PI
* @brief    : 归化角度到0—2PI
* @param	: float angle
* @retval   : float new_angle
* @Note     :
************************************************************************************************************************
**/
float Transform_Angle_0_2PI(float angle)
{
    float new_angle=fmod(angle+2*PI,2*PI);
    {
        return (new_angle<0)?new_angle+2*PI:new_angle;
    }
}



/**
************************************************************************************************************************
* @Name     : Motor_Out_Limit
* @brief    : 电机输出限幅
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Motor_Out_Limit(Balance_Chassis_t* Chassis)
{
    VAL_LIMIT(Chassis->joint_T[1],-JOINT_MAX_T,JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[2], -JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->driving_T[0], -WHEEL_MAX_T, WHEEL_MAX_T);

    VAL_LIMIT(Chassis->joint_T[0], -JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[3], -JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->driving_T[1], -WHEEL_MAX_T, WHEEL_MAX_T);
}


/**
************************************************************************************************************************
* @Name     : Motor_Torque_Set
* @brief    : 电机力矩设定
* @param	: Balance_Chassis_t* Chassis,float Joint_T_0,float Joint_T_1,float Joint_T_2,float Joint_T_3,float Driving_T_1,float Driving_T_2
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Motor_Torque_Set(Balance_Chassis_t* Chassis,float Joint_T_0,float Joint_T_1,float Joint_T_2,float Joint_T_3,float Driving_T_1,float Driving_T_2)
{
    //左
    Chassis->joint_T[1] = Joint_T_1;//前
    Chassis->joint_T[2] = Joint_T_2;
    Chassis->driving_T[0] = Driving_T_1;
    //右
    Chassis->joint_T[0] = Joint_T_0;//前
    Chassis->joint_T[3] = Joint_T_3;
    Chassis->driving_T[1] = Driving_T_2;
}




/*********************支持力解算*******************/
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
/**
************************************************************************************************************************
* @Name     : FN_calculate
* @brief    : 支持力解算
* @param	: CH040DATA_t* Chassis_GYRO, Leg_State_t* Leg_State, Lpf1stObj *ft,float MT1_torque,float MT4_torque
* @retval   : void
* @Note     : 计算支持力，带气弹簧
************************************************************************************************************************
**/
void FN_calculate(CH040DATA_t* Chassis_GYRO, Leg_State_t* Leg_State, Lpf1stObj *ft,float MT1_torque,float MT4_torque)
{
    static float  last_dtheta;
    float costheta = arm_cos_f32(Leg_State->phi0  - Chassis_GYRO->Pitch_Angle*DEG_TO_RAD);
    float sintheta = arm_sin_f32(Leg_State->phi0  - Chassis_GYRO->Pitch_Angle*DEG_TO_RAD);

    Leg_State->ddtheta = (Leg_State->dtheta - last_dtheta) / ((TIME_STEP * 0.001));//ddzw的计算   差分
    float ddz = Chassis_GYRO->Z_Acc * Chassis_GYRO->Pitch_Angle*DEG_TO_RAD;//机体加速度 ddz
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

    float P = (Leg_State->F_fdb - Leg_State->Gasspring_FN)*costheta + (Leg_State->Tp_fdb*sintheta)/Leg_State->l0;
    //支持力的计算
    Leg_State->Leg_FN = WHEEL_MASS * Leg_State->ddzw + P + WHEEL_MASS * 9.81;
    
    last_dtheta = Leg_State->dtheta;
}


/**
************************************************************************************************************************
* @Name     : wheel_state_estimate
* @brief    : 底盘离地检测函数
* @param	: Leg_State_t *Leg_State
* @retval   : wheel_state
* @Note     :
************************************************************************************************************************
**/
uint8_t Wheel_State_Estimate(Leg_State_t *Leg_State)
{
    if (Leg_State->Leg_FN < 35) // 如果支持力小于 35N 离地 轮子状态为0 
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


/**
************************************************************************************************************************
* @Name     : Init_Tp_Calc
* @brief    : 初始化扭矩计算
* @param    : float Ref_Leglength,float Harmonize,float Init_Tp,Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 计算腿长，双腿协调，初始化力矩，再解到电机上
************************************************************************************************************************
**/
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



/**
************************************************************************************************************************
* @Name     : Chassis_Param_Init
* @brief    : 底盘参数初始化
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Chassis_Param_Init(Balance_Chassis_t* Chassis)
{
    memset(Chassis,0,sizeof(*Chassis));//清零底盘结构体
 //   Chassis->Chassis_Remote_Ref.Leglength = 0.25;
//    Chassis->Chassis_Ref.Leglength = 0.4;
    //初始化力矩
    PID_Init(&Chassis->Init_Tp_PID,PID_POSITION,0,0,0,500,200);
    
    //左腿腿长
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,2500,0,6000,4000,20000);
    
    //右腿腿长
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,2500,0,6000,20000,20000);
    
    //双腿协调 
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner,PID_POSITION,9.3,0,0.7f,35,3);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer,PID_POSITION,23,0,2.2f,50,3);
//    
    //roll平衡
     PID_Init(&Chassis->Roll_Balance_FN_PID,PID_POSITION,35,0,25,100,0);
    
    //小陀螺roll平衡
//    PID_Init(&Chassis->Roll_leg_F_Rotate_Pid,PID_POSITION,2,0,1,20,10);
    
    //普通模式
    PID_Init(&Chassis->Roll_Leg_F_Pid,PID_POSITION,35,0.01,10,600,10);
    
    PID_Init(&Chassis->V_w_Pid,PID_POSITION,3.5,0,0,5,5);
    
    //底盘跟随云台pid
    PID_Init(&Chassis->Pid_Follow_Gimbal,PID_POSITION,12,0,1,100,200);
    
    //先写这些，其他的再说
   
}


/**
************************************************************************************************************************
* @Name     : Chassis_State_Update
* @brief    : 底盘状态获取
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 获取腿长，摆角，机体角度等，用于平衡和判断初始化的姿态
************************************************************************************************************************
**/
void Chassis_State_Update(Balance_Chassis_t* Chassis)
{
    //底盘各数据获取
    VMC_Data_Get(&Chassis->Right_Leg,Chassis->Joint_Motor[3].Angle_Rad_fdb*JM4_POLARITY + PI,Chassis->Joint_Motor[3].Omega_Rad_fdb*JM4_POLARITY,
    Chassis->Joint_Motor[0].Angle_Rad_fdb*JM1_POLARITY + PI,Chassis->Joint_Motor[0].Omega_Rad_fdb*JM1_POLARITY);//求得右腿状态
    VMC_Data_Get(&Chassis->Left_Leg,Chassis->Joint_Motor[2].Angle_Rad_fdb*JM3_POLARITY + PI,Chassis->Joint_Motor[2].Omega_Rad_fdb*JM3_POLARITY,
    Chassis->Joint_Motor[1].Angle_Rad_fdb*JM2_POLARITY + PI,Chassis->Joint_Motor[1].Omega_Rad_fdb*JM2_POLARITY);//求得左腿状态
   

    Chassis->Left_Leg.dtheta = Chassis->Left_Leg.dphi0  - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD;
    Chassis->Right_Leg.dtheta = Chassis->Right_Leg.dphi0  - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*DEG_TO_RAD;
    Chassis->Left_Leg.theta = Chassis->Left_Leg.phi0  - Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD;
    Chassis->Right_Leg.theta = Chassis->Right_Leg.phi0 - Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD;

    Chassis->balance_loop.L0 = (Chassis->Left_Leg.l0 + Chassis->Right_Leg.l0)/2.0f;
    Chassis->balance_loop.theta = ((Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f - Chassis->Chassis_GYRO.Pitch_Angle*DEG_TO_RAD);
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

}
    

/**
************************************************************************************************************************
* @Name     : Chassis_Mode_Select
* @brief    : 底盘模式选择
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 底盘模式选择，通过遥控和机体姿态来判断模式
************************************************************************************************************************
**/
void Chassis_Mode_Select(Balance_Chassis_t* Chassis)
{
    //模式切换判断
    if((Chassis->Driving_Motor[0].online_flag == 1) && (Chassis->Driving_Motor[1].online_flag == 1))
    {

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
        
//        if( ( Chassis->Control_Mode == MANUAL_FOLLOW_REMOTE) && (fabs(Chassis->Chassis_GYRO.Pitch_Angle)>15) ) //抬头太多进初始化，之后还要改的
//        {
//            Chassis->Control_Mode = CHASSIS_INIT ;

//        }
//        
        if(
            ((Chassis->balance_loop.L0 > 0.25 && Chassis->Control_Mode != CHASSIS_RELAX && fabs(Chassis->balance_loop.theta)>0.7f) || //磕台阶
            (fabs(Chassis->Chassis_GYRO.Roll_Angle) > 95 || fabs(Chassis->Chassis_GYRO.Pitch_Angle)>45) )&& (Chassis->Control_Mode != CHASSIS_RELAX)//翻车
        )
        {
            Chassis->Control_Mode = CHASSIS_INIT ;
        }
          

    }
    else
    {
         Chassis->Control_Mode = CHASSIS_RELAX ;
    }
    
    
    if(gimbal_control_online_detective() != 1)
    {
        Chassis->Control_Mode = CHASSIS_RELAX ;
    }
    
    Chassis->Last_Control_Mode = Chassis->Control_Mode;
}
   



/**
************************************************************************************************************************
* @Name     : Chassis_Referance_Update
* @brief    : 底盘参考值更新
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 获取遥控数据，并进行简单的数据处理，包括底盘转角
************************************************************************************************************************
**/
void Chassis_Referance_Update(Balance_Chassis_t* Chassis)    
{     
    
    //遥控数据获取
    if(Chassis->Control_Mode != CHASSIS_INIT)
    {
        Chassis->Chassis_Remote_Ref.V_y = Chassis->USART_Chassis_Data.V_y ;
        Chassis->Chassis_Remote_Ref.V_x = Chassis->USART_Chassis_Data.V_x ;
        
    
       if(Chassis->USART_Chassis_Data.Cmd_Leg_Length == LOW_LEGLENGTH_CMD)
       {
           Chassis->Chassis_Remote_Ref.Leglength = 0.11f;
           Chassis->Max_Speed = 2.5f;
           Chassis->Min_Speed = -2.0f;
       }
       else if(Chassis->USART_Chassis_Data.Cmd_Leg_Length == MIDDLE_LEGLENGTH_CMD)
       {
           Chassis->Chassis_Remote_Ref.Leglength = 0.19f;
           Chassis->Max_Speed = 2.5f;
           Chassis->Min_Speed = -1.8f;
           
       }
       else if(Chassis->USART_Chassis_Data.Cmd_Leg_Length == HIGH_LEGLENGTH_CMD)
       {
           Chassis->Chassis_Remote_Ref.Leglength = 0.32f;
           Chassis->Max_Speed = 1.4f;
           Chassis->Min_Speed = -1.4f;
       }
    }
    
    
    
 //获取底盘转角
    
    //速度，角度参考值更新
    float V_y;
    float V_x;
    float Temp_Angle;
    
    
    Chassis->Yaw_Angle_0_To_2PI = 2*PI - Chassis->USART_Chassis_Data.Yaw_Encoder_Angle;
    
    
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
    else if(Chassis->Leg_Length == HIGH_LEG_LENGTH )
    {
        Chassis->Chassis_Ref.Remote_Angle = 0;
        Chassis->Chassis_Ref.Remote_Speed = sqrtf(V_x*V_x + V_y*V_y);
    }
    else
    {
        Chassis->Chassis_Ref.Remote_Speed = sqrtf(V_x*V_x + V_y*V_y);
        Temp_Angle = atan2f(V_y,V_x) - 1.57f;
        if(Temp_Angle < -PI)
        {
            Chassis->Chassis_Ref.Remote_Angle = Temp_Angle + 2*PI;
        }
        else
        {
            Chassis->Chassis_Ref.Remote_Angle = Temp_Angle;
        }
    }
    
    //速度限幅
    if(Chassis->Chassis_Remote_Ref.V_x == 0)
    {
       VAL_LIMIT(Chassis->Chassis_Ref.Remote_Speed , Chassis->Min_Speed , Chassis->Max_Speed);
    }
    else
    {
       VAL_LIMIT(Chassis->Chassis_Ref.Remote_Speed , -1.5f , 1.5f);
    }
}

    


/**
************************************************************************************************************************
* @Name     : Chassis_Relax_Handle
* @brief    : 底盘失能
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Relax_Handle(Balance_Chassis_t* Chassis)
{   
    //置零关节输出
    Chassis->joint_T[0] = 0;
    Chassis->joint_T[1] = 0;
    Chassis->joint_T[2] = 0;
    Chassis->joint_T[3] = 0;
    Chassis->driving_T[0] = 0;
    Chassis->driving_T[1] = 0;
    
    //置零Tp
    Chassis->Balance_Tpgain = 0;
    Chassis->Balance_Tpoutlandgain = 0;
    Chassis->Gimbal_Init_Cmd = 0;
    
    Chassis->Chassis_Ref.Pitch = 0;
    Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x;
    Chassis->Roll_Balance_FN_PID.Iout = 0;
    Chassis->Chassis_Ref.Roll = 0;
    
    Chassis->Init_State = (Init_State_e)0;
    Chassis->rotate_flag = 0;//后续会改成枚举 7878
    
    PID_Clear(&Chassis->Leg_Harmonize_Pid_Inner);
    PID_Clear(&Chassis->Leg_Harmonize_Pid_Outer);
    PID_Clear(&Chassis->normal_init_dphi0_pid_left);
    PID_Clear(&Chassis->Roll_Leg_F_Pid);
    PID_Clear(&Chassis->Init_Tp_PID);
    PID_Clear(&Chassis->Left_Leg.Leg_Length_PID);
    PID_Clear(&Chassis->Right_Leg.Leg_Length_PID);
    PID_Clear(&Chassis->flip_init_dphi0_pid_left);
    PID_Clear(&Chassis->flip_init_dphi0_pid_right);
    
    
    Chassis->Left_Leg.Leg_FN = 100;
    Chassis->Right_Leg.Leg_FN = 100;

}


/**
************************************************************************************************************************
* @Name     : Chassis_Init_Handle
* @brief    : 初始化收腿
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Init_Handle(Balance_Chassis_t* Chassis)
{
    PID_Init(&Chassis->Init_Tp_PID,PID_POSITION,30,0,0,1000,200);
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,3200,0,4000,4000,4000);
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,3200,0,4000,4000,4000);
    PID_Init(&Chassis->normal_init_dphi0_pid_right,PID_POSITION,3,0.002,0,500,500);
    PID_Init(&Chassis->normal_init_dphi0_pid_left,PID_POSITION,3,0.002,0,500,500);
    PID_Init(&Chassis->flip_init_dphi0_pid_left,PID_POSITION,4,0.004,0,1000,1500);
    PID_Init(&Chassis->flip_init_dphi0_pid_right,PID_POSITION,4,0.004,0,1000,1500);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner,PID_POSITION,9.3,0,0.7f,100,3);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer,PID_POSITION,25,0,1.8f,100,3);
    
    Chassis->Chassis_Ref.V_y = 0;
    Chassis->Chassis_Ref.V_x = 0;
    Chassis->Chassis_Ref.V_w = 0;
    
    float phi0_0_2PI_Left = Transform_Angle_0_2PI(Chassis->Left_Leg.phi0);
    float phi0_0_2PI_Right = Transform_Angle_0_2PI(Chassis->Right_Leg.phi0);
    float phi0_0_2PI = (phi0_0_2PI_Left + phi0_0_2PI_Right)/2.0f;
    float Left_Leg_phi1  = Normalize_Angle_PI(Chassis->Left_Leg.phi1);
    float Right_Leg_phi1 = Normalize_Angle_PI(Chassis->Right_Leg.phi1);
    float phi0 = (Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f;
     

 
//初始化状态决判断  
     //倒扣状态
     if(fabs(Chassis->Chassis_GYRO.Roll_Angle)>95 && fabs(Chassis->Chassis_GYRO.Pitch_Angle) > 45 )
     {
          //倒扣状态1，
          if(Chassis->Chassis_GYRO.Pitch_Angle < -45 /*||  (Chassis->Chassis_GYRO.Pitch_Angle == 90 && Last_Pitch_GYRO_Angle <-10)*/)//发现角度有个跳变，尝试打补丁2
          {
              Chassis->Init_State = FLIP_STATE_1;
              Chassis->Gimbal_Init_Cmd = 0;
          }
          //倒扣状态2，摆腿方向不同
          else/* if(Chassis->Chassis_GYRO.Pitch_Angle > 35 || (Chassis->Chassis_GYRO.Pitch_Angle == 90 && Last_Pitch_GYRO_Angle > 10))*/
          {
              Chassis->Init_State = FLIP_STATE_2;
              Chassis->Gimbal_Init_Cmd = 0;
          }
     }
     
     //侧翻状态1
     else if(Chassis->Chassis_GYRO.Pitch_Angle <-45 /*|| (Chassis->Chassis_GYRO.Pitch_Angle == 90 && Last_Pitch_GYRO_Angle <-10)*/)
     {
         Chassis->Init_State = ROLL_STATE_1;
         Chassis->Gimbal_Init_Cmd = 0;
     }
     else if(Chassis->Chassis_GYRO.Pitch_Angle > 45 /*|| (Chassis->Chassis_GYRO.Pitch_Angle == 90 && Last_Pitch_GYRO_Angle > 10)*/)
     {
         Chassis->Init_State = ROLL_STATE_2;
         Chassis->Gimbal_Init_Cmd = 0;
     }
     
     //正坐状态1 双腿在后直接起
     else if(  (fabs(phi0) >= 6*PI/180) && (phi0_0_2PI_Left<1.7f||phi0_0_2PI_Left>5.4f) && (phi0_0_2PI_Right<1.7f||phi0_0_2PI_Right>5.4f) )
     {
          Chassis->Init_State = NORMAL_STATE_1;
          Chassis->Gimbal_Init_Cmd = 1;
     }
     //正坐状态2 双腿不在后 摆腿到后
     else if((phi0_0_2PI_Left >=1.7f&&phi0_0_2PI_Left<=5.4f) || (phi0_0_2PI_Right >= 1.7f&&phi0_0_2PI_Right <=5.4f) )
     {
         Chassis->Init_State = NORMAL_STATE_2;
         Chassis->Gimbal_Init_Cmd = 1;
     }
     
     
     
     
//初始化反应
    switch (Chassis->Init_State)
    {
        case NORMAL_STATE_1:
        { 
                if( (fabs(Chassis->phi0) <= 6*PI/180) && (fabs(Chassis->Right_Leg.l0 - Chassis->Left_Leg.l0)<0.08) ) //腿摆角偏离竖直方向 且 双腿腿长差距小  正常姿势初始化
                {
                    Chassis->Control_Mode = CHASSIS_STAND_MODE;
                    Chassis->Init_State = INIT_FINISH;
                }
                else if(Chassis->USART_Chassis_Data.Gimbal_Init_Finish_Flag == 1)
                {
                    Chassis->Init_Tp = PID_Calc(&Chassis->Init_Tp_PID,Chassis->phi0,0.0f);
                    Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer , Normalize_Angle_PI(Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0),0);
                    Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner ,(Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0),Chassis->Harmonize_Outer);
                    Init_Tp_Calc(0.10f,Chassis->Harmonize_Inner/2,Chassis->Init_Tp,Chassis);
                    Motor_Torque_Set(Chassis,Chassis->Right_Leg.T_Set[0]*JM1_POLARITY, Chassis->Left_Leg.T_Set[0]*JM2_POLARITY, Chassis->Left_Leg.T_Set[1]*JM3_POLARITY, Chassis->Right_Leg.T_Set[1]*JM4_POLARITY, 0, 0);
                    Motor_Out_Limit(Chassis);
                }
         }
         break;
                
        case NORMAL_STATE_2 :
        {
            if(Chassis->USART_Chassis_Data.Gimbal_Init_Finish_Flag == 1)
            {
                if((phi0_0_2PI_Left >=1.7f&&phi0_0_2PI_Left<=5.4f) || (phi0_0_2PI_Right >= 1.7f&&phi0_0_2PI_Right <=5.4f))
                    {
                        if(phi0_0_2PI_Left >1.6f)
                        {
                            float Init_dphi0_Tp = PID_Calc(&Chassis->normal_init_dphi0_pid_left,Chassis->Left_Leg.dphi4,-6);
                            Chassis->joint_T[1] = Init_dphi0_Tp*JM2_POLARITY;
                            Chassis->joint_T[2] = 0;
                        }
                        else
                        {
                             Chassis->joint_T[1] = 0;
                             Chassis->joint_T[2] = 0;
                        }
                        
                        
                        if(phi0_0_2PI_Right >1.6f)
                        {
                            float Init_dphi0_Tp = PID_Calc(&Chassis->normal_init_dphi0_pid_right,Chassis->Right_Leg.dphi4,-6);
                            Chassis->joint_T[0] = Init_dphi0_Tp*JM1_POLARITY;
                            Chassis->joint_T[3] = 0;                    
                        }
                        else
                        {
                            Chassis->joint_T[0] = 0;
                            Chassis->joint_T[3] = 0;
                        }
                    }
            }
            else
            {
                Chassis->joint_T[0] = 0;
                Chassis->joint_T[1] = 0;
                Chassis->joint_T[2] = 0;
                Chassis->joint_T[3] = 0; 
                Chassis->driving_T[0] = 0;
                Chassis->driving_T[1] = 0;
            }
         }
        break;
         
        case FLIP_STATE_1:
        {
            Chassis->Gimbal_Init_Cmd = 0;
            Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, Normalize_Angle_PI(Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0), 0);
            Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0),  Chassis->Harmonize_Outer);
            
            float Init_dphi0_Tp_Left = PID_Calc(&Chassis->flip_init_dphi0_pid_left, Chassis->Left_Leg.dphi0, 2);
            float Init_dphi0_Tp_Right = PID_Calc(&Chassis->flip_init_dphi0_pid_right, Chassis->Right_Leg.dphi0, 2);
            
            leg_conv(0, Init_dphi0_Tp_Left - Chassis->Harmonize_Inner/2.0f, Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
            leg_conv(0, Init_dphi0_Tp_Right + Chassis->Harmonize_Inner/2.0f, Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
            
            Chassis->joint_T[0] = JM1_POLARITY*Chassis->Right_Leg.T_Set[0];
            Chassis->joint_T[3] = JM4_POLARITY*Chassis->Right_Leg.T_Set[1];
            Chassis->driving_T[1] = 0;
            
            Chassis->joint_T[1] = JM2_POLARITY*Chassis->Left_Leg.T_Set[0];
            Chassis->joint_T[2] = JM3_POLARITY*Chassis->Left_Leg.T_Set[1];
            Chassis->driving_T[0] = 0;
        }
        break;
        
        case FLIP_STATE_2 :
        {
            Chassis->Gimbal_Init_Cmd = 0;
            Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, Normalize_Angle_PI(Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0), 0);
            Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0),  Chassis->Harmonize_Outer);
            
            float Init_dphi0_Tp_Left = PID_Calc(&Chassis->flip_init_dphi0_pid_left, Chassis->Left_Leg.dphi0, -2);
            float Init_dphi0_Tp_Right = PID_Calc(&Chassis->flip_init_dphi0_pid_right, Chassis->Right_Leg.dphi0, -2);
            leg_conv(0, Init_dphi0_Tp_Left - Chassis->Harmonize_Inner/2.0f, Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
            leg_conv(0, Init_dphi0_Tp_Right + Chassis->Harmonize_Inner/2.0f, Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
            Chassis->joint_T[0] = JM1_POLARITY*Chassis->Right_Leg.T_Set[0];
            Chassis->joint_T[3] = JM4_POLARITY*Chassis->Right_Leg.T_Set[1];
            Chassis->driving_T[1] = 0;
            
            Chassis->joint_T[1] = JM2_POLARITY*Chassis->Left_Leg.T_Set[0];
            Chassis->joint_T[2] = JM3_POLARITY*Chassis->Left_Leg.T_Set[1];
            Chassis->driving_T[0] = 0;
        }
        break;
        
        case ROLL_STATE_1:
        {
           Chassis->Gimbal_Init_Cmd = 0;
            Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, Normalize_Angle_PI(Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0), 0);
            Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0),  Chassis->Harmonize_Outer);
            
            float Init_dphi0_Tp_Left = PID_Calc(&Chassis->flip_init_dphi0_pid_left, Chassis->Left_Leg.dphi0, 2);
            float Init_dphi0_Tp_Right = PID_Calc(&Chassis->flip_init_dphi0_pid_right, Chassis->Right_Leg.dphi0, 2);
            
            leg_conv(0, Init_dphi0_Tp_Left - Chassis->Harmonize_Inner/2.0f, Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
            leg_conv(0, Init_dphi0_Tp_Right + Chassis->Harmonize_Inner/2.0f, Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
            Chassis->joint_T[0] = JM1_POLARITY*Chassis->Right_Leg.T_Set[0];
            Chassis->joint_T[3] = JM4_POLARITY*Chassis->Right_Leg.T_Set[1];
            Chassis->driving_T[1] = 0;
            
            Chassis->joint_T[1] = JM2_POLARITY*Chassis->Left_Leg.T_Set[0];
            Chassis->joint_T[2] = JM3_POLARITY*Chassis->Left_Leg.T_Set[1];
            Chassis->driving_T[0] = 0;
 
        }
        break;
        case ROLL_STATE_2:
        {
           Chassis->Gimbal_Init_Cmd = 0;
            Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, Normalize_Angle_PI(Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0), 0);
            Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0),  Chassis->Harmonize_Outer);
            
            float Init_dphi0_Tp_Left = PID_Calc(&Chassis->flip_init_dphi0_pid_left, Chassis->Left_Leg.dphi0, -2);
            float Init_dphi0_Tp_Right = PID_Calc(&Chassis->flip_init_dphi0_pid_right, Chassis->Right_Leg.dphi0, -2);
            
            leg_conv(0, Init_dphi0_Tp_Left - Chassis->Harmonize_Inner/2.0f, Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
            leg_conv(0, Init_dphi0_Tp_Right + Chassis->Harmonize_Inner/2.0f, Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
            Chassis->joint_T[0] = JM1_POLARITY*Chassis->Right_Leg.T_Set[0];
            Chassis->joint_T[3] = JM4_POLARITY*Chassis->Right_Leg.T_Set[1];
            Chassis->driving_T[1] = 0;
            
            Chassis->joint_T[1] = JM2_POLARITY*Chassis->Left_Leg.T_Set[0];
            Chassis->joint_T[2] = JM3_POLARITY*Chassis->Left_Leg.T_Set[1];
            Chassis->driving_T[0] = 0;
        }
        break;
        default :
            break;
            
    }
}



/**
************************************************************************************************************************
* @Name     : Chassis_Standup_Handle
* @brief    : 起立模式
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Standup_Handle(Balance_Chassis_t* Chassis)
{
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner, PID_POSITION, 9.3f, 0.0f, 1.0f, 35.0f, 3.0f);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer, PID_POSITION, 35.0f, 0.0f, 0.8f, 50.0f, 3.0f);
      //左腿腿长
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,2500,0,40000,4000,20000);
    
    //右腿腿长
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,2500,0,40000,4000,20000);
    
    Chassis->Chassis_Ref.Leglength = 0.12f;
    Chassis->Chassis_Ref.V_y = 0;
    Chassis->Chassis_Ref.V_x = 0;
    Chassis->Chassis_Ref.V_w = 0;
    Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x;
//    if(fabs(Chassis->balance_loop.state_err[3]) < 10*DEG_TO_RAD)
//    {
    Chassis->Control_Mode = (Chassis_Mode_e)Chassis->USART_Chassis_Data.Chassis_Mode;
//    }
}
 

/**
************************************************************************************************************************
* @Name     : Chassis_Stop_Handle
* @brief    : 停止模式，打符用
* @param    : Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Stop_Handle(Balance_Chassis_t* Chassis)
{
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION,2500,0,40000,2000,0);
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION,2500,0,40000,2000,0);
    PID_Init(&Chassis->Roll_Balance_FN_PID, PID_POSITION,30,0,12,0400,10);
    
    Chassis->Chassis_Ref.Leglength = trackRamp_leg(0.0008,Chassis->Chassis_Ref.Leglength,Chassis->Chassis_Remote_Ref.Leglength);
    
    Chassis->normal_Y_erroffset -= Chassis->balance_loop.dx * 0.0007 *TIME_STEP ;
    Chassis->Chassis_Ref.V_y = 0;                                                     // 设置底盘的参考速度为零 y轴方向速度
    Chassis->Chassis_Ref.V_w = 0;                                                     // 设置底盘的参考角速度为零
}



/**
************************************************************************************************************************
* @Name     : Chassis_Fallow_Gimbal_Handle
* @brief    : 底盘跟随云台
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Fallow_Gimbal_Handle(Balance_Chassis_t* Chassis)
{
    //PID初始化
    PID_Init(&Chassis->Roll_Balance_FN_PID, PID_POSITION,30,0,12,400,10);
   // PID_Init(&Chassis->Roll_leg_F_Rotate_Pid, PID_POSITION,0,0,0,400,10);
  //  PID_Init(&Chassis->Init_Tp_PID, PID_POSITION,100,0,0,700,10);
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION,2500,0,40000,2000,0);
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION,2500,0,40000,2000,0);
   
    PID_Init(&Chassis->Leg_Harmonize_Pid_Inner,PID_POSITION,8.0,0,0.4f,35,3);
    PID_Init(&Chassis->Leg_Harmonize_Pid_Outer,PID_POSITION,21,0,3.2f,50,3);
    PID_Init(&Chassis->V_w_Pid,PID_POSITION,3.5,0,2,10,10);
  
   
    PID_Init(&Chassis->Pid_Follow_Gimbal,PID_POSITION,10,0,5,1000,200);
    
    
    //位移处理
    if(fabs(Chassis->balance_loop.dx)>0.8f || Chassis->Chassis_Ref.V_y != 0 || fabs(Chassis->Chassis_Ref.V_w) >= 1.75 || Chassis->Control_Mode ==CHASSIS_STOP)
    {
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->normal_Y_erroffset = NORMAL_Y_ERROEOFFSET ;
    }
    else 
    {
        if(Chassis->Leg_Length == HIGH_LEG_LENGTH)
        {
//            if(Chassis->Leg_Length == HIGH_LEG_LENGTH && Chassis->Leg_Length != HIGH_LEG_LENGTH)
//            {
//                Chassis->normal_Y_erroffset -= NORMAL_Y_ERROEOFFSET;
//            }
            Chassis->normal_Y_erroffset_H -= Chassis->balance_loop.dx * 0.001 *TIME_STEP ;
            
        }
        else
        {
             Chassis->normal_Y_erroffset -= Chassis->balance_loop.dx * 0.001 *TIME_STEP ;
        }
       
    }
    
    //跳下台阶后，伸中腿长一段时间，认为头发来的腿长是中腿长，控制遥控腿长
    if(Chassis->Jump_Finish_Middle_Leg_Flag == 1 && Chassis->Jump_Finish_Middle_Leg_Cnt < 1000)
    {
        Chassis->Chassis_Remote_Ref.Leglength = 0.20f;
        Chassis->Jump_Finish_Middle_Leg_Cnt ++;
    }
    
    if(Chassis->Jump_Finish_Middle_Leg_Cnt >= 1000)
    {
        Chassis->Chassis_Remote_Ref.Leglength = 0.20f;
        Chassis->Jump_Finish_Middle_Leg_Cnt ++;
    }
    if(Chassis->Chassis_Remote_Ref.Leglength != 0.32f || (Chassis->Chassis_Remote_Ref.Leglength == 0.32f && fabs(Chassis->Yaw_Angle__PI_To_PI)<= 45*PI/180.0f ))
     Chassis->Chassis_Ref.Leglength = trackRamp_leg(0.001,Chassis->Chassis_Ref.Leglength,Chassis->Chassis_Remote_Ref.Leglength);//这些都不能调换位置
    
    //腿长变化检测
    if(Chassis->balance_loop.L0 <= 0.40f && Chassis->balance_loop.L0 >=0.22f)
    {
        Chassis->Leg_Length = HIGH_LEG_LENGTH ;
    }
    else if(Chassis->balance_loop.L0 < 0.22f && Chassis->balance_loop.L0 >= 0.14f)
    {
         Chassis->Leg_Length = MIDDLE_LEG_LENGTH ;
    }
    else if(Chassis->balance_loop.L0 < 0.14f && Chassis->balance_loop.L0 > 0)
    {
        Chassis->Leg_Length = LOW_LEG_LENGTH ;
    }
  
    
    //中腿长被压下低腿长，保持低腿长一段时间，直接控制最终参考腿长
    if(Chassis->Leg_Length == LOW_LEG_LENGTH && Chassis->Last_Leg_Length == MIDDLE_LEG_LENGTH && Chassis->USART_Chassis_Data.Cmd_Leg_Length != LOW_LEGLENGTH_CMD)
    {
        Chassis->Low_Leglength_Flag = 1;
    }
    
    if(Chassis->Low_Leglength_Flag == 1)
    {
        Chassis->Chassis_Ref.Leglength = 0.12f;
        Chassis->Low_Leglength_Cnt++;
    }
    
    if(Chassis->Low_Leglength_Cnt == 1500)
    {
        Chassis->Low_Leglength_Cnt = 0;
        Chassis->Low_Leglength_Flag = 0;
    }
    
     Chassis->Last_Leg_Length = Chassis->Leg_Length;
    
     
    //角度优化
    if(Chassis->Chassis_Remote_Ref.Leglength >= 0.26f)//高腿长一定转正 相当于一键转头
    {   
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle ;
        if(fabs(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI) < PI/2)
        {
            Chassis->Chassis_Target_Speed = Chassis->Chassis_Remote_Ref.V_y;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 3*PI/2)
        {
            Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI <-3*PI/2)
        {
            Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 0)
        {
            Chassis->Chassis_Target_Speed = -Chassis->Chassis_Ref.Remote_Speed;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < 0)
        {
            Chassis->Chassis_Target_Speed = -Chassis->Chassis_Ref.Remote_Speed;
        }
        Chassis->Chassis_Ref.Roll = 0;
    }
    else//其他腿长做运动上的优化
    {
        if(fabs(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI) < PI/2) //若云台角度与底盘目标速度差值小于PI/2，说明在同一象限内
        {
            Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle ;
            Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
            Chassis->Chassis_Ref.Roll = 0;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 3*PI/2) //如果云台角度与底盘偏航角差值大于3*PI/2
        {
            Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - 2*PI;
            Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
            Chassis->Chassis_Ref.Roll = 0;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < -3*PI/2) //如果差值的绝对值小于-3π/2
        {
            Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + 2*PI;
            Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
            Chassis->Chassis_Ref.Roll = 0;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 0)//如果差值的绝对值大于0
        {
            Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - PI;
            Chassis->Chassis_Target_Speed = - Chassis->Chassis_Ref.Remote_Speed ;
            Chassis->Chassis_Ref.Roll = -0;
        }
        else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < 0)
        {
            Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + PI;
            Chassis->Chassis_Target_Speed = -Chassis->Chassis_Ref.Remote_Speed ;
            Chassis->Chassis_Ref.Roll = -0;
        }
    }
    Chassis->Chassis_Ref.V_y = trackRamp(Chassis->Chassis_Ref.V_y,Chassis->Chassis_Target_Speed);
    Chassis->Chassis_Ref.V_w = -PID_Calc(&Chassis->Pid_Follow_Gimbal,Chassis->Yaw_Angle__PI_To_PI,Chassis->Chassis_Target_Angle);
}

float temp_distance ;
/**
************************************************************************************************************************
* @Name     : Chassis_Jump_Up_Handle
* @brief    : 跳上台阶
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Jump_Up_Handle(Balance_Chassis_t* Chassis)
{
    static uint16_t Air_Cnt;//腾空计数
    Chassis->Chassis_Ref.V_w = 0;//准备跳的时候不能有Vx Vw
    Chassis->Chassis_Ref.V_x = 0;
    PID_Init(&Chassis->Roll_leg_F_Rotate_Pid, PID_POSITION ,0,0,0,0,0);
    
    if(Chassis->USART_Chassis_Data.Jump_Height == 200)//跳一级
    {
        if(temp_distance<60 && temp_distance>0 && Chassis->Jump_State == NO_JUMPING)//判断何时起跳
        {
            PID_Init(&Chassis->Init_Tp_PID,PID_POSITION,100,0,0,700,0);
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,0,4000,0);
            Chassis->Chassis_Ref.Leglength = 0.39f;
            Chassis->Jump_Feedforward = 300;
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Jump_Process = JUMP_EXTEND ;
            Chassis->Jump_State = JUMPING;
        }
        
        if(Chassis->Jump_Process == JUMP_EXTEND)//伸腿阶段
        {
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,0,4000,0);
            Chassis->Chassis_Ref.Leglength = 0.39f;
            Chassis->Jump_State = JUMPING;
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Jump_Finish_Flag = 0;
            if(Chassis->Left_Leg.l0 > 0.30f && Chassis->Right_Leg.l0 > 0.30f)//伸腿后，该进入收腿进程
            {
                Chassis->Jump_Process = JUMP_RETRACT ;
                Chassis->Jump_Feedforward = 0;
            }
        }
        else if(Chassis->Jump_Process == JUMP_RETRACT)//收腿阶段
        {
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,0,4000,0);
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Chassis_Ref.Leglength = 0.1f;
            Chassis->Jump_State = JUMPING;
            
            if(Chassis->Left_Leg.l0 < 0.2f && Chassis->Right_Leg.l0 < 0.2f)
            {
                Chassis->Jump_Process = NO_JUMP;
                Chassis->Jump_Feedforward = 0;
                Chassis->Jump_Finish_Flag = 1;
            }
        }
        else//跳前准备
        {
            Chassis->Jump_Feedforward = 0;
            Chassis->Jump_State = NO_JUMPING;
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
            PID_Init(&Chassis->Init_Tp_PID , PID_POSITION,30,0,0,400,200);
            Chassis->Chassis_Ref.Leglength = 0.1f;
            Chassis->Jump_Finish_Flag = 0;
        }
    }
    
    else if(Chassis->USART_Chassis_Data.Jump_Height == 350 )//跳二级
    {
        if(temp_distance < 58 && temp_distance > 0 && Chassis->Jump_State == NO_JUMPING)//判断何时起跳
        {
            PID_Init(&Chassis->Init_Tp_PID,PID_POSITION,100,0,0,700,0);
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            Chassis->Chassis_Ref.Leglength = 1.0f;
            Chassis->Jump_Feedforward = 12000;
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Jump_Process = JUMP_EXTEND;
            Chassis->Jump_State = JUMPING;
        }
        
        if(Chassis->Jump_Process == JUMP_EXTEND)//伸腿阶段
        {
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            Chassis->Chassis_Ref.Leglength = 1.0f;
            Chassis->Jump_State = JUMPING;
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Jump_Finish_Flag = 0;
            if(Chassis->Left_Leg.l0 > 0.35f && Chassis->Right_Leg.l0 > 0.35f)//伸腿后，该进入收腿进程
            {
                Chassis->Jump_Process = JUMP_RETRACT ;
                Chassis->Jump_Feedforward = 0;
            }
        }
        else if(Chassis->Jump_Process == JUMP_RETRACT)//收腿阶段
        {
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Chassis_Ref.Leglength = 0.05f;
            Chassis->Jump_State = JUMPING;
            
            if(Chassis->Left_Leg.l0 < 0.12f && Chassis->Right_Leg.l0 < 0.12f)
            {
                Air_Cnt++;
                if(Air_Cnt > 350/2)
                {
                    Chassis->Jump_State = NO_JUMPING;
                    Chassis->Jump_Process = NO_JUMP;
                    Chassis->Jump_Feedforward = 0;
                    Chassis->Jump_Finish_Flag = 1;
                }
                else
                {
                    Chassis->Jump_Finish_Flag = 0;
                }
            }
        }
        else//跳前准备
        {
            Chassis->Jump_Feedforward = 0;
            Chassis->Jump_State = NO_JUMPING;
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
            PID_Init(&Chassis->Init_Tp_PID , PID_POSITION,30,0,0,400,200);
            Chassis->Chassis_Ref.Leglength = 0.1f;
            Chassis->Jump_Finish_Flag = 0;
        }
    }
    
    else if(Chassis->USART_Chassis_Data.Jump_Height == 400)//跳400
    {
        if(temp_distance<75 && temp_distance>0 && Chassis->Jump_State == NO_JUMPING)//判断何时起跳
        {
            PID_Init(&Chassis->Init_Tp_PID,PID_POSITION,100,0,0,700,0);
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            Chassis->Chassis_Ref.Leglength = 0.39f;
            Chassis->Jump_Feedforward = 1000;
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Jump_Process = JUMP_EXTEND ;
            Chassis->Jump_State = JUMPING;
        }
        
        if(Chassis->Jump_Process == JUMP_EXTEND)//伸腿阶段
        {
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            Chassis->Chassis_Ref.Leglength = 0.39f;
            Chassis->Jump_State = JUMPING;
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Jump_Finish_Flag = 0;
            if(Chassis->Left_Leg.l0 > 0.30f && Chassis->Right_Leg.l0 > 0.30f)//伸腿后，该进入收腿进程
            {
                Chassis->Jump_Process = JUMP_RETRACT ;
                Chassis->Jump_Feedforward = 0;
            }
        }
        else if(Chassis->Jump_Process == JUMP_RETRACT)//收腿阶段
        {
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
            Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
            Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
            Chassis->Chassis_Ref.Leglength = 0.1f;
            Chassis->Jump_State = JUMPING;
            
            if(Chassis->Left_Leg.l0 < 0.2f && Chassis->Right_Leg.l0 < 0.2f)
            {
                Chassis->Jump_Process = NO_JUMP;
                Chassis->Jump_Feedforward = 0;
                Chassis->Jump_Finish_Flag = 0;
            }
        }
        else//跳前准备
        {
            Chassis->Jump_Feedforward = 0;
            Chassis->Jump_State = NO_JUMPING;
            PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
            PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
            PID_Init(&Chassis->Init_Tp_PID , PID_POSITION,30,0,0,400,200);
            Chassis->Chassis_Ref.Leglength = 0.1f;
            Chassis->Jump_Finish_Flag = 0;
        }
    }
    
    
    //常规的角度优化
    if(fabs(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI) < PI/2) //若云台角度与底盘目标速度差值小于PI/2，说明在同一象限内
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle ;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 3*PI/2) //如果云台角度与底盘偏航角差值大于3*PI/2
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - 2*PI;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < -3*PI/2) //如果差值的绝对值小于-3π/2
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + 2*PI;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 0)//如果差值的绝对值大于0
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - PI;
        Chassis->Chassis_Target_Speed = - Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = -0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < 0)
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + PI;
        Chassis->Chassis_Target_Speed = -Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = -0;
    }
    
    
    //位移处理
    if(fabs(Chassis->balance_loop.dx)>0.5f || Chassis->Chassis_Ref.V_y != 0 || fabs(Chassis->Chassis_Ref.V_w) >= 1.75 || Chassis->Control_Mode ==CHASSIS_STOP)
    {
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->normal_Y_erroffset = NORMAL_Y_ERROEOFFSET ;
    }
    else
    {
        Chassis->normal_Y_erroffset -= Chassis->balance_loop.dx * 0.0005 *TIME_STEP ;
    }
    
    if(Chassis->USART_Chassis_Data.Jump_Height == 200)
    {
        VAL_LIMIT(Chassis->Chassis_Ref.V_y, -1.6,1.6);
    }
    else if(Chassis->USART_Chassis_Data.Jump_Height == 350)
    {
        VAL_LIMIT(Chassis->Chassis_Ref.V_y, -2.5,2.5);
    }
    else if(Chassis->USART_Chassis_Data.Jump_Height == 400)
    {
        VAL_LIMIT(Chassis->Chassis_Ref.V_y, -2.5,2.5);
    }
    
    Chassis->Chassis_Ref.V_w = -PID_Calc(&Chassis->Pid_Follow_Gimbal,Chassis->Yaw_Angle__PI_To_PI,Chassis->Chassis_Target_Angle);
}



/**
************************************************************************************************************************
* @Name     : Chassis_AntiFly_Slope_Handle
* @brief    : 反飞坡
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_AntiFly_Slope_Handle(Balance_Chassis_t* Chassis)
{
    static uint16_t Air_Cnt;//腾空计数
    Chassis->Chassis_Ref.V_w = 0;//准备跳的时候不能有Vx Vw
    Chassis->Chassis_Ref.V_x = 0;
    PID_Init(&Chassis->Roll_leg_F_Rotate_Pid, PID_POSITION ,0,0,0,0,0);
    
    if(temp_distance<125 && temp_distance>10 && Chassis->Jump_State == NO_JUMPING)//判断何时起跳
    {
        PID_Init(&Chassis->Init_Tp_PID,PID_POSITION,100,0,0,700,0);
        PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
        PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
        Chassis->Chassis_Ref.Leglength = 1.0f;
        Chassis->Jump_Feedforward = 50000;
        Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->Jump_Process = JUMP_EXTEND ;
        Chassis->Jump_State = JUMPING;
    }
    
    if(Chassis->Jump_Process == JUMP_EXTEND)//伸腿阶段
    {
        PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
        PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
        Chassis->Chassis_Ref.Leglength = 1.0f;
        Chassis->Jump_State = JUMPING;
        Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->Jump_Finish_Flag = 0;
        if(Chassis->Left_Leg.l0 > 0.35f && Chassis->Right_Leg.l0 > 0.35f)//伸腿后，该进入收腿进程
        {
            Chassis->Jump_Process = JUMP_RETRACT ;
            Chassis->Jump_Feedforward = 0;
        }
    }
    else if(Chassis->Jump_Process == JUMP_RETRACT)//收腿阶段
    {
        PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
        PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 6000,0,0,4000,0);
        Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->Chassis_Ref.Leglength = 0.05f;
        Chassis->Jump_State = JUMPING;
        
        if(Chassis->Left_Leg.l0 < 0.12f && Chassis->Right_Leg.l0 < 0.12f)
        {
            Air_Cnt++;
            if(Air_Cnt>150/2)
            {
                Air_Cnt = 0;
                Chassis->Jump_Process = NO_JUMP;
                Chassis->Jump_Feedforward = 0;
                Chassis->Jump_Finish_Flag = 1;
            }
        }
    }
    else//跳前准备
    {
        Chassis->Jump_Feedforward = 0;
        Chassis->Jump_State = NO_JUMPING;
        PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
        PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 2500,0,40000,2000,0);
        PID_Init(&Chassis->Init_Tp_PID , PID_POSITION,30,0,0,400,200);
        Chassis->Chassis_Ref.Leglength = 0.1f;
        Chassis->Jump_Finish_Flag = 0;
    }
    
    
    //常规的角度优化
    if(fabs(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI) < PI/2) //若云台角度与底盘目标速度差值小于PI/2，说明在同一象限内
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle ;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 3*PI/2) //如果云台角度与底盘偏航角差值大于3*PI/2
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - 2*PI;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < -3*PI/2) //如果差值的绝对值小于-3π/2
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + 2*PI;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 0)//如果差值的绝对值大于0
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - PI;
        Chassis->Chassis_Target_Speed = - Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = -0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < 0)
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + PI;
        Chassis->Chassis_Target_Speed = -Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = -0;
    }
    
    
    //位移处理
    if(fabs(Chassis->balance_loop.dx)>0.5f || Chassis->Chassis_Ref.V_y != 0 || fabs(Chassis->Chassis_Ref.V_w) >= 1.75 || Chassis->Control_Mode ==CHASSIS_STOP)
    {
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->normal_Y_erroffset = NORMAL_Y_ERROEOFFSET ;
    }
    else
    {
        Chassis->normal_Y_erroffset -= Chassis->balance_loop.dx * 0.0005 *TIME_STEP ;
    }
    
    VAL_LIMIT(Chassis->Chassis_Ref.V_y, -2.5,2.5);
    Chassis->Chassis_Ref.V_w = -PID_Calc(&Chassis->Pid_Follow_Gimbal,Chassis->Yaw_Angle__PI_To_PI,Chassis->Chassis_Target_Angle);
}



/**
************************************************************************************************************************
* @Name     : Chassis_Jump_Down_Handle
* @brief    : 跳下台阶
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Jump_Down_Handle(Balance_Chassis_t* Chassis)
{
    static uint16_t Air_Cnt;//腾空计数
    Chassis->Chassis_Ref.V_w = 0;//准备跳的时候不能有Vx Vw
    Chassis->Chassis_Ref.V_x = 0;
    PID_Init(&Chassis->Roll_leg_F_Rotate_Pid, PID_POSITION ,0,0,0,0,0);
    PID_Init(&Chassis->Left_Leg.Leg_Length_PID, PID_POSITION, 3200,0,4000,4000,0);
    PID_Init(&Chassis->Right_Leg.Leg_Length_PID, PID_POSITION, 3200,0,4000,4000,0);
    
    if(Chassis->Jump_Process == NO_JUMP)
    {
        Chassis->Chassis_Ref.Leglength = 1.0f;
        Chassis->Jump_Feedforward = 1000;
        Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->Jump_Process = JUMP_EXTEND;
        Chassis->Jump_Finish_Flag = 0;
    }
    else if(Chassis->Jump_Process == JUMP_EXTEND)//伸腿阶段
    {
        Chassis->Chassis_Ref.Leglength = 1.0f;
        Chassis->Jump_State = JUMPING;
        Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->Jump_Finish_Flag = 0;
        if(Chassis->Left_Leg.l0 > 0.22f && Chassis->Right_Leg.l0 > 0.22f)//伸腿后，该进入收腿进程
        {
            Chassis->Jump_Process = JUMP_RETRACT ;
            Chassis->Jump_Feedforward = 0;
        }
    }
    else if(Chassis->Jump_Process == JUMP_RETRACT)//收腿阶段
    {
        Chassis->Chassis_Ref.V_y = Chassis->balance_loop.dx ;
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->Chassis_Ref.Leglength = 0.05f;
        Chassis->Jump_State = JUMPING;
        
        if(Chassis->Left_Leg.l0 < 0.12f && Chassis->Right_Leg.l0 < 0.12f)
        {
            Air_Cnt++;
            if(Air_Cnt>150/2)
            {
                Air_Cnt = 0;
                Chassis->Jump_Finish_Middle_Leg_Flag = 1;
                Chassis->Jump_Process = NO_JUMP;
                Chassis->Jump_Feedforward = 0;
                Chassis->Jump_Finish_Flag = 1;
            }
        }
    }
    
    //常规的角度优化
    if(fabs(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI) < PI/2) //若云台角度与底盘目标速度差值小于PI/2，说明在同一象限内
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle ;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 3*PI/2) //如果云台角度与底盘偏航角差值大于3*PI/2
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - 2*PI;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < -3*PI/2) //如果差值的绝对值小于-3π/2
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + 2*PI;
        Chassis->Chassis_Target_Speed = Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = 0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI > 0)//如果差值的绝对值大于0
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle - PI;
        Chassis->Chassis_Target_Speed = - Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = -0;
    }
    else if(Chassis->Chassis_Ref.Remote_Angle - Chassis->Yaw_Angle__PI_To_PI < 0)
    {
        Chassis->Chassis_Target_Angle = Chassis->Chassis_Ref.Remote_Angle + PI;
        Chassis->Chassis_Target_Speed = -Chassis->Chassis_Ref.Remote_Speed ;
        Chassis->Chassis_Ref.Roll = -0;
    }
    
    
    //位移处理
    if(fabs(Chassis->balance_loop.dx)>0.5f || Chassis->Chassis_Ref.V_y != 0 || fabs(Chassis->Chassis_Ref.V_w) >= 1.75 || Chassis->Control_Mode ==CHASSIS_STOP)
    {
        Chassis->Chassis_Ref.Y_position = Chassis->balance_loop.x ;
        Chassis->normal_Y_erroffset = NORMAL_Y_ERROEOFFSET ;
    }
    else
    {
        Chassis->normal_Y_erroffset -= Chassis->balance_loop.dx * 0.0005 *TIME_STEP ;
    }
    
    
    VAL_LIMIT(Chassis->Chassis_Ref.V_y, -1.4,1.4);
    Chassis->Chassis_Ref.V_w = -PID_Calc(&Chassis->Pid_Follow_Gimbal,Chassis->Yaw_Angle__PI_To_PI,Chassis->Chassis_Target_Angle);
}


/**
************************************************************************************************************************
* @Name     : Chassis_Rotate_Handle
* @brief    : 小陀螺
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :
************************************************************************************************************************
**/
void Chassis_Rotate_Handle(Balance_Chassis_t* Chassis)
{
      //腿长变化检测
    if(Chassis->balance_loop.L0 <= 0.40f && Chassis->balance_loop.L0 >=0.22f)
    {
        Chassis->Leg_Length = HIGH_LEG_LENGTH ;
    }
    else if(Chassis->balance_loop.L0 < 0.22f && Chassis->balance_loop.L0 >= 0.14f)
    {
         Chassis->Leg_Length = MIDDLE_LEG_LENGTH ;
    }
    else if(Chassis->balance_loop.L0 < 0.14f && Chassis->balance_loop.L0 > 0)
    {
        Chassis->Leg_Length = LOW_LEG_LENGTH ;
    }
    
    
    //角度优化
    Chassis->Yaw_Angle__PI_To_PI = Normalize_Angle_PI(Chassis->Yaw_Angle_0_To_2PI);
    //固定腿长0.1m
    Chassis->Chassis_Ref.Leglength = trackRamp_leg(0.001,Chassis->Chassis_Ref.Leglength, 0.1f);
    if(Chassis->Control_Mode == CHASSIS_CLOCKWISE_ROTATE || Chassis->Control_Mode == CHASSIS_CLOCKWISE_ROTATE_VAR_SPEED)
    {
        Chassis->Chassis_Ref.V_y = -(Chassis->Chassis_Remote_Ref.V_y * arm_sin_f32(Chassis->Yaw_Angle__PI_To_PI - PI/4) + Chassis->Chassis_Remote_Ref.V_x*arm_cos_f32(Chassis->Yaw_Angle__PI_To_PI - PI/4));
    }
    else
    {
         Chassis->Chassis_Ref.V_y = (Chassis->Chassis_Remote_Ref.V_y * arm_sin_f32(Chassis->Yaw_Angle__PI_To_PI + PI/4) + Chassis->Chassis_Remote_Ref.V_x*arm_cos_f32(Chassis->Yaw_Angle__PI_To_PI + PI/4));
    }
    
    Chassis->Chassis_Ref.V_x = 0;
    
    if(Chassis->Control_Mode == CHASSIS_CLOCKWISE_ROTATE)
    {
        Chassis->Chassis_Ref.V_w = 10 - fabs(Chassis->Chassis_Ref.V_y);
    }
    else if(Chassis->Control_Mode == CHASSIS_ANTI_CLOCKWISE_ROTATE)
    {
        Chassis->Chassis_Ref.V_w = -fabs(10 - fabs(Chassis->Chassis_Ref.V_y));
    }
    else if(Chassis->Control_Mode == CHASSIS_CLOCKWISE_ROTATE_VAR_SPEED)
    {
        Chassis->Chassis_Ref.V_w = 10 - fabs(Chassis->Chassis_Ref.V_y) + 5*Sinusoidal_Waveform_Generator_2(0.001/*函数执行周期*/,0.5/*正弦波频率*/);
    }
    else if(Chassis->Control_Mode == CHASSIS_ANTI_CLOCKWISE_ROTATE_VAR_SPEED)
    {
        Chassis->Chassis_Ref.V_w = -10 + fabs(Chassis->Chassis_Ref.V_y) - 5*Sinusoidal_Waveform_Generator_2(0.001/*函数执行周期*/,0.5/*正弦波频率*/);//后续再完善7878
    }
    else
    {
        Chassis->Chassis_Ref.V_w = 0 ;
    }
    VAL_LIMIT(Chassis->Chassis_Ref.V_w, -15, 15);
    VAL_LIMIT(Chassis->Chassis_Ref.V_y, -0, 0);
}




/**
************************************************************************************************************************
* @Name     : Chassis_Single_Leg_Control_Handle
* @brief    : 底盘单腿控制处理
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 翻车和UWB支架干涉时用，使用灰控手动摆腿
************************************************************************************************************************
**/
void Chassis_Single_Leg_Control_Handle(Balance_Chassis_t* Chassis)
{
    PID_Init(&Chassis->Init_phi0_pid_left,PID_POSITION,2,0,0,100,100);
    PID_Init(&Chassis->Init_phi0_pid_right,PID_POSITION,2,0,0,100,100);
    
    float phi_0_2PI_Left = Transform_Angle_0_2PI(Chassis->Left_Leg.phi0);
    float phi_0_2PI_Right = Transform_Angle_0_2PI(Chassis->Right_Leg.phi0);
    
    float Ref_phi0_Left = Chassis->USART_Chassis_Data.leg_single_angle_handle_left + phi_0_2PI_Left;
    float Ref_phi0_Right = Chassis->USART_Chassis_Data.leg_single_angle_handle_right + phi_0_2PI_Right;
    
    float Init_Tp_Left = PID_Calc(&Chassis->Init_phi0_pid_left,phi_0_2PI_Left,Ref_phi0_Left);
    float Init_Tp_Right = PID_Calc(&Chassis->Init_phi0_pid_right,phi_0_2PI_Right,Ref_phi0_Right);
    
    leg_conv(0,Init_Tp_Left,Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4,Chassis->Left_Leg.T_Set);
    leg_conv(0,Init_Tp_Left,Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4,Chassis->Right_Leg.T_Set);
    
    Motor_Torque_Set(Chassis,Chassis->Right_Leg.T_Set[0],Chassis->Left_Leg.T_Set[0],Chassis->Left_Leg.T_Set[1],Chassis->Right_Leg.T_Set[1],0,0);
}




/**
************************************************************************************************************************
* @Name     : Balance_Task
* @brief    : 平衡底盘解算
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     :	一定要注意弧度制的转化啊
                            电机极性要仔细检查
                            检查好各个传感器的单位与性能
************************************************************************************************************************
**/
void Balance_Task(Balance_Chassis_t* Chassis)
{
    static uint8_t follow_gimbal_flag = 0;
    //balance_loop数据获取
 //   if(Chassis->Control_Mode == CHASSIS_ROTATE)//小陀螺补偿phi0
 //   {
  //      Chassis->balance_loop.phi = (Chassis->Chassis_GYRO.Pitch_Angle+0.5f)*DEG_TO_RAD;
 //   }
 //   else
 //   {
    Chassis->balance_loop.phi = Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f;
  //  }
    
    Chassis->balance_loop.dphi = Chassis->Chassis_GYRO.Pitch_Gyro_Omega*PI/180.0f;
    Chassis->balance_loop.x = ((LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Angle_Rad_Total_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Angle_Rad_Total_fdb)/2.0f) * WHEEL_R ;
    Chassis->balance_loop.dx = Mileage_kalman_filter.velocity;
    Chassis->balance_loop.theta = ((Chassis->Left_Leg.phi0 + Chassis->Right_Leg.phi0)/2.0f) - Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f;
    Chassis->balance_loop.dtheta = (Chassis->Left_Leg.dphi0 + Chassis->Right_Leg.dphi0)/2.0f - Chassis->Chassis_GYRO.Pitch_Gyro_Omega*PI/180.0f;

    //机体重力加速度
    Chassis->balance_loop.ddz = Chassis->Chassis_GYRO.Z_Acc * arm_cos_f32(Chassis->Chassis_GYRO.Pitch_Angle*PI/180.0f);
    //底盘轮子平均线速度变化
    Chassis->balance_loop.wheel_dx = ((LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Omega_Rad_fdb)/2.0f) * WHEEL_R ;
    //底盘轮子平均转速
    Chassis->balance_loop.RPM = (LEFT_WHEEL_POLARITY * Chassis->Driving_Motor[0].Omega_Rad_fdb + RIGHT_WHEEL_POLARITY * Chassis->Driving_Motor[1].Omega_Rad_fdb)/2.0f;
    //腿长平均值
    Chassis->balance_loop.L0 = (Chassis->Left_Leg.l0 + Chassis->Right_Leg.l0)/2.0f;
    //不用陀螺仪的向心力
    Chassis->balance_loop.Fm = Chassis->Chassis_Ref.V_w*Chassis->Chassis_Ref.V_y * BODY_MASS;

    //气弹簧解算 
    Chassis->Left_Leg.Gasspring_FN = Get_Left_GasSpring_FN(Chassis->Left_Leg.l0);
    Chassis->Right_Leg.Gasspring_FN = Get_Right_GasSpring_FN(Chassis->Right_Leg.l0);
    
    //支持力计算
    FN_calculate(&Chassis->Chassis_GYRO,&Chassis->Left_Leg,&Chassis->L_DDZW_LPF,Chassis->Joint_Motor[1].Torque*JM2_POLARITY,Chassis->Joint_Motor[2].Torque*JM3_POLARITY);//没调极性7878
    FN_calculate(&Chassis->Chassis_GYRO,&Chassis->Right_Leg,&Chassis->R_DDZW_LPF,Chassis->Joint_Motor[0].Torque*JM1_POLARITY,Chassis->Joint_Motor[3].Torque*JM4_POLARITY);
    
    
    //LQR增益获取
    lqr_k(Chassis->balance_loop.L0,Chassis->balance_loop.K);
    for(uint8_t i = 0; i < 6; i++)
    {
        for(uint8_t j = 0; j < 2; j++)
        {
            Chassis->balance_loop.k[j][i] = Chassis->balance_loop.K[i * 2 + j];
        }
    }
    
    if(Chassis->Leg_Length == MIDDLE_LEG_LENGTH)//中腿长，pitch偏置
    {
       Chassis->balance_loop.state_err[4] = 3*PI/180.0f - Chassis->balance_loop.phi;
    }
    else
    {
        Chassis->balance_loop.state_err[4] = 0 - Chassis->balance_loop.phi;
    }
    
    //中腿长离地，phi偏置
    if(Chassis->Leg_Length == MIDDLE_LEG_LENGTH && Wheel_State_Estimate(&Chassis->Left_Leg) == 0 && Wheel_State_Estimate(&Chassis->Right_Leg) == 0)
    {
         Chassis->balance_loop.state_err[0] = -10.0f*PI/180.0f - Chassis->balance_loop.theta;
    }
    else
    {
        Chassis->balance_loop.state_err[0] = 0 - Chassis->balance_loop.theta;
    }
    
    //误差计算
   
    Chassis->balance_loop.state_err[1] = 0 - Chassis->balance_loop.dtheta;
    Chassis->balance_loop.state_err[2] = Chassis->Chassis_Ref.Y_position - Chassis->balance_loop.x ;
    Chassis->balance_loop.state_err[3] = Chassis->Chassis_Ref.V_y - Chassis->balance_loop.dx;
    Chassis->balance_loop.state_err[5] = 0 - Chassis->balance_loop.dphi;
    
    if(Chassis->Leg_Length == HIGH_LEG_LENGTH)
    {
        Chassis->x_error = (Chassis->balance_loop.state_err[2] + Chassis->normal_Y_erroffset_H);
    }
    else
    {
        Chassis->x_error = (Chassis->balance_loop.state_err[2] + Chassis->normal_Y_erroffset);
    }
    
    
    
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
    
    float speed_k;
    if(fabs(Chassis->Left_Acc)>310 || fabs(Chassis->Right_Acc)>310)
    {
        speed_k = 0;
    }
    else
    {
        speed_k = Chassis->balance_loop.K_error[0][3];
    }
    
    //触地增益计算
    Chassis->Balance_Tgain = Chassis->balance_loop.K_error[0][0] + 
                             Chassis->balance_loop.K_error[0][1] + 
                             Chassis->balance_loop.k[0][2] * Chassis->x_error + 
                             speed_k + 
                             Chassis->balance_loop.K_error[0][4] + 
                             Chassis->balance_loop.K_error[0][5] ;
                             
    Chassis->Balance_Tpgain = Chassis->balance_loop.K_error[1][0] + 
                              Chassis->balance_loop.K_error[1][1] + 
                              Chassis->balance_loop.k[1][2] * Chassis->x_error + 
                              Chassis->balance_loop.K_error[1][3] + 
                              Chassis->balance_loop.K_error[1][4] + 
                              Chassis->balance_loop.K_error[1][5] ;
          
//Chassis->Balance_Tpgain = Chassis->Balance_Tpgain;    
    
    //离地增益计算
    Chassis->Balance_Toutlandgain = 0.0f;
    Chassis->Balance_Tpoutlandgain = Chassis->balance_loop.K_error[1][0] + Chassis->balance_loop.K_error[1][1];
     Chassis->Balance_Tpoutlandgain = Chassis->Balance_Tpoutlandgain/2.0f;
    
    //双腿协调PID
    Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, (Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0), 0.0f);
    Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0), Chassis->Harmonize_Outer);
    
    
    //转向PID
    Chassis->V_w_Torque = PID_Calc(&Chassis->V_w_Pid, Chassis->Chassis_GYRO.Yaw_Gyro_Omega*DEG_TO_RAD, Chassis->Chassis_Ref.V_w);
    Chassis->vw_limit_rate = 1.0f;
    
    
    
    //打的补丁，希望小陀螺回正快
    if((Chassis->Last_Control_Mode == CHASSIS_CLOCKWISE_ROTATE || Chassis->Last_Control_Mode == CHASSIS_ANTI_CLOCKWISE_ROTATE) && fabs(Chassis->Yaw_Angle__PI_To_PI-Chassis->Chassis_Target_Angle)>15.0/180.0f*PI )
    {
        follow_gimbal_flag = 1;
    }
    
    if( fabs(Chassis->Yaw_Angle__PI_To_PI-Chassis->Chassis_Target_Angle)<15.0/180.0f*PI)
    {
       follow_gimbal_flag = 0;
    }

    
    if(follow_gimbal_flag == 1)
    {
        VAL_LIMIT(Chassis->V_w_Torque,-7,7);
    }
    else if(follow_gimbal_flag == 0)
    {
        VAL_LIMIT(Chassis->V_w_Torque,-5,5);
    }
    
    
    
    
    //roll平衡PID
    Chassis->Roll_Balance_F = PID_Calc(&Chassis->Roll_Balance_FN_PID,Chassis->Chassis_GYRO.Roll_Angle, 0);
    
    if(Chassis->Roll_Balance_F > 0)
    {
        Chassis->Roll_Balance_F_Left = Chassis->Roll_Balance_F;
        Chassis->Roll_Balance_F_Right = 0;
    }
    else if(Chassis->Roll_Balance_F < 0)
    {
        Chassis->Roll_Balance_F_Left = 0;
        Chassis->Roll_Balance_F_Right = -Chassis->Roll_Balance_F;
    }
    
    //腿部竖直力F的计算
    if(Chassis->Control_Mode == CHASSIS_ANTI_FLY_SLOPE || Chassis->Control_Mode == CHASSIS_JUMP_DOWN || Chassis->Control_Mode == CHASSIS_JUMP_UP )
    {
        Chassis->Left_Leg.Leg_F = Chassis->Jump_Feedforward+PID_Calc(&Chassis->Left_Leg.Leg_Length_PID,Chassis->Left_Leg.l0,Chassis->Chassis_Ref.Leglength) + BODY_MASS/2*9.81f + Chassis->Roll_Balance_F_Left + Chassis->Left_Leg.Gasspring_FN;
        Chassis->Right_Leg.Leg_F = Chassis->Jump_Feedforward+PID_Calc(&Chassis->Right_Leg.Leg_Length_PID,Chassis->Right_Leg.l0,Chassis->Chassis_Ref.Leglength) + BODY_MASS/2*9.81f + Chassis->Roll_Balance_F_Right + Chassis->Right_Leg.Gasspring_FN;
    }
    else
    {
        Chassis->Left_Leg.Leg_F = PID_Calc(&Chassis->Left_Leg.Leg_Length_PID,Chassis->Left_Leg.l0,Chassis->Chassis_Ref.Leglength) + BODY_MASS/2*9.81f + Chassis->Roll_Balance_F_Left + Chassis->Left_Leg.Gasspring_FN;
        Chassis->Right_Leg.Leg_F = PID_Calc(&Chassis->Right_Leg.Leg_Length_PID,Chassis->Right_Leg.l0,Chassis->Chassis_Ref.Leglength) + BODY_MASS/2*9.81f + Chassis->Roll_Balance_F_Right + Chassis->Right_Leg.Gasspring_FN;
    }
    
    
    //设置左腿关节扭矩
    if(Wheel_State_Estimate(&Chassis->Left_Leg) || Chassis->balance_loop.L0 >= 0.25)
    {
        
         leg_conv(Chassis->Left_Leg.Leg_F, (Chassis->Balance_Tpgain - Chassis->Harmonize_Inner)/2.0f,
         Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
        
         Chassis->joint_T[1] = JM2_POLARITY * Chassis->Left_Leg.T_Set[0];
         Chassis->joint_T[2] = JM3_POLARITY * Chassis->Left_Leg.T_Set[1];
        
         Chassis->driving_T[0] = ((Chassis->Balance_Tgain + Chassis->V_w_Torque*Chassis->vw_limit_rate)/2) * LEFT_WHEEL_POLARITY;//还得叠加转向
      
    }
    else//离地
    {
        leg_conv(Chassis->Left_Leg.Leg_F, (Chassis->Balance_Tpoutlandgain - Chassis->Harmonize_Inner)/2.0f,
        Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
       
        Chassis->joint_T[1] = JM2_POLARITY * Chassis->Left_Leg.T_Set[0];
        Chassis->joint_T[2] = JM3_POLARITY * Chassis->Left_Leg.T_Set[1];
        
        Chassis->driving_T[0] = 0;
    }
    
    
    //设置右腿关节扭矩
    if(Wheel_State_Estimate(&Chassis->Right_Leg) || Chassis->balance_loop.L0 >= 0.25)
    {
        leg_conv(Chassis->Right_Leg.Leg_F,(Chassis->Balance_Tpgain + Chassis->Harmonize_Inner)/2.0f, 
        Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
        
        Chassis->joint_T[0] = JM1_POLARITY * Chassis->Right_Leg.T_Set[0];
        Chassis->joint_T[3] = JM4_POLARITY * Chassis->Right_Leg.T_Set[1];
        
        Chassis->driving_T[1] = (Chassis->Balance_Tgain - Chassis->V_w_Torque*Chassis->vw_limit_rate/2) * RIGHT_WHEEL_POLARITY;
      
    }
    else//离地
    {
        leg_conv(Chassis->Right_Leg.Leg_F, (Chassis->Balance_Tpoutlandgain + Chassis->Harmonize_Inner)/2.0f, //7878
        Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
       
        Chassis->joint_T[0] = JM1_POLARITY * Chassis->Right_Leg.T_Set[0];
        Chassis->joint_T[3] = JM4_POLARITY * Chassis->Right_Leg.T_Set[1];
       
        Chassis->driving_T[1] = 0;
    }
    
    
    
    //力矩限幅
    VAL_LIMIT(Chassis->joint_T[1],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[2],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[0],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[3],-JOINT_MAX_T, JOINT_MAX_T);
    
    VAL_LIMIT(Chassis->driving_T[0],-WHEEL_MAX_T,WHEEL_MAX_T);
    VAL_LIMIT(Chassis->driving_T[1],-WHEEL_MAX_T,WHEEL_MAX_T);
}



float temp_leg = 0.15f;
void Chassis_Test_Handle(Balance_Chassis_t* Chassis)
{
//        //左腿腿长
//    PID_Init(&Chassis->Left_Leg.Leg_Length_PID,PID_POSITION,150,0,500,20000,20000);
//    
//    //右腿腿长
//    PID_Init(&Chassis->Right_Leg.Leg_Length_PID,PID_POSITION,150,0,500,20000,20000);
//    
//    if(Remote_DT7_data.Remote_clicker.s2 == DOWN)
//    {
//        Chassis->Chassis_Remote_Ref.Leglength = 0.14f;
//    }
//    if(Remote_DT7_data.Remote_clicker.s2 == MIDDLE)
//    {
//        Chassis->Chassis_Remote_Ref.Leglength = 0.20f;
//    }
//    if(Remote_DT7_data.Remote_clicker.s2 == UP)
//    {
//        Chassis->Chassis_Remote_Ref.Leglength = 0.30f;
//    }
  
      Chassis->Chassis_Ref.Leglength = trackRamp_leg(0.008,Chassis->Chassis_Ref.Leglength,temp_leg);
    
 //   Chassis->Harmonize_Outer = PID_Calc(&Chassis->Leg_Harmonize_Pid_Outer, (Chassis->Right_Leg.phi0 - Chassis->Left_Leg.phi0), 0.0f);
 //   Chassis->Harmonize_Inner = PID_Calc(&Chassis->Leg_Harmonize_Pid_Inner, (Chassis->Right_Leg.dphi0 - Chassis->Left_Leg.dphi0), Chassis->Harmonize_Outer);
    
      //腿部竖直力F的计算
      Chassis->Left_Leg.Leg_F = PID_Calc(&Chassis->Left_Leg.Leg_Length_PID,Chassis->Left_Leg.l0,Chassis->Chassis_Ref.Leglength); //+ BODY_MASS/2*9.81f + Chassis->Roll_Balance_F_Left;
      Chassis->Right_Leg.Leg_F = PID_Calc(&Chassis->Right_Leg.Leg_Length_PID,Chassis->Right_Leg.l0,Chassis->Chassis_Ref.Leglength);// + BODY_MASS/2*9.81f + Chassis->Roll_Balance_F_Right;

    
    

      leg_conv(Chassis->Left_Leg.Leg_F, (Chassis->Balance_Tpgain - Chassis->Harmonize_Inner)/2.0f, //7878
      Chassis->Left_Leg.phi1, Chassis->Left_Leg.phi4, Chassis->Left_Leg.T_Set);
        
      Chassis->joint_T[1] = JM2_POLARITY * Chassis->Left_Leg.T_Set[0];
      Chassis->joint_T[2] = JM3_POLARITY * Chassis->Left_Leg.T_Set[1];
        
      Chassis->driving_T[0] = 0;//((Chassis->Balance_Tgain + Chassis->V_w_Torque*Chassis->vw_limit_rate)/2) * LEFT_WHEEL_POLARITY;//还得叠加转向
  
  
  
      leg_conv(Chassis->Right_Leg.Leg_F,(Chassis->Balance_Tpgain + Chassis->Harmonize_Inner)/2.0f, 
      Chassis->Right_Leg.phi1, Chassis->Right_Leg.phi4, Chassis->Right_Leg.T_Set);
        
      Chassis->joint_T[0] = JM1_POLARITY * Chassis->Right_Leg.T_Set[0];
      Chassis->joint_T[3] = JM4_POLARITY * Chassis->Right_Leg.T_Set[1];
        
      Chassis->driving_T[1] = (Chassis->Balance_Tgain - Chassis->V_w_Torque*Chassis->vw_limit_rate/2) * RIGHT_WHEEL_POLARITY;
   
   
    
    
    
    //力矩限幅
    VAL_LIMIT(Chassis->joint_T[1],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[2],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[0],-JOINT_MAX_T, JOINT_MAX_T);
    VAL_LIMIT(Chassis->joint_T[3],-JOINT_MAX_T, JOINT_MAX_T);
    
    VAL_LIMIT(Chassis->driving_T[0],-WHEEL_MAX_T,WHEEL_MAX_T);
    VAL_LIMIT(Chassis->driving_T[1],-WHEEL_MAX_T,WHEEL_MAX_T);
}


/**
************************************************************************************************************************
* @Name     : Chassis_Control_Loop
* @brief    : 底盘控制循环
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/
void Chassis_Control_Loop(Balance_Chassis_t* Chassis)
{
    switch (Chassis->Control_Mode)
    {
        //底盘失能
        case CHASSIS_RELAX :
        {
            Chassis_Relax_Handle(Chassis);
        }
        break;
        //初始化
        case CHASSIS_INIT :
        {
            if(Chassis->USART_Chassis_Data.fn_2_trigger_flag == 1)
            {
                Chassis_Single_Leg_Control_Handle(Chassis);
            }
            else
            {
                Chassis_Init_Handle(Chassis);
            }
        }
        break;
        //站立模式
        case CHASSIS_STAND_MODE :
        {
            Chassis_Standup_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //手动跟随遥控
        case MANUAL_FOLLOW_REMOTE :
        {
            Chassis_Fallow_Gimbal_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //停止模式，打符用
        case CHASSIS_STOP:
        {
            Chassis_Stop_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //小陀螺
        case CHASSIS_CLOCKWISE_ROTATE :
        {
            Chassis_Rotate_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //小陀螺
        case CHASSIS_ANTI_CLOCKWISE_ROTATE :
        {
            Chassis_Rotate_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //小陀螺
        case CHASSIS_CLOCKWISE_ROTATE_VAR_SPEED :
        {
             Chassis_Rotate_Handle(Chassis);
             Balance_Task(Chassis);
        }
        break;
        //小陀螺
        case CHASSIS_ANTI_CLOCKWISE_ROTATE_VAR_SPEED :
        {
             Chassis_Rotate_Handle(Chassis);
             Balance_Task(Chassis);
        }
        break;
        //大跳上台阶
        case CHASSIS_JUMP_UP :
        {
            Chassis_Jump_Up_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //反飞坡
        case CHASSIS_ANTI_FLY_SLOPE :
        {
            Chassis_AntiFly_Slope_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        //小跳下台阶
        case CHASSIS_JUMP_DOWN :
        {
            Chassis_Jump_Down_Handle(Chassis);
            Balance_Task(Chassis);
        }
        break;
        default:
        break;
    }
}



/**
************************************************************************************************************************
* @Name     : Chassis_Task
* @brief    : 底盘控制任务
* @param	: Balance_Chassis_t* Chassis
* @retval   : void
* @Note     : 底盘控制任务，放定时器
************************************************************************************************************************
**/
void Chassis_Task(Balance_Chassis_t* Chassis)
{
    Chassis_State_Update(Chassis);//底盘状态更新
    Chassis_Mode_Select(Chassis);//底盘模式选择
    Chassis_Referance_Update(Chassis);//底盘参考值更新
    Chassis_Control_Loop(Chassis);//底盘控制循环
}


