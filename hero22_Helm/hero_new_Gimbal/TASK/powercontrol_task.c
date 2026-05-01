#include "powercontrol_task.h"


/**
 * @brief 底盘功率控制任务
 *        
 *        
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */

float Set_Power_test=100;
/**
 * @brief 获取底盘最大可输出功率
 * 
 * @return float 最大可输出功率
 */
float PowerControl_MaxPower_Get(Chassis_t *pChassis)
{
    if(SuperCap_Recv.cap_voltage_filt > 22.5f)
    {
        pChassis->SuperCap_State = Cap_Fulling;
    }
    else if(SuperCap_Recv.cap_voltage_filt < 11.0f)     //超电电压不够预警
    {
        pChassis->SuperCap_State = Cap_Deading;
    }

    if((pChassis->SuperCap_State == Cap_Fulling || pChassis->SuperCap_State == Cap_Useing)
        &&SuperCap_Recv.cap_voltage_filt < 22.5f)
    {
        pChassis->SuperCap_State = Cap_Useing;
    }
    else if((pChassis->SuperCap_State == Cap_Deading || pChassis->SuperCap_State == Cap_Chargeing)
        &&SuperCap_Recv.cap_voltage_filt < 22.5f)
    {
        pChassis->SuperCap_State = Cap_Chargeing;
    }

   
    if(pChassis->SuperCap_State == Cap_Fulling || pChassis->SuperCap_State == Cap_Useing)
    {
        pChassis->MaxPower = 150.0f;
    }
    else if(pChassis->SuperCap_State == Cap_Chargeing)
    {
        pChassis->MaxPower = judge_rece_mesg.game_robot_state.chassis_power_limit - 30.0f;
    }else if(pChassis->SuperCap_State == Cap_Deading)
    {
        pChassis->MaxPower == judge_rece_mesg.game_robot_state.chassis_power_limit -50.0f;
    }

    //先设置个50瓦测测
    // return pChassis->MaxPower = Set_Power_test;

    return pChassis->MaxPower;
}

float m_test,n_test,l_test,delta_test,v_rate_test;	//2025/11/16fuck终于找到bug了
float sqrt_test,result_test;
//舵向电机和行进电机数量
int HELM_NUM = 4;
int DRIVE_NUM = 4;
/**
 * @brief 获取底盘速度限制系数
 * 
 * @param pChassis 
 * @return float 速度ref限制系数
 */
float PowerControl_SpeedLimitRate_Get(Chassis_t *pChassis)
{
#if PWC_FITTING_FORMULA == 0
    //i = a*k_rate + b;
    static float a[4];
    for(int i=0;i<4;i++)
        a[i] = pChassis->Driving_Speed_Ref[i]*(pChassis->Pid_Driving_Speed_Motor[i].Kp + pChassis->Pid_Driving_Speed_Motor[i].Kd);
    static float b[4];
    for(int i=0;i<4;i++)
        b[i] = -pChassis->Pid_Driving_Speed_Motor[i].Kp * pChassis->Driving_Speed_Fdb[i] \
               +pChassis->Pid_Driving_Speed_Motor[i].iout \
               -pChassis->Pid_Driving_Speed_Motor[i].Kd * pChassis->Driving_Speed_Fdb[i] \
               -pChassis->Pid_Driving_Speed_Motor[i].Kd * pChassis->Pid_Driving_Speed_Motor[i].err[LAST];
    
    //P = m*k_rate**2 + n*k_rate + l
    float m=0;
    for(int i=0;i<4;i++)
        m += a[i]*a[i]*PWC_K4;

    float n=0;
    for(int i=0;i<4;i++)
        n +=  2.0f*a[i]*b[i]*PWC_K4 \
             +PWC_K3*pChassis->Driving_Speed_Fdb[i]*a[i] \
             +a[i]*PWC_K1;
             
    float l=0;
    for(int i=0;i<4;i++)
        l +=  PWC_K4*b[i]*b[i] \
             +PWC_K3*pChassis->Driving_Speed_Fdb[i]*b[i] \
             +PWC_K5*pChassis->Driving_Speed_Fdb[i]*pChassis->Driving_Speed_Fdb[i] \
             +PWC_K0 \
             +PWC_K2*pChassis->Driving_Speed_Fdb[i] \
             +PWC_K1*b[i];
    l -= pChassis->MaxPower;         

    //判断二元一次方程是否有解
    static float delta;
    delta = n*n - 4.0f*m*l;
	
	//test
	m_test = m;
	n_test = n;
	l_test = l;
	delta_test = delta;
	
//	sqrt_test = sqrtf(delta);
//	result_test = (-n + sqrt_test)/()
	
    if(delta > 0)   
		pChassis->SpeedLimitRate = (-n + sqrtf(delta))/(2*m);
    else	//无解
        pChassis->SpeedLimitRate = 0.5f;

	v_rate_test = pChassis->SpeedLimitRate;
    //返回对速度的限制系数
    return pChassis->SpeedLimitRate;
#endif

#if PWC_FITTING_FORMULA == 1
/*
	float a[4];
    for(int i=0;i<4;i++)
        a[i] = (float)pChassis->Driving_Speed_Ref[i]*(pChassis->Pid_Driving_Speed_Motor[i].Kp + pChassis->Pid_Driving_Speed_Motor[i].Kd);
    float b[4];
    for(int i=0;i<4;i++)
        b[i] = (float)(-pChassis->Pid_Driving_Speed_Motor[i].Kp*pChassis->Driving_Speed_Fdb[i] \
               +pChassis->Pid_Driving_Speed_Motor[i].iout \
               -pChassis->Pid_Driving_Speed_Motor[i].Kd*pChassis->Driving_Speed_Fdb[i] \
               -pChassis->Pid_Driving_Speed_Motor[i].Kd*pChassis->Pid_Driving_Speed_Motor[i].err[LAST]);

    float m=0;
    for(int i=0;i<4;i++)
    {
        m += a[i]*a[i]*FACTOR_2;
    }
    float n=0;
    for(int i=0;i<4;i++)
    {
        n += 2*FACTOR_2*a[i]*b[i] \
             +FACTOR_1*a[i] \
             +I_TIMES_V_TO_WATT*a[i]*pChassis->Driving_Speed_Fdb[i];
    }
    float l=0;
    for(int i=0;i<4;i++)
    {
        l += b[i]*b[i]*FACTOR_2 \
             +b[i]*FACTOR_1 \
             +I_TIMES_V_TO_WATT*b[i]*pChassis->Driving_Speed_Fdb[i] \
             +FACTOR_0 \
             -pChassis->MaxPower/4.0f;
    }
    float delta = n*n - 4*m*l;
    if((int32_t)delta < 0)
	{
		pChassis->SpeedLimitRate = 0.5f;
	}
    else 
	{
		pChassis->SpeedLimitRate = (-n + sqrtf(delta)+1.0f)/(2*m);
	}
	
    return pChassis->SpeedLimitRate;
*/
    float a[4];
    for(int i=0;i<4;i++)
        a[i] = (float)pChassis->Driving_Speed_Ref[i]*(pChassis->Pid_Driving_Speed_Motor[i].Kp);
    float b[4];
    for(int i=0;i<4;i++)
        b[i] = (float)(-pChassis->Pid_Driving_Speed_Motor[i].Kp*pChassis->Driving_Speed_Fdb[i]);

    float m=0;
    for(int i=0;i<4;i++)
    {
        m += a[i]*a[i]*FACTOR_2;
    }
    float n=0;
    for(int i=0;i<4;i++)
    {
        n += 2*FACTOR_2*a[i]*b[i] \
             +FACTOR_1*a[i] \
             +I_TIMES_V_TO_WATT*a[i]*pChassis->Driving_Speed_Fdb[i];
    }
    float l=0;
    for(int i=0;i<4;i++)
    {
        l += b[i]*b[i]*FACTOR_2 \
             +b[i]*FACTOR_1 \
             +I_TIMES_V_TO_WATT*b[i]*pChassis->Driving_Speed_Fdb[i] \
             +FACTOR_0 \
             -pChassis->MaxPower/4.0f;
    }
    float delta = n*n - 4*m*l;
	
	//test
	m_test = m;
	n_test = n;
	l_test =l;
	delta_test = delta;
	
    if((int32_t)delta < 0)
	{
		pChassis->SpeedLimitRate = 0.5f;
	}
    else 
	{
		pChassis->SpeedLimitRate = (-n + sqrtf(delta))/(2*m);
	}
	
	v_rate_test = pChassis->SpeedLimitRate;
	
    return pChassis->SpeedLimitRate;
#endif
}

/**
 * @brief 底盘行进轮功率重新分配
 * 
 * @param pChassis 
 */
void PowerControl_DrivingPower_ReDistribution(Chassis_t *pChassis)
{
    static float Power_Distribution_min = 7.0f;     //单电机最小可分配的功率

    if(1)//误差分配
    {
        float Err_Sum=0,Err[4],Err_Null_Cnt;
        for(int i=0;i<4;i++)
        {
            Err[i] = fabs(pChassis->Driving_Speed_Ref[i]) - fabs(pChassis->Driving_Speed_Fdb[i]);
			Err[i] = (Err[i]>0)?Err[i]:0;
            Err_Sum += Err[i];
            if(Err[i] == 0)Err_Null_Cnt++;
        }
        
        for(int i=0;i<4;i++)
        {
            if(Err[i] == 0)pChassis->Power_Limit_ReDistrib[i] = Power_Distribution_min;
            else{
                if(Err_Sum != 0)
                    pChassis->Power_Limit_ReDistrib[i] = (pChassis->MaxPower - Err_Null_Cnt*Power_Distribution_min)*(Err[i]/Err_Sum);
            }
        }
    }

}

/**
 * @brief 获取输出电流限制系数 系数将乘在速度环的输出上
 * 
 * @param pChassis 
 */
void PowerControl_CurrentLimitRate_Get(Chassis_t *pChassis)
{
#if PWC_FITTING_FORMULA == 0
    //I = i_out*k_rate
    //P = k0 + k1*I + k2*u + k3*I*u + k4*I**2 + k5*u**2
    float a,b,c;
    for(int i=0;i<4;i++)
    {
        a = PWC_K4*pChassis->Pid_Driving_Speed_Motor[i].out*pChassis->Pid_Driving_Speed_Motor[i].out;
        b = (PWC_K1 + PWC_K3*pChassis->Driving_Speed_Fdb[i])*pChassis->Pid_Driving_Speed_Motor[i].out;
        c =  PWC_K0 \
            +PWC_K2*pChassis->Pid_Driving_Speed_Motor[i].out \
            +PWC_K3*pChassis->Driving_Speed_Fdb[i]*pChassis->Pid_Driving_Speed_Motor[i].out
			-pChassis->Power_Limit_ReDistrib[i];

        float delta;
        delta = b*b - 4.0f*a*c;
        if(delta > 0)
		{
			pChassis->CurrentLimitRate[i]=fabs((-b + sqrt(delta))/(2*a) / pChassis->Pid_Driving_Speed_Motor[i].out);
			Limit_To_Range(pChassis->CurrentLimitRate[i],0,1);
		}
        else	//无解
            pChassis->CurrentLimitRate[i]=0.1f;
    }
#endif

#if PWC_FITTING_FORMULA == 1
    float a=0,b=0,c=0;
    for(int i=0;i<4;i++)
    {
        a = FACTOR_2*pChassis->Pid_Driving_Speed_Motor[i].out*pChassis->Pid_Driving_Speed_Motor[i].out;
        b = FACTOR_1*pChassis->Pid_Driving_Speed_Motor[i].out \
            +I_TIMES_V_TO_WATT*pChassis->Driving_Speed_Fdb[i]*pChassis->Pid_Driving_Speed_Motor[i].out;
        c = FACTOR_0 - pChassis->Power_Limit_ReDistrib[i];

        float delta = b*b - 4*a*c;
        if(delta > 0)
		{
			pChassis->CurrentLimitRate[i] = (-b + sqrtf(delta))/(2*a);
			pChassis->CurrentLimitRate[i] = Limit_To_Range(pChassis->CurrentLimitRate[i],-1,1);
		}
        else
            pChassis->CurrentLimitRate[i] = 0.1f;
    }
#endif
}

/**
 * @brief 底盘速度Ref大小限制 乘以速度限制系数
 * 
 * @param pChassis 
 */
void PowerControl_Chassis_SpeedRef_Limit(Chassis_t *pChassis)
{
    pChassis->SpeedLimitRate = Limit_To_Range(pChassis->SpeedLimitRate,0,1);
    for(int i=0;i<4;i++)
        pChassis->Driving_Speed_Ref[i] = pChassis->Driving_Speed_Ref[i]*pChassis->SpeedLimitRate; 
}

/**
 * @brief 底盘速度环输出的转矩电流限制 乘以电流输出限制系数
 * 
 * @param pChassis 
 */
void PowerControl_Chassis_PID_Out_Limit(Chassis_t *pChassis)
{   
    for(int i=0;i<4;i++)
        pChassis->Pid_Driving_Speed_Motor[i].out = pChassis->Pid_Driving_Speed_Motor[i].out * pChassis->CurrentLimitRate[i];
}

float wheel_power[4];
/**
 * @brief 底盘功率观测
 * 
 * @param pChassis 
 */
void PowerControl_Power_Detect(Chassis_t *pChassis)
{
    for(int i=0;i<4;i++)
    {
        wheel_power[i] = I_TIMES_V_TO_WATT*pChassis->Driving_Speed_Fdb[i]*pChassis->Pid_Driving_Speed_Motor[i].out \
                        +FACTOR_2*pChassis->Pid_Driving_Speed_Motor[i].out*pChassis->Pid_Driving_Speed_Motor[i].out \
                        +FACTOR_1*pChassis->Pid_Driving_Speed_Motor[i].out \
                        +FACTOR_0 ;
    }
}
