#include "public.h"
extern chassis_t 	chassis;
extern can_capacitance_message_t can_capacitance_message;
float Power_Limit_Rate;
float Power_Limit_Rate_2;
u16  Max_Power;

void New_Speed_PID_Init(chassis_t* chassis)
{
    PID_struct_init(&chassis->chassis_new_speed[0],POSITION_PID,10000,1500,7,0.002,0);
    PID_struct_init(&chassis->chassis_new_speed[1],POSITION_PID,10000,1500,7,0.002,0);
    PID_struct_init(&chassis->chassis_new_speed[2],POSITION_PID,10000,1000,4,0.002,0);
    PID_struct_init(&chassis->chassis_new_speed[3],POSITION_PID,10000,1000,4,0.002,0);
}





/***********************************↓功率控制函数↓***************************************/
/**********************
*@Brief:驱动电机分配函数
*@Call:内部或外部
*@Param:
@*_Chassis：底盘结构体
*@Note:无
*@RetVal:无
**********************/
float Chassis_Error[4],Chassis_Error_Sum;
u8 Power_Surplus_CNT;
void Chassis_Driver_Power_Distribution(chassis_t *_Chassis)
{
    Chassis_Error_Sum = 0;
    Power_Surplus_CNT = 0;
    
	float static Power_Distribution_min=5,Power_Distribution_Ratio=1/3;
	for(int i=0;i<4;i++)
	{
		//设定值绝对值与反馈值绝对值的差作差作为功率分配的判断条件，当反馈值绝对值大于设定值绝对值时认为功率过剩
		//功率过剩为一切原因导致的反馈速度高于设定速度，如底盘倾斜导致的摩擦力减弱甚至悬空等
		Chassis_Error[i]=(fabs(_Chassis->Speed_Ref[i])-fabs(_Chassis->Speed_Fdb[i]));
		Chassis_Error[i]=(Chassis_Error[i]>0)?Chassis_Error[i]:0;
		Chassis_Error_Sum+=Chassis_Error[i];
		if(Chassis_Error[i]==0)Power_Surplus_CNT++;
	}
	if(1)//误差分配
	{
		for(int i=0;i<4;i++)
		{
			if(Chassis_Error[i]==0)_Chassis->Power_Limit[i]=Power_Distribution_min;//该值为最小分配额度，可调整
			else
			{
				//一般情况下Chassis_Error_Sum不可能等于零但为防止特俗情况导致其等于0出现inf加该判断
				if(Chassis_Error_Sum!=0)
					_Chassis->Power_Limit[i]=(_Chassis->Max_Power-Power_Distribution_min*Power_Surplus_CNT)*Chassis_Error[i]/Chassis_Error_Sum;	
			}
		}
        
//        for(int i=0; i < 4; i++) 
//		{
//            //chassis.Power_Limit_Total+=chassis.Power_Limit[i];
//			chassis.Power_Limit_By_Current_k[i] = 0.9;
////            VAL_LIMIT(chassis.Power_Limit_By_Current_k[i],0,1);
//		}
		
	}
	
	Power_Surplus_CNT=0;
	Chassis_Error_Sum=0;
}

/**********************
*@Brief:限制系数计算函数
*@Call:内部或外部
*@Param:
*@Note:通过所设定的最大功率来计算速度的限制系数，最后的速度输出==速度*该系数
*@RetVal:无
**********************/
float Limit_Rate_Get(float max_power)
{
  float a[4];
  for(int i=0; i<4; i++)
    a[i]=(float)chassis.Speed_Ref[i]*(chassis.pid_3508_motor_speed[i].p+chassis.pid_3508_motor_speed[i].d);
  float b[4];
  for(int i=0; i<4; i++)
    b[i]=-chassis.pid_3508_motor_speed[i].p*(float)chassis.Speed_Fdb[i]+chassis.pid_3508_motor_speed[i].iout \
         -chassis.pid_3508_motor_speed[i].d*(float)chassis.Speed_Fdb[i]-chassis.pid_3508_motor_speed[i].d*chassis.chassis_new_speed[i].err[LAST];
  // Max_power=heat_power+drive_power
  //	i_n=a[n]*k+b[n]	带入
  //Max_Power=m*k^2+n*k+o
  //0=m*k^2+n*k+l(l=o-Max_Power)
  float m=(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3])*FACTOR_2;
    
  if(fabsf(m) < 1e-6f)
  {
    return 0.5f;
  }

  float n=2*FACTOR_2*(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]) + \
          FACTOR_1*(a[0] + a[1] + a[2] + a[3]) + \
          I_TIMES_V_TO_WATT*(a[0]*(float)chassis.Speed_Fdb[0] + \
                             a[1]*(float)chassis.Speed_Fdb[1] + \
                             a[2]*(float)chassis.Speed_Fdb[2] + \
                             a[3]*(float)chassis.Speed_Fdb[3]);

  float l=(b[0]*b[0] + b[1]*b[1] + b[2]*b[2] + b[3]*b[3])*FACTOR_2 + \
          (b[0] + b[1] + b[2] + b[3])*FACTOR_1 + \
          I_TIMES_V_TO_WATT*(b[0]*(float)chassis.Speed_Fdb[0] + \
                             b[1]*(float)chassis.Speed_Fdb[1] + \
                             b[2]*(float)chassis.Speed_Fdb[2] + \
                             b[3]*(float)chassis.Speed_Fdb[3])+ \
          4*FACTOR_0 - \
          max_power;
	if((float)(n*n-4*m*l)<0)//防止开根号负数出现浮点型运算错误
		return 0.5;
	else
		return (-n+(float)sqrt((double)(n*n-4*m*l)+1.0f))/(2*m);
}





/**********************
*@Brief:限制系数计算函数
*@Call:内部或外部
*@Param:
*@Note:
*@RetVal:无
**********************/
void Limit_Rate_Get_By_Current(chassis_t *_Chassis)
{
	float a,b,c;
	for(int i=0;i<4;i++)
	{
		a=FACTOR_2*_Chassis->pid_3508_motor_speed[i].out*_Chassis->pid_3508_motor_speed[i].out;
		b=FACTOR_1*_Chassis->pid_3508_motor_speed[i].out+I_TIMES_V_TO_WATT*_Chassis->Speed_Fdb[i]*_Chassis->pid_3508_motor_speed[i].out;
		c=FACTOR_0-_Chassis->Power_Limit[i];
		
		if(b*b-4*a*c>0)
		{
			if(chassis.Power_Limit[i]>5)
				_Chassis->Power_Limit_By_Current_k[i]=(-b+sqrt(b*b-4*a*c))/(2*a);
//			else if(chassis.Power_Limit[i]==5)
//				_Chassis->Power_Limit_By_Current_k[i]=-0.02;
		}
		else
			_Chassis->Power_Limit_By_Current_k[i]=0.1;
		
		VAL_LIMIT(_Chassis->Power_Limit_By_Current_k[i],-1,1);
		
	}
}



/**********************
*@Brief:最大功率限制函数
*@Call:内部或外部
*@Param:
voltage：功率控制反馈的电压值
*@Note:限制电压防止电压过低导致电机复位
*@RetVal:无
**********************/
float get_max_power(float voltage)//
{ 
	int static max_power=0;
	
  if(voltage>WARNING_VOLTAGE)	
		max_power=800;
  else
    max_power=70;//(voltage-WARNING_VOLTAGE)/3.0f*200;
	
	if(max_power>=8000)
		max_power=8000;
	else if(max_power<=0)
		max_power=0;
	
  return max_power;
}


/**********************
*@Brief:缓冲功率计算函数
*@Call:内部或外部
*@Param:void
*@Note:算出缓冲功率
*@RetVal:无
**********************/
void Buffer_Power(void)
{
	if(can_capacitance_message.cap_voltage_filte<22.5)
		Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+(judge_rece_mesg.power_heat_data.buffer_energy-10); //5
	else
		Max_Power=0;
	
	if(Max_Power>150)
		Max_Power=150;
	else if(Max_Power<0)
		Max_Power=0;
//	VAL_LIMIT(Max_Power,0,150);
}


/**********************
*@Brief:功率限制函数
*@Call:内部或外部
*@Param:
*@Note:计算出最后的速度限制系数
*@RetVal:无
**********************/
void Power_Limit_Handle(void)
{
	float static Voltage_Out_Medium;
	
	Voltage_Out_Medium=can_capacitance_message.cap_voltage_filte;
	//Capacitance_Message.Capacitance_Voltage=Voltage_Out_Medium/100;
	Power_Limit_Rate=Limit_Rate_Get(get_max_power(can_capacitance_message.cap_voltage_filte));

	if(Power_Limit_Rate>=1)
		Power_Limit_Rate=1;
	else if(Power_Limit_Rate<=0)
		Power_Limit_Rate=0;
	else if(Power_Limit_Rate!=Power_Limit_Rate)
		Power_Limit_Rate=0;
	
	Buffer_Power();
}







float Power_3508_Max;
float Power_Test=55;
u8 Power_Up_CNT;
u8 Power_Up_Flag;
u8 Power_Keep_CNT;
u8 Power_Keep_Flag;
float Power_Keep_V;
float Power_Up_V;

float Power=50;
u8 Flag_Fly_Ready;
uint8_t Using_Cap_Flag;
void Mec_Power_Limit_Handle(void)
{		
	/*↓补给区禁止用超电 防止超功率↓*/
//	if(judge_rece_mesg.ext_rfid_status.My_Recharge_Area||judge_rece_mesg.ext_rfid_status.My_Recharge_Area_)
//	{
//		chassis.Flag_Out_Break=0;
//	}
	/*↑补给区禁止用超电 防止超功率↑*/
	
	/*↓超电正常↓*/
		if(/*chassis.Flag_Out_Break==1&&*/can_capacitance_message.cap_voltage_filte>8&&Peripheral_State.Super_Cap.Link_State==CONNECTED)
		{
//			if(chassis.chassis_Mode==CHASSIS_FLYING_SLOPE)
//			{
//				chassis.Max_Power=330;
//			}
//			else
//			
			
			if(chassis.Chassis_Move_State==CHAS_STATE_STOP)
			{

				chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit;
			}
			if(chassis.Chassis_Move_State==CHAS_UP_SLOPE)
				chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+80;
			else if(chassis.Chassis_Move_State==CHAS_DOWN_SLOPE)
				chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit;
			if(chassis.chassis_speed_mode==HIGH_SPEED_MODE)
			{
				if(chassis.Chassis_Move_State==CHAS_STATE_RUN_READY)
				{
					chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+100;
				}
				else if(chassis.Chassis_Move_State==CHAS_STATE_RUN)
				{
					chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+60;
				}
			}
			else if(chassis.chassis_speed_mode==NORMAL_SPEED_MODE)
			{
				if(chassis.Chassis_Move_State==CHAS_STATE_RUN_READY)
				{
					chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+50;
				}
				else if(chassis.Chassis_Move_State==CHAS_STATE_RUN)
				{
					chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+30;
				}
			}
//				if(chassis.Chassis_Move_State == CHAS_STATE_RUN_READY)
//				chassis.Max_Power=judge_rece_mesg.game_robot_state.chassis_power_limit+100;
//				else if(chassis.Chassis_Move_State == CHAS_STATE_RUN)
//				chassis.Max_Power=judge_rece_mesg.game_robot_state.chassis_power_limit+50;
//				else if(chassis.Chassis_Move_State == CHAS_STATE_UP_SLOPE)
//				chassis.Max_Power=judge_rece_mesg.game_robot_state.chassis_power_limit+80;
//				else if(chassis.Chassis_Move_State == CHAS_STATE_DOWN_LOW_SLOPE
//					||chassis.Chassis_Move_State == CHAS_STATE_DOWN_HIGH_SLOPE)
//				chassis.Max_Power=judge_rece_mesg.game_robot_state.chassis_power_limit;
//			}
		}
	/*↑超电正常↑*/	
		
/*↓超电断联||电压过低↓*/	
		else
		{
			Using_Cap_Flag=0;
			chassis.Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit;
		}
		
/*↑超电断联||电压过低↑*/	
		
//测试用
//	chassis.Max_Power = Power_Test;
//	if(can_capacitance_message.cap_voltage_filte*16-50>0)
//	{
//		VAL_LIMIT(chassis.Max_Power,0,can_capacitance_message.cap_voltage_filte*16-50);
//	}
	
//	get_6020power();
	
	Power_3508_Max=chassis.Max_Power;
	
	/*↓超电坏了的时候解开我~↓*/
//	chassis.Max_Power=judge_rece_mesg.game_robot_state.chassis_power_limit+judge_rece_mesg.power_heat_data.buffer_energy-10;
	/*↑超电坏了的时候解开我~↑*/

//	chassis.Max_Power=80;//超电寄时或测试用
	
	Power_Limit_Rate_2 = Limit_Rate_Get(chassis.Max_Power);
	
	
//	Power_Limit_Rate_2=1;//容易直接过流死了
	
	VAL_LIMIT(Power_Limit_Rate_2,0,1);
}


/***********************************↑功率控制函数↑***************************************/

