#include "public.h"
#define POWER_LIMIT_SWITCH 1
uint8_t EN_POWER_CONTROL=1;   
chassis_t 	chassis;
float Helm_Chassis_bias[4]={90,153,31,-30};



void Chassis_Encoder_Get(void)
{
	for(int i=0;i<4;i++)
	{
		chassis.Speed_Fdb[i]=Helm_chassis.Driving_Encoder[i].rate_rpm;
		chassis.Helm_angle_fdb[i]=Helm_chassis.Heading_Encoder[i].ecd_angle;
		chassis.Helm_speed_fdb[i]=Helm_chassis.Heading_Encoder[i].rate_rpm;
	}
}

void Chassis_Pid_Cal(void)
{
	for(int i=0;i<4;i++)
	{
		chassis.Helm_3508_Out[i]=pid_calc(&chassis.pid_3508_motor_speed[i],chassis.Speed_Fdb[i],chassis.Speed_Ref[i]);
		chassis.Helm_speed_ref[i]=pid_calc(&chassis.pid_6020_motor_angle[i],chassis.Helm_angle_fdb[i],chassis.Helm_angle_ref[i]);
		chassis.Helm_6020_Out[i]=pid_calc(&chassis.pid_6020_motor_speed[i],chassis.Helm_speed_fdb[i],chassis.Helm_speed_ref[i]);
		
	}
	
}

void Chassis_Real_V_Cal(chassis_t *Chassis)
{
	Chassis->Vx_real=Chassis->Vx*Power_Limit_Rate_2;
	Chassis->Vy_real=Chassis->Vy*Power_Limit_Rate_2;
	Chassis->Vw_real=Chassis->Vw*Power_Limit_Rate_2;
}

/**********************
*@Brief:底盘输出限制函数
*@Call:内部或外部
*@Param:
*@Note:限制系数由功率控制部分计算得出
*@RetVal:无
**********************/
void Chassis_Out_Limit(chassis_t* Chassis)
{
	for(int i=0;i<4;i++)
	{
		Chassis->Speed_Ref[i]=Chassis->Speed_Ref[i]*Power_Limit_Rate_2;
	}
}

/**********************
*@Brief:舵轮解算函数
*@Call:内部或外部
*@Param:
*@Note:无
*@RetVal:无
**********************/
int Speed_Polarity[4];
	float Heading_Angle_Medium[4],Heading_Speed_Medium[4];
uint8_t Stop_Not_Turn_Flag;
void Chassis_Helm_Cal(chassis_t *Chassis)
{
	
	float static V_x_Medium[4],V_y_Medium[4];
	static float cosf45;
	cosf45 = cosf(PI/4);
	
	if(Chassis->Vx==0&&Chassis->Vy==0&&fabs(Chassis->Vw)<0.1)
		Stop_Not_Turn_Flag=1;
	if(Chassis->Vx!=0||Chassis->Vy!=0||fabs(Chassis->Vw)>0.1)
		Stop_Not_Turn_Flag=0;
	if(!Stop_Not_Turn_Flag)
	{
		for(int i=0;i<4;i++)
		{
		/**************************↓基础解算↓***************************/
		/**方向计算 */
//		Chassis->Vw=Chassis->Vw*cosf45*0.4f;
		
		Heading_Angle_Medium[0] = atan2(Chassis->Vy + Chassis->Vw * cosf45,
								Chassis->Vx + Chassis->Vw * cosf45);
		Heading_Angle_Medium[1] = atan2(Chassis->Vy - Chassis->Vw * cosf45,
								Chassis->Vx + Chassis->Vw * cosf45);
		Heading_Angle_Medium[2] = atan2(Chassis->Vy - Chassis->Vw * cosf45,
								Chassis->Vx - Chassis->Vw * cosf45);
		Heading_Angle_Medium[3] = atan2(Chassis->Vy + Chassis->Vw * cosf45,
								Chassis->Vx - Chassis->Vw * cosf45);
		/**速度解算 */
		Heading_Speed_Medium[0] = sqrt(	(Chassis->Vy + Chassis->Vw*cosf45)*(Chassis->Vy + Chassis->Vw*cosf45) +
										(Chassis->Vx + Chassis->Vw*cosf45)*(Chassis->Vx + Chassis->Vw*cosf45));
		Heading_Speed_Medium[1] = sqrt(	(Chassis->Vy - Chassis->Vw*cosf45)*(Chassis->Vy - Chassis->Vw*cosf45) +
										(Chassis->Vx + Chassis->Vw*cosf45)*(Chassis->Vx + Chassis->Vw*cosf45));
		Heading_Speed_Medium[2] = sqrt(	(Chassis->Vy - Chassis->Vw*cosf45)*(Chassis->Vy - Chassis->Vw*cosf45) +
										(Chassis->Vx - Chassis->Vw*cosf45)*(Chassis->Vx - Chassis->Vw*cosf45));
		Heading_Speed_Medium[3] = sqrt(	(Chassis->Vy + Chassis->Vw*cosf45)*(Chassis->Vy + Chassis->Vw*cosf45) +
										(Chassis->Vx - Chassis->Vw*cosf45)*(Chassis->Vx - Chassis->Vw*cosf45));

		/************************↑基础解算↑***************************/
			Heading_Angle_Medium[i]=Heading_Angle_Medium[i]*180/PI;
			
			/**************************↓取劣弧↓***************************/
			while((Heading_Angle_Medium[i]+180*Speed_Polarity[i])-Chassis->Helm_angle_fdb[i]-Helm_Chassis_bias[i]>90)
			{
				Speed_Polarity[i]-=1;
			}
			while((Heading_Angle_Medium[i]+180*Speed_Polarity[i])-Chassis->Helm_angle_fdb[i]-Helm_Chassis_bias[i]<-90)
			{
				Speed_Polarity[i]+=1;
			}
		
			Chassis->Helm_angle_ref[i]=Heading_Angle_Medium[i]+180*Speed_Polarity[i]-Helm_Chassis_bias[i];
			
			if(Speed_Polarity[i]%2==1||Speed_Polarity[i]%2==-1)
				//3508转速给定值（rpm）=解算出的单轮速度（m/s）* 轮毂3508减速比 * 60/轮子直径
				Chassis->Speed_Ref[i]=Heading_Speed_Medium[i]*16.9/*轮毂电机减速比*/*60/*RPM->RPS*//(PI*0.12f/*轮子周长*/);
			else
				Chassis->Speed_Ref[i]=-Heading_Speed_Medium[i]*16.9/*轮毂电机减速比*/*60/*RPM->RPS*//(PI*0.12f/*轮子周长*/);
			
			/*************************↑取劣弧↑***************************/
		}
	}
	else
	{
		for(int i=0;i<4;i++)
			Chassis->Speed_Ref[i]=0;
	}
}

void Power_Limit(void)
{

	Helm_Power_Limit_Handle();
	get_6020power();
	Chassis_Driver_Power_Distribution(&chassis);
	
	Limit_Rate_Get_By_Current(&chassis);
	
	#if POWER_LIMIT_SWITCH
	Chassis_Out_Limit(&chassis);//限制速度
	#endif 
	Chassis_Real_V_Cal(&chassis);//方便调试
	
	Chassis_Pid_Cal();
	
	for(int i=0;i<4;i++)
		chassis.Helm_3508_Out[i]=chassis.Power_Limit_By_Current_k[i]*chassis.Helm_3508_Out[i];
	
	
}






void Chassis_Follow_Gimbal_Handle(void)
{    
	Chassis_Helm_Cal(&chassis);
	if(EN_POWER_CONTROL==0)
	{
		Chassis_Pid_Cal();
	}
	else
	{
		Power_Limit();
	}
}

void Chassis_Separate_Gimbal_Handle(void)
{
	Chassis_Helm_Cal(&chassis);
	Chassis_Pid_Cal();
}

void Chassis_Rotate_Handle(void)
{
	Chassis_Helm_Cal(&chassis);
	if(EN_POWER_CONTROL==0)
	{
		Chassis_Pid_Cal();
	}
	else
	{
		Power_Limit();
	}
}



void Chassis_Stop_Handle(void)
{
	for(int i=0;i<4;i++)
	{
		chassis.Helm_angle_ref[i]=-45-Helm_Chassis_bias[i]+i*90;
		while((chassis.Helm_angle_ref[i]-chassis.Helm_angle_fdb[i])>90)
		{
			chassis.Helm_angle_ref[i]-=180;
		}
		while((chassis.Helm_angle_ref[i]-chassis.Helm_angle_fdb[i])<-90)
		{
			chassis.Helm_angle_ref[i]+=180;
		}
	}
	if(EN_POWER_CONTROL==0)
	{
		Chassis_Pid_Cal();
	}
	else
	{
		Power_Limit();
	}
	for(int i=0;i<4;i++)
		chassis.Helm_3508_Out[i]=0;
//		chassis.Speed_Ref[i]=0;
}

void Chassis_Relax_Handle(void)
{
	for(int i=0;i<4;i++)
	{
		chassis.Helm_3508_Out[i]=0;
		chassis.Helm_6020_Out[i]=0;
	}
}

void Chassis_ModeSelect(void)
{
    
    //Chassis_Ramp(&chassis);
    

    switch(chassis.ctrl_mode)
    {
        case AUTO_FOLLOW_GIMBAL:
            Chassis_Follow_Gimbal_Handle();
			chassis.last_ctrl_mode = chassis.ctrl_mode;
		break;
        
        case CHASSIS_STOP:
            Chassis_Stop_Handle();
			chassis.last_ctrl_mode = chassis.ctrl_mode;
        break;
		
		case CHASSIS_ROTATE:
			Chassis_Rotate_Handle();
			chassis.last_ctrl_mode = chassis.ctrl_mode;
		break;
		
		case CHASSIS_REVERSE_ROTATE:
			Chassis_Rotate_Handle();
			chassis.last_ctrl_mode = chassis.ctrl_mode;
		break;
		
		case CHASSIS_RELAX:
			Chassis_Relax_Handle();
        default:
			
        break;
    }
}

/*缓起步函数*/
void Chassis_Ramp(chassis_t *_Chassis)
{
		float static V_x_Last,V_y_Last,W_Last;
		u16 static Ramp_CNT;
		
		u16 static Chassis_Fast_CNT_x,Chassis_Fast_CNT_y,Chassis_Fast_CNT;
		
		Chassis_Fast_CNT=0;
		if(fabs(_Chassis->Speed_Fdb[0])>100
		 ||fabs(_Chassis->Speed_Fdb[1])>100
		 ||fabs(_Chassis->Speed_Fdb[2])>100
		 ||fabs(_Chassis->Speed_Fdb[3])>100)
		{
			if(Chassis_Fast_CNT_x<1000)
				Chassis_Fast_CNT++;
		}
		else
			Chassis_Fast_CNT=0;
		
		
		if(Ramp_CNT<10)
		{
			//该段作用：使速度参考值100ms刷新一次
			Ramp_CNT++;
			_Chassis->ChassisSpeed_Ref.forward_back_ref=V_x_Last;
			_Chassis->ChassisSpeed_Ref.left_right_ref=V_y_Last;
			//_Chassis->Vw=W_Last;
		}
		else
		{
			Ramp_CNT=0;
			/**************************↓V_x↓***************************/
			//速度反向时，将速度先设为0再慢慢贴近参考值
			if(_Chassis->ChassisSpeed_Ref.forward_back_ref!=0&&Chassis_Fast_CNT<1000)
			{
				if(V_x_Last*_Chassis->ChassisSpeed_Ref.forward_back_ref<0)
					_Chassis->ChassisSpeed_Ref.forward_back_ref=0;
				else //if(Chassis_Fast_CNT_x!=0)
				{
					
					if(_Chassis->ChassisSpeed_Ref.forward_back_ref-V_x_Last>CHASSIS_RAMP_X_STEP_SIZE)
						_Chassis->ChassisSpeed_Ref.forward_back_ref=V_x_Last+CHASSIS_RAMP_X_STEP_SIZE;
					else if(_Chassis->ChassisSpeed_Ref.forward_back_ref-V_x_Last<-CHASSIS_RAMP_X_STEP_SIZE)
						_Chassis->ChassisSpeed_Ref.forward_back_ref=V_x_Last-CHASSIS_RAMP_X_STEP_SIZE;
				}
			}
			/*************************↑V_x↑***************************/
		
		
			/**************************↓V_y↓***************************/
			if(_Chassis->ChassisSpeed_Ref.left_right_ref!=0&&Chassis_Fast_CNT<1000)
			{
				if(V_y_Last*_Chassis->ChassisSpeed_Ref.left_right_ref<0)
				{
					_Chassis->ChassisSpeed_Ref.left_right_ref=0;
				}
				else// if(Chassis_Fast_CNT_y!=0)
				{
					if(_Chassis->ChassisSpeed_Ref.left_right_ref-V_y_Last>CHASSIS_RAMP_Y_STEP_SIZE)
						_Chassis->ChassisSpeed_Ref.left_right_ref=V_y_Last+CHASSIS_RAMP_Y_STEP_SIZE;
					else if(_Chassis->ChassisSpeed_Ref.left_right_ref-V_y_Last<-CHASSIS_RAMP_Y_STEP_SIZE)
						_Chassis->ChassisSpeed_Ref.left_right_ref=V_y_Last-CHASSIS_RAMP_Y_STEP_SIZE;
				}
			}

			
			/*************************↑V_y↑***************************/
		
		
			/**************************↓w↓***************************/
			
			
			
			
			
			
			
			
			
			
			/*************************↑w↑***************************/
		
		}
		
		
		V_x_Last=_Chassis->ChassisSpeed_Ref.forward_back_ref;
		V_y_Last=_Chassis->ChassisSpeed_Ref.left_right_ref;
}






