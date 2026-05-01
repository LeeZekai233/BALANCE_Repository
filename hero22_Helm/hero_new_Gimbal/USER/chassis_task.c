#include "public.h"
#define LK_Angle 36		//YAW轴电机偏移
#define ROTATE_SPEED 4//小陀螺转速（rad/s）
#define POWER_LIMIT_SWITCH 1
uint8_t EN_POWER_CONTROL=0;   
chassis_t 	chassis;
Chassis_angle_t Chassis_angle;
int16_t wheel_rpm[4];
float Follow_Angle1,temp;
float Follow_Angle2;
uint8_t Flag_Dir_Reverse;
float Rate=19;
pid_t mec[4];
//float Power_Limit_Rate_2;

float Chassis_Dir;

int a[4];



extern Encoder_plus yaw_Encoder;
extern pid_t pid_follow_angle;
extern pid_t pid_follow_speed;
extern LK_M_t LK_M_Gimbal_Yaw;

//float test=0;
//float first;

float xy_k,w_k;

double last_angle1=0,angle1=0,angle1_end,follow_plus_W1,Chassis_Dir1;
int16_t cnt1=0;

double last_angle2,angle2,angle2_end,follow_plus_W2,Chassis_Dir2;
int16_t cnt2=0;

//power_t Power;


float testa,testb,testc,testi1,testi2,testiout;
float k_in_test[4];
float temp_new;
float temp_end;



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

/**
************************************************************************************************************************
* @Name     : Chassis_State_Detect
* @brief    : 底盘停止、起步、达到预设速度的阶段/上下坡判定
* @param	: None
* @retval   : void
* @Note     : 
************************************************************************************************************************
**/

void Chassis_State_Detect(chassis_t *chassis)
{
	u8 RUN_CNT=0;
	int static RUN_Timer=0;
	
	chassis->Inclination_Angle=gimbal_gyro.pitch_angle+(Pitch_Encoder.ecd_angle-Pitch_Encoder.Init_Angle)/100*0.475f;
	
	if((fabs(chassis->_3508_motor_speed_rpm[0]) + fabs(chassis->_3508_motor_speed_rpm[1])+ fabs(chassis->_3508_motor_speed_rpm[2])+ fabs(chassis->_3508_motor_speed_rpm[3])) < 3200)
	{
		chassis->Chassis_Move_State = CHAS_STATE_STOP;
	}
	else
	{
		for(int i=0;i<4;i++)
		{
			if(fabs(chassis->_3508_motor_speed_rpm[i])>CHAS_RUN_THRESHOLD)RUN_CNT++;
		}
		if(RUN_CNT>=2)
		{
			if(RUN_Timer<800)
			RUN_Timer++;
		}
		else RUN_Timer=0;
		
		if(chassis->Inclination_Angle>6.5)
		{
			chassis->Chassis_Move_State =CHAS_UP_SLOPE;
		}
		else if( chassis->Inclination_Angle < -15)
		{
			chassis->Chassis_Move_State = CHAS_DOWN_SLOPE;
		}
		else
		{
			if(RUN_Timer>=800)chassis->Chassis_Move_State=CHAS_STATE_RUN;
			else chassis->Chassis_Move_State=CHAS_STATE_RUN_READY;
		}
		
		
		
	}
	
}

void Power_Limit(void)
{
	Mec_Power_Limit_Handle();
	
	Chassis_Driver_Power_Distribution(&chassis);
	
	Limit_Rate_Get_By_Current(&chassis);
	
	#if POWER_LIMIT_SWITCH
	Chassis_Out_Limit(&chassis);//限制速度
	#endif 
	Chassis_Real_V_Cal(&chassis);//方便调试
	
	pid_calc(&chassis.pid_3508_motor_speed[0],Mecanum_chassis.Driving_Encoder[0].rate_rpm,chassis.Speed_Ref[0]);
	pid_calc(&chassis.pid_3508_motor_speed[1],Mecanum_chassis.Driving_Encoder[1].rate_rpm,chassis.Speed_Ref[1]);
	pid_calc(&chassis.pid_3508_motor_speed[2],Mecanum_chassis.Driving_Encoder[2].rate_rpm,chassis.Speed_Ref[2]);
	pid_calc(&chassis.pid_3508_motor_speed[3],Mecanum_chassis.Driving_Encoder[3].rate_rpm,chassis.Speed_Ref[3]);
	
	for(int i=0;i<4;i++)
	chassis.current[i]=chassis.Power_Limit_By_Current_k[i]*chassis.pid_3508_motor_speed[i].out;
	
}



void Mec_steel_target_rpm_cal(float V__x,float V__y,float w,chassis_t* chassis)//解算3508目标转速
{    
    wheel_rpm[0]=((-V__x+V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);//x型布局从第一象限开始逆时针标定轮子
    wheel_rpm[1]=((V__x+V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);//系数单位为米
    wheel_rpm[2]=((V__x-V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);//计算得出每分钟转速
    wheel_rpm[3]=((-V__x-V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);

    chassis->Speed_Fdb[0]=Mecanum_chassis.Driving_Encoder[0].rate_rpm;
    chassis->Speed_Fdb[1]=Mecanum_chassis.Driving_Encoder[1].rate_rpm;
    chassis->Speed_Fdb[2]=Mecanum_chassis.Driving_Encoder[2].rate_rpm;
    chassis->Speed_Fdb[3]=Mecanum_chassis.Driving_Encoder[3].rate_rpm;
    
    chassis->Speed_Ref[0]=((-V__x+V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);
    chassis->Speed_Ref[1]=((V__x+V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);
    chassis->Speed_Ref[2]=((V__x-V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);
    chassis->Speed_Ref[3]=((-V__x-V__y+0.39*w)/0.05*Rate)*60/(2*3.1415);
}


void Rpm_to_Rad_powercontrol(chassis_t* T)
{
    T->After_Mec_Cal_W_Ref[0]=wheel_rpm[0]/60*2*PI;
    T->After_Mec_Cal_W_Ref[1]=wheel_rpm[1]/60*2*PI;
    T->After_Mec_Cal_W_Ref[2]=wheel_rpm[2]/60*2*PI;
    T->After_Mec_Cal_W_Ref[3]=wheel_rpm[3]/60*2*PI;
}




float try= 0;
//int16_t aaa[4];
float test_angle;
void Chassis_Follow_Gimbal_Handle(void)
{    
//        if(chassis.last_ctrl_mode==CHASSIS_ROTATE || chassis.last_ctrl_mode==CHASSIS_REVERSE)//;劣弧优化
//       {
//            test_angle=LK_M_Gimbal_Yaw.Circle_Angle;
//            if(fabs(angle2-52)<=90)//正常摆头
//            {
//                
//            }
//            else//以尾为头
//            {
//                
//            }
//        }
//        else
        {
            last_angle1=angle1;
            angle1=LK_M_Gimbal_Yaw.Circle_Angle;
        
            if((angle1-last_angle1)<-320)
            {
                cnt1++;                        
            }
            else if((angle1-last_angle1)>320)
            {
                cnt1--;           
            }
        
//            temp=LK_M_Gimbal_Yaw.Circle_Angle+cnt1*360;
        
            angle1_end=LK_M_Gimbal_Yaw.Circle_Angle+cnt1*360-LK_Angle/*云台yaw轴电机偏置*/;
//            
//            if(test==0)
//            {
//                first=angle1_end;
//                test=1;
//            }
			//底盘跟随云台的劣弧优化
			while(angle1_end > 180)
			angle1_end -= 360;
			while(angle1_end < -180)
			angle1_end += 360;
			
			Chassis_Dir1=angle1_end;
			
			chassis.Vx = chassis.ChassisSpeed_Ref.forward_back_ref;
			chassis.Vy = chassis.ChassisSpeed_Ref.left_right_ref;
			//云台坐标系投影至底盘
			Chassis_Dir1=(Chassis_Dir1/180)*PI;//转为弧度制
			chassis.Vcx = chassis.Vy * sin(Chassis_Dir1) + chassis.Vx * cos(Chassis_Dir1);
			chassis.Vcy = chassis.Vy * cos(Chassis_Dir1) - chassis.Vx * sin(Chassis_Dir1);
			

			
            Follow_Angle1=(gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb+angle1_end-Follow_Angle_Medium);//底盘所需要跟随的角度,正负待测试

			while(Follow_Angle1 > 180)
			Follow_Angle1 -= 360;
			while(Follow_Angle1 < -180)
			Follow_Angle1 += 360;
			
			if(!((angle1_end>75&&angle1_end<105)||(angle1_end>-105&&angle1_end<-75)))
			{
				if(Follow_Angle1 > 90)
				Follow_Angle1 -= 180;
				if(Follow_Angle1 < -90)
				Follow_Angle1 += 180;
			}
			
			Follow_Angle1=Follow_Angle1/360*2*PI;
			if(Follow_Angle1<0.03&&Follow_Angle1>-0.03) Follow_Angle1=0;
			
			
			Chassis_Dir=Follow_Angle1;
			
            chassis.Vw=2.5*pid_double_loop_cal(&pid_follow_angle,&pid_follow_speed,0,Follow_Angle1,&pid_follow_angle.out,0,0);//目标偏差角度经过pid控制后为0,实现底盘跟随云台
			
//			Mec_steel_target_rpm_cal(chassis.Vcx,chassis.Vcy,chassis.Vw,&chassis);
			
//            if(EN_POWER_CONTROL==0)
//            {
//				
//                chassis.current[0]=pid_calc(&chassis.pid_3508_motor_speed[0],Mecanum_chassis.Driving_Encoder[0].rate_rpm,wheel_rpm[0]);
//                chassis.current[1]=pid_calc(&chassis.pid_3508_motor_speed[1],Mecanum_chassis.Driving_Encoder[1].rate_rpm,wheel_rpm[1]);
//                chassis.current[2]=pid_calc(&chassis.pid_3508_motor_speed[2],Mecanum_chassis.Driving_Encoder[2].rate_rpm,wheel_rpm[2]);
//                chassis.current[3]=pid_calc(&chassis.pid_3508_motor_speed[3],Mecanum_chassis.Driving_Encoder[3].rate_rpm,wheel_rpm[3]);
//            }
//            else
//			{
//				Power_Limit();
//            }
        }
}

void Chassis_Separate_Gimbal_Handle(void)
{
    Mec_steel_target_rpm_cal(chassis.ChassisSpeed_Ref.forward_back_ref,chassis.ChassisSpeed_Ref.left_right_ref,chassis.ChassisSpeed_Ref.rotate_ref,&chassis);
    chassis.current[0]=pid_calc(&chassis.pid_3508_motor_speed[0],Mecanum_chassis.Driving_Encoder[0].rate_rpm,wheel_rpm[0]);
    chassis.current[1]=pid_calc(&chassis.pid_3508_motor_speed[1],Mecanum_chassis.Driving_Encoder[1].rate_rpm,wheel_rpm[1]);
    chassis.current[2]=pid_calc(&chassis.pid_3508_motor_speed[2],Mecanum_chassis.Driving_Encoder[2].rate_rpm,wheel_rpm[2]);
    chassis.current[3]=pid_calc(&chassis.pid_3508_motor_speed[3],Mecanum_chassis.Driving_Encoder[3].rate_rpm,wheel_rpm[3]);
}

void Chassis_Rotate_Handle(void)
{
    //为头时编码器大概为LK_Angle度,编码器返回角度值为0-360,LK_Angle+180=229
        last_angle2=angle2;
        angle2=LK_M_Gimbal_Yaw.Circle_Angle;
        if((angle2-last_angle2)<-320)
        {
            cnt2++;
            
        }
        else if((angle2-last_angle2)>320)
        {
            cnt2--;
        }
        angle2_end=(angle2+cnt2*360-LK_Angle);//yaw连续绝对角度值
        while(angle2_end > 180)
		angle2_end -= 360;
		while(angle2_end < -180)
		angle2_end += 360;
//      Follow_Angle2=-angle2_end;//云台底盘角度差值弧度值
//      chassis.Vcx=chassis.Vy*sin(Follow_Angle2)+chassis.Vx*cos(Follow_Angle2);
//      chassis.Vcy=chassis.Vy*cos(Follow_Angle2)-chassis.Vx*sin(Follow_Angle2);
        
        chassis.Vx = chassis.ChassisSpeed_Ref.forward_back_ref;
		chassis.Vy = chassis.ChassisSpeed_Ref.left_right_ref;
        
		Chassis_Dir2=angle2_end;
		//云台坐标系投影至底盘
		Chassis_Dir2=(Chassis_Dir2/180)*PI;//转为弧度制
		chassis.Vcx = chassis.Vy * sin(Chassis_Dir2) + chassis.Vx * cos(Chassis_Dir2);
		chassis.Vcy = chassis.Vy * cos(Chassis_Dir2) - chassis.Vx * sin(Chassis_Dir2);
		
		Chassis_Dir=Chassis_Dir2;
        if(chassis.ctrl_mode==CHASSIS_ROTATE)
        {
            if(chassis.chassis_speed_mode==HIGH_SPEED_MODE)
            {
                chassis.Vw=ROTATE_SPEED*2;
            }
			else
            {
                chassis.Vw=ROTATE_SPEED;//660*0.002
            }
        }
        else if(chassis.ctrl_mode==CHASSIS_REVERSE_ROTATE)
        {
            
			if(chassis.chassis_speed_mode==HIGH_SPEED_MODE)
            {
                chassis.Vw=-ROTATE_SPEED*2;
            }
			else
            {
                chassis.Vw=-ROTATE_SPEED;//660*0.002
            }
        }
      
}



void Chassis_Stop_Handle(void)
{
    
    chassis.current[0]=0;
    chassis.current[1]=0;
    chassis.current[2]=0;
    chassis.current[3]=0;
    for(uint8_t i = 0; i<4; i++)
    {
        VAL_LIMIT(chassis.current[i],-8000,8000);  
    }
}
float V_x_Set,V_y_Set,W_Set;
float V_x_Now,V_y_Now,W_Now;
/*缓起步函数*/
void Chassis_Ramp(chassis_t *_Chassis)
{

	V_x_Set=_Chassis->ChassisSpeed_Ref.forward_back_ref;
	V_y_Set=_Chassis->ChassisSpeed_Ref.left_right_ref;
	if(V_x_Now>V_x_Set&&fabsf(V_x_Now-V_x_Set)>CHASSIS_RAMP_X_STEP_SIZE)
	{
		V_x_Now-=CHASSIS_RAMP_X_STEP_SIZE;
	}
	else if(V_x_Now<V_x_Set&&fabsf(V_x_Now-V_x_Set)>CHASSIS_RAMP_X_STEP_SIZE)
	{
		V_x_Now+=CHASSIS_RAMP_X_STEP_SIZE;
	}
	else
	{
		V_x_Now=0;
	}
	
	_Chassis->ChassisSpeed_Ref.forward_back_ref=V_x_Now;
	
	if(V_y_Now>V_y_Set&&fabsf(V_y_Now-V_y_Set)>CHASSIS_RAMP_Y_STEP_SIZE)
	{
		V_y_Now-=CHASSIS_RAMP_Y_STEP_SIZE;
	}
	else if(V_y_Now<V_y_Set&&fabsf(V_y_Now-V_y_Set)>CHASSIS_RAMP_Y_STEP_SIZE)
	{
		V_y_Now+=CHASSIS_RAMP_Y_STEP_SIZE;
	}
	else
	{
		V_y_Now=0;
	}
	
	_Chassis->ChassisSpeed_Ref.left_right_ref=V_y_Now;

}

void Chassis_ModeSelect(void)
{
//    if(chassis.ChassisSpeed_Ref.forward_back_ref==0&&chassis.ChassisSpeed_Ref.left_right_ref==0)
 //   Chassis_Ramp(&chassis);
    

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
		
        default:
			
        break;
    }
    Chassis_State_Detect(&chassis);
}








