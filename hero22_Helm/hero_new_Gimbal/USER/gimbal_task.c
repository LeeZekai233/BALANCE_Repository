#include "public.h"
float yaw_sys_input;
gimbal_t gimbal_data;
pid_t pid_follow_angle;
pid_t pid_follow_speed;
extern general_gyro_t gimbal_gyro;
extern gimbal_ESO_t yaw_gimbal_ESO;
Encoder Pitch_Encoder={0};
Encoder Scope_Encoder={0};
Encoder_plus yaw_Encoder = {0};
float pitch_min = -10;
float pitch_max = 45;		
int Scope_Init_Cnt=0,Scope_Init_Finished_Cnt=0;
uint8_t reversal_flag,reversing_flag;
float Follow_Angle_Medium;
void Gimbal_parameter_Init(void)
{
    
    //对跟随云台模式进行pid初始化
    PID_struct_init(&gimbal_data.pid_pit_Angle,POSITION_PID,1000,13,23,0,0);
    PID_struct_init(&gimbal_data.pid_pit_speed,POSITION_PID,15000,500,100,0.2,50);
    
    PID_struct_init(&gimbal_data.pid_yaw_Angle,POSITION_PID,800,30,20,0,250);
    PID_struct_init(&gimbal_data.pid_yaw_speed,POSITION_PID,2048,1000,20,0,0);
    
    //对底盘跟随云台消除跟随角的pid初始化
     PID_struct_init(&pid_follow_angle,POSITION_PID,800,30,0.12,0,0);
     PID_struct_init(&pid_follow_speed,POSITION_PID,1,1,12,0,0);
	
	   gimbal_ESO_param_Init(&yaw_gimbal_ESO,0.01611328125,180.0/PI,0.002,0.102,0.3829,50000,1200,0,0,0);

    //对吊射模式pid初始化
    PID_struct_init(&gimbal_data.pid_auto_pit_Angle,POSITION_PID,120,13,20,0,300);
    PID_struct_init(&gimbal_data.pid_auto_pit_speed,POSITION_PID,10000,1000,90,1,0);
    
    PID_struct_init(&gimbal_data.pid_auto_yaw_Angle,POSITION_PID,120,30,20,0,350);
    PID_struct_init(&gimbal_data.pid_auto_yaw_speed,POSITION_PID,2048,200,19,0.3,0);
    
    PID_struct_init(&gimbal_data.pid_scope_angle,POSITION_PID,720,0,100,0,15);
	PID_struct_init(&gimbal_data.pid_scope_speed,POSITION_PID,5000,500,23,0.5,15);
    //对自瞄模式pid初始化
    PID_struct_init(&gimbal_data.pid_pit_follow_1,POSITION_PID,1000,13,23,0,80);
    PID_struct_init(&gimbal_data.pid_pit_speed_follow_1,POSITION_PID,15000,300,132,0.05,78);
	
    PID_struct_init(&gimbal_data.pid_pit_follow_2,POSITION_PID,1000,13,26,0,50);
    PID_struct_init(&gimbal_data.pid_pit_speed_follow_2,POSITION_PID,15000,300,152,0.05,78);
	
    PID_struct_init(&gimbal_data.pid_pit_follow_3,POSITION_PID,1000,13,32,0,25);
    PID_struct_init(&gimbal_data.pid_pit_speed_follow_3,POSITION_PID,15000,300,165,0.05,78);
	
	
    PID_struct_init(&gimbal_data.pid_yaw_follow_1,POSITION_PID,800,30,18,0,13);
    PID_struct_init(&gimbal_data.pid_yaw_speed_follow_1,POSITION_PID,2048,1000,13,0,4);
	
    PID_struct_init(&gimbal_data.pid_yaw_follow_2,POSITION_PID,800,30,22,0,25);
    PID_struct_init(&gimbal_data.pid_yaw_speed_follow_2,POSITION_PID,2048,1000,17,0,3);
	
    PID_struct_init(&gimbal_data.pid_yaw_follow_3,POSITION_PID,800,30,28,0,20);
    PID_struct_init(&gimbal_data.pid_yaw_speed_follow_3,POSITION_PID,2048,1000,25,0,3);
	
    PID_struct_init(&gimbal_data.pid_yaw_follow_4,POSITION_PID,800,30,30,0,0);
    PID_struct_init(&gimbal_data.pid_yaw_speed_follow_4,POSITION_PID,2048,1000,26,0,3);
    
    
   
}

void Gimbal_Init_Handle(void)
{
    int init_rotate_num=0;
    
    gimbal_data.gim_ref_and_fdb.pit_angle_ref=0;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb=gimbal_gyro.pitch_angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref=180;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb=gimbal_gyro.yaw_angle;
    
    
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb=gimbal_gyro.pitch_palstance;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb=gimbal_gyro.yaw_palstance;
    
    
    while(gimbal_data.gim_ref_and_fdb.yaw_angle_fdb>180)gimbal_data.gim_ref_and_fdb.yaw_angle_fdb-=360;
	while(gimbal_data.gim_ref_and_fdb.yaw_angle_fdb<-180)gimbal_data.gim_ref_and_fdb.yaw_angle_fdb+=360;
    
    
    init_rotate_num=(gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)/360;//求出陀螺仪一开始可能累积的多圈角度值
    gimbal_data.gim_ref_and_fdb.yaw_angle_ref=init_rotate_num*360+gimbal_data.gim_ref_and_fdb.yaw_angle_ref;//将目标值归化到陀螺仪值同一圈内进行后续处理
    if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<-180)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref+=360;
    }
    else if((gimbal_data.gim_ref_and_fdb.yaw_angle_ref-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>180)
    {
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref-=360;
    }
    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,&gimbal_data.pid_yaw_speed,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_yaw_Angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
    
	//小云台初始化
	if(Scope_Encoder.cal_data.can_cnt>10)
	{
		gimbal_data.gim_ref_and_fdb.scope_motor_input=pid_calc(&gimbal_data.pid_scope_speed,Scope_Encoder.rate_rpm,500);
		Scope_Init_Cnt++;
	}
	if(Scope_Init_Cnt>10&&Scope_Encoder.rate_rpm<5&&Scope_Encoder.rate_rpm>-5)
	{
		Scope_Init_Finished_Cnt++;
	}
	if(Scope_Init_Finished_Cnt>20)
	{
		gimbal_data.gim_ref_and_fdb.scope_angle_Init=Scope_Encoder.cal_data.ecd_value*0.001220703125f;
		gimbal_data.if_finish_Init=1;
		gimbal_data.gim_ref_and_fdb.scope_motor_input=0;
	}
}
void Gimbal_Follow_Gyro_Handle(void)
{
	
    if(gimbal_data.last_ctrl_mode!=GIMBAL_FOLLOW_ZGYRO&&gimbal_data.last_ctrl_mode!=GIMBAL_SNIPE&&gimbal_data.last_ctrl_mode!=GIMBAL_RADAR_ASSISTANT_SNIPE)
    {
        gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_gyro.pitch_angle;
        gimbal_data.gim_ref_and_fdb.yaw_angle_ref=gimbal_gyro.yaw_angle;
    }
    gimbal_data.gim_ref_and_fdb.scope_angle_fdb=Scope_Encoder.cal_data.ecd_value*0.001220703125f;
    gimbal_data.gim_ref_and_fdb.scope_speed_fdb=Scope_Encoder.rate_rpm;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb=gimbal_gyro.pitch_angle;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb=gimbal_gyro.yaw_angle;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb=gimbal_gyro.pitch_palstance;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb=gimbal_gyro.yaw_palstance;
    
    
	if(1)
	{
		VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref,pitch_min,pitch_max);
	}
	else if(Pitch_Encoder.ecd_angle>Pitch_Encoder.Init_Angle+2500&&gimbal_data.gim_ref_and_fdb.pit_angle_ref<=gimbal_data.gim_ref_and_fdb_last.pit_angle_ref)
	{
		gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
	}
	else if(Pitch_Encoder.ecd_angle<Pitch_Encoder.Init_Angle-9000&&gimbal_data.gim_ref_and_fdb.pit_angle_ref>=gimbal_data.gim_ref_and_fdb_last.pit_angle_ref)
    {
		gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
	}
    gimbal_data.gim_ref_and_fdb.pitch_motor_input=pid_double_loop_cal(&gimbal_data.pid_pit_Angle,
                                                                      &gimbal_data.pid_pit_speed,
                                                                      gimbal_data.gim_ref_and_fdb.pit_angle_ref,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,&gimbal_data.pid_pit_Angle.out,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,0);
    

	if(reversing_flag==1)
	{
		PID_struct_init(&gimbal_data.pid_yaw_Angle,POSITION_PID,540,30,13,0,30);
		if(fabsf(gimbal_data.gim_ref_and_fdb.yaw_angle_fdb-gimbal_data.gim_ref_and_fdb.yaw_angle_ref)<10)
		{
			reversing_flag=0;
			PID_struct_init(&gimbal_data.pid_yaw_Angle,POSITION_PID,800,30,20,0,30);
		}
	}
    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_yaw_Angle,&gimbal_data.pid_yaw_speed,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_angle_ref,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_yaw_Angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
	gimbal_data.gim_ref_and_fdb.scope_angle_ref=-4;
    gimbal_data.gim_ref_and_fdb.scope_motor_input=pid_double_loop_cal(&gimbal_data.pid_scope_angle,&gimbal_data.pid_scope_speed,
                                                                    gimbal_data.gim_ref_and_fdb.scope_angle_ref+gimbal_data.gim_ref_and_fdb.scope_angle_Init,gimbal_data.gim_ref_and_fdb.scope_angle_fdb,&gimbal_data.pid_scope_angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.scope_speed_fdb,0);
    
    
    yaw_sys_input = gimbal_data.gim_ref_and_fdb.yaw_motor_input- yaw_gimbal_ESO.output;
    VAL_LIMIT(yaw_sys_input,-2048,2048);
    gimbal_ESO_cal(&yaw_gimbal_ESO,yaw_sys_input,gimbal_data.gim_ref_and_fdb.yaw_speed_fdb);
  
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pitch_motor_input,-5040,5040);
    
}

float snipe_yaw_benchmark,snipe_pitch_bench_mark;

void Gimbal_Snipe_Handle(void)
{
    gimbal_data.gim_ref_and_fdb.scope_angle_fdb=Scope_Encoder.cal_data.ecd_value*0.001220703125f;
    gimbal_data.gim_ref_and_fdb.scope_speed_fdb=Scope_Encoder.rate_rpm;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb=gimbal_gyro.pitch_angle;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb=gimbal_gyro.pitch_palstance;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb=gimbal_gyro.yaw_angle;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb=gimbal_gyro.yaw_palstance;
    
    snipe_pitch_bench_mark=20;
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pit_angle_ref,pitch_min-20,pitch_max-20);
    gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref=snipe_pitch_bench_mark+gimbal_data.gim_ref_and_fdb.pit_angle_ref;
    gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref=snipe_yaw_benchmark+gimbal_data.gim_ref_and_fdb.yaw_angle_ref;
    
	if(1)
	{
		VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref,pitch_min,pitch_max);
	}
	else if(Pitch_Encoder.ecd_angle>Pitch_Encoder.Init_Angle+2500&&gimbal_data.gim_ref_and_fdb.pit_angle_ref<=gimbal_data.gim_ref_and_fdb_last.pit_angle_ref)
	{
		gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
	}
	else if(Pitch_Encoder.ecd_angle<Pitch_Encoder.Init_Angle-9000&&gimbal_data.gim_ref_and_fdb.pit_angle_ref>=gimbal_data.gim_ref_and_fdb_last.pit_angle_ref)
    {
		gimbal_data.gim_ref_and_fdb.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
	}
    gimbal_data.gim_ref_and_fdb.pitch_motor_input=pid_double_loop_cal(&gimbal_data.pid_auto_pit_Angle,
                                                                      &gimbal_data.pid_auto_pit_speed,
                                                                      gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,&gimbal_data.pid_auto_pit_Angle.out,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,0);
    
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_auto_yaw_Angle,&gimbal_data.pid_auto_yaw_speed,
                                                                    gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_auto_yaw_Angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
   	gimbal_data.gim_ref_and_fdb.scope_angle_ref=-40;
    gimbal_data.gim_ref_and_fdb.scope_motor_input=pid_double_loop_cal(&gimbal_data.pid_scope_angle,&gimbal_data.pid_scope_speed,
                                                                    gimbal_data.gim_ref_and_fdb.scope_angle_ref+gimbal_data.gim_ref_and_fdb.scope_angle_Init,gimbal_data.gim_ref_and_fdb.scope_angle_fdb,&gimbal_data.pid_scope_angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.scope_speed_fdb,0);
    
                             
    yaw_sys_input = gimbal_data.gim_ref_and_fdb.yaw_motor_input- yaw_gimbal_ESO.output; 
    VAL_LIMIT(yaw_sys_input,-2048,2048);
    gimbal_ESO_cal(&yaw_gimbal_ESO,yaw_sys_input,gimbal_data.gim_ref_and_fdb.yaw_speed_fdb);
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pitch_motor_input,-5040,5040);
}

void Gimbal_Radar_Assistant_Snipe_Handle(void)
{
    gimbal_data.gim_ref_and_fdb.scope_angle_fdb=Scope_Encoder.cal_data.ecd_value*0.001220703125f;
    gimbal_data.gim_ref_and_fdb.scope_speed_fdb=Scope_Encoder.rate_rpm;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb=gimbal_gyro.pitch_angle;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb=gimbal_gyro.pitch_palstance;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb=gimbal_gyro.yaw_angle;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb=gimbal_gyro.yaw_palstance;
    
    
    gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref=My_Auto_Snipe.Auto_Aim.Pitch_Angle;
    gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref=/*gimbal_data.gim_ref_and_fdb.yaw_angle_ref */My_Auto_Snipe.Auto_Aim.Yaw_Angle+gimbal_gyro.Yaw_count*360;
    
	VAL_LIMIT(gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref,pitch_min,pitch_max);
	
    gimbal_data.gim_ref_and_fdb.pitch_motor_input=pid_double_loop_cal(&gimbal_data.pid_auto_pit_Angle,
                                                                      &gimbal_data.pid_auto_pit_speed,
                                                                      gimbal_data.gim_dynamic_ref.pitch_angle_dynamic_ref,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,&gimbal_data.pid_auto_pit_Angle.out,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,0);
    
    
    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_auto_yaw_Angle,&gimbal_data.pid_auto_yaw_speed,
                                                                    gimbal_data.gim_dynamic_ref.yaw_angle_dynamic_ref,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_auto_yaw_Angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
	gimbal_data.gim_ref_and_fdb.scope_angle_ref=-40;
    gimbal_data.gim_ref_and_fdb.scope_motor_input=pid_double_loop_cal(&gimbal_data.pid_scope_angle,&gimbal_data.pid_scope_speed,
                                                                    gimbal_data.gim_ref_and_fdb.scope_angle_ref+gimbal_data.gim_ref_and_fdb.scope_angle_Init,gimbal_data.gim_ref_and_fdb.scope_angle_fdb,&gimbal_data.pid_scope_angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.scope_speed_fdb,0);
    
    
    yaw_sys_input = gimbal_data.gim_ref_and_fdb.yaw_motor_input- yaw_gimbal_ESO.output; 
    VAL_LIMIT(yaw_sys_input,-2048,2048);
    gimbal_ESO_cal(&yaw_gimbal_ESO,yaw_sys_input,gimbal_data.gim_ref_and_fdb.yaw_speed_fdb);
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pitch_motor_input,-5040,5040);
}


void Gimbal_AUTO_AIM_Handle(void)
{
    gimbal_data.gim_ref_and_fdb.scope_angle_fdb=Scope_Encoder.cal_data.ecd_value*0.001220703125f;
    gimbal_data.gim_ref_and_fdb.scope_speed_fdb=Scope_Encoder.rate_rpm;
    gimbal_data.gim_ref_and_fdb.pit_angle_fdb=gimbal_gyro.pitch_angle;
    gimbal_data.gim_ref_and_fdb.pit_speed_fdb=gimbal_gyro.pitch_palstance;
    gimbal_data.gim_ref_and_fdb.yaw_angle_fdb=gimbal_gyro.yaw_angle;
    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb=gimbal_gyro.yaw_palstance;
    
	if(Pitch_Init_Flag==0)
	{
		VAL_LIMIT(My_Auto_Shoot.Auto_Aim.Pitch_Angle,pitch_min,pitch_max);
	}
	else if(Pitch_Encoder.ecd_angle>Pitch_Encoder.Init_Angle+2500)
	{
		My_Auto_Shoot.Auto_Aim.Pitch_Angle=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
	}
	else if(Pitch_Encoder.ecd_angle<Pitch_Encoder.Init_Angle-9000)
    {
		My_Auto_Shoot.Auto_Aim.Pitch_Angle=gimbal_data.gim_ref_and_fdb.pit_angle_fdb;
	}
	
	if(fabsf(My_Auto_Shoot.Auto_Aim.Yaw_Angle-gimbal_gyro.Yaw_count*360-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<1)
	{
		    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_yaw_follow_1,&gimbal_data.pid_yaw_speed_follow_1,
                                                                    My_Auto_Shoot.Auto_Aim.Yaw_Angle+gimbal_gyro.Yaw_count*360,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_yaw_follow_1.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
	}
	else if(fabsf(My_Auto_Shoot.Auto_Aim.Yaw_Angle-gimbal_gyro.Yaw_count*360-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<3)
	{
		    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_yaw_follow_2,&gimbal_data.pid_yaw_speed_follow_2,
                                                                    My_Auto_Shoot.Auto_Aim.Yaw_Angle+gimbal_gyro.Yaw_count*360,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_yaw_follow_2.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
	}
	else if(fabsf(My_Auto_Shoot.Auto_Aim.Yaw_Angle-gimbal_gyro.Yaw_count*360-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)<7)
	{
		    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_yaw_follow_3,&gimbal_data.pid_yaw_speed_follow_3,
                                                                    My_Auto_Shoot.Auto_Aim.Yaw_Angle+gimbal_gyro.Yaw_count*360,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_yaw_follow_3.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
	}
	else// if(fabsf(My_Auto_Shoot.Auto_Aim.Yaw_Angle-gimbal_gyro.Yaw_count*360-gimbal_data.gim_ref_and_fdb.yaw_angle_fdb)>8)
	{
		    gimbal_data.gim_ref_and_fdb.yaw_motor_input=pid_double_loop_cal(&gimbal_data.pid_yaw_follow_4,&gimbal_data.pid_yaw_speed_follow_4,
                                                                    My_Auto_Shoot.Auto_Aim.Yaw_Angle+gimbal_gyro.Yaw_count*360,gimbal_data.gim_ref_and_fdb.yaw_angle_fdb,&gimbal_data.pid_yaw_follow_4.out,
                                                                    gimbal_data.gim_ref_and_fdb.yaw_speed_fdb,0);
	}
	if(fabsf(My_Auto_Shoot.Auto_Aim.Pitch_Angle-gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<0.5)
    {
    gimbal_data.gim_ref_and_fdb.pitch_motor_input=pid_double_loop_cal(&gimbal_data.pid_pit_follow_1,
                                                                      &gimbal_data.pid_pit_speed_follow_1,
                                                                      My_Auto_Shoot.Auto_Aim.Pitch_Angle,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,&gimbal_data.pid_pit_follow_1.out,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,0);
	}
	else if(fabsf(My_Auto_Shoot.Auto_Aim.Pitch_Angle-gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<2)
    {
    gimbal_data.gim_ref_and_fdb.pitch_motor_input=pid_double_loop_cal(&gimbal_data.pid_pit_follow_2,
                                                                      &gimbal_data.pid_pit_speed_follow_2,
                                                                      My_Auto_Shoot.Auto_Aim.Pitch_Angle,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,&gimbal_data.pid_pit_follow_2.out,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,0);
	}
	else// if(fabsf(My_Auto_Shoot.Auto_Aim.Pitch_Angle-gimbal_data.gim_ref_and_fdb.pit_angle_fdb)<3)
    {
    gimbal_data.gim_ref_and_fdb.pitch_motor_input=pid_double_loop_cal(&gimbal_data.pid_pit_follow_3,
                                                                      &gimbal_data.pid_pit_speed_follow_3,
                                                                      My_Auto_Shoot.Auto_Aim.Pitch_Angle,gimbal_data.gim_ref_and_fdb.pit_angle_fdb,&gimbal_data.pid_pit_follow_3.out,
                                                                      gimbal_data.gim_ref_and_fdb.pit_speed_fdb,0);
	gimbal_data.gim_ref_and_fdb.scope_angle_ref=-4;
    gimbal_data.gim_ref_and_fdb.scope_motor_input=pid_double_loop_cal(&gimbal_data.pid_scope_angle,&gimbal_data.pid_scope_speed,
                                                                    gimbal_data.gim_ref_and_fdb.scope_angle_ref+gimbal_data.gim_ref_and_fdb.scope_angle_Init,gimbal_data.gim_ref_and_fdb.scope_angle_fdb,&gimbal_data.pid_scope_angle.out,
                                                                    gimbal_data.gim_ref_and_fdb.scope_speed_fdb,0);
    
    
	}
    

    
    yaw_sys_input = gimbal_data.gim_ref_and_fdb.yaw_motor_input- yaw_gimbal_ESO.output;
    VAL_LIMIT(yaw_sys_input,-1848,1848);
    gimbal_ESO_cal(&yaw_gimbal_ESO,yaw_sys_input,gimbal_data.gim_ref_and_fdb.yaw_speed_fdb);
    VAL_LIMIT(gimbal_data.gim_ref_and_fdb.pitch_motor_input,-5040,5040);
}







void Gimbal_task(void)
{
	
    switch(gimbal_data.ctrl_mode)
    {
        case GIMBAL_INIT:
            Gimbal_Init_Handle();
            break;
        
        case GIMBAL_FOLLOW_ZGYRO:
            Gimbal_Follow_Gyro_Handle();
            break;
        
        case GIMBAL_SNIPE:
            Gimbal_Snipe_Handle();
            break;
        
        case GIMBAL_AUTO_AIM:
            Gimbal_AUTO_AIM_Handle();
            break;
		case GIMBAL_RADAR_ASSISTANT_SNIPE:
			Gimbal_Radar_Assistant_Snipe_Handle();
		break;
        
        default:
					sys_input=0;
					yaw_gimbal_ESO.z1=0;
					yaw_gimbal_ESO.z2=0;
					yaw_gimbal_ESO.z3=0;
		gimbal_data.gim_ref_and_fdb.scope_motor_input=0;
            break;
    }
    gimbal_data.last_ctrl_mode=gimbal_data.ctrl_mode;
	gimbal_data.gim_ref_and_fdb_last.pit_angle_ref=gimbal_data.gim_ref_and_fdb.pit_angle_ref;
}






