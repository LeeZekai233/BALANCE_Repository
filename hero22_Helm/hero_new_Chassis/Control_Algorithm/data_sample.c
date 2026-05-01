#include "public.h"

extern sin_signal_t sin_signal;

pid_t yaw_gimbal_speed_pid;
pid_t yaw_gimbal_angle_pid;
float ref_signal;
float sys_input;
float angle_output;
float signal_frequency = 0.5;
int64_t exertion_time_tick;
u8 start_flag;
float Am=300;
float set;
int16_t disterb;
int dis_tick;

//gimbal_ESO_t yaw_gimbal_ESO;

//0.5 1 5 10 20 30 40 50 60 70 80 90 100 120 150 200

void sample_task(int frequency)
{
	
    sin_signal.start_flag = start_flag;
    start_sin_cal(frequency);
    if(start_flag==1)
    {
        exertion_time_tick++;
        if(exertion_time_tick<=40000)
        {
            signal_frequency = 0.5;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
            
        }else if(exertion_time_tick<=60000)
        {
            signal_frequency = 1;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=64000)
        {
            signal_frequency = 5;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=66000)
        {
            signal_frequency = 10;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=67000)
        {
            signal_frequency = 20;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=67666)
        {
            signal_frequency = 30;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=68166)
        {
            signal_frequency = 40;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }else if(exertion_time_tick<=68566)
        {
            signal_frequency = 50;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }else if(exertion_time_tick<=68899)
        {
            signal_frequency = 60;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=69185)
        {
            signal_frequency = 70;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=69435)
        {
            signal_frequency = 80;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=69657)
        {
            signal_frequency = 90;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=69857)
        {
            signal_frequency = 100;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=70024)
        {
            signal_frequency = 120;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=70157)
        {
            signal_frequency = 150;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else if(exertion_time_tick<=70257)
        {
            signal_frequency = 200;
            sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
        }
        else
        {
            start_flag = 0;
        }

        if(exertion_time_tick%2000==0)
        {
            disterb = 5000;
        }
        if(disterb!=0)
        {
            dis_tick++;
            if(dis_tick>100)
            {
                dis_tick=0;
                disterb=0;
            }
        }
        
        ref_signal = set;
        sample_control_task();
    }else
    {
        exertion_time_tick = 0;
        sys_input = 0;
        ref_signal = 0;
//        sample_control_task();
    }
		
}




void sample_control_task(void)
{
    ref_signal = sin_signal.output;
//    angle_output = pid_calc(&yaw_gimbal_angle_pid,Gimbal.Yaw_Angle_Fdb,ref_signal); 
//    sys_input = pid_calc(&yaw_gimbal_speed_pid,Gimbal.Yaw_Spped_Fdb,yaw_gimbal_angle_pid.out) - yaw_gimbal_ESO.output;//pid_calc(&yaw_gimbal_pid,gimbal_gyro.yaw_Angle,ref_signal);
//    VAL_LIMIT(sys_input,-16380,+16380);
//    gimbal_ESO_cal(&yaw_gimbal_ESO,sys_input,Gimbal.Yaw_Spped_Fdb);  
    sys_input = ref_signal;
     //sin_signal.seconds_point
    //yaw_Encoder.Torque
    //gimbal_gyro.yaw_Gyro

}


void sample_task_Init(void)
{
    Am = 3000;
    PID_struct_init(&yaw_gimbal_speed_pid,POSITION_PID,27000,20000,35,0,0);
    PID_struct_init(&yaw_gimbal_angle_pid,POSITION_PID,27000,200, 20,0.001,80); //38 0.005 0
    sin_signal_Init(Am,2*PI/(1/signal_frequency),0);
    
//	   gimbal_ESO_param_Init(&yaw_gimbal_ESO,0.01611328125,180.0/PI,0.002,0.102,0.3829,180,400,0,0,-200000);
}
