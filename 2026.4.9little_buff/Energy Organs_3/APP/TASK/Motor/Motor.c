#include <Motor.h>

pid_t pid_620_angle           = {0};
pid_t pid_big           = {0};
pid_t pid_small           = {0};
pid_t pid_6020_angle           = {0};
pid_t pid_6020_speed           = {0};
pid_t pid_c610andc620_angle    = {0};
pid_t pid_c610andc620_speed    = {0};

float motor620_set;
float aaaa;
float motor_c610andc620_set;

int aaaaaaa=122;
void Motor_Run(void)
{
//	CAN1_Send();
	
	//Motor620
	//motor620_set=50;
	
	//pid_calc(&pid_big,Motor620_Encoder.filter_rate,motor620_set);
	
	Motor_620_out1(CAN1,0,0,0,0);
	
	
	
//	//Motor_c610andc620
//	motor_c610andc620_set=100;
//	
//	pid_calc(&pid_c610andc620_speed,Motor_c610and620_Encoder1.filter_rate,motor_c610andc620_set);
//	
////	Motor_c610andc620_out1(CAN1, pid_c610andc620_speed.out , pid_c610andc620_speed.out , pid_c610andc620_speed.out , pid_c610andc620_speed.out );
	
    //Motor_c610andc620_out1(CAN1, 5111, 1111 , 1111, 5111 );
}


void motor_run_init(void)
{
	
//	PID_struct_init(&pid_big,        POSITION_PID, 1000, 2000, 50 , 0.1 , 10);

//PID_struct_init(&pid_big,        POSITION_PID, 25000,8000,12,2, 20);	//双环值
//PID_struct_init(&pid_620_angle,        POSITION_PID, 2000  ,1000  ,0.9, 0.0009 , 20);	
	PID_struct_init(&pid_big,         POSITION_PID, 25000,8000,15,0.5, 0);	//双环值
		PID_struct_init(&pid_small,       DELTA_PID, 25000,8000,15,0.5, 0);	//双环值
//	PID_struct_init(&pid_6020_angle,        POSITION_PID, 320,200,13, 8 ,15);	
//	PID_struct_init(&pid_6020_speed,        POSITION_PID, 30000,25000,50,1 ,50);	

//	PID_struct_init(&pid_6020_angle,        POSITION_PID, 100,80,7, 4 ,7);	
//	PID_struct_init(&pid_6020_speed,        POSITION_PID, 30000,25000,50,1 ,50);	

	
	
//	motor6020_set=100;
//	motor_c610andc620_set=100;

}




void Motor_620_Speed(int32_t motor620_setbig,int32_t motor620_setsmall)
{
	
	
	pid_calc(&pid_big,Motor620_Encoder.rotate_rate,motor620_setbig);
	pid_calc(&pid_small,Motor620_Encoder.rotate_rate,motor620_setsmall);
	
	Motor_620_out1(CAN2,pid_big.out,pid_big.out,0 ,0 ); 	
    //测试
		Motor_620_out1(CAN1,pid_small.out,pid_small.out,0 ,0 );
}


void Motor_620_Angle(int32_t angle)
{
	pid_620_angle.set=angle;
	pid_calc(&pid_620_angle,Motor620_Encoder.ecd_angle,pid_620_angle.set);//get set
	pid_calc(&pid_big,Motor620_Encoder.rotate_rate,pid_620_angle.out);//get set
	Motor_620_out1(CAN1,pid_big.out, 0  ,0 ,0 );	
}

void Motor_6020_Angle(int32_t angle)
{
	pid_6020_angle.set=angle;
	pid_calc(&pid_6020_angle,Motor6020_Encoder.ecd_angle,pid_6020_angle.set);//get set
	pid_calc(&pid_6020_speed,Motor6020_Encoder.rotate_rate,pid_6020_angle.out);//get set
	Motor_6020_out(CAN1,pid_6020_speed.out, 0  ,0 ,0 );	
}


void pid_clear(void)
{	
	pid_clr(&pid_big);
	pid_clr(&pid_small);
	pid_clr(&pid_620_angle);
}


