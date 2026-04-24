#include <Motor.h>

pid_t pid_620_angle           = {0};
pid_t pid_620_speed           = {0};
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
	motor620_set=1000;
	
	pid_calc(&pid_620_speed,Motor620_Encoder.filter_rate,motor620_set);
	
	Motor_620_out1(CAN1,2222, 8500 ,  2000 , 2000 );
	
	
	
//	//Motor_c610andc620
//	motor_c610andc620_set=100;
//	
//	pid_calc(&pid_c610andc620_speed,Motor_c610and620_Encoder1.filter_rate,motor_c610andc620_set);
//	
////	Motor_c610andc620_out1(CAN1, pid_c610andc620_speed.out , pid_c610andc620_speed.out , pid_c610andc620_speed.out , pid_c610andc620_speed.out );
	Motor_c610andc620_out1(CAN1, 5111, 1111 , 1111, 5111 );
}


void motor_run_init(void)
{
	
//	PID_struct_init(&pid_620_speed,        POSITION_PID, 1000, 2000, 50 , 0.1 , 10);
//	PID_struct_init(&pid_c610andc620_speed, POSITION_PID, 5000, 2000, 15 , 1 , 5);
//	PID_struct_init(&pid_620_angle,        POSITION_PID, 3000,2000,15, 1 , 5);	
//	PID_struct_init(&pid_620_speed,        POSITION_PID, 25000,2000,50, 0.1 , 10);	
	
	PID_struct_init(&pid_620_angle,        POSITION_PID, 500  ,300  ,10, 0.5 , 5);	
	PID_struct_init(&pid_620_speed,        POSITION_PID, 16384,13000,25, 0.5 , 15);	

//	PID_struct_init(&pid_6020_angle,        POSITION_PID, 320,200,13, 8 ,15);	
//	PID_struct_init(&pid_6020_speed,        POSITION_PID, 30000,25000,50,1 ,50);	

//	PID_struct_init(&pid_6020_angle,        POSITION_PID, 100,80,7, 4 ,7);	
//	PID_struct_init(&pid_6020_speed,        POSITION_PID, 30000,25000,50,1 ,50);	

	
	
//	motor6020_set=100;
//	motor_c610andc620_set=100;

}




void Motor_620_Speed(void)
{
	motor620_set=1000;
	
	pid_calc(&pid_620_speed,Motor620_Encoder.filter_rate,motor620_set);
	Motor_620_out1(CAN1,0, pid_620_speed.out ,0 ,0 );	
}


void Motor_620_Angle(int32_t angle)
{
	pid_620_angle.set=angle;
	pid_calc(&pid_620_angle,Motor620_Encoder.ecd_angle,pid_620_angle.set);//get set
	pid_calc(&pid_620_speed,Motor620_Encoder.rotate_rate,pid_620_angle.out);//get set
	Motor_620_out1(CAN1,pid_620_speed.out, 0  ,0 ,0 );	
}

void Motor_6020_Angle(int32_t angle)
{
	pid_6020_angle.set=angle;
	pid_calc(&pid_6020_angle,Motor6020_Encoder.ecd_angle,pid_6020_angle.set);//get set
	pid_calc(&pid_6020_speed,Motor6020_Encoder.rotate_rate,pid_6020_angle.out);//get set
	Motor_6020_out(CAN1,pid_6020_speed.out, 0  ,0 ,0 );	
}






