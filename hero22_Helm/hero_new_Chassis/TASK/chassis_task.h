#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#define X_CHASSIS_AUTO_FOLLOW_GIMBAL_SENSITIVITY    0.002
#define Y_CHASSIS_AUTO_FOLLOW_GIMBAL_SENSITIVITY    0.002
#define W_CHASSIS_AUTO_FOLLOW_GIMBAL_SENSITIVITY    0.0025 //94度每秒,1.65rad/s

#define CHASSIS_RAMP_X_STEP_SIZE									0.07
#define CHASSIS_RAMP_Y_STEP_SIZE									0.07

#define CHASSIS_VX_LIMIT 3
#define CHASSIS_VY_LIMIT 3
#define CHASSIS_Vw_LIMIT 3.14
#define CHAS_RUN_THRESHOLD 800

typedef struct
{
  /* position loop */

	float angle_ref[4];
	float angle_fdb[4];
  /* speed loop */

	int16_t speed_ref[4];
	int16_t speed_fdb[4];
	
} cha_pid_t;

typedef enum
{
  CHASSIS_RELAX          = 0,	//关控
  CHASSIS_STOP           = 1,	//锁轮
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,//底盘云台反转
  CHASSIS_CHANGE_REVERSE = 9,
  CHASSIS_SEPARATE 		 = 10,
  CHASSIS_AUTO_SUP       = 11,
  CHASSIS_REVERSE_ROTATE = 12,	
}chassis_mode_e;


typedef enum
{
	CHAS_STATE_STOP			,
	CHAS_STATE_RUN_READY	,
	CHAS_STATE_RUN			,
	CHAS_UP_SLOPE			,
	CHAS_DOWN_SLOPE			,
	CHAS_FLY				,
}chassis_move_state_e;

typedef enum
{
  NORMAL_SPEED_MODE          = 2,
	HIGH_SPEED_MODE            = 3,
	LOW_SPEED_MODE             = 1,
	FLY_SLPOE				= 4,
} chassis_speed_mode_e;

typedef __packed struct
{
    float forward_back_ref;
    float left_right_ref;
    float rotate_ref;
}ChassisSpeed_Ref_t;

typedef struct
{
		float start_angle[4];
		float include_angle[4];
		float Remote_angle;
		float Remote_speed;
		float deviation_angle[4];
		int16_t handle_speed[4];
		int16_t handle_speed_lim[4];
		float get_speedw;
		float yaw_angle_0_2pi;
		float yaw_angle__pi_pi;
		double yaw_encoder_ecd_angle;
}Chassis_angle_t;

typedef struct
{
		float           Vx; // forward/back
		float           Vy; // left/right
		float           Vw; // 控直接发的速度
	
		float 			Vx_real;
		float 			Vy_real;
		float 			Vw_real;
		
		float 			Vcx;
		float 			Vcy;
		float 			Vcw;	//底盘跟随模式下，由云台坐标系变换到底盘坐标系的速度
		chassis_mode_e  			ctrl_mode;
		chassis_mode_e  			last_ctrl_mode;
        chassis_mode_e              last_last_ctrl_mode;
		chassis_speed_mode_e  		chassis_speed_mode;
		chassis_move_state_e		Chassis_Move_State;

		ChassisSpeed_Ref_t  ChassisSpeed_Ref;
	
		float          gyro_angle;
		float          gyro_palstance;

		int16_t        Helm_3508_Out[4];
		int16_t		   Helm_6020_Out[4];
		
		
		pid_t		   pid_6020_motor_speed[4];
		pid_t		   pid_6020_motor_angle[4];
		
		pid_t 		   pid_3508_motor_speed[4];
		
		float			_3508_motor_ref[4];
		float 			_3508_motor_fdb[4];
		float			_3508_motor_speed_rpm[4];	//filter_rate的反馈有点问题
		float 			Max_Power;
		float			Inclination_Angle;
		float 			Power_Limit[4];
        float           Power_Limit_Total;
		float			Power_Limit_By_Current_k[4];
        
        float           After_Mec_Cal_W_Ref[4];
        
        int16_t Speed_Ref[4];//速度参考值
	    int16_t Speed_Fdb[4];//速度反馈值
		
		float Helm_angle_ref[4];
		float Helm_angle_fdb[4];
		int16_t Helm_speed_ref[4];
		int16_t Helm_speed_fdb[4];
		
		
//        float Power_Limit[4];
//	    float Power_Limit_By_Current_k[4];
        pid_t chassis_new_speed[4];
		
} chassis_t;



//#define k0	2.650886f
//#define k1	0.00020922f
//#define k2	0.003969621f
//#define k3	-5.604004e-08f
//#define k4	1.432481e-07f
//#define k5	1.927039e-06f



typedef struct
{
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    
    float power_limit_single[4];
    float end_i_to_motor[4];
    
    
    
}power_t;


extern chassis_t 	chassis;
extern float Chassis_Dir;
extern float Helm_Chassis_bias[4];


void Chassis_Encoder_Get(void);
void Mec_steel_target_rpm_cal(float V__x,float V__y,float w,chassis_t* chassis);
void Chassis_ModeSelect(void);
void Chassis_Follow_Gimbal_Handle(void);
void Chassis_Separate_Gimbal_Handle(void);
void Chassis_Rotate_Handle(void);
void Chassis_Stop_Handle(void);
//void Power_Num_Init(void);
//float Power_foumule(power_t P,int32_t i,int32_t w);
void Power_Limit_Control_Handle(void);
//void Power_Get_k_in_and_Reset_PID_set(power_t* P,float power_target,chassis_t* T);
//void Error_Distribution_Cal(chassis_t* T,float power_target,power_t* P);
//void Power_End_Cal(power_t* P,chassis_t* T);
void Rpm_to_Rad_powercontrol(chassis_t* T);
void Chassis_Ramp(chassis_t *_Chassis);

#endif
