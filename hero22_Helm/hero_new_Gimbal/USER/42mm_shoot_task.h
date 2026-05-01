#ifndef __42MM_SHOOT_TASK_H__
#define __42MM_SHOOT_TASK_H__

typedef enum
{
	_42MM_SHOOT_NORMAL = 2,		//普通模式
	_42MM_SHOOT_BURST = 1,		//爆发
	_42MM_SHOOT_RELAX = 0,		//关控或遥控信号丢失
}
shoot_mode_e;

typedef enum
{

	FRICTION_OFF = 0,	
	FRICTION_ON = 1,
	FRICTION_TRAP = 2,
	FRICTION_RELAX = 3,
}
friction_state_e;

typedef enum
{
	POKE_RELAX = 0,
	POKE_NORMAL = 1,
	POKE_TRAP = 2,
	POKE_INIT = 3,
}
poke_state_e;

typedef struct
{
   
//   float up_poke_seed_fdb;
//   float up_poke_speed_ref;
    
   float down_poke_speed_ref;
   float down_poke_speed_fdb;
   float down_poke_angle_ref;
   float down_poke_angle_fdb;
    
//   float down_poke_angle_dynamic_ref;
//   float down_poke_speed_dynamic_ref;

   float left_friction_speed_ref[2];
   float left_friction_speed_fdb[2];

   float right_friction_speed_ref[2];
   float right_friction_speed_fdb[2];

//   int16_t up_poke_motor_input;
   int16_t down_poke_motor_input;
    
   int16_t left_friction_motor_input[2];
   int16_t right_friction_motor_input[2];
   int16_t down_friction_motor_input[2];
	
}
shoot_ref_and_fdb_t;


typedef struct
{
   //----------------↓电机反馈结构体嵌套↓-------------//
	
	shoot_ref_and_fdb_t shoot_ref_and_fdb;
   //----------------↑电机反馈结构体嵌套↑-------------//
	
	
	//----------------↓发射机构模式枚举嵌套↓-------------//

    shoot_mode_e ctrl_mode;
	shoot_mode_e last_ctrl_mode;
	friction_state_e friction_state;
	poke_state_e poke_state;
	
	//----------------↑发射机构模式枚举嵌套↑-------------//
	
	
    u8 shoot_flag; //开火标志位
	u8 inverse_flag;//拨盘反转标志位
	
	
	//----------------↓pid结构体嵌套↓-------------//
	
//	pid_t pid_up_poke_speed;
	
    pid_t pid_downpoke_speed;
	pid_t pid_downpoke_angle;
    
    pid_t pid_left_friction_speed[2];
    pid_t pid_right_friction_speed[2];
    pid_t pid_down_friction_speed[2];
	
	//----------------↑pid结构体嵌套↑-------------//
	
	////////////////////摩擦轮电机输出///////////////
	float left_friction_current[2];
	float right_friction_current[2];
    float down_friction_current[2];
	
} 
_42mm_shoot_t;

typedef struct 
{
    volatile Encoder right_up_motor;
    volatile Encoder left_up_motor;
    volatile Encoder left_down_motor;
    volatile Encoder right_down_motor;
    volatile Encoder down_up_motor;
    volatile Encoder down_down_motor;
}friction_t;


static struct {
    uint8_t state;          // 0:空闲, 1:反转解卡, 2:正转尝试, 3:等待确认, 4:复位
    uint32_t timer;         // 计时器
    uint8_t try_count;      // 尝试次数
} trap_recovery = {0};

static struct {
    uint32_t rotate_start_time;      // 旋转开始时间
    uint32_t low_speed_timer;        // 低速计时器
    uint32_t no_move_timer;          // 无运动计时器
    float last_angle;                // 上次角度
    uint8_t is_rotating;             // 是否正在旋转
    uint8_t consecutive_jams;        // 连续卡弹次数
    uint32_t last_jam_time;          // 上次卡弹时间
} jam_detect = {0};



void _42mm_Shoot_PID_Init(void);
void _42mm_Shoot_Normal_Hdandle(void);
void _42mm_Shoot_Task(void);
void Poke_Handle(void);
void Poke_Init(void);
//static void Handle_Trap_Recovery(void);
//static uint8_t Update_Rotation_Detection(float current_angle, int16_t current_rpm, float target_angle);
//static void Start_Rotation_Detection(float start_angle);
void Heat_Control(void);
//void Time_cal(void);
extern float residue_heart;
extern float heat_over_flag;
extern uint32_t total_time;
extern Encoder test_down_1;
extern float residue_heart1;
extern float Last_Bullet_speed;
extern uint8_t AUTOAIM_ENABLE_SHOOT;
extern float Heat1;
extern uint8_t Heat2_Finished_Flag,Heat2_Shoot_Flag;
extern _42mm_shoot_t _42mm_shoot;
extern float friction_speed_ref;
#endif
