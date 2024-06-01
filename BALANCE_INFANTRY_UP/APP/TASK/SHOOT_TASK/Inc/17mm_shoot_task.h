#ifndef __17MM_SHOOT_TASK
#define __17MM_SHOOT_TASK
#include "public.h"


#define FRICTION_BUF_SPEED_30  (-970)
#define FRICTION_SPEED_30  (-950)
#define BULLET_SPEED_TARGET     28
#define BULLET_SPEED_SELF_ADAPTATION_K    100

typedef struct{
        float Error_Mea;
        float Error_Est;
        float Error_Est_Last;
        float Kalman_Gain;
        float X_hat;
        float X_hat_Last;
}First_Order_Kalman_Filter_t;


typedef enum
{
  OUTBURST  = 0,
  COOLING   = 1,
  ZOOM     = 2,

} shoot_mode_selection_e;

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,

} shoot_mode_e;

typedef enum
{
  TRIG_INIT       = 0,
  TRIG_PRESS_DOWN = 1,
  TRIG_BOUNCE_UP  = 2,
  TRIG_ONE_DONE   = 3,
} trig_state_e;

typedef struct
{
	float speed_ref[2];
	float speed_fdb[2];
	float angle_ref[2];
	float angle_fdb[2];
	
}shoot_pid_friction_t;

typedef struct
{
	float speed_ref;
	float speed_fdb;
	float angle_ref;
	float angle_fdb;
	
}shoot_pid_poke_t;

typedef struct
{
  /* shoot task relevant param */
	shoot_mode_selection_e  shoot_mode_selection;
  shoot_mode_e 						ctrl_mode;
	shoot_pid_poke_t        poke_pid;
	shoot_pid_friction_t		friction_pid;
	int16_t        poke_current;
	int16_t        fric_current[2];
	uint8_t      poke_run;
	uint8_t      bulletspead_level;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
  float     will_time_shoot;
  float     remain_bullets;
	uint8_t        single_angle;
	float        shoot_frequency;
  float        total_speed;
  float        limit_heart0;
	float        limit_heart1;
  uint16_t     max_heart0;
  uint16_t     cooling_ratio;
    First_Order_Kalman_Filter_t Bullet_Speed_Kalman;
} shoot_t;


void shot_param_init(void);
void heat0_limit(void);
void shoot_task(void);
void performance_select(void);
void shoot_mode_switch(void);
void speed_switch(void);
void heat_switch(void);
void shoot_bullet_handle(void);
void shoot_friction_handle(void);
void shoot_state_mode_switch(void);
void heat_shoot_frequency_limit(void);
void bullets_spilling(void);
float First_Order_Kalman_Filter_Cal
        (First_Order_Kalman_Filter_t *_First_Order_Kalman_Filter
        ,float _Z/*≤‚¡ø÷µ*/);

extern shoot_t shoot;
extern u8 press_l_state_switch;

extern pid_t pid_trigger_speed_buf;
extern pid_t pid_trigger_angle_buf;
#endif
