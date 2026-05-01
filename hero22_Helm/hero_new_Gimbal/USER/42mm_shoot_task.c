#include "public.h"

#define INIT_ANGLE  40
#define SINGLE_ANGLE 60
#define FIRE_COOLING_MAX_MANUAL 300
#define FIRE_COOLING_MAX_AUTO 350

float Fric_Speed_Normal[2]={3220,3680};

float Fric_Speed_Snipe[2]={4650,5190};

float friction_speed_ref;

_42mm_shoot_t _42mm_shoot;
friction_t general_friction = {0};

uint8_t AUTOAIM_ENABLE_SHOOT=1;
Encoder test_down_1;

float residue_heart,heat_over_flag; //剩余热量
float residue_heart1,residue_heart2;
float Heat1,Heat2;

uint8_t Heat2_Shoot_Flag,Heat2_Finished_Flag;
uint8_t Shoot_Flag;
uint16_t Heat2_TimeOut_Cnt;

int Fire_Cooling_cnt_manual=FIRE_COOLING_MAX_MANUAL;
int Fire_Cooling_cnt_auto=FIRE_COOLING_MAX_AUTO;

extern Encoder Poke_3508;
pid_t Poke_Reverse_PID;
uint8_t back_flag=0;
uint16_t back_cnt=0;
uint8_t back_trap=0;
uint8_t back_normal_flag=0;
uint8_t poke_init_flag=0;
float angle_real;
float temp11;
uint32_t work_time;
uint8_t single_use=0;
uint16_t Init_flag=0;

uint8_t flag1=0;
uint16_t last_shoot;
uint32_t cnt111=0;

uint8_t time_start_flag=0;
uint32_t time_test_cnt;

int16_t last_rpm;
uint32_t total_time;

float Last_Bullet_speed;

//对发射的热量进行限制,要先根据等级给定初始热量,一级为100
//这里三个剩余热量是尝试对热量进行两种不同的方式计算：依赖50hz的传统裁判系统实时返回数据；每次射击后枪管会立刻发送弹速数据，自行计算热量；完全自行计算热量，只在裁判系统返回弹速时更新总热量
void Heat_Control(void)
{
    if(flag1==0)//上电后一次性用途,先将热量初始化为当前等级热量上限
    {
        residue_heart=judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit;
        residue_heart1=judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit;
        flag1=1;
    }
    //剩余热量等于热量上限-实时消耗热量（均来于裁判系统）
    residue_heart=judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit - judge_rece_mesg.power_heat_data.shooter_42mm_barrel_heat;
	//剩余热量等于热量上限-实时消耗热量（实时热量由测速反馈计算）
    residue_heart1=judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit - Heat1;
    //剩余热量等于热量上限-实时消耗热量（实时热量由拨盘计算）
	if(Heat2_Finished_Flag==1)
		Heat2=judge_rece_mesg.power_heat_data.shooter_42mm_barrel_heat;
    residue_heart2=judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit - Heat2;
    //剩余热量按照当前等级每秒冷却值不断增加
    if(residue_heart<judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit)
    {
        residue_heart+=(float)judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value/1000;
    }
    else//加到上限停止
    {
        residue_heart=judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit;
    }
	if(Heat1>0)
	{
		Heat1-=(float)judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value/1000;
	}
	else
	{
		Heat1=0;
	}
	if(Heat2>0)
	{
		Heat2-=(float)judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value/1000;
	}
	else
	{
		Heat2=0;
	}
    
     if(residue_heart1<residue_heart)//三条热量计算取限制比较死的那一条
    {
        residue_heart=residue_heart1;
    }
	if(residue_heart2<residue_heart)
	{
		residue_heart=residue_heart2;
	}
    //剩下的热量足够时标志位为0,反之为1
    if(residue_heart>=100)
    {
        heat_over_flag=0;
    }
    else
    {
        heat_over_flag=1;
        _42mm_shoot.shoot_flag=0;
    }
    
    
}

//对摩擦轮和拨盘pid初始化
void _42mm_Shoot_PID_Init(void)
{
	PID_struct_init(&_42mm_shoot.pid_left_friction_speed[0],POSITION_PID,5000,200,10,0.005,1);
	PID_struct_init(&_42mm_shoot.pid_left_friction_speed[1],POSITION_PID,5000,200,10,0.005,1);
	PID_struct_init(&_42mm_shoot.pid_right_friction_speed[0],POSITION_PID,5000,200,10,0.005,1);
	PID_struct_init(&_42mm_shoot.pid_right_friction_speed[1],POSITION_PID,5000,200,10,0.005,1);
    PID_struct_init(&_42mm_shoot.pid_down_friction_speed[0],POSITION_PID,5000,200,10,0.005,1);
    PID_struct_init(&_42mm_shoot.pid_down_friction_speed[1],POSITION_PID,5000,200,10,0.005,1);

//拨盘的pid给得很极限,略微超调保证发弹响应速度    
    PID_struct_init(&_42mm_shoot.pid_downpoke_angle,POSITION_PID,3800,10000,90,0,0);
	PID_struct_init(&_42mm_shoot.pid_downpoke_speed,POSITION_PID,16000,5000,20,0.7,10);
    PID_struct_init(&Poke_Reverse_PID,POSITION_PID,10000,10000,15,0,0);
    
}

void _42mm_Shoot_Normal_Hdandle(void)
{
    if(gimbal_data.ctrl_mode==GIMBAL_SNIPE||gimbal_data.ctrl_mode==GIMBAL_RADAR_ASSISTANT_SNIPE)
    {
        //16.5m/s,在调试阶段实际弹速与16.5有出入,修改后面pid目标值即可,用rpm做的速度闭环
        _42mm_shoot.left_friction_current[0]=pid_calc(&_42mm_shoot.pid_left_friction_speed[0],general_friction.left_up_motor.rate_rpm,Fric_Speed_Snipe[1]);
        _42mm_shoot.left_friction_current[1]=pid_calc(&_42mm_shoot.pid_left_friction_speed[1],general_friction.left_down_motor.rate_rpm,Fric_Speed_Snipe[0]);
        _42mm_shoot.right_friction_current[0]=pid_calc(&_42mm_shoot.pid_right_friction_speed[0],general_friction.right_up_motor.rate_rpm,Fric_Speed_Snipe[1]);
        _42mm_shoot.right_friction_current[1]=pid_calc(&_42mm_shoot.pid_right_friction_speed[1],general_friction.right_down_motor.rate_rpm,Fric_Speed_Snipe[0]);
        _42mm_shoot.down_friction_current[0]=pid_calc(&_42mm_shoot.pid_down_friction_speed[0],general_friction.down_up_motor.rate_rpm,Fric_Speed_Snipe[1]);
        _42mm_shoot.down_friction_current[1]=pid_calc(&_42mm_shoot.pid_down_friction_speed[1],general_friction.down_down_motor.rate_rpm,Fric_Speed_Snipe[0]);
		friction_speed_ref=Fric_Speed_Snipe[1];
    }
    else if(gimbal_data.ctrl_mode==GIMBAL_FOLLOW_ZGYRO || gimbal_data.ctrl_mode==GIMBAL_AUTO_AIM)
    {
        //12m/s
        _42mm_shoot.left_friction_current[0]=pid_calc(&_42mm_shoot.pid_left_friction_speed[0],general_friction.left_up_motor.rate_rpm,Fric_Speed_Normal[1]);
        _42mm_shoot.left_friction_current[1]=pid_calc(&_42mm_shoot.pid_left_friction_speed[1],general_friction.left_down_motor.rate_rpm,Fric_Speed_Normal[0]);
        _42mm_shoot.right_friction_current[0]=pid_calc(&_42mm_shoot.pid_right_friction_speed[0],general_friction.right_up_motor.rate_rpm,Fric_Speed_Normal[1]);
        _42mm_shoot.right_friction_current[1]=pid_calc(&_42mm_shoot.pid_right_friction_speed[1],general_friction.right_down_motor.rate_rpm,Fric_Speed_Normal[0]);
        _42mm_shoot.down_friction_current[0]=pid_calc(&_42mm_shoot.pid_down_friction_speed[0],general_friction.down_up_motor.rate_rpm,Fric_Speed_Normal[1]);
        _42mm_shoot.down_friction_current[1]=pid_calc(&_42mm_shoot.pid_down_friction_speed[1],general_friction.down_down_motor.rate_rpm,Fric_Speed_Normal[0]);
		friction_speed_ref=Fric_Speed_Normal[1];
        
                
    }
    else
    {
        _42mm_shoot.left_friction_current[0]=pid_calc(&_42mm_shoot.pid_left_friction_speed[0],general_friction.left_up_motor.rate_rpm,0);
        _42mm_shoot.left_friction_current[1]=pid_calc(&_42mm_shoot.pid_left_friction_speed[1],general_friction.left_down_motor.rate_rpm,0);
        _42mm_shoot.right_friction_current[0]=pid_calc(&_42mm_shoot.pid_right_friction_speed[0],general_friction.right_up_motor.rate_rpm,0);
        _42mm_shoot.right_friction_current[1]=pid_calc(&_42mm_shoot.pid_right_friction_speed[1],general_friction.right_down_motor.rate_rpm,0);
        _42mm_shoot.down_friction_current[0]=pid_calc(&_42mm_shoot.pid_down_friction_speed[0],general_friction.down_up_motor.rate_rpm,0);
        _42mm_shoot.down_friction_current[1]=pid_calc(&_42mm_shoot.pid_down_friction_speed[1],general_friction.down_down_motor.rate_rpm,0);
		friction_speed_ref=0;
    }
    
    

    
    
    
}

float poke_target_angle;

float Bias_angle;
uint16_t reverse_cnt;
uint8_t back_normal_flag2;
uint32_t reverse_cnt2;
void Poke_Init(void)
{
    if(poke_init_flag==0)//进行拨盘初始化
    {
        //闭拨盘速度环先将拨盘反转
        _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=pid_calc(&Poke_Reverse_PID,Poke_3508.rate_rpm,1000);
        if(Poke_3508.rate_rpm>500)back_normal_flag2=1;//判断拨盘是否正常反转
        
        
        if(back_normal_flag2==1)//此时拨盘能正常反转
        {
            if(fabs(Poke_3508.rate_rpm)<5)//当反转使得弹丸卡到机械限位时停止
            {    
                reverse_cnt2++;
                if(reverse_cnt2==300)//略做等待
                {
                    Bias_angle=Poke_3508.ecd_angle/19.2;//记录下此时的偏置角度(拨盘实际角度,=电机反馈角度/减速比)
                    reverse_cnt2=0;
                    poke_init_flag=1;//初始化完成
                    Init_flag=1;
                }
            }                  
        }        
    }
}


uint8_t bias_finish=0;
uint8_t sing_use=0;
void Poke_Handle(void)
{   
    if(single_use==0 && poke_init_flag==1)//初始化时先将拨盘进入正常状态,一次性,只使用一次
    {
        _42mm_shoot.poke_state=POKE_NORMAL;
        single_use=1;
    }
    
    if(_42mm_shoot.poke_state==POKE_NORMAL)
    {
        angle_real=Poke_3508.ecd_angle/19.2;//实时记录拨盘的真实角度值                
        if(bias_finish==0)
        {
            //之前初始化时拨盘是卡在机械限位,这里对拨盘给定一个初始角度,这个角度是实测出来的,能够使得弹丸贴合摩擦轮前面的限位轴承,用角度环驱动拨盘到这个限位
            _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=pid_double_loop_cal(&_42mm_shoot.pid_downpoke_angle,&_42mm_shoot.pid_downpoke_speed,
                                                                                Bias_angle-INIT_ANGLE,angle_real,&_42mm_shoot.pid_downpoke_angle.out,Poke_3508.rate_rpm,0);
            if(Poke_3508.rate_rpm==0 && fabs(Bias_angle-INIT_ANGLE-angle_real)<2)
            {
                bias_finish=1;//判断是否达到这个初始角度
            }
        }                
        else
        {
            if(sing_use==0)//先转到初始角度
            {
                temp11=Bias_angle-INIT_ANGLE+SINGLE_ANGLE;//这里是后面发现拨盘初始角度少了一个弹丸加上的,和上面初始化拨盘角度作用相同,一次性
                sing_use=1;
            }
						if(fabsf(general_friction.down_up_motor.rate_rpm-3610)>50&&Shoot_Flag==1)
						{
							Shoot_Flag=0;
							total_time=TIM_GetCounter(TIM2);
						}
            
            /******************开火判定******************/
            if(gimbal_data.ctrl_mode==GIMBAL_AUTO_AIM)//自瞄模式下的开火判定
            {
                if((RC_CtrlData.RemoteSwitch.trigger==1||Auto_Shoot_Fire==1) &&
					My_Auto_Shoot.Auto_Aim.enable_shoot==1 && heat_over_flag==0/* && last_shoot!=1 &&AUTOAIM_ENABLE_SHOOT==1*/
					&&Fire_Cooling_cnt_auto==FIRE_COOLING_MAX_AUTO&&
					general_friction.left_up_motor.rate_rpm>3000&&
					general_friction.left_down_motor.rate_rpm>3000&&
					general_friction.right_down_motor.rate_rpm>3000&&
					general_friction.right_up_motor.rate_rpm>3000&&
					general_friction.down_down_motor.rate_rpm>3000&&
					general_friction.down_up_motor.rate_rpm>3000&&
					work_time==0)/*&&_42mm_shoot.shoot_flag==1*/
                {
//					AUTOAIM_ENABLE_SHOOT=0;
                    temp11-=SINGLE_ANGLE;//angle_real;//记录当前角度,下面的temp11-SINGLE_ANGLE即为拨出一发弹丸
//                    _42mm_shoot.shoot_flag=0;
                    New_Auto_Aim_Send.poke_state=1;//自瞄需要接收我们是否开火
                    Fire_Cooling_cnt_auto=0;
                    TIM_SetCounter(TIM2,0);//开火时间清零,开始计时,这里这个定时器用于计算开火响应时间,从代码向拨盘发出开火指令到枪管检测到有弹丸通过
					Heat2+=100;
					Heat2_Shoot_Flag=1;
					Heat2_Finished_Flag=0;
				}
                else
                {
                    New_Auto_Aim_Send.poke_state=0;
                }
                
          
                
                
                last_shoot=My_Auto_Shoot.Auto_Aim.enable_shoot;
                _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=pid_double_loop_cal(&_42mm_shoot.pid_downpoke_angle,&_42mm_shoot.pid_downpoke_speed,
                                                                                temp11-SINGLE_ANGLE,angle_real,&_42mm_shoot.pid_downpoke_angle.out,Poke_3508.rate_rpm,0);
                if(fabsf(Poke_3508.rate_rpm)<200 && fabsf(temp11-SINGLE_ANGLE-angle_real)>5)//与想要达到的目标角度相差超过5度且速度为零判断为卡弹
                {
                    work_time++;
               
                }
				else
					work_time=0;
                //当卡弹计时1s时,拨盘进入卡住状态,后续反转进行处理
                if(work_time>=300)
                {
                    _42mm_shoot.poke_state=POKE_TRAP;
                    work_time=0;
               
                }   
            }
            else//非自瞄模式下的普通开火逻辑,基本和上面一样
            {
                if(_42mm_shoot.shoot_flag==1 && heat_over_flag==0&&Fire_Cooling_cnt_manual==FIRE_COOLING_MAX_MANUAL&&
					general_friction.left_up_motor.rate_rpm>3000&&
					general_friction.left_down_motor.rate_rpm>3000&&
					general_friction.right_down_motor.rate_rpm>3000&&
					general_friction.right_up_motor.rate_rpm>3000&&
					general_friction.down_down_motor.rate_rpm>3000&&
					general_friction.down_up_motor.rate_rpm>3000&&
					work_time==0)
                {
                    temp11-=SINGLE_ANGLE;//angle_real;
                    _42mm_shoot.shoot_flag=0;
//                    New_Auto_Aim_Send.poke_state=1;
                    TIM_SetCounter(TIM2,0);
					Fire_Cooling_cnt_manual=0;
					Heat2+=100;
					Heat2_Shoot_Flag=1;
					Heat2_Finished_Flag=0;
					Shoot_Flag=1;
                }
				
                _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=pid_double_loop_cal(&_42mm_shoot.pid_downpoke_angle,&_42mm_shoot.pid_downpoke_speed,
                                                                                temp11-SINGLE_ANGLE,angle_real,&_42mm_shoot.pid_downpoke_angle.out,Poke_3508.rate_rpm,0);
                        
                if(fabsf(Poke_3508.rate_rpm)<200 && fabsf(temp11-SINGLE_ANGLE-angle_real)>5)
                {
                    work_time++;
                }
				else
					work_time=0;
            
                if(work_time>=300)
                {
                    _42mm_shoot.poke_state=POKE_TRAP;
                    work_time=0;
                }   
            }
			
				if(Fire_Cooling_cnt_manual!=FIRE_COOLING_MAX_MANUAL)
					Fire_Cooling_cnt_manual++;
				if(Fire_Cooling_cnt_manual>FIRE_COOLING_MAX_MANUAL)
					Fire_Cooling_cnt_manual=FIRE_COOLING_MAX_MANUAL;
				if(Fire_Cooling_cnt_auto!=FIRE_COOLING_MAX_AUTO)
					Fire_Cooling_cnt_auto++;
				if(Fire_Cooling_cnt_auto>FIRE_COOLING_MAX_AUTO)
					Fire_Cooling_cnt_auto=FIRE_COOLING_MAX_AUTO;
				
				if(Heat2_Shoot_Flag==1&&Heat2_Finished_Flag==0)
				{
					Heat2_TimeOut_Cnt++;
				}
				if(Heat2_TimeOut_Cnt>290&&Heat2_Finished_Flag==0)
				{
					Heat2-=100;
					Heat2_Shoot_Flag=0;
					Heat2_Finished_Flag=1;
					Heat2_TimeOut_Cnt=0;
				}
            
        }     
    }
    else if(_42mm_shoot.poke_state==POKE_TRAP)
    {
        //速度环用于反转
        _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=pid_calc(&Poke_Reverse_PID,Poke_3508.rate_rpm,1000);//目标值速度为负值时为发射,正为反转 

        
        
//            if(fabsf(Poke_3508.rate_rpm)<5)//当反转速度小于五时反转成功计时器++
            {    
                reverse_cnt++;
                if(reverse_cnt==300)
                {
                    _42mm_shoot.poke_state=POKE_NORMAL;
                    back_normal_flag=0;
                    reverse_cnt=0;
					Fire_Cooling_cnt_manual=0;
					Fire_Cooling_cnt_auto=0;
//                    temp11+=SINGLE_ANGLE;//45.3;//若不对temp11处理,这时拨盘进入到正常状态时按照原来角度会直接打出一发弹,因此这里对原来的角度进行后移处理,保证反转后不直接发弹
                }
            }                       
    }
}





void _42mm_Shoot_Task(void)
{
    switch(_42mm_shoot.ctrl_mode)
	{
        
		case _42MM_SHOOT_NORMAL:
		{
            if(Init_flag==0)
            {
                Poke_Init();
                
            }
            else
            {
                _42mm_Shoot_Normal_Hdandle();
                Poke_Handle();
            }
		}
		break;
        
        case _42MM_SHOOT_RELAX:
			_42mm_shoot.left_friction_current[0]=pid_calc(&_42mm_shoot.pid_left_friction_speed[0],general_friction.left_up_motor.rate_rpm,0);
			_42mm_shoot.left_friction_current[1]=pid_calc(&_42mm_shoot.pid_left_friction_speed[1],general_friction.left_down_motor.rate_rpm,0);
			_42mm_shoot.right_friction_current[0]=pid_calc(&_42mm_shoot.pid_right_friction_speed[0],general_friction.right_up_motor.rate_rpm,0);
			_42mm_shoot.right_friction_current[1]=pid_calc(&_42mm_shoot.pid_right_friction_speed[1],general_friction.right_down_motor.rate_rpm,0);
			_42mm_shoot.down_friction_current[0]=pid_calc(&_42mm_shoot.pid_down_friction_speed[0],general_friction.down_up_motor.rate_rpm,0);
			_42mm_shoot.down_friction_current[1]=pid_calc(&_42mm_shoot.pid_down_friction_speed[1],general_friction.down_down_motor.rate_rpm,0);
			_42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input=0;
        break;
		default:
			
		break;
    }
}
    

