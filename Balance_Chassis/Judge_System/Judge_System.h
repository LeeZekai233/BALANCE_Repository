/**
  ******************************************************************************
  * @file    EMBEDDED\Judgement_System\Judge_System.h 
  * @author  William
  * @version V1.0.0
  * @date    11-September-2025
  * @brief   The file contains the headers of Judgement_System configuration  
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __JUDGE_SYSTEM_H__
#define __JUDGE_SYSTEM_H__
#include "stm32f4xx.h"                  // Device header


/*******************************************************************************#definition***************************************************************************************/
#define Judge_System_UART5_DMA_TX_BUF_LEN			150
#define Judge_System_UART5_DMA_RX_BUF_LEN			512

#define UP_REG_ID                       0xA0  	//up layer regional id, i.e., the identification ID of the higher level
#define DN_REG_ID                       0xA5  	//down layer regional id, i.e., the identification ID of the lower level
#define HEADER_LEN                      sizeof(frame_header_t)
#define CMD_LEN                         2    	//cmdid bytes
#define CRC_LEN                         2    	//crc16 bytes

/****************************************************************************Enum Definition**************************************************************************************/
typedef enum
{
	GAME_STATE_ID                      	=0x0001,//比赛状态数据:0x0001。发送频率:1Hz 
	GAME_RESULT_ID                     	=0x0002,//比赛结果数据:0x0002。发送频率:比赛结束后发送 
	GAME_ROBOT_SURVIVORS_ID            	=0x0003,//机器人存活数据:0x0003。发送频率:1Hz 
	EVENT_DADA_ID                      	=0x0101,//场地事件数据:0x0101。发送频率:事件改变后发送
	REFEREE_WARNING_ID                 	=0x0104,//裁判警告信息:0x0104。发送频率:警告发生后发送
	DART_LAUNCH_ID                		=0x0105,//飞镖发送相关数据:0x0105。发送频率:1Hz
	GAME_ROBOT_STATE_ID                	=0x0201,//比赛机器人状态:0x0201。发送频率:10Hz 
	POWER_HEAT_DATA_ID                 	=0x0202,//实时功率热量数据:0x0202。发送频率:50Hz 
	GAME_ROBOT_POS_ID                  	=0x0203,//机器人位置:0x0203。发送频率:10Hz 
	BUFF_MUSK_ENERGY_ID                	=0x0204,//机器人增益和底盘能量数据:0x0204。发送频率:状态改变后发送
	ROBOT_HURT_ID                      	=0x0206,//伤害状态:0x0206。发送频率:伤害发生后发送
	SHOOT_DATA_ID                      	=0x0207,//实时射击信息:0x0207。发送频率:射击后发送
	PROJECTILE_ALLOWANCE_ID            	=0x0208,//子弹剩余发射数:0x0208。发送频率:1Hz 周期发送，空中机器人以及哨兵主控发送
	RFID_STATE_ID                      	=0x0209,//RFID检测数据
	DART_PLAYER_COMMAND_ID             	=0x020A,//飞镖选手端指令数据
	GROUND_ROBOT_POSITION_ID           	=0x020B,//地面机器人位置数据
	RADAR_MARK_PROGRESS_ID             	=0x020C,//雷达标记进度数据
	SENTINEL_DECISION_ID               	=0x020D,//哨兵自主决策信息同步
	RADAR_DECISION_ID    			    =0x020E,//雷达自主决策信息同步
	
	STUDENT_INTERACTIVE_HEADER_DATA_ID 	=0x0301,//交互数据接收信息:0x0301。发送频率:上限 30Hz 
	CLIENT_CUSTOM_DATA_ID              	=0x0301,//客户端 客户端自定义数据:cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	ROBOT_INTERACTIVE_DATA_ID          	=0x0301,//交互数据 机器人间通信:0x0301。发送频率:上限10Hz 
	CLIENT_GRAPHIC_DRAW_ID             	=0x0301,//客户端自定义图形 机器人间通信:0x0301。发送频率:上限 10Hz 

	ROBOT_CUSTOM_CONTROLLER_DATA_ID    	=0x0302,//自定义控制器(图传链路):0x0302。发送频率：上限 30hz
	MAP_COMMAND_ID                     	=0x0303,//小地图交互
	KEY_MOUSE_REMOTE_DATA_ID    		=0x0304,//键鼠遥控器(图传链路):。发送频率：上限30hz
	CUSTOM_CONTROLLER_BACK_DATA_ID     	=0x0309,//自定义控制器回传(图传链路):。发送频率：上限10hz
}judge_data_id_e;

//机器人颜色枚举
typedef enum
{
  unkown = 0,
  blue 	 = 1,
  red    = 2,
}robot_color_e;


/****************************************************************************Struct Definition************************************************************************************/
//比赛状态数据
/* 
 0：未开始比赛 
 1：准备阶段 
 2：十五秒裁判系统自检阶段 
 3：五秒倒计时 
 4：比赛中  
 5：比赛结算中 
*/
typedef __packed struct
{
	uint8_t 	game_type 		 : 4;	//比赛类型
	uint8_t 	game_progress 	 : 4;	//当前比赛阶段
	uint16_t 	stage_remain_time	;	//当前阶段剩余时间，单位:s
	uint64_t 	SyncTimeStamp		;	//UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
}ext_game_status_t;

//比赛结果数据
typedef __packed struct
{
	uint8_t winner;
}ext_game_result_t;

//机器人血量数据
typedef __packed struct
{
	uint16_t red_1_robot_HP; 
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t reserved_; 
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP; 
	uint16_t red_base_HP; 
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t reserved; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP; 
	uint16_t blue_base_HP; 
}ext_game_robot_HP_t;

//人工智能挑战赛加成\惩罚区分布与潜伏模式状态
typedef __packed struct
{
	uint8_t 	F1_zone_status				:1;
	uint8_t 	F1_zone_buff_debuff_status	:3; 
	uint8_t 	F2_zone_status				:1;
	uint8_t 	F2_zone_buff_debuff_status	:3; 
	uint8_t 	F3_zone_status				:1;
	uint8_t 	F3_zone_buff_debuff_status	:3; 
	uint8_t 	F4_zone_status				:1;
	uint8_t 	F4_zone_buff_debuff_status	:3; 
	uint8_t 	F5_zone_status				:1;
	uint8_t 	F5_zone_buff_debuff_status	:3; 
	uint8_t 	F6_zone_status				:1;
	uint8_t 	F6_zone_buff_debuff_status	:3;
	uint16_t 	red1_bullet_left;
	uint16_t 	red2_bullet_left;
	uint16_t 	blue1_bullet_left;
	uint16_t 	blue2_bullet_left;
	uint8_t 	lurk_mode;
	uint8_t 	res;
}ext_ICRA_buff_debuff_zone_and_lurk_status_t;

//场地事件数据:

//bit 0：己方补给站前补血点的占领状态，1为已占领 
//bit 1：己方补给站内部补血点的占领状态，1为已占领 
//bit 2：己方补给区的占领状态，1为已占领（仅RMUL适用） 
//bit 3：己方能量机关激活点的占领状态，1为已占领 
//bit 4：己方小能量机关的激活状态，1为已激活 
//bit 5：己方大能量机关的激活状态，1为已激活 
//bit 6-7：己方环形高地的占领状态，1为被己方占领，2为被对方占领 
//bit 8-9：己方梯形高地的占领状态，1为被己方占领，2为被对方占领 
//bit 10-11：己方梯形高地的占领状态，1为被己方占领，2为被对方占领 
//bit 12-18：己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数） 
//bit 19-27：飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0） 
//bit 28-29：飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3为击中基地随机目标  
//bit 30-31：中心增益点的占领情况，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领。（仅RMUL适用）
typedef __packed struct
{uint32_t event_type;}
ext_event_data_t;		

//裁判警告信息:
//level——己方最后一次受到判罚的等级： 
//1：双方黄牌 
//2：黄牌 
//3：红牌 
//4：判负
//offending_robot_id——己方最后一次受到判罚的违规机器人ID。
//（如红1机器人ID为1，蓝1机器人ID为101） 判负和双方黄牌时，该值为0 
typedef __packed struct
{
	uint8_t level; 
	uint8_t offending_robot_id; 
	uint8_t count;
}ext_referee_warning_t;

//飞镖发射口倒计时
typedef __packed struct
{
	uint8_t 	dart_remaining_time;//己方飞镖发射剩余时间
	uint16_t 	dart_info; 			//包含:(1)最近一次击中的目标 (2)击中计次数 (3)飞镖此时选定的击打目标
}ext_dart_info_t;			

//比赛机器人状态
typedef __packed struct
{
  uint8_t robot_id; 							//本机器人ID
  uint8_t robot_level; 							//机器人等级
  uint16_t current_HP;  						//机器人当前血量
  uint16_t maximum_HP; 							//机器人血量上限
  uint16_t shooter_barrel_cooling_value; 		//机器人射击热量每秒冷却值
  uint16_t shooter_barrel_heat_limit; 			//机器人射击热量上限
  uint16_t chassis_power_limit;  				//机器人底盘功率上限
  uint8_t power_management_gimbal_output: 1; 	//0为无输出，1为24V输出 
  uint8_t power_management_chassis_output: 1;   //0为无输出，1为24V输出 
  uint8_t power_management_shooter_output: 1; 	//0为无输出，1为24V输出 
}ext_game_robot_status_t;

//实时功率热量数据
typedef __packed struct
{
  uint16_t 	reserved_chassis_volt; 
  uint16_t 	reserved_chassis_current; 
  float 	reserved_chassis_power; 
  uint16_t 	buffer_energy; 				//缓冲能量
  uint16_t 	shooter_17mm_1_barrel_heat; //17mm枪1射击热量
  uint16_t 	shooter_17mm_2_barrel_heat; //17mm枪2射击热量
  uint16_t 	shooter_42mm_barrel_heat;	//42mm枪 射击热量
}ext_power_heat_data_t;

//本机器人位置
typedef __packed struct
{
  float x; 			//x坐标，单位：m
  float y; 			//y坐标，单位：m
  float angle; 		//本机器人测速模块的朝向，单位：度。正北为0度
}ext_game_robot_pos_t;

//机器人增益
typedef __packed struct
{
	uint8_t power_rune_buff;		//机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
	uint8_t shoot_cool_buff;		//机器人枪口冷却倍率（直接值，值为5表示5倍冷却）
	uint8_t car_defence_buff;		//机器人防御增益（百分比，值为50表示50%防御增益）
	uint8_t car_vulnerability_buff;	//机器人负防御增益（百分比，值为30表示-30%防御增益）
	uint16_t car_attack_buff;		//机器人攻击增益（百分比，值为50表示50%攻击增益）
	uint8_t remaining_energy;		//??
}ext_buff_t;

//伤害状态
//bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
//该4 bit组成的数值为装甲模块或测速模块的ID编号；当其他原因导致扣血时，该数值为0 
//bit 4-7：血量变化类型 
//0：装甲模块被弹丸攻击导致扣血 
//1：裁判系统重要模块离线导致扣血 
//2：射击初速度超限导致扣血 
//3：枪口热量超限导致扣血 
//4：底盘功率超限导致扣血 
//5：装甲模块受到撞击导致扣血
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
}ext_robot_hurt_t;

//实时射击信息
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;	//弹丸射速(单位:Hz)
	float initial_speed;			//弹丸初速度(单位:m/s)
}ext_shoot_data_t;

//允许射击信息
typedef __packed struct 
{ 
  uint16_t projectile_allowance_17mm; 	//17mm弹丸允许发弹量
  uint16_t projectile_allowance_42mm;  	//42mm弹丸允许发弹量
  uint16_t remaining_gold_coin; 		//剩余金币数量
}projectile_allowance_t; 

//机器人 RFID 状态
typedef __packed struct
{
	uint32_t My_Base_GainPoint:1;						//己方基地增益点
	uint32_t My_Central_Tablelands_GainPoint:1;			//己方中央高地增益点
	uint32_t Enemy_Central_Tablelands_GainPoint:1;		//敌方中央高地增益点
	uint32_t My_Trapezoidal_elevation_GainPoint:1;		//己方梯形高地增益点
	uint32_t Enemy_Trapezoidal_elevation_GainPoint:1;	//敌方梯形高地增益点
	uint32_t My_FlyingSlope_GainPoint_1:1;				//靠近己方一侧飞坡前
	uint32_t My_FlyingSlope_GainPoint_2:1;				//靠近己方一侧飞坡后
	uint32_t Enemy_FlyingSlope_GainPoint_1:1;			//靠近对方一侧飞坡前
	uint32_t Enemy_FlyingSlope_GainPoint_2:1;			//靠近对方一侧飞坡后
	uint32_t My_Crossing_Step_GainPoint_1:1;			//中央高地下方
	uint32_t My_Crossing_Step_GainPoint_2:1;			//中央高地上方
	uint32_t Enemy_Crossing_Step_GainPoint_1:1;			//下
	uint32_t Enemy_Crossing_Step_GainPoint_2:1;			//上
	uint32_t My_Crossing_Highway_GainPoint_1:1;			//公路下方
	uint32_t My_Crossing_Highway_GainPoint_2:1;			//公路上方
	uint32_t Enemy_Crossing_Highway_GainPoint_1:1;		//下
	uint32_t Enemy_Crossing_Highway_GainPoint_2:1;		//上
	uint32_t My_Fort_GainPoint:1;						//己方堡垒增益点
	uint32_t My_Outpost_GainPoint:1;					//己方前哨站增益点
	uint32_t My_Recharge_Area:1;						//己方与兑换区不重叠的补给区/RMUL补给区
	uint32_t My_Recharge_Area_:1;						//己方与兑换区重叠的补给区
	uint32_t My_Gold_GainPoint:1;						//己方大资源岛增益点
	uint32_t Enemy_Gold_GainPoint:1;					//对方大资源岛增益点
	uint32_t RMUL_Centrol_GainPoint:1;					//中心增益点(仅 RMUL 适用)
	uint32_t RFID_Reverse:8;	
}ext_rfid_status_t;

//飞镖机器人客户端指令数据
typedef __packed struct
{
	uint8_t dart_launch_opening_status; //当前飞镖发射站的状态 1关闭 2再动 0开启
	uint8_t reserved;  
	uint16_t target_change_time;  		//切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0
	uint16_t latest_launch_cmd_time; 	//最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0
}ext_dart_client_cmd_t;

//客户端自定义数据:cmd_id:0x0301。内容 ID:0xD180
typedef __packed struct
{ 
	float data1;
	float data2;
	float data3; 
	uint8_t masks; 
}client_custom_data_t;

//车辆在场地的坐标
typedef __packed struct 
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float reserved_standard_5_x;
  float reserved_standard_5_y;  
}ground_robot_position_t; 

//机器人易伤情况
typedef __packed struct 
{uint8_t mark_progress;}
radar_mark_data_t;

//哨兵的战场数据（见手册）
typedef __packed struct 
{  
	uint32_t sentry_info; 
	uint16_t sentry_info_2; 
}sentry_info_t; 

//交互数据接收信息
//子内容ID 内容数据段长度 功能说明 
//0x0100   2 			 选手端删除图层 
//0x0101   15 			 选手端绘制一个图形 
//0x0102   30 			 选手端绘制两个图形 
//0x0103   75 			 选手端绘制五个图形 
//0x0104   105 			 选手端绘制七个图形 
//0x0110   45 			 选手端绘制字符图形 
//0x0120   4 			 哨兵自主决策指令 
//0x0121   1             雷达自主决策指令 
typedef __packed struct
{
  uint16_t data_cmd_id; 	//子内容ID
  uint16_t sender_id; 		//发送者ID
  uint16_t receiver_id; 	//接收者ID
//uint8_t user_data[x]; 	//x最大为112
}ext_student_interactive_header_data_t;

//交互数据 机器人间通信
typedef __packed struct
{
	uint8_t data[6];
}robot_interactive_data_t;

//子内容ID：0x0100  客户端删除图形 机器人间通信
typedef __packed struct
{
	uint8_t operate_tpye; 
	uint8_t layer; 
}ext_client_custom_graphic_delete_t;

//子内容ID：0x0101  图形数据
/*
-----------------------------------------------------------------------
类型  |  details_a |details_b | details_c |  details_d   | details_e  |
-----------------------------------------------------------------------
直线  |     -      |    -     |     -     |  终点x坐标   | 终点y坐标   |
-----------------------------------------------------------------------
矩形  |     -      |    -     |     -     |对角顶点x坐标 |对角顶点y坐标|
-----------------------------------------------------------------------
正圆  |     -      |    -     |    半径   |      -       |     -      |
-----------------------------------------------------------------------
椭圆  |     -      |    -     |     -     |  x半轴长度   |  y半轴长度  |
-----------------------------------------------------------------------
圆弧  |  起始角度  | 终止角度  |     -     |  x半轴长度   |  y半轴长度  |
-----------------------------------------------------------------------
浮点数|  字体大小  |  无作用   |    该值除以1000即实际显示值 			  |
-----------------------------------------------------------------------
整型数|  字体大小  |    -      |     32位整型数，int32_t               |
-----------------------------------------------------------------------
字符  |  字体大小  |字符长度   |     -     |     -        |     -      |
-----------------------------------------------------------------------
*/
typedef __packed struct
{ 
	uint8_t  figure_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t figure_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t details_a:9;//start angle
	uint32_t details_b:9;//end angle
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t details_c:10;//radius
	uint32_t details_d:11;//endx
	uint32_t details_e:11;//endy
}graphic_data_struct_t;

//客户端绘制一个图形 机器人间通信
typedef __packed struct
{graphic_data_struct_t grapic_data_struct;}
ext_client_custom_graphic_single_t;

//子内容ID：0x0102  客户端绘制两个图形 机器人间通信
typedef __packed struct
{graphic_data_struct_t grapic_data_struct[2];}
ext_client_custom_graphic_double_t;

//子内容ID：0x0103  客户端绘制五个图形 机器人间通信
typedef __packed struct
{graphic_data_struct_t grapic_data_struct[5];}
ext_client_custom_graphic_five_t;

//子内容ID：0x0104  客户端绘制七个图形 机器人间通信
typedef __packed struct
{graphic_data_struct_t grapic_data_struct[7];}
ext_client_custom_graphic_seven_t;

//子内容ID：0x0110  客户端绘制字符 机器人间通信
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
}ext_client_custom_character_t;

//自定义 交互数据接收信息
typedef __packed struct
{uint8_t data[30];}
robot_custon_controller_data_t;

//小地图下发信息标识:0x0303
typedef __packed struct
{
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; 	//云台手按下的键盘按键通用键值
	uint8_t target_robot_id;//对方机器人ID
	uint16_t cmd_source; 	//信息来源ID 
}ext_robot_command_t;

//客户端接收信息
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
}ext_client_map_command_t;

//雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据 0x305
typedef __packed struct 
{  
	uint16_t hero_position_x; 
	uint16_t hero_position_y; 
	uint16_t engineer_position_x; 
	uint16_t engineer_position_y; 
	uint16_t infantry_3_position_x; 
	uint16_t infantry_3_position_y; 
	uint16_t infantry_4_position_x; 
	uint16_t infantry_4_position_y; 
	uint16_t infantry_5_position_x; 
	uint16_t infantry_5_position_y; 
	uint16_t sentry_position_x; 
	uint16_t sentry_position_y; 
} map_robot_data_t; 

//图传遥控信息标识
typedef __packed struct
{
	int16_t mouse_x; 			//鼠标x轴移动速度,负标识向左移动
	int16_t mouse_y; 			//鼠标y轴移动速度,负标识向下移动
	int16_t mouse_z; 			//鼠标滚轮移动速度,负标识向后滚动
	int8_t left_button_down; 	//鼠标左键是否按下:0为未按下;1为按下
	int8_t right_button_down; 	//鼠标右键是否按下:0为未按下;1为按下
	uint16_t keyboard_value; 	//键盘按键信息，每个bit对应一个按键:0为未按下;1为按下
	uint16_t reserved; 
}remote_control_t; 

//crc8 generator polynomial:G(x)=x^8+x^5+x^4+1 
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
}frame_header_t;

//哨兵自主决策指令：0x0120 ???
typedef __packed struct 
{ 
	uint8_t 	Relive_Confirm  			:1;
	uint8_t  	Relive_At_Once  			:1;
	uint16_t 	Num_Of_Exchange_Bullet  	:11;
	uint8_t  	Remote_Exchange_Bullet_Time :4;
	uint8_t  	Remote_Exchange_Blood_Time  :4;
	
	uint16_t 	sentry_cmd_Reserve  		:10;
}sentry_cmd_t;

//the data structure receive from judgement
typedef struct
{ 
	ext_game_status_t                     	game_state;						//比赛状态数据
	ext_game_result_t                     	game_result;					//比赛结果数据
	ext_game_robot_HP_t                   	game_robot_HP;					//机器人存活数据
	ext_event_data_t                      	event_data;						//场地事件数据
	ext_referee_warning_t                 	referee_warning;				//裁判警告信息
	ext_dart_info_t							dart_info;						//飞镖发射
	ext_game_robot_status_t               	game_robot_state;				//比赛机器人状态
	ext_power_heat_data_t                 	power_heat_data;				//实时功率热量数
	ext_game_robot_pos_t                  	game_robot_pos;					//机器人位置
	ext_buff_t                            	buff_musk;						//机器人增益
	ext_robot_hurt_t                      	robot_hurt;						//伤害状态
	ext_shoot_data_t                      	shoot_data;						//实时射击信息
	projectile_allowance_t				  	Projectile_Allowance;			//0x208       
	ext_rfid_status_t                     	ext_rfid_status;				//RFID状态
	ext_dart_client_cmd_t					dart_client_cmd;				//飞镖机器人客户端指令
	client_custom_data_t                  	client_custom_data;				//客户端 客户端自定义数据  
	ground_robot_position_t					ground_robot_position;			//车辆在场地的坐标   
	radar_mark_data_t						radar_mark_data;				//机器人易伤情况  
	sentry_info_t							sentry_info;					//哨兵的战场数据           
	ext_student_interactive_header_data_t 	student_interactive_header_data;//交互数据接收信息
	robot_interactive_data_t              	robot_interactive_data;			//交互数据 机器人间通信
	graphic_data_struct_t            		graphic_data_struct;			//客户端自定义图形
	robot_custon_controller_data_t      	robot_custon_controller_data;	//自定义控制器数据
	ext_robot_command_t						robot_command;					//小地图下发信息        
	map_robot_data_t						map_robot_data;					//雷达发送对方机器人的坐标数据
	remote_control_t						remote_control;					//键鼠遥控数据
}receive_judge_t;                                                           
                                                                            
typedef __packed struct                                                      
{
  uint16_t data_cmd_id;		//子内容ID 需为开放的子内容ID
  uint16_t sender_id; 		//发送者ID 需与自身ID匹配,ID编号详见附录
  uint16_t receiver_id; 	//接收者ID 仅限己方通信 需为规则允许的多机通讯接收者 若接收者为选手端，则仅可发送至发送者对应的选手端 ID编号详见附录
}id_data_t;


/****************************************************************************Extern variable**************************************************************************************/
extern uint8_t  Judge_System_UART5_DMA_RX_BUF[2][Judge_System_UART5_DMA_RX_BUF_LEN];
extern uint8_t  Judge_System_UART5_DMA_TX_BUF[Judge_System_UART5_DMA_TX_BUF_LEN];
extern uint8_t  ddata[120];
extern uint8_t  dddata[120];
extern int Robot_Remain_HP;
extern int Robot_Max_HP;

extern receive_judge_t	judge_rece_mesg; 
extern sentry_cmd_t  	sentry_cmd;
extern robot_color_e 	robot_color;  
extern id_data_t 		send_to_aerial;	//专门用于发送给无人机的结构体

extern const unsigned char CRC8_INIT;
extern const unsigned char CRC8_TAB[256];
extern uint16_t CRC_INIT;
extern const uint16_t wCRC_Table[256];


/****************************************************************************Extern Function**************************************************************************************/


/****************************************************************************Function Declaration*********************************************************************************/	
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned char get_crc8(unsigned char* data, unsigned int length);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void MYDMA_Enable (DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);
void data_upload_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);
void Send_bullet_remaining_num(void);
void judgement_data_handle (uint8_t *p_frame, uint16_t rec_len);
void Judge_System_Receive (uint8_t *p_frame);


#endif /*_JUDGE_SYSTEM_H*/

