#ifndef __JUDGE_SYSTEM_H__
#define __JUDGE_SYSTEM_H__

#include <stm32f4xx.h>

/** 
  * @brief  judgement data command id
  */
	
#define UART5_TX_BUF_LENGTH             150
#define BSP_UART5_DMA_RX_BUF_LEN        512  

#define UP_REG_ID                       0xA0  //up layer regional id
#define DN_REG_ID                       0xA5  //down layer regional id
#define HEADER_LEN                      sizeof(frame_header_t)
#define CMD_LEN                         2    //cmdid bytes
#define CRC_LEN                         2    //crc16 bytes
	
typedef enum
{
	GAME_STATE_ID                      =0x0001,//±ÈÈü×´Ì¬Êı¾İ:0x0001¡£·¢ËÍÆµÂÊ:1Hz 
	GAME_RESULT_ID                     =0x0002,//±ÈÈü½á¹ûÊı¾İ:0x0002¡£·¢ËÍÆµÂÊ:±ÈÈü½áÊøºó·¢ËÍ 
	GAME_ROBOT_SURVIVORS_ID            =0x0003,//»úÆ÷ÈË´æ»îÊı¾İ:0x0003¡£·¢ËÍÆµÂÊ:1Hz 
	EVENT_DADA_ID                      =0x0101,//³¡µØÊÂ¼şÊı¾İ:0x0101¡£·¢ËÍÆµÂÊ:ÊÂ¼ş¸Ä±äºó·¢ËÍ
	REFEREE_WARNING_ID                 =0x0104,//²ÃÅĞ¾¯¸æĞÅÏ¢:0x0104¡£·¢ËÍÆµÂÊ:¾¯¸æ·¢Éúºó·¢ËÍ
	DART_LAUNCH_ID                		 =0x0105,//·ÉïÚ·¢ËÍÏà¹ØÊı¾İ:0x0105¡£·¢ËÍÆµÂÊ:1Hz
	GAME_ROBOT_STATE_ID                =0x0201,//±ÈÈü»úÆ÷ÈË×´Ì¬:0x0201¡£·¢ËÍÆµÂÊ:10Hz 
	POWER_HEAT_DATA_ID                 =0x0202,//ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ:0x0202¡£·¢ËÍÆµÂÊ:50Hz 
	GAME_ROBOT_POS_ID                  =0x0203,//»úÆ÷ÈËÎ»ÖÃ:0x0203¡£·¢ËÍÆµÂÊ:10Hz 
	BUFF_MUSK_ENERGY_ID                =0x0204,//»úÆ÷ÈËÔöÒæºÍµ×ÅÌÄÜÁ¿Êı¾İ:0x0204¡£·¢ËÍÆµÂÊ:×´Ì¬¸Ä±äºó·¢ËÍ
	ROBOT_HURT_ID                      =0x0206,//ÉËº¦×´Ì¬:0x0206¡£·¢ËÍÆµÂÊ:ÉËº¦·¢Éúºó·¢ËÍ
	SHOOT_DATA_ID                      =0x0207,//ÊµÊ±Éä»÷ĞÅÏ¢:0x0207¡£·¢ËÍÆµÂÊ:Éä»÷ºó·¢ËÍ
	PROJECTILE_ALLOWANCE_ID            =0x0208,//×Óµ¯Ê£Óà·¢ÉäÊı:0x0208¡£·¢ËÍÆµÂÊ:1Hz ÖÜÆÚ·¢ËÍ£¬¿ÕÖĞ»úÆ÷ÈËÒÔ¼°ÉÚ±øÖ÷¿Ø·¢ËÍ
	RFID_STATE_ID                      =0x0209,//RFID¼ì²âÊı¾İ
	DART_PLAYER_COMMAND_ID             =0x020A,//·ÉïÚÑ¡ÊÖ¶ËÖ¸ÁîÊı¾İ
	GROUND_ROBOT_POSITION_ID           =0x020B,//µØÃæ»úÆ÷ÈËÎ»ÖÃÊı¾İ
	RADAR_MARK_PROGRESS_ID             =0x020C,//À×´ï±ê¼Ç½ø¶ÈÊı¾İ
	SENTINEL_DECISION_ID               =0x020D,//ÉÚ±ø×ÔÖ÷¾ö²ßĞÅÏ¢Í¬²½
	RADAR_DECISION_ID    			         =0x020E,//À×´ï×ÔÖ÷¾ö²ßĞÅÏ¢Í¬²½
	
	STUDENT_INTERACTIVE_HEADER_DATA_ID =0x0301,//½»»¥Êı¾İ½ÓÊÕĞÅÏ¢:0x0301¡£·¢ËÍÆµÂÊ:ÉÏÏŞ 30Hz 
	CLIENT_CUSTOM_DATA_ID              =0x0301,//¿Í»§¶Ë ¿Í»§¶Ë×Ô¶¨ÒåÊı¾İ:cmd_id:0x0301¡£ÄÚÈİ ID:0xD180¡£·¢ËÍÆµÂÊ£ºÉÏÏŞ 10Hz 
	ROBOT_INTERACTIVE_DATA_ID          =0x0301,//½»»¥Êı¾İ »úÆ÷ÈË¼äÍ¨ĞÅ:0x0301¡£·¢ËÍÆµÂÊ:ÉÏÏŞ10Hz 
	CLIENT_GRAPHIC_DRAW_ID             =0x0301,//¿Í»§¶Ë×Ô¶¨ÒåÍ¼ĞÎ »úÆ÷ÈË¼äÍ¨ĞÅ:0x0301¡£·¢ËÍÆµÂÊ:ÉÏÏŞ 10Hz 

	ROBOT_CUSTOM_CONTROLLER_DATA_ID    =0x0302,//×Ô¶¨Òå¿ØÖÆÆ÷(Í¼´«Á´Â·):0x0302¡£·¢ËÍÆµÂÊ£ºÉÏÏŞ 30hz
	MAP_COMMAND_ID                     =0x0303,//Ğ¡µØÍ¼½»»¥
	KEY_MOUSE_REMOTE_DATA_ID    			 =0x0304,//¼üÊóÒ£¿ØÆ÷(Í¼´«Á´Â·):¡£·¢ËÍÆµÂÊ£ºÉÏÏŞ30hz
	CUSTOM_CONTROLLER_BACK_DATA_ID     =0x0309,//×Ô¶¨Òå¿ØÖÆÆ÷»Ø´«(Í¼´«Á´Â·):¡£·¢ËÍÆµÂÊ£ºÉÏÏŞ10hz
} judge_data_id_e;

//±ÈÈü×´Ì¬Êı¾İ
/* 
 0£ºÎ´¿ªÊ¼±ÈÈü 
 1£º×¼±¸½×¶Î 
 2£ºÊ®ÎåÃë²ÃÅĞÏµÍ³×Ô¼ì½×¶Î 
 3£ºÎåÃëµ¹¼ÆÊ± 
 4£º±ÈÈüÖĞ  
 5£º±ÈÈü½áËãÖĞ 
*/
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

//±ÈÈü½á¹ûÊı¾İ
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;

//»úÆ÷ÈËÑªÁ¿Êı¾İ
typedef __packed struct
{
uint16_t ally_1_robot_HP; //¼º·½ 1 ºÅÓ¢ĞÛ»úÆ÷ÈËÑªÁ¿£¬Èô¸Ã»úÆ÷ÈËÎ´ÉÏ³¡»òÕß±»·£ÏÂ£¬ÔòÑªÁ¿Îª 0£¬ÏÂÎÄÍ¬Àí
uint16_t ally_2_robot_HP; //¼º·½ 2 ºÅ¹¤³Ì»úÆ÷ÈËÑªÁ¿ 
uint16_t ally_3_robot_HP; //¼º·½ 3 ºÅ²½±ø»úÆ÷ÈËÑªÁ¿ 
uint16_t ally_4_robot_HP; //¼º·½ 4 ºÅ²½±ø»úÆ÷ÈËÑªÁ¿ 
uint16_t reserved; 				//±£ÁôÎ» 
uint16_t ally_7_robot_HP; //¼º·½ 7 ºÅÉÚ±ø»úÆ÷ÈËÑªÁ¿ 
uint16_t ally_outpost_HP; //¼º·½Ç°ÉÚÕ¾ÑªÁ¿
uint16_t ally_base_HP; 		//¼º·½»ùµØÑªÁ¿ 
} ext_game_robot_HP_t;

//ÈË¹¤ÖÇÄÜÌôÕ½Èü¼Ó³É\³Í·£Çø·Ö²¼ÓëÇ±·üÄ£Ê½×´Ì¬
typedef __packed struct
{
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3; 
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3; 
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3; 
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3; 
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3; 
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
	uint16_t red1_bullet_left;
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
	uint8_t lurk_mode;
	uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;

//³¡µØÊÂ¼şÊı¾İ:
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;		//Ã¿Ò»Î»¶ÔÓ¦Ò»ÖÖ±ÈÈü³¡µØÊÂ¼ş£¬Ïê¼ûÊÖ²á

//²ÃÅĞ¾¯¸æĞÅÏ¢:
typedef __packed struct
{
 uint8_t level; 
 uint8_t offending_robot_id; 
 uint8_t count;
} ext_referee_warning_t;

//·ÉïÚ·¢Éä¿Úµ¹¼ÆÊ±
typedef __packed struct
{
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 		//°üº¬:(1)×î½üÒ»´Î»÷ÖĞµÄÄ¿±ê (2)»÷ÖĞ¼Æ´ÎÊı (3)·ÉïÚ´ËÊ±Ñ¡¶¨µÄ»÷´òÄ¿±ê
} ext_dart_info_t;			

//±ÈÈü»úÆ÷ÈË×´Ì¬
typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 	//»úÆ÷ÈËµÈ¼¶
  uint16_t current_HP;  //»úÆ÷ÈËµ±Ç°ÑªÁ¿
  uint16_t maximum_HP; 	//»úÆ÷ÈËÑªÁ¿ÉÏÏŞ
  uint16_t shooter_barrel_cooling_value; //»úÆ÷ÈËÉä»÷ÈÈÁ¿Ã¿ÃëÀäÈ´Öµ
  uint16_t shooter_barrel_heat_limit; //»úÆ÷ÈËÉä»÷ÈÈÁ¿ÉÏÏŞ
  uint16_t chassis_power_limit;  //»úÆ÷ÈËµ×ÅÌ¹¦ÂÊÉÏÏŞ
  uint8_t power_management_gimbal_output 	: 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
} ext_game_robot_status_t;

//ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ
typedef __packed struct
{
  uint16_t reserved; 
  uint16_t reserved_; 
  float reserved__; 
  uint16_t buffer_energy; 	//»º³åÄÜÁ¿
  uint16_t shooter_17mm_1_barrel_heat; 	//17mmÇ¹1Éä»÷ÈÈÁ¿
  uint16_t shooter_42mm_barrel_heat;	//42mmÇ¹ Éä»÷ÈÈÁ¿
} ext_power_heat_data_t;

//ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ
//typedef __packed struct
//{
//	uint16_t chassis_volt; 
//	uint16_t chassis_current; 
//	float chassis_power; 
//	uint16_t chassis_power_buffer; 
//	uint16_t shooter_id1_17mm_cooling_heat;
//	uint16_t shooter_id2_17mm_cooling_heat;
//	uint16_t shooter_id1_42mm_cooling_heat;
//} ext_power_heat_data_t;

//»úÆ÷ÈËÎ»ÖÃ
typedef __packed struct
{
  float x; 	//±¾»úÆ÷ÈËÎ»ÖÃx×ø±ê£¬µ¥Î»£ºm
  float y; 
  float angle; 		//	±¾»úÆ÷ÈË²âËÙÄ£¿éµÄ³¯Ïò£¬µ¥Î»£º¶È¡£Õı±±Îª0¶È
} ext_game_robot_pos_t;

//»úÆ÷ÈËÔöÒæ
typedef __packed struct 
{ 
 uint8_t recovery_buff; 		 //»úÆ÷ÈË»ØÑªÔöÒæ£¨°Ù·Ö±È£¬ÖµÎª 10 ±íÊ¾Ã¿Ãë»Ö¸´ÑªÁ¿ÉÏÏŞµÄ 10%£© 
 uint16_t cooling_buff; 		 //»úÆ÷ÈËÉä»÷ÈÈÁ¿ÀäÈ´ÔöÒæ¾ßÌåÖµ£¨Ö±½ÓÖµ£¬ÖµÎª x ±íÊ¾ÈÈÁ¿ÀäÈ´Ôö¼Ó x/s£©
 uint8_t defence_buff; 			 //»úÆ÷ÈË·ÀÓùÔöÒæ£¨°Ù·Ö±È£¬ÖµÎª 50 ±íÊ¾ 50%·ÀÓùÔöÒæ£© 
 uint8_t vulnerability_buff; //»úÆ÷ÈË¸º·ÀÓùÔöÒæ£¨°Ù·Ö±È£¬ÖµÎª 30 ±íÊ¾-30%·ÀÓùÔöÒæ£©
 uint16_t attack_buff; 		 	//»úÆ÷ÈË¹¥»÷ÔöÒæ£¨°Ù·Ö±È£¬ÖµÎª 50 ±íÊ¾ 50%¹¥»÷ÔöÒæ£© 
 uint8_t remaining_energy; 
}buff_t; 

//ÉËº¦×´Ì¬
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//ÊµÊ±Éä»÷ĞÅÏ¢
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;//µ¯ÍèÉäËÙ(µ¥Î»:Hz)
 float initial_speed;//µ¯Íè³õËÙ¶È(µ¥Î»:m/s)
}ext_shoot_data_t;

//ÔÊĞíÉä»÷ĞÅÏ¢
typedef __packed struct 
{ 
  uint16_t projectile_allowance_17mm; 		//»úÆ÷ÈË×ÔÉíÓµÓĞµÄ 17mm µ¯ÍèÔÊĞí·¢µ¯Á¿ 
 uint16_t projectile_allowance_42mm;  		//42mm µ¯ÍèÔÊĞí·¢µ¯Á¿
 uint16_t remaining_gold_coin; 			  		//Ê£Óà½ğ±ÒÊıÁ¿ 
 uint16_t projectile_allowance_fortress; //±¤ÀİÔöÒæµãÌá¹©µÄ´¢±¸ 17mm µ¯ÍèÔÊĞí·¢µ¯Á¿£» ¸ÃÖµÓë»úÆ÷ÈËÊÇ·ñÊµ¼ÊÕ¼Áì±¤ÀİÎŞ¹Ø 
}projectile_allowance_t; 

//»úÆ÷ÈË RFID ×´Ì¬
typedef __packed struct
{
	uint32_t My_Base_GainPoint:1;											//¼º·½»ùµØÔöÒæµã          	  														0
	uint32_t My_Central_Tablelands_GainPoint:1;			  //¼º·½ÖĞÑë¸ßµØÔöÒæµã       															1
	uint32_t Enemy_Central_Tablelands_GainPoint:1;	  //¶Ô·½ÖĞÑë¸ßµØÔöÒæµã       															2
	uint32_t My_Trapezoidal_elevation_GainPoint:1;		//ÌİĞÎ¸ßµØÔöÒæµã																					3
	uint32_t Enemy_Trapezoidal_elevation_GainPoint:1;	//¶Ô·½ÌİĞÎ¸ßµØÔöÒæµã 																		4
	uint32_t My_FlyingSlope_GainPoint_1:1;						//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨·ÉÆÂ£©£¨¿¿½ü¼º·½Ò»²à·ÉÆÂÇ°£©					5
	uint32_t My_FlyingSlope_GainPoint_2:1;						//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨·ÉÆÂ£©£¨¿¿½ü¼º·½Ò»²à·ÉÆÂºó£©					6
	uint32_t Enemy_FlyingSlope_GainPoint_1:1;					//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨·ÉÆÂ£©£¨¿¿½ü¶Ô·½Ò»²à·ÉÆÂÇ°£©					7
	uint32_t Enemy_FlyingSlope_GainPoint_2:1;					//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨·ÉÆÂ£©£¨¿¿½ü¶Ô·½Ò»²à·ÉÆÂºó£©					8
	uint32_t My_Crossing_Step_GainPoint_1:1;					//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨ÖĞÑë¸ßµØÏÂ·½£© 											9
	uint32_t My_Crossing_Step_GainPoint_2:1;					//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨ÖĞÑë¸ßµØÉÏ·½£©												10
	uint32_t Enemy_Crossing_Step_GainPoint_1:1;				//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨ÖĞÑë¸ßµØÏÂ·½£©												11
	uint32_t Enemy_Crossing_Step_GainPoint_2:1;				//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨ÖĞÑë¸ßµØÉÏ·½£©												12
	uint32_t My_Crossing_Highway_GainPoint_1:1;				//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨¹«Â·ÏÂ·½£©													13
	uint32_t My_Crossing_Highway_GainPoint_2:1;				//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨¹«Â·ÉÏ·½£© 													14
	uint32_t Enemy_Crossing_Highway_GainPoint_1:1;		//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨¹«Â·ÏÂ·½£©													15
	uint32_t Enemy_Crossing_Highway_GainPoint_2:1;		//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨¹«Â·ÉÏ·½£© 													16
	uint32_t My_Fort_GainPoint:1;											//¼º·½±¤ÀİÔöÒæµã																					17
	uint32_t My_Outpost_GainPoint:1;									//¼º·½Ç°ÉÚÕ¾ÔöÒæµã																				18
	uint32_t My_Recharge_Area:1;											//¼º·½Óë¶Ò»»Çø²»ÖØµşµÄ²¹¸øÇø/RMUL²¹¸øÇø										19
	uint32_t My_Recharge_Area_:1;											//¼º·½Óë¶Ò»»ÇøÖØµşµÄ²¹¸øÇø																20
	uint32_t My_Assemble_Area :1;											//¼º·½×°ÅäÔöÒæµã																					21
	uint32_t Enemy_Assemble_Area :1; 										//¶Ô·½×°ÅäÔöÒæµã																					22
	uint32_t RMUL_Centrol_GainPoint:1;								//ÖĞĞÄÔöÒæµã(½ö RMUL ÊÊÓÃ)																23
	uint32_t Enemy_Fort_GainPoint:1;									//¶Ô·½±¤ÀİÔöÒæµã																					24
	uint32_t Enemy_Outpost_GainPoint:1;								//¶Ô·½Ç°ÉÚÕ¾ÔöÒæµã																				25
	uint32_t My_Tunnle_GainPoint_1:	1;								//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¼º·½Ò»²à¹«Â·ÇøÏÂ·½£©     	26
	uint32_t My_Tunnle_GainPoint_2:	1;								//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¼º·½Ò»²à¹«Â·ÇøÉÏ·½£©    	27
	uint32_t My_Tunnle_GainPoint_3:	1;								//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¼º·½ÌİĞÎ¸ßµØ½ÏµÍ´¦£©			28
	uint32_t My_Tunnle_GainPoint_4:	1;								//¼º·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¼º·½ÌİĞÎ¸ßµØ½Ï¸ß´¦£©			29
	uint32_t Enemy_Tunnle_GainPoint_1: 1;							//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¶Ô·½Ò»²à¹«Â·ÇøÏÂ·½£© 			30
	uint32_t Enemy_Tunnle_GainPoint_2: 1;							//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¶Ô·½Ò»²à¹«Â·ÇøÉÏ·½£© 			31
	uint32_t Enemy_Tunnle_GainPoint_3: 1;							//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¶Ô·½ÌİĞÎ¸ßµØ½ÏµÍ´¦£©
	uint32_t Enemy_Tunnle_GainPoint_4: 1;							//¶Ô·½µØĞÎ¿çÔ½ÔöÒæµã£¨ËíµÀ£©£¨¿¿½ü¶Ô·½ÌİĞÎ¸ßµØ½Ï¸ß´¦£©

} ext_rfid_status_t;

//·ÉïÚ»úÆ÷ÈË¿Í»§¶ËÖ¸ÁîÊı¾İ
typedef __packed struct
{
  uint8_t dart_launch_opening_status;  //µ±Ç°·ÉïÚ·¢ÉäÕ¾µÄ×´Ì¬ 1¹Ø±Õ 2ÔÙ¶¯ 0¿ªÆô
  uint8_t reserved;  
  uint16_t target_change_time;  
  uint16_t latest_launch_cmd_time; 
} ext_dart_client_cmd_t;

//¿Í»§¶Ë×Ô¶¨ÒåÊı¾İ:cmd_id:0x0301¡£ÄÚÈİ ID:0xD180
typedef __packed struct
{ 
	float data1;
	float data2;
	float data3; 
	uint8_t masks; 
} client_custom_data_t;

//³µÁ¾ÔÚ³¡µØµÄ×ø±ê
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
  float reserved;  
  float reserved_; 
}ground_robot_position_t; 

//»úÆ÷ÈËÒ×ÉËÇé¿ö
typedef __packed struct 
{ 
  uint8_t mark_progress;  
}radar_mark_data_t;



/*
bit 0-1£º
À×´ïÊÇ·ñÓµÓĞ´¥·¢Ë«±¶Ò×ÉËµÄ»ú»á£¬¿ª¾ÖÎª 0£¬ÊıÖµÎªÀ×´ïÓµÓĞ´¥
·¢Ë«±¶Ò×ÉËµÄ»ú»á£¬ÖÁ¶àÎª 2 
bit 2£º¶Ô·½ÊÇ·ñÕıÔÚ±»´¥·¢Ë«±¶Ò×ÉË 
 0£º¶Ô·½Î´±»´¥·¢Ë«±¶Ò×ÉË 
 1£º¶Ô·½ÕıÔÚ±»´¥·¢Ë«±¶Ò×ÉË 
bit 3-4£º¼º·½¼ÓÃÜµÈ¼¶£¨¼´¶Ô·½¸ÉÈÅ²¨ÄÑ¶ÈµÈ¼¶£©£¬¿ª¾ÖÎª 1£¬×î¸ßÎª 3 
bit 5£ºµ±Ç°ÊÇ·ñ¿ÉÒÔĞŞ¸ÄÃÜÔ¿£¬1 Îª¿ÉĞŞ¸Ä 
bit 6-7£º±£ÁôÎ» */
typedef __packed struct 
{ 
 uint8_t radar_info; 
} radar_info_t;


//ÉÚ±øµÄÕ½³¡Êı¾İ£¨¼ûÊÖ²á£©
typedef __packed struct 
{  
	uint32_t sentry_info; 
  uint16_t sentry_info_2; 
}sentry_info_t; 

//1 ½»»¥Êı¾İ½ÓÊÕĞÅÏ¢
typedef __packed struct
{
  uint16_t data_cmd_id; 
  uint16_t sender_id; 
  uint16_t receiver_id; 
//  uint8_t user_data[x]; //x×î´óÎª112
}ext_student_interactive_header_data_t;

//½»»¥Êı¾İ »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
	uint8_t data[6];
}robot_interactive_data_t;

//2 ¿Í»§¶ËÉ¾³ıÍ¼ĞÎ »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; 
} ext_client_custom_graphic_delete_t;

//Í¼ĞÎÊı¾İ
typedef __packed struct
{ 
uint8_t figure_name[3]; 
uint32_t operate_tpye:3; 
uint32_t figure_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t details_a:9; //start angle
uint32_t details_b:9; //end angle
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t details_c:10; //radius
uint32_t details_d:11; //endx
uint32_t details_e:11;//endy
} graphic_data_struct_t;

//¿Í»§¶Ë»æÖÆÒ»¸öÍ¼ĞÎ »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

//¿Í»§¶Ë»æÖÆÁ½¸öÍ¼ĞÎ »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//¿Í»§¶Ë»æÖÆÎå¸öÍ¼ĞÎ »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//¿Í»§¶Ë»æÖÆÆß¸öÍ¼ĞÎ »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//¿Í»§¶Ë»æÖÆ×Ö·û »úÆ÷ÈË¼äÍ¨ĞÅ
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
uint8_t data[30];
} ext_client_custom_character_t;

//×Ô¶¨Òå ½»»¥Êı¾İ½ÓÊÕĞÅÏ¢
typedef __packed struct
{
uint8_t data[30];
} robot_custon_controller_data_t;

//Ğ¡µØÍ¼ÏÂ·¢ĞÅÏ¢±êÊ¶:0x0303
typedef __packed struct
{
	float target_position_x; 
	float target_position_y; 
	uint8_t cmd_keyboard; //ÔÆÌ¨ÊÖ°´ÏÂµÄ¼üÅÌ°´¼üÍ¨ÓÃ¼üÖµ
	uint8_t target_robot_id; //¶Ô·½»úÆ÷ÈËID
	uint16_t cmd_source; //ĞÅÏ¢À´Ô´ID 
} ext_robot_command_t;

//¿Í»§¶Ë½ÓÊÕĞÅÏ¢
typedef __packed struct
{
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;

//À×´ï¿ÉÍ¨¹ı³£¹æÁ´Â·Ïò¼º·½ËùÓĞÑ¡ÊÖ¶Ë·¢ËÍ¶Ô·½»úÆ÷ÈËµÄ×ø±êÊı¾İ 0x305
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


//ÉÚ±ø»úÆ÷ÈË»ò°ë×Ô¶¯¿ØÖÆ·½Ê½µÄ»úÆ÷ÈË¿ÉÍ¨¹ı³£¹æÁ´Â·Ïò¶ÔÓ¦µÄ²Ù×÷ÊÖÑ¡ÊÖ¶Ë·¢ËÍÂ·¾¶×ø±êÊı¾İ£¬¸ÃÂ·¾¶»áÔÚĞ¡µØÍ¼ÉÏÏÔÊ¾0x0307
typedef __packed struct  
{ 
uint8_t intention; //1£ºµ½Ä¿±êµã¹¥»÷ 2£ºµ½Ä¿±êµã·ÀÊØ 3£ºÒÆ¶¯µ½Ä¿±êµã 
uint16_t start_position_x; 
uint16_t start_position_y; 
int8_t delta_x[49]; 
int8_t delta_y[49]; 
uint16_t sender_id; 
}map_data_t;

//Í¼´«Ò£¿ØĞÅÏ¢±êÊ¶
typedef __packed struct
{
	int16_t mouse_x; //Êó±êxÖáÒÆ¶¯ËÙ¶È,¸º±êÊ¶Ïò×óÒÆ¶¯
	int16_t mouse_y; //Êó±êyÖáÒÆ¶¯ËÙ¶È,¸º±êÊ¶ÏòÏÂÒÆ¶¯
	int16_t mouse_z; //Êó±ê¹öÂÖÒÆ¶¯ËÙ¶È,¸º±êÊ¶Ïòºó¹ö¶¯
	int8_t left_button_down; //Êó±ê×ó¼üÊÇ·ñ°´ÏÂ:0ÎªÎ´°´ÏÂ;1Îª°´ÏÂ
	int8_t right_button_down; //Êó±êÓÒ¼üÊÇ·ñ°´ÏÂ:0ÎªÎ´°´ÏÂ;1Îª°´ÏÂ
	uint16_t keyboard_value; //¼üÅÌ°´¼üĞÅÏ¢£¬Ã¿¸öbit¶ÔÓ¦Ò»¸ö°´¼ü:0ÎªÎ´°´ÏÂ;1Îª°´ÏÂ
	uint16_t reserved; 
}remote_control_t; 

//crc8 generator polynomial:G(x)=x8+x5+x4+1 

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef __packed struct 
{ 
	uint32_t 	Relive_Confirm  :1;
	uint32_t  Relive_At_Once  :1;
	uint32_t 	Num_Of_Exchange_Bullet  :11;
	uint32_t  Remote_Exchange_Bullet_Time  :4;
	uint32_t  Remote_Exchange_Blood_Time  :4;
	uint32_t	Posture_Change:2;
	uint32_t  Buff_Process:1;
	uint32_t sentry_cmd_Reserve  :7;
} sentry_cmd_t;


typedef __packed struct 
{ 
	uint8_t radar_cmd; //À×´ïÊÇ·ñÈ·ÈÏ´¥·¢Ë«±¶Ò×ÉË 
	uint8_t password_cmd; //1ĞŞ¸Ä¼º·½ 2ÆÆ½â·¢ËÍ
	uint8_t password_1; //byte2-7 ÎªÃÜÔ¿Öµ
	uint8_t password_2; 
	uint8_t password_3; 
	uint8_t password_4; 
	uint8_t password_5; 
	uint8_t password_6; 
} radar_cmd_t; 


/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{ 
		ext_game_status_t                     game_state;//±ÈÈü×´Ì¬Êı¾İ
		ext_game_result_t                     game_result;//±ÈÈü½á¹ûÊı¾İ
		ext_game_robot_HP_t                   game_robot_HP;//»úÆ÷ÈË´æ»îÊı¾İ
		ext_event_data_t                      event_data;//³¡µØÊÂ¼şÊı¾İ
	  ext_referee_warning_t                 referee_warning;//²ÃÅĞ¾¯¸æĞÅÏ¢
		ext_dart_info_t												dart_info;//·ÉïÚ·¢Éä
		ext_game_robot_status_t               game_robot_state;//±ÈÈü»úÆ÷ÈË×´Ì¬
		ext_power_heat_data_t                 power_heat_data;//ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾
		ext_game_robot_pos_t                  game_robot_pos;//»úÆ÷ÈËÎ»ÖÃ
		buff_t                            buff_musk;//»úÆ÷ÈËÔöÒæ
		ext_robot_hurt_t                      robot_hurt;//ÉËº¦×´Ì¬
		ext_shoot_data_t                      shoot_data;//ÊµÊ±Éä»÷ĞÅÏ¢
		projectile_allowance_t				  			Projectile_Allowance;//0x208       
		ext_rfid_status_t                     ext_rfid_status;//RFID×´Ì¬
		ext_dart_client_cmd_t									dart_client_cmd;//·ÉïÚ»úÆ÷ÈË¿Í»§¶ËÖ¸Áî
		client_custom_data_t                  client_custom_data;//¿Í»§¶Ë ¿Í»§¶Ë×Ô¶¨ÒåÊı¾İ  
		ground_robot_position_t								ground_robot_position;//³µÁ¾ÔÚ³¡µØµÄ×ø±ê   
		radar_mark_data_t											radar_mark_data;//»úÆ÷ÈËÒ×ÉËÇé¿ö  
		sentry_info_t													sentry_info;//ÉÚ±øµÄÕ½³¡Êı¾İ           
		ext_student_interactive_header_data_t student_interactive_header_data;//½»»¥Êı¾İ½ÓÊÕĞÅÏ¢
		robot_interactive_data_t              robot_interactive_data;//½»»¥Êı¾İ »úÆ÷ÈË¼äÍ¨ĞÅ
		graphic_data_struct_t            			graphic_data_struct;//¿Í»§¶Ë×Ô¶¨ÒåÍ¼ĞÎ
		robot_custon_controller_data_t      	robot_custon_controller_data;//×Ô¶¨Òå¿ØÖÆÆ÷Êı¾İ
		ext_robot_command_t										robot_command;//Ğ¡µØÍ¼ÏÂ·¢ĞÅÏ¢        
		map_robot_data_t											map_robot_data;////À×´ï·¢ËÍ¶Ô·½»úÆ÷ÈËµÄ×ø±êÊı¾İ
		remote_control_t											remote_control;//¼üÊóÒ£¿ØÊı¾İ
} receive_judge_t;                                                           
          

//typedef __packed struct                                                      
//{
//  uint16_t data_cmd_id; /*×ÓÄÚÈİID ĞèÎª¿ª·ÅµÄ×ÓÄÚÈİID*/
//  uint16_t sender_id; /*·¢ËÍÕßID ĞèÓë×ÔÉíIDÆ¥Åä,ID±àºÅÏê¼û¸½Â¼ */
//  uint16_t receiver_id; /*½ÓÊÕÕßID ½öÏŞ¼º·½Í¨ĞÅ ĞèÎª¹æÔòÔÊĞíµÄ¶à»úÍ¨Ñ¶½ÓÊÕÕß Èô½ÓÊÕÕßÎªÑ¡ÊÖ¶Ë£¬Ôò½ö¿É·¢ËÍÖÁ·¢ËÍÕß¶ÔÓ¦µÄÑ¡ÊÖ¶Ë ID±àºÅÏê¼û¸½Â¼*/
//}id_data_t;

//0x0308
typedef __packed struct 
{ 
uint16_t sender_id; //ĞèÒªĞ£Ñé·¢ËÍÕßµÄ ID ÕıÈ·ĞÔ 
uint16_t receiver_id; //ĞèÒªĞ£Ñé½ÓÊÕÕßµÄ ID ÕıÈ·ĞÔ£¬½öÖ§³Ö·¢ËÍ¼º·½Ñ¡ÊÖ¶Ë 
uint8_t user_data[30]; //ÒÔ utf-16 ¸ñÊ½±àÂë·¢ËÍ£¬Ö§³ÖÏÔÊ¾ÖĞÎÄ¡£±àÂë·¢ËÍÊ±Çë×¢ÒâÊı¾İµÄ´óĞ¡¶ËÎÊÌâ 
} custom_info_t; 

typedef enum
{
  unkown = 0,
  blue = 1,
  red  = 2,
} robot_color_e;

extern sentry_cmd_t  sentry_cmd;
extern receive_judge_t judge_rece_mesg; 
extern robot_color_e robot_color;

extern int Robot_Remain_HP;
extern int Robot_Max_HP;
extern uint8_t  USART5_Tx_Buf[150];
//extern uint8_t  ddata[120];

void USART5_DMA_R_T_JUDGE_Init(void);
void judgement_data_handle(uint8_t *p_frame,uint16_t	rec_len);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,uint16_t ndtr);

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned char get_crc8(unsigned char* data, unsigned int length);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);  //´®¿Ú·¢ËÍĞ­Òé
void data_upload_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);
void Sentry_Cmd_Send(void);
void Send_bullet_remaining_num(void);
//
#endif /*_JUDGE_SYSTEM_H*/

