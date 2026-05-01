//#include "UI.h"






//u8  draw_cnt=0;
//u16 draw_data_ID=0x0101;
//u16 data_ID=0xD180;
//u16 client_custom_ID=0;

//#define WIDTH    3
//uint8_t  pdata[19];
//uint8_t  ddata[62];
//uint8_t  dddata[62];


//UI_t UI=UI_DEFAULT;



///*创建图形对象*/




////pitch角度显示 + 方框位置 + 摩擦轮状态提示 + 摩擦轮电机转速显示
////interaction_figure_t _0001 = FLOAT_NUM(ADD,0,1,0,1715,470,0,15,WIDTH,1,UI_GREEN);
//interaction_figure_t _0001 = FLOAT_NUM(ADD,0,1,0,1715,470,0,20,WIDTH,1,UI_GREEN);
//interaction_figure_t _0002 = RECTANGLE(ADD,0,3,4,1700,1808,700,817,2,1,UI_CYAN);//相对位置矩形边框
//interaction_figure_t _0003 = CIRCLE(ADD,0,1,9,1730,400,10,20,1,2);
//interaction_figure_t _0004 = CIRCLE(ADD,0,2,0,1790,400,10,20,1,2);
//interaction_figure_t _0007 = LINE(ADD,0,3,5,1754,1754,758,758,2,1,UI_PURPLE);//相对位置直线
//interaction_figure_t _0008 = CHARACTER(ADD,0,3,6,150,700,20,14,2,1,UI_CYAN);//摩擦轮转速字符显示friction_speed
//interaction_figure_t _0009 = FLOAT_NUM(ADD,0,3,7,250,650,0,20,2,1,UI_ORANGE);//摩擦轮转速数字显示


//interaction_figure_t _00010 = CHARACTER(ADD,0,3,8,0,0,0,0,0,0,0);//空
//interaction_figure_t _00011 = CHARACTER(ADD,0,3,9,0,0,0,0,0,0,0);//空
//interaction_figure_t _00012 = CHARACTER(ADD,0,4,0,0,0,0,0,0,0,0);//空
//interaction_figure_t _00013 = CHARACTER(ADD,0,4,1,0,0,0,0,0,0,0);//空



////静态字符
//interaction_figure_t _0005 = CHARACTER(ADD,0,0,7,1600,470,20,6,2,1,UI_WHITE);//pitch
//interaction_figure_t _0006 = CHARACTER(ADD,0,3,2,540,137,20,4,2,1,UI_YELLOW);//cap



////小陀螺图形提示
//interaction_figure_t _000003 = CIRCLE(ADD,0,2,1,1760,580,50,1,1,UI_ORANGE);
//interaction_figure_t _000004 = CIRCLE(ADD,0,2,2,1730,610,10,7,1,UI_ORANGE);
//interaction_figure_t _000005 = CIRCLE(ADD,0,2,3,1790,610,10,7,1,UI_ORANGE);
//interaction_figure_t _000006 = CIRCLE(ADD,0,2,4,1730,550,10,7,1,UI_ORANGE);
//interaction_figure_t _000007 = CIRCLE(ADD,0,2,5,1790,550,10,7,1,UI_ORANGE);
//interaction_figure_t _000001 = RECTANGLE(ADD,0,3,1,583,1208,85,135,2,1,0); //电容电量框
//interaction_figure_t _000002 = LINE(ADD,0,3,3,600,600,110,110,2,1,UI_PINK);//电容电量

////吊射模式提示
//interaction_figure_t _00001 = LINE(ADD,0,1,2,920,958,572,572,2,1,UI_CYAN);
//interaction_figure_t _00002 = CIRCLE(ADD,0,1,3,948,574,2,2,1,2);
//interaction_figure_t _00003 = LINE(ADD,0,1,4,940,940,590,550,2,1,UI_CYAN);

//interaction_figure_t _00004 = RECTANGLE(ADD,0,1,5,583,1148,320,710,2,1,7); //自瞄框
//interaction_figure_t _00005 = CIRCLE(ADD,0,1,6,150,620,5,2,1,4);
//interaction_figure_t _00006 = FLOAT_NUM(ADD,0,1,7,1458,654,0,40,5,9,UI_ORANGE);  // LINE(ADD,0,1,7,135,165,610,610,2,1,5);
//interaction_figure_t _00007 = LINE(ADD,0,1,8,150,150,610,530,2,1,5);


////瞄准线提示
//interaction_figure_t _0000001 = CIRCLE(ADD,0,0,0,949,579,2,4,1,1);//点
//interaction_figure_t _0000002 = CIRCLE(ADD,0,0,1,949,440,4,6,1,UI_PINK);//点
//interaction_figure_t _0000003 = LINE(ADD,0,0,2,765,1109,520,520,1,1,2);//空
//interaction_figure_t _0000004 = LINE(ADD,0,0,3,829,1069,480,480,1,1,6);//空
//interaction_figure_t _0000005 = LINE(ADD,0,0,4,869,1029,440,440,2,1,4);//空
//interaction_figure_t _0000006 = LINE(ADD,0,0,5,909,989,537,537,1,1,2);//空
//interaction_figure_t _0000007 = LINE(ADD,0,0,6,949,949,280,558,2,1,2);//竖线

////过洞提示
////////////////////////////////////  -17°  ///////////////////////////////////////////////////
////interaction_figure_t _00000001 = LINE(ADD,0,4,0,68,668,10,645,4,1,2);
////interaction_figure_t _00000002 = LINE(ADD,0,4,1,1848,1248,10,645,4,1,2);

////////////////////////////////////    -10°    ///////////////////////////////////////////////////
//interaction_figure_t _00000001 = LINE(ADD,0,4,0,503,763,74,405,4,1,2);
//interaction_figure_t _00000002 = LINE(ADD,0,4,1,1503,1173,84,425,4,1,2);


////瞄准线提示
//interaction_figure_t _00000003 = LINE(ADD,0,4,2,869,1029,400,400,2,1,4);
//interaction_figure_t _00000004 = LINE(ADD,0,4,3,869,1029,360,360,2,1,2);
//interaction_figure_t _00000005 = LINE(ADD,0,4,4,869,1029,305,305,2,1,6);
//interaction_figure_t _00000006 ;//= LINE(ADD,0,0,6,958,858,220,258,2,1,2);
//interaction_figure_t _00000007 ;//= LINE(ADD,0,0,6,958,858,220,258,2,1,2);

//interaction_figure_t SB = CHARACTER(ADD,9,9,9,200,200,20,2,4,1,4);

///*创建 组合图形对象*/
//interaction_figure_4_t A;
//interaction_figure_4_t AA;
//interaction_figure_4_t AB;
//interaction_figure_4_t BB;

//interaction_figure_4_t TEST;

///*创建 字符对象*/
//client_custom_character_t B;uint8_t dataB[]="Pitch-";
//client_custom_character_t C;uint8_t dataC[]="Cap:";
//client_custom_character_t D;uint8_t dataD[]="friction_speed";
//client_custom_character_t friction_speed;
//client_custom_character_t TRAP; uint8_t dataTRAP[]="POKE_TRAP";





///*UI刷新主函数*/
//void Client_Send_Handle()
//{	
//  UI.id=judge_rece_mesg.game_robot_state.robot_id;
//  switch(UI.id)
//    {
//    case 1:
//      client_custom_ID=0x0101;
//      break;
//    case 3:
//      client_custom_ID=0x0103;
//      break;
//    case 4:
//      client_custom_ID=0x0104;
//      break;
//    case 5:
//      client_custom_ID=0x0105;
//      break;
//    case 101:
//      client_custom_ID=0x0165;
//      break;
//    case 103:
//      client_custom_ID=0x0167;
//      break;
//    case 104:
//      client_custom_ID=0x0168;
//      break;
//    case 105:
//      client_custom_ID=0x0169;
//      break;
//    }
//	
//	switch(UI.cnt)
//		{
//		case 1:
//		{
//			UI.ADD_7Graph(A,_0001,_0002,_0003,_0004,_0007,_0006,_0009);
//		}break;
//        
//        case 2:
//        {
//            UI.ADD_7Graph(AA,_000001,_000002,_000003,_000004,_000005,_000006,_000007);
//        }break;
//        
//        case 3:
//        {
//			  UI.ADD_7Graph(AB,_00001,_00002,_00003,_00004,_00005,_00006,_00007);
//            
//        }break;
//        
//        case 4:
//        {
//            UI.ADD_7Graph(BB,_0000001,_0000007,_0000003,_0000004,_0000005,_0000006,_0000002);
//			
//        }break;
//        
//		case 5:
//		{
//			UI.ADD_Char(B,_0005,dataB,6);//pitch
//		}break;
//        
//        case 6:
//        {
//            UI.ADD_Char(D,_0008,dataD,14);//friction_speed
//        }break;
//		
//		case 7:
//        {
//			UI.ADD_7Graph(TEST,_00000001,_00000002,_00000003,_00000004,_00000005,_00000006,_00000007);
//        }break;
//        
//		case 8:/*动态显示*/
//		{
//			UI.MODIFY_7Graph_0(A,_0001,_0002,_0003,_0004,_0007,_0006,_0009);
//		}break;
//        
//		case 9:
//		{
//			UI.MODIFY_7Graph_1(AA,_000003,_000004,_000005,_000006,_000007,_000001,_000002);
//		}break;
//        
//        case 10:
//        {
////            UI.MODIFY_7Graph_2(AB,_00001,_00002,_00003,_00004,_00005,_00006,_00007);
//			  UI.MODIFY_7Graph_3(BB,_0000001,_0000007,_0000003,_0000004,_0000005,_0000006,_0000002);
//        }break;
//        
//        case 11:
//        {
//            UI.MODIFY_7Graph_2(AB,_00001,_00002,_00003,_00004,_00005,_00006,_00007);
//        }break;
//		
//		default:
//     break;
//    }
//		
//	
//	u8 static CNT_TIME=0;
//	if(UI.cnt<8)
//	{
//		CNT_TIME++;
//		if(CNT_TIME%2==1)
//			UI.cnt++;
//	}
//	else
//		UI.cnt++;
//  if(UI.cnt>11)/*在需要刷新的图层刷新*/
//     UI.cnt=8;

//}



////建议范围 x（960+-120*2.75） y（540+-280）
//typedef struct
//{
//  int16_t x;
//  int16_t y;
//} point;

//point rotate_point(int16_t x,int16_t y,float angle)
//{
//  point result;
//  float rad_angle=angle*ANGLE_TO_RAD;
//  result.x=(int)(x*cos(rad_angle)-y*sin(rad_angle));
//  result.y=(int)(x*sin(rad_angle)+y*cos(rad_angle));
//  return result;
//}



//void ADD_Character(client_custom_character_t _0,interaction_figure_t __0,uint8_t *data0,uint8_t size0)
//{
//		robot_interaction_data_t UI_data;
// 
//		UI_data.id_data.data_cmd_id=0x0110;
//		UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//        UI_data.id_data.receiver_id=client_custom_ID; //客户端id

//		memcpy((uint8_t *)ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//	
//		_0.interaction_figure=__0;
//		memcpy(_0.data,data0,size0);
//		*(client_custom_character_t*)(&ddata[6])=_0;

//	
//		memcpy((uint8_t *)(ddata+6+sizeof(client_custom_character_t)),(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//	
//		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,ddata,2*sizeof(UI_data.id_data)+sizeof(client_custom_character_t),DN_REG_ID,tx_buf);
//}

//void ADD_7_Graph(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
//{
//      robot_interaction_data_t UI_data;
//	
//	  UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
//      UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//      UI_data.id_data.receiver_id=client_custom_ID; //客户端id

//			_7.interaction_figure[0]=_0;
//			_7.interaction_figure[1]=_1;
//			_7.interaction_figure[2]=_2;
//			_7.interaction_figure[3]=_3;
//			_7.interaction_figure[4]=_4;
//			_7.interaction_figure[5]=_5;
//			_7.interaction_figure[6]=_6;

//			memcpy(ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//		  *(interaction_figure_4_t*)(&ddata[6])=_7;
////			memcpy(dddata+sizeof(UI_data.id_data),(interaction_figure_4_t *)&_7,sizeof(interaction_figure_4_t));
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,ddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
//}

//void MODIFY_2_Character_Num(client_custom_character_t _0,interaction_figure_t __0,float data0,client_custom_character_t _1,interaction_figure_t __1,float data1)
//{
//		robot_interaction_data_t UI_data;

//		UI_data.id_data.data_cmd_id=0x0110;
//		UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
//	
//    memcpy(ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));	
//	
//		_0.interaction_figure=__0;
//		_0.interaction_figure=__1;
//	
//		_0.interaction_figure.operate_tpye=2;
//		sprintf((char *)_0.data,"%f",data0);
//		*(client_custom_character_t*)(&ddata[6])=_0;
//	
//		_1.interaction_figure.operate_tpye=2;
//		sprintf((char *)_1.data,"%f",data1);
//		*(client_custom_character_t*)(&ddata[6+sizeof(client_custom_character_t)])=_1;
////		memcpy((uint8_t *)&UI_data.user_data,(uint8_t *)&_0,sizeof(client_custom_character_t));
////		memcpy((uint8_t *)&UI_data.user_data+sizeof(client_custom_character_t),(uint8_t *)&_1,sizeof(client_custom_character_t));
//	 data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,ddata ,sizeof(UI_data.id_data)+2*sizeof(client_custom_character_t),DN_REG_ID,tx_buf);
//}

//void MODIFY_7_Graph_DIY(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
//{
//		robot_interaction_data_t UI_data;
//	
//		UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
//    UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
//	
//		_7.interaction_figure[0]=_0;
//		_7.interaction_figure[1]=_1;
//		_7.interaction_figure[2]=_2;
//		_7.interaction_figure[3]=_3;
//		_7.interaction_figure[4]=_4;
//		_7.interaction_figure[5]=_5;
//		_7.interaction_figure[6]=_6;
//	
//		_7.interaction_figure[0].operate_tpye=MODIFY;
//		_7.interaction_figure[1].operate_tpye=MODIFY;
//		_7.interaction_figure[2].operate_tpye=MODIFY;
//		_7.interaction_figure[3].operate_tpye=MODIFY;
//		_7.interaction_figure[4].operate_tpye=MODIFY;
//		_7.interaction_figure[5].operate_tpye=MODIFY;
//		_7.interaction_figure[6].operate_tpye=MODIFY;
////图形1pitch角度显示
//			int32_t  pitch_angle=(gimbal_gyro.pitch_angle*1000.0f);
//			_7.interaction_figure[0].details_c=pitch_angle;
//		    _7.interaction_figure[0].details_d=pitch_angle>>10;
//			_7.interaction_figure[0].details_e=pitch_angle>>21;
//		 
////图形2方框云台相对位置	
//            _7.interaction_figure[1].width = 2;
//		
//        
////图形3摩擦轮状态提示			
//		    if(_42mm_shoot.friction_state == FRICTION_ON)
//            {
//                _7.interaction_figure[2].width = 20;
//                _7.interaction_figure[3].width = 20;
//                
//            }
//            else
//            {
//                _7.interaction_figure[2].width = 0;
//                _7.interaction_figure[3].width = 0;
//            }
//             
////图形4相对位置直线			
//            
//                _7.interaction_figure[4].width = 4;
////                _7.interaction_figure[4].details_d = 1754-sin(Chassis_angle.yaw_angle_0_2pi)*80;
////                _7.interaction_figure[4].details_e = 758+cos(Chassis_angle.yaw_angle_0_2pi)*80;
//				_7.interaction_figure[4].details_d = 1754-sin(Chassis_Dir)*80;
//                _7.interaction_figure[4].details_e = 758+cos(Chassis_Dir)*80;
////图形5摩擦轮电机转速显示
////               uint32_t  friction_speed = (frictionSpeed_42*1000.0f);
//              uint32_t  friction_speed = (friction_speed_ref*1000.0f);
//			_7.interaction_figure[6].details_c=friction_speed;
//		    _7.interaction_figure[6].details_d=friction_speed>>10;
//			_7.interaction_figure[6].details_e=friction_speed>>21;
//            

//           
//            
//			
//			memcpy((uint8_t *)ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//			*(interaction_figure_4_t*)(&ddata[6])=_7;
//            data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
//}

//void MODIFY_7_Graph_DIY1(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
//{
//		robot_interaction_data_t UI_data;
//	
//		UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
//        UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//        UI_data.id_data.receiver_id=client_custom_ID; //客户端id
//	
//		_7.interaction_figure[0]=_0;
//		_7.interaction_figure[1]=_1;
//		_7.interaction_figure[2]=_2;
//		_7.interaction_figure[3]=_3;
//		_7.interaction_figure[4]=_4;
//		_7.interaction_figure[5]=_5;
//		_7.interaction_figure[6]=_6;
//	
//		_7.interaction_figure[0].operate_tpye=MODIFY;
//		_7.interaction_figure[1].operate_tpye=MODIFY;
//		_7.interaction_figure[2].operate_tpye=MODIFY;
//		_7.interaction_figure[3].operate_tpye=MODIFY;
//		_7.interaction_figure[4].operate_tpye=MODIFY;
//		_7.interaction_figure[5].operate_tpye=MODIFY;
//		_7.interaction_figure[6].operate_tpye=MODIFY;
//		
////图形1小陀螺提示
//        if(chassis.ctrl_mode == CHASSIS_ROTATE  || chassis.ctrl_mode == CHASSIS_REVERSE_ROTATE)
//        {
//            _7.interaction_figure[0].width = 1;
//            _7.interaction_figure[1].width = 7;
//            _7.interaction_figure[2].width = 7;
//            _7.interaction_figure[3].width = 7;
//            _7.interaction_figure[4].width = 7;
//        }
//        else
//        {
//            _7.interaction_figure[0].width = 0;
//            _7.interaction_figure[1].width = 0;
//            _7.interaction_figure[2].width = 0;
//            _7.interaction_figure[3].width = 0;
//            _7.interaction_figure[4].width = 0;
//            
//        }
//        
//			
//            
//            
//		 
////图形2电容
//		  if(can_capacitance_message.cap_voltage_filte>=0.0f&&can_capacitance_message.cap_voltage_filte<=8.0f)
//        {
//          _7.interaction_figure[6].details_d = 600;
//          _7.interaction_figure[6].color=UI_PINK;
//        }
//         else if(can_capacitance_message.cap_voltage_filte>8.0f)
//        {
//          _7.interaction_figure[6].details_d=600+((can_capacitance_message.cap_voltage_filte-8.0))*28.8f;
//          _7.interaction_figure[6].color=UI_GREEN;
//        }
//          _7.interaction_figure[6].width=30;
//        
////图形3电容方框
//        
//        
//        

//        
//        
///*第4个图形 yaw*/
////			float yaw__180_180;
////			float yaw_0_360	=fmod(yaw_Encoder.ecd_angle*1,360);	
////			if(yaw_0_360<0){yaw_0_360+=360;}
////				if(yaw_0_360>=180)/*将0-2PI转换到-PI-PI范围内*/
////					{yaw__180_180=yaw_0_360-360;}
////				else
////					{yaw__180_180=yaw_0_360;}
//		
////		uint32_t bullet_supply;
////		bullet_supply = bullet_supply_num*1000.0;
////		_7.interaction_figure[3].details_a = /*bullet_supply*/gimbal_gyro.yaw_Angle+15;		

////		if(yaw_0_360+345>360)
////			yaw_0_360=yaw_0_360-360;
////		_7.interaction_figure[3].details_b = /*bullet_supply;*/gimbal_gyro.yaw_Angle+345;
//		
////		_7.interaction_figure[3].details_c = bullet_supply;
////		_7.interaction_figure[3].details_d = bullet_supply>>10;
////		_7.interaction_figure[3].details_d = bullet_supply>>21;
//		
///*第5个图形 big buff*/			
////		if(gimbal_data.ctrl_mode==GIMBAL_AUTO_BIG_BUFF)
////		 {
////			  _7.interaction_figure[4].width=10;
//// 
////				_7.interaction_figure[4].details_a=UI.circle_360;
////				_7.interaction_figure[4].details_b=UI.circle_360-50;						
////		 }
////		else
////		 {
////			  _7.interaction_figure[4].width=5;
////				_7.interaction_figure[4].details_a=0;
////				_7.interaction_figure[4].details_b=360;
////		 }
//		 
///*第6个图形 small buf*/			
//		if(chassis.chassis_speed_mode == FLY_SLPOE)
//		 {
//				_7.interaction_figure[5].width=10;
//				_7.interaction_figure[5].details_a=UI.circle_360;
//				_7.interaction_figure[5].details_b=UI.circle_360-50;						
//		 }
//		else
//		 {
//			  _7.interaction_figure[5].width=5;
//				_7.interaction_figure[5].details_a=0;
//				_7.interaction_figure[5].details_b=360;
//		 }

//			
//			memcpy((uint8_t *)ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//			*(interaction_figure_4_t*)(&ddata[6])=_7;
//            data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
//}
//void MODIFY_7_Graph_DIY2(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
//{
//    robot_interaction_data_t UI_data;
//	
//	UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
//    UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
//	
//		_7.interaction_figure[0]=_0;
//		_7.interaction_figure[1]=_1;
//		_7.interaction_figure[2]=_2;
//		_7.interaction_figure[3]=_3;
//		_7.interaction_figure[4]=_4;
//		_7.interaction_figure[5]=_5;
//		_7.interaction_figure[6]=_6;
//	
//		_7.interaction_figure[0].operate_tpye=MODIFY;
//		_7.interaction_figure[1].operate_tpye=MODIFY;
//		_7.interaction_figure[2].operate_tpye=MODIFY;
//		_7.interaction_figure[3].operate_tpye=MODIFY;
//		_7.interaction_figure[4].operate_tpye=MODIFY;
//		_7.interaction_figure[5].operate_tpye=MODIFY;
//		_7.interaction_figure[6].operate_tpye=MODIFY;
////吊射模式提示
//        
//        if(gimbal_data.ctrl_mode == GIMBAL_SNIPE)
//        {
//            _7.interaction_figure[0].width = 2;
//            _7.interaction_figure[1].width = 2;
//            _7.interaction_figure[2].width = 2;
////            _7.interaction_figure[3].width = 2;
//            _7.interaction_figure[4].width = 2;
////            _7.interaction_figure[5].width = 2;
//            _7.interaction_figure[6].width = 2;
//        
//        }
//        else
//        {
//            _7.interaction_figure[0].width = 0;
//            _7.interaction_figure[1].width = 0;
//            _7.interaction_figure[2].width = 0;
////            _7.interaction_figure[3].width = 0;
//            _7.interaction_figure[4].width = 0;
////           _7.interaction_figure[5].width = 0;
//            _7.interaction_figure[6].width = 0;
//        
//        }
//		
//		if(Peripheral_State.Equipment_Visual_Equipment_Auto_Aim.Link_State == CONNECTED)
//		{
//			if(My_Auto_Shoot.Auto_Aim.Flag_Get_Target == 1)
//			{
//				_7.interaction_figure[3].color = UI_GREEN;
//			}
//			else
//			{
//				_7.interaction_figure[3].color = UI_WHITE;
//			}
//		}
//		else
//		{
//			_7.interaction_figure[3].color = UI_BLACK;
//		}
//		
//		uint32_t bullet_supply;
//		bullet_supply = 1*1000.0;
//		_7.interaction_figure[5].details_c = bullet_supply;
//		_7.interaction_figure[5].details_d = bullet_supply>>10;
//		_7.interaction_figure[5].details_e = bullet_supply>>21;		
//		
//            memcpy((uint8_t *)ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//			*(interaction_figure_4_t*)(&ddata[6])=_7;
//            data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
//        

//}
//void MODIFY_7_Graph_DIY3(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
//{
//    robot_interaction_data_t UI_data;
//	
//	UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
//    UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
//    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
//	
//		_7.interaction_figure[0]=_0;
//		_7.interaction_figure[1]=_1;
//		_7.interaction_figure[2]=_2;
//		_7.interaction_figure[3]=_3;
//		_7.interaction_figure[4]=_4;
//		_7.interaction_figure[5]=_5;
//		_7.interaction_figure[6]=_6;
//	
//		_7.interaction_figure[0].operate_tpye=MODIFY;
//		_7.interaction_figure[1].operate_tpye=MODIFY;
//		_7.interaction_figure[2].operate_tpye=MODIFY;
//		_7.interaction_figure[3].operate_tpye=MODIFY;
//		_7.interaction_figure[4].operate_tpye=MODIFY;
//		_7.interaction_figure[5].operate_tpye=MODIFY;
//		_7.interaction_figure[6].operate_tpye=MODIFY;
//		
//		if(DM_4310.ERR != NORMAL)
//			_7.interaction_figure[6].color = UI_WHITE;
////		else if(_42mm_shoot.poke_state == POKE_TRAP)
////			_7.interaction_figure[6].color = UI_GREEN;
////		else if(_42mm_shoot.poke_state != POKE_TRAP)
//		else
//			_7.interaction_figure[6].color = UI_PINK;
//		
//		

//		
//		
//		
//		
//		
//            memcpy((uint8_t *)ddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
//			*(interaction_figure_4_t*)(&ddata[6])=_7;
//            data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);







//}

//id_data_t send_to_aerial;
//void Send_bullet_remaining_num(void)
//{
//    
//    send_to_aerial.data_cmd_id = 0x0201;//按兵种标号向后顺序排 0x0202 0x0203 0x0204----0x02FF
//    send_to_aerial.sender_id = judge_rece_mesg.game_robot_state.robot_id;
//    if(judge_rece_mesg.game_robot_state.robot_id == 1)//红方一号
//    {
//        send_to_aerial.receiver_id = 6;
//    }
//    else if(judge_rece_mesg.game_robot_state.robot_id == 101)//蓝方一号
//    {
//        send_to_aerial.receiver_id = 106;
//    }
//    
//   
//    
//    
////    memcpy((uint8_t *)dddata,(uint8_t *)&send_to_aerial,sizeof(send_to_aerial));
////    dddata[6] = (uint8_t)judge_rece_mesg.Projectile_Allowance.bullet_remaining_num_42mm;
////    dddata[7] = (uint8_t)(judge_rece_mesg.Projectile_Allowance.bullet_remaining_num_42mm >> 8);
////    data_upload_handle(ROBOT_INTERACTIVE_DATA_ID,dddata,sizeof(send_to_aerial)+sizeof(judge_rece_mesg.Projectile_Allowance.bullet_remaining_num_42mm),DN_REG_ID,tx_buf);

//}



////id_data_t send_to_aerial;
////void Send_bullet_remaining_num(void)
////{
////    switch(judge_rece_mesg.game_robot_state.robot_id)
////	{
////		case 3:
////			send_to_aerial.data_cmd_id=0x0203;//按兵种标号向后顺序排 0x0202 0x0203 0x0204----0x02FF
////			send_to_aerial.receiver_id = 6;
////		break;
////		case 4:
////			send_to_aerial.data_cmd_id=0x0204;
////			send_to_aerial.receiver_id = 6;
////		break;

////		case 103://蓝色
////			send_to_aerial.data_cmd_id=0x0203;
////			send_to_aerial.receiver_id = 106;
////		break;
////		case 104:
////			send_to_aerial.data_cmd_id=0x204;
////			send_to_aerial.receiver_id = 106;
////		break;
////	}
////    send_to_aerial.sender_id = judge_rece_mesg.game_robot_state.robot_id;
////    
////    memcpy((uint8_t *)dddata,(uint8_t *)&send_to_aerial,sizeof(send_to_aerial));
////    dddata[6] = (uint8_t)judge_rece_mesg.Projectile_Allowance.bullet_remaining_num_17mm;
////    dddata[7] = (uint8_t)(judge_rece_mesg.Projectile_Allowance.bullet_remaining_num_17mm >> 8);
////    data_upload_handle(ROBOT_INTERACTIVE_DATA_ID,dddata,sizeof(send_to_aerial)+sizeof(judge_rece_mesg.Projectile_Allowance.bullet_remaining_num_17mm),DN_REG_ID,tx_buf);

////}


//void delete_Coverage(u8 coverage)
//{
//	ddata[6]=4;//1增加2修改3删除单个4删除图层5删除所有
//	ddata[13]=coverage;//图层0-9
//}


