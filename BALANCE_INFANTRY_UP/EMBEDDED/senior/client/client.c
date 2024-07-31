#include "client.h"

/**
  ******************************************************************************
  * @file    client.c
  * @date    2024.7.5
  * @brief    各图形定义及引用方法见client.h，除字符外共用结构体interaction_figure_t
	*	@introduction UI绘制步骤
			静态UI ：1.创建interaction_figure_t图形对象或client_custom_character_t字符对象
							 2.调用UI.结构体中ADD函数进行绘制 如UI.ADD_7Graph（入口参数）
			动态UI ：前两步同静态UI
							 3.调用UI.结构体中相应modify函数进行动态自定义更新
							 4.自行调整case和UI.cnt
 ===============================================================================
 **/
#define WIDTH    3
u16 client_custom_ID=0;
uint8_t dddata[120];
uint8_t  tx_buf[150];
uint8_t ddata[120];

UI_t UI=UI_DEFAULT;

VMC_t VMC_point;

id_data_t send_to_aerial;


/*创建图形对象*/
/*电容警示圈*/
interaction_figure_t cap_down_arc =         ARC(ADD,0,0,1,960,540,140*3-30,140*3-30,225,315,6,0,UI_BLACK);
interaction_figure_t cap_up_arc =           ARC(ADD,0,0,2,960,540,140*3-30,140*3-30,225,315,5,1,UI_YELLOW);
/*热量圈*/ 
interaction_figure_t heat_down_arc =        ARC(ADD,0,0,3,960,540,140*3-30,140*3-30,45,135,6,0,UI_BLACK);
interaction_figure_t heat_up_arc =          ARC(ADD,0,0,4,960,540,140*3-30,140*3-30,45,135,5,1,UI_RB);
/*底盘状态*/    
interaction_figure_t yaw_down_arc =         ARC(ADD,0,0,5,960,130,40,40,0,359,15,0,UI_YELLOW);/*yaw底圈*/
interaction_figure_t yaw_up_arc =           ARC(ADD,0,0,6,960,130,40,40,0,359,15,1,UI_CYAN);
/*视觉状态*/    
interaction_figure_t vision_cl =         CIRCLE(ADD,0,0,7,1650,530,9,5,0,UI_RB);
/*剩余发弹量*/
interaction_figure_t bullet =         FLOAT_NUM(ADD,0,0,8,230,540,0,20,4,0,UI_RB);
/*等级*/                             
interaction_figure_t level  =         FLOAT_NUM(ADD,0,0,9,230,570,0,20,4,0,UI_RB);
/*电池*/
interaction_figure_t battery_ract =   RECTANGLE(ADD,0,1,0,600,730,115,155,3,0,UI_RB);
interaction_figure_t battery =        FLOAT_NUM(ADD,0,1,1,607,144,0,17,4,1,UI_CYAN);
/*准星*/
interaction_figure_t sight =             CIRCLE(ADD,0,1,2,960,540,1,3,0,UI_YELLOW);
interaction_figure_t sight1 =            CIRCLE(ADD,0,1,9,970,490,2,4,0,UI_YELLOW);   
/*五连杆*/
interaction_figure_t l1 =                  LINE(ADD,0,2,0,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l2 =                  LINE(ADD,0,2,1,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l3 =                  LINE(ADD,0,2,2,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l4 =                  LINE(ADD,0,2,3,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l5 =                  LINE(ADD,0,2,4,0,0,0,0,WIDTH,1,UI_RB);
interaction_figure_t wheel =             CIRCLE(ADD,0,2,5,0,0,100/2,WIDTH,1,UI_RB); 

interaction_figure_t X1;

/*字符*/
interaction_figure_t _b1=CHARACTER(ADD,0,1,3,1800,540,20,20,2,0,UI_YELLOW);/*bullet*/    interaction_figure_t _r1=CHARACTER(ADD,0,1,3,50,540,20,20,2,0,UI_YELLOW);/*bullet*/
interaction_figure_t _b2=CHARACTER(ADD,0,1,4,1800,570,20,20,2,0,UI_YELLOW);/*level*/     interaction_figure_t _r2=CHARACTER(ADD,0,1,4,50,570,20,20,2,0,UI_YELLOW);/*level*/ 
interaction_figure_t _b3=CHARACTER(ADD,0,1,5,1700,660,20,20,2,0,UI_RB);/*BIG_BUFF*/      interaction_figure_t _r3=CHARACTER(ADD,0,1,5,50,660,20,20,2,0,UI_RB);/*BIG_BUFF*/
interaction_figure_t _b4=CHARACTER(ADD,0,1,6,1700,630,20,20,2,0,UI_RB);/*SMALL_BUFF*/    interaction_figure_t _r4=CHARACTER(ADD,0,1,6,50,630,20,20,2,0,UI_RB);/*SMALL_BUFF*/ 
interaction_figure_t _b5=CHARACTER(ADD,0,1,7,1700,600,20,20,2,0,UI_RB);/*AIM_NORMAL*/    interaction_figure_t _r5=CHARACTER(ADD,0,1,7,50,600,20,20,2,0,UI_RB);/*AIM_NORMAL*/ 

/*创建 组合图形对象*/
interaction_figure_4_t A;
interaction_figure_4_t AA;
interaction_figure_4_t AB;

/*创建 字符对象*/
client_custom_character_t B;uint8_t dataB[]="BULLET";
client_custom_character_t C;uint8_t dataC[]="LEVEL";
client_custom_character_t D;uint8_t dataD[]="BIG_BUFF";
client_custom_character_t E;uint8_t dataE[]="SMALL_BUFF";
client_custom_character_t F;uint8_t dataF[]="AIM_NORMAL";


/*UI刷新主函数*/
void Client_Send_Handle()
{	
  UI.id=judge_rece_mesg.game_robot_state.robot_id;
  switch(UI.id)
    {
    case 3:
      client_custom_ID=0x0103;
      break;
    case 4:
      client_custom_ID=0x0104;
      break;
    case 5:
      client_custom_ID=0x0105;
      break;
    case 103://蓝色
      client_custom_ID=0x0167;
      break;
    case 104:
      client_custom_ID=0x0168;
      break;
    case 105:
      client_custom_ID=0x0169;
      break;
    }
	
	switch(UI.cnt)
		{
		case 1:/*静态显示*/
		{
			UI.ADD_7Graph(A,cap_down_arc,cap_up_arc,heat_down_arc,heat_up_arc,yaw_down_arc,yaw_up_arc,vision_cl);
		}break;
		case 2:
		{
			UI.ADD_7Graph(AA,bullet,level,battery_ract,battery,sight,sight1,wheel);
		}break;
        case 3:
        {
            UI.ADD_7Graph(AB,l1,l2,l3,l4,l5,X1,X1);
        }break;
		case 4:
		{
           if(UI.id < 50)
           {
               UI.ADD_Char(B,_r1,dataB,6);
           }else
           {
               UI.ADD_Char(B,_b1,dataB,6);
           }
			
		}break;
		case 5:
		{
            if(UI.id < 50)
           {
               UI.ADD_Char(C,_r2,dataC,5);
           }else
           {
               UI.ADD_Char(C,_b2,dataC,5);
           }
		}break;
		case 6:
		{
            if(UI.id < 50)
           {
               UI.ADD_Char(D,_r3,dataD,8);
           }else
           {
               UI.ADD_Char(D,_b3,dataD,8);
           }
		}break;
		case 7:
		{
            if(UI.id < 50)
           {
               UI.ADD_Char(E,_r4,dataE,10);
           }else
           {
               UI.ADD_Char(E,_b4,dataE,10);
           }
		}break;
		case 8:
		{
            if(UI.id < 50)
           {
               UI.ADD_Char(F,_r5,dataF,10);
           }else
           {
               UI.ADD_Char(F,_b5,dataF,10);
           }
		}break;
		case 9:/*动态显示*/
		{
			UI.MODIFY_7Graph_0(A,cap_up_arc,heat_up_arc,yaw_up_arc,vision_cl,bullet,level,battery);
		}break;
		case 10:
		{
			UI.MODIFY_7Graph_1(AA,sight,l1,l2,l3,l4,l5,wheel);
		}break;
		
		default:
     break;
    }
		
	
	if(UI.circle_360<360)
	  {UI.circle_360+=40;}
	else
	  {UI.circle_360-=360;}
		
	UI.cnt++;
  if(UI.cnt>10)/*在需要刷新的图层刷新*/
     UI.cnt=9;
  
  if(RC_CtrlData.Key_Flag.Key_R_Flag)
	{
		UI.cnt = 1;
	}
}



//建议范围 x（960+-120*2.75） y（540+-280）
typedef struct
{
  int16_t x;
  int16_t y;
} point;

point rotate_point(int16_t x,int16_t y,float angle)
{
  point result;
  float rad_angle=angle*ANGLE_TO_RAD;
  result.x=(int)(x*cos(rad_angle)-y*sin(rad_angle));
  result.y=(int)(x*sin(rad_angle)+y*cos(rad_angle));
    
  return result;
}


void VMC_ui_cal(int x1,int y1)
{
    VMC_point.x[0] = x1;VMC_point.y[0] = y1;
    
    VMC_point.x[1] = x1+L5;VMC_point.y[1] = y1;
    
    VMC_point.x[2] = x1 - L4*cosf(usart_gimbal_data.phi4);VMC_point.y[2] = y1 - L4*sinf(usart_gimbal_data.phi4);
    
    VMC_point.x[3] = VMC_point.x[1] - L1*cosf(usart_gimbal_data.phi1);VMC_point.y[3] = VMC_point.y[1] - L1*sinf(usart_gimbal_data.phi1);
    
    VMC_point.x[4] = x1 + L5/2 - (usart_gimbal_data.L0*1000/2)*cosf(usart_gimbal_data.phi0);VMC_point.y[4] = y1 - (usart_gimbal_data.L0*1000/2)*sinf(usart_gimbal_data.phi0);
}

void ADD_Character(client_custom_character_t _0,interaction_figure_t __0,uint8_t *data0,uint8_t size0)
{
		robot_interaction_data_t UI_data;

		UI_data.id_data.data_cmd_id=0x0110;
		UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
    UI_data.id_data.receiver_id=client_custom_ID; //客户端id

		memcpy((uint8_t *)dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
	
		_0.interaction_figure=__0;
		memcpy(_0.data,data0,size0);
		*(client_custom_character_t*)(&dddata[6])=_0;
//		memcpy((uint8_t *)&UI_data.user_data,(uint8_t *)&_0,sizeof(client_custom_character_t));
	
		memcpy((uint8_t *)(dddata+6+sizeof(client_custom_character_t)),(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
	
		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,dddata,2*sizeof(UI_data.id_data)+sizeof(client_custom_character_t),DN_REG_ID,tx_buf);
}

void ADD_7_Graph(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
{
     robot_interaction_data_t UI_data;
	
			UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
      UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
      UI_data.id_data.receiver_id=client_custom_ID; //客户端id

			_7.interaction_figure[0]=_0;
			_7.interaction_figure[1]=_1;
			_7.interaction_figure[2]=_2;
			_7.interaction_figure[3]=_3;
			_7.interaction_figure[4]=_4;
			_7.interaction_figure[5]=_5;
			_7.interaction_figure[6]=_6;

			memcpy(dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
		  *(interaction_figure_4_t*)(&dddata[6])=_7;
//			memcpy(dddata+sizeof(UI_data.id_data),(interaction_figure_4_t *)&_7,sizeof(interaction_figure_4_t));
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,dddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
		}

void MODIFY_2_Character_Num(client_custom_character_t _0,interaction_figure_t __0,float data0,client_custom_character_t _1,interaction_figure_t __1,float data1)
{
		robot_interaction_data_t UI_data;

		UI_data.id_data.data_cmd_id=0x0110;
		UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
	
    memcpy(dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));	
	
		_0.interaction_figure=__0;
		_0.interaction_figure=__1;
	
		_0.interaction_figure.operate_tpye=2;
		sprintf((char *)_0.data,"%f",data0);
		*(client_custom_character_t*)(&dddata[6])=_0;
	
		_1.interaction_figure.operate_tpye=2;
		sprintf((char *)_1.data,"%f",data1);
		*(client_custom_character_t*)(&dddata[6+sizeof(client_custom_character_t)])=_1;
//		memcpy((uint8_t *)&UI_data.user_data,(uint8_t *)&_0,sizeof(client_custom_character_t));
//		memcpy((uint8_t *)&UI_data.user_data+sizeof(client_custom_character_t),(uint8_t *)&_1,sizeof(client_custom_character_t));
	 data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,dddata ,sizeof(UI_data.id_data)+2*sizeof(client_custom_character_t),DN_REG_ID,tx_buf);
}

void MODIFY_7_Graph_DIY(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
{
		robot_interaction_data_t UI_data;
	
		UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
    UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
	
		_7.interaction_figure[0]=_0;
		_7.interaction_figure[1]=_1;
		_7.interaction_figure[2]=_2;
		_7.interaction_figure[3]=_3;
		_7.interaction_figure[4]=_4;
		_7.interaction_figure[5]=_5;
		_7.interaction_figure[6]=_6;
	
		_7.interaction_figure[0].operate_tpye=MODIFY;
		_7.interaction_figure[1].operate_tpye=MODIFY;
		_7.interaction_figure[2].operate_tpye=MODIFY;
		_7.interaction_figure[3].operate_tpye=MODIFY;
		_7.interaction_figure[4].operate_tpye=MODIFY;
		_7.interaction_figure[5].operate_tpye=MODIFY;
		_7.interaction_figure[6].operate_tpye=MODIFY;
/*第1个图形*/
		_7.interaction_figure[0].details_b=225+usart_gimbal_data.cap_v*90.0f/24.0f;
        VAL_LIMIT(_7.interaction_figure[0].details_b,225,315);
		if(usart_gimbal_data.cap_v<=0)
			{
					_7.interaction_figure[0].color=UI_YELLOW;
					_7.interaction_figure[0].details_b=315;
			}
		
		if(usart_gimbal_data.cap_v>5)
			{
               if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag==1)
               {
                   _7.interaction_figure[0].color=UI_GREEN;
               }else
               {
				   _7.interaction_figure[0].color=UI_YELLOW;
               }
			}

		 
/*第2个图形*/	
		_7.interaction_figure[1].details_a=45+(judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat)*90.0f/judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit;
		_7.interaction_figure[1].details_b = 135;
		
		if(shoot.fric_wheel_run==1)
			{
               _7.interaction_figure[1].color=UI_RB;
			}
		else 
		  {
			_7.interaction_figure[1].color=UI_YELLOW;

		  }
/*第3个图形*/			
		float yaw__180_180;
			float yaw_0_360	=fmod(yaw_Encoder.ecd_angle*YAW_POLARITY,360);	
			if(yaw_0_360<0){yaw_0_360+=360;}
				if(yaw_0_360>=180)/*将0-2PI转换到-PI-PI范围内*/
					{yaw__180_180=yaw_0_360-360;}
				else
					{yaw__180_180=yaw_0_360;}
		
		_7.interaction_figure[2].details_a=yaw_0_360+15;//gimbal_gyro.yaw_Angle+15;		

		if(yaw_0_360+345>360)
			yaw_0_360=yaw_0_360-360;
		_7.interaction_figure[2].details_b=yaw_0_360+345;//gimbal_gyro.yaw_Angle+345;
        
        if(RC_CtrlData.Key_Flag.Key_F_TFlag)
        {
            _7.interaction_figure[2].color = UI_PINK;
        }else
        {
            _7.interaction_figure[2].color = UI_CYAN;
        }
/*第4个图形*/			
         switch(gimbal_data.vision_mode)
            {
                case AIM_NORMAL:
                {
                      _7.interaction_figure[3].start_y=590;
                }break;
                case SMALL_BUFF:
                {
                      _7.interaction_figure[3].start_y=620;
                }break;
                case BIG_BUFF:
                {
                      _7.interaction_figure[3].start_y=650;
                }break;
                
            }
            if(UI.id < 50)
           {
               _7.interaction_figure[3].start_x=300;
           }else
           {
               _7.interaction_figure[3].start_x=1650;
           }
                   
            if(chassis.ctrl_mode == CHASSIS_SEPARATE)
            {
                _7.interaction_figure[3].color = UI_RB;
            }else
            {
               _7.interaction_figure[3].color = UI_YELLOW; 
            }
/*第5个图形 */			
			uint32_t  cap_temp=((580-already_shoot)*1000.0f);
			_7.interaction_figure[4].details_c=cap_temp;
		  _7.interaction_figure[4].details_d=cap_temp>>10;
			_7.interaction_figure[4].details_e=cap_temp>>21;
             if(UI.id < 50)
           {
               _7.interaction_figure[4].start_x = 230;
           }else
           {
               _7.interaction_figure[4].start_x = 1700;
           }
/*第6个图形*/			
		uint32_t  cap_temp1=(judge_rece_mesg.game_robot_state.robot_level*1000.0f);
			_7.interaction_figure[5].details_c=cap_temp1;
		  _7.interaction_figure[5].details_d=cap_temp1>>10;
			_7.interaction_figure[5].details_e=cap_temp1>>21;
            if(UI.id < 50)
           {
               _7.interaction_figure[5].start_x = 230;
           }else
           {
               _7.interaction_figure[5].start_x = 1700;
           }
/*第7个图形*/			
		uint32_t  cap_temp2=((usart_gimbal_data.input_V-22)/3.0*100*1000.0f);
			_7.interaction_figure[6].details_c=cap_temp2;
		  _7.interaction_figure[6].details_d=cap_temp2>>10;
			_7.interaction_figure[6].details_e=cap_temp2>>21;
			
			memcpy((uint8_t *)dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
			*(interaction_figure_4_t*)(&dddata[6])=_7;
//			memcpy((uint8_t *)(dddata+sizeof(UI_data.id_data)),(interaction_figure_4_t*)&_7,sizeof(interaction_figure_4_t));
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, dddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
}

void MODIFY_7_Graph_DIY1(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
{
		robot_interaction_data_t UI_data;
	
		UI_data.id_data.data_cmd_id=0x0104;//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形
    UI_data.id_data.sender_id =judge_rece_mesg.game_robot_state.robot_id;
    UI_data.id_data.receiver_id=client_custom_ID; //客户端id
	
		_7.interaction_figure[0]=_0;
		_7.interaction_figure[1]=_1;
		_7.interaction_figure[2]=_2;
		_7.interaction_figure[3]=_3;
		_7.interaction_figure[4]=_4;
		_7.interaction_figure[5]=_5;
		_7.interaction_figure[6]=_6;
	
		_7.interaction_figure[0].operate_tpye=MODIFY;
		_7.interaction_figure[1].operate_tpye=MODIFY;
		_7.interaction_figure[2].operate_tpye=MODIFY;
		_7.interaction_figure[3].operate_tpye=MODIFY;
		_7.interaction_figure[4].operate_tpye=MODIFY;
		_7.interaction_figure[5].operate_tpye=MODIFY;
		_7.interaction_figure[6].operate_tpye=MODIFY;
		
/*第1个图形 累计发弹数*/
		if(My_Auto_Shoot.Auto_Aim.Flag_Get_Target)
			{
                 _7.interaction_figure[0].color =  UI_RB;
            }else
            {
                _7.interaction_figure[0].color =  UI_YELLOW;
            }
		 
/*第2-6个图形 预计发弹数*/	
         VMC_ui_cal(1250,200);
		_7.interaction_figure[1].start_x = VMC_point.x[1];
        _7.interaction_figure[1].start_y = VMC_point.y[1];
        _7.interaction_figure[1].details_d = VMC_point.x[3];
        _7.interaction_figure[1].details_e = VMC_point.y[3];
            
        _7.interaction_figure[2].start_x = VMC_point.x[3];
        _7.interaction_figure[2].start_y = VMC_point.y[3];
        _7.interaction_figure[2].details_d = VMC_point.x[4];
        _7.interaction_figure[2].details_e = VMC_point.y[4];
            
        _7.interaction_figure[3].start_x = VMC_point.x[2];
        _7.interaction_figure[3].start_y = VMC_point.y[2];
        _7.interaction_figure[3].details_d = VMC_point.x[4];
        _7.interaction_figure[3].details_e = VMC_point.y[4];
            
        _7.interaction_figure[4].start_x = VMC_point.x[0];
        _7.interaction_figure[4].start_y = VMC_point.y[0];
        _7.interaction_figure[4].details_d = VMC_point.x[2];
        _7.interaction_figure[4].details_e = VMC_point.y[2];
        
        _7.interaction_figure[5].start_x = VMC_point.x[0];
        _7.interaction_figure[5].start_y = VMC_point.y[0];
        _7.interaction_figure[5].details_d = VMC_point.x[1];
        _7.interaction_figure[5].details_e = VMC_point.y[1];
			
		_7.interaction_figure[6].start_x = VMC_point.x[4];	
		_7.interaction_figure[6].start_y = VMC_point.y[4];
        
        if(RC_CtrlData.Key_Flag.Key_Z_TFlag)
        {
            _7.interaction_figure[1].color = UI_GREEN;
            _7.interaction_figure[2].color = UI_GREEN;
            _7.interaction_figure[3].color = UI_GREEN;
            _7.interaction_figure[4].color = UI_GREEN;
        }else if(usart_gimbal_data.jump_flag==1)
        {
           _7.interaction_figure[1].color = UI_PINK;
           _7.interaction_figure[2].color = UI_PINK;
           _7.interaction_figure[3].color = UI_PINK;
           _7.interaction_figure[4].color = UI_PINK; 
        }else
        {
           _7.interaction_figure[1].color = UI_YELLOW;
           _7.interaction_figure[2].color = UI_YELLOW;
           _7.interaction_figure[3].color = UI_YELLOW;
           _7.interaction_figure[4].color = UI_YELLOW; 
        }
			
			memcpy((uint8_t *)dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
			*(interaction_figure_4_t*)(&dddata[6])=_7;
//			memcpy((uint8_t *)(dddata+sizeof(UI_data.id_data)),(interaction_figure_4_t*)&_7,sizeof(interaction_figure_4_t));
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, dddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);
}


void Send_bullet_remaining_num(void)
{
    
    send_to_aerial.data_cmd_id = 0x0200 + (uint16_t)(judge_rece_mesg.game_robot_state.robot_id%100);//按兵种标号向后顺序排 0x0202 0x0203 0x0204----0x02FF
    send_to_aerial.sender_id = judge_rece_mesg.game_robot_state.robot_id;
    if(judge_rece_mesg.game_robot_state.robot_id < 50)//红方一号
    {
        send_to_aerial.receiver_id = 6;
    }
    else//蓝方一号
    {
        send_to_aerial.receiver_id = 106;
    }
    
   
    
    
    memcpy((uint8_t *)ddata,(uint8_t *)&send_to_aerial,sizeof(send_to_aerial));
    ddata[6] = (uint8_t)judge_rece_mesg.ext_bullet_remaining.bullet_remaining_num_17mm;
    ddata[7] = (uint8_t)(judge_rece_mesg.ext_bullet_remaining.bullet_remaining_num_17mm >> 8);
    data_upload_handle(ROBOT_INTERACTIVE_DATA_ID,ddata,sizeof(send_to_aerial)+sizeof(judge_rece_mesg.ext_bullet_remaining.bullet_remaining_num_17mm),DN_REG_ID,tx_buf);

}	