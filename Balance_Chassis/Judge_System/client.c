#include "main.h"


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
//准星坐标位置
#define SIGHT1_X 950 
#define SIGHT1_Y 498
//弹仓容量
#define BULLET_SUM 495

u16 client_custom_ID=0;
uint8_t dddata[120];
uint8_t  tx_buf[150];
uint8_t ddata[120];

UI_t UI=UI_DEFAULT;
//id_data_t send_to_aerial;

/*创建图形对象*/
/*电容警示圈*///老
interaction_figure_t cap_down_arc =         ARC(ADD,0,0,1,960,540,140*3-30,140*3-30,225,315,6,0,UI_BLACK);
interaction_figure_t cap_up_arc =           ARC(ADD,0,0,2,960,540,140*3-30,140*3-30,225,315,5,1,UI_YELLOW);

//interaction_figure_t cap_up_rectangle = RECTANGLE(ADD,0,0,1,)
/*热量圈*/
interaction_figure_t heat_down_arc =        ARC(ADD,0,0,3,960,540,140*3-30,140*3-30,45,135,6,0,UI_BLACK);
interaction_figure_t heat_up_arc =          ARC(ADD,0,0,4,960,540,140*3-30,140*3-30,45,135,5,1,UI_RB);
/*底盘状态*/
interaction_figure_t yaw_down_arc =         ARC(ADD,0,0,5,960,130,40,40,0,359,15,0,UI_YELLOW);/*yaw底圈*/
interaction_figure_t yaw_up_arc =           ARC(ADD,0,0,6,960,130,40,40,0,359,15,1,UI_CYAN);
/*视觉状态*/
interaction_figure_t vision_c2 =         RECTANGLE(ADD,0,2,8,750,1170,700,380,1,1,UI_BLACK);//op,n1,n2,n3,x1,x2,y1,y2,w,l,c //自瞄锁定：2
/*剩余发弹量*/
interaction_figure_t bullet =            FLOAT_NUM(ADD,0,0,8,230,540,0,20,4,0,UI_RB);
/*等级*/
interaction_figure_t level  =            FLOAT_NUM(ADD,0,0,9,230,570,0,20,4,0,UI_RB);
/*腿长*/
interaction_figure_t leg  =              FLOAT_NUM(ADD,0,2,5,230,700,0,20,4,0,UI_RB);
/*电容实际使用*/
interaction_figure_t cap_use  =          FLOAT_NUM(ADD,0,0,7,230,630,0,20,4,0,UI_YELLOW);
/*跳跃距离*/
interaction_figure_t jump_distance  =    FLOAT_NUM(ADD,0,2,7,1340,130,0,17,4,0,UI_PURPLE);

/*电池*/
interaction_figure_t battery_ract =   RECTANGLE(ADD,0,1,0,600,730,115,155,3,0,UI_RB);
interaction_figure_t battery =        FLOAT_NUM(ADD,0,1,1,607,144,0,17,4,1,UI_CYAN);//op,n1,n2,n3,x1,y1,_float,size,w,l,c
/*准星*/
interaction_figure_t sight =             CIRCLE(ADD,0,1,2,960,540,1,3,0,UI_YELLOW);
interaction_figure_t sight1 =            CIRCLE(ADD,0,1,9,0,0,3,4,0,UI_YELLOW); // CIRCLE(ADD,0,1,9,950,498,3,4,0,UI_YELLOW);//465

/*五连杆*/
interaction_figure_t l1 =                  LINE(ADD,0,2,0,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l2 =                  LINE(ADD,0,2,1,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l3 =                  LINE(ADD,0,2,2,0,0,0,0,WIDTH,1,UI_YELLOW);//op,n1,n2,n3,x1,x2,y1,y2,w,l,c
interaction_figure_t l4 =                  LINE(ADD,0,2,3,0,0,0,0,WIDTH,1,UI_YELLOW);
interaction_figure_t l5 =                  LINE(ADD,0,2,4,0,0,0,0,WIDTH,1,UI_RB);

interaction_figure_t wheel =             CIRCLE(ADD,0,2,6,0,0,100/2,WIDTH,1,UI_RB);

/*箭头/棒棒糖*/
interaction_figure_t arrows1_l1 =  LINE(ADD,0,2,9,0,0,850,650, WIDTH,1,UI_RB);//440,520,850,650
interaction_figure_t arrows1_l2 =  LINE(ADD,0,3,0,0,0,850,800, WIDTH,1,UI_RB);//440,480,850,800
interaction_figure_t arrows1_l3 =  LINE(ADD,0,3,1,0,0,850,800, WIDTH,1,UI_RB);// 440,440,850,800

/*小棒棒糖*/
interaction_figure_t lollipop_line   =  LINE(  ADD,0,3,2,0,0,450,350,WIDTH,1,UI_RB);//红方 1560,1520,450,350   //蓝方1760,1800
interaction_figure_t lollipop_circle =  CIRCLE(ADD,0,3,3,0,343,7,8,0,UI_RB);//op,n1,n2,n3,x1,y1,r,w,l,c//红方1518,343,7,7,  //1802,343,7,7

//跳跃模式提示线
interaction_figure_t jump_hint_line1  =  LINE(  ADD,0,3,4,0,0,0,0,WIDTH,1,UI_PURPLE);//红方 1560,1520,450,350   //蓝方1760,1800
interaction_figure_t jump_hint_line2  =  LINE(  ADD,0,3,5,0,0,0,0,WIDTH,1,UI_PURPLE);//红方 1560,1520,450,350   //蓝方1760,1800

/*底盘模式切换*/
interaction_figure_t chassis_mode_circle =  CIRCLE(ADD,0,3,6,0,0,7,8,1,UI_GREEN);

/*字符*/ //op,n1,n2,n3,x1,y1,size,len,w,l,c
interaction_figure_t _b1=CHARACTER(ADD,0,1,3,1600,540,20,20,2,0,UI_YELLOW);/*bullet*/    interaction_figure_t _r1=CHARACTER(ADD,0,1,3,50,540,20,20,2,0,UI_YELLOW);/*bullet*/
interaction_figure_t _b2=CHARACTER(ADD,0,1,4,1600,570,20,20,2,0,UI_YELLOW);/*level*/     interaction_figure_t _r2=CHARACTER(ADD,0,1,4,50,570,20,20,2,0,UI_YELLOW);/*level*/

interaction_figure_t _b3=CHARACTER(ADD,0,1,5,50,660,20,20,2,0,UI_PURPLE);/*NORMAL*/       interaction_figure_t _r3=CHARACTER(ADD,0,1,5,1660,660,20,20,2,0,UI_PURPLE);/*NORMAL*/
interaction_figure_t _b5=CHARACTER(ADD,0,1,7,50,630,20,20,2,0,UI_PURPLE);/*SLOPE*/     	 interaction_figure_t _r5=CHARACTER(ADD,0,1,7,1660,630,20,20,2,0,UI_PURPLE);/*SLOPE*/
interaction_figure_t _b7=CHARACTER(ADD,1,1,9,50,600,20,20,2,0,UI_PURPLE);/*GROUND*/    	 interaction_figure_t _r7=CHARACTER(ADD,1,1,9,1660,600,20,20,2,0,UI_PURPLE);/*GROUND*/
interaction_figure_t _b8=CHARACTER(ADD,0,2,0,50,690,20,20,2,0,UI_PURPLE);/*JUMP1700,130,17,15 */ interaction_figure_t _r8=CHARACTER(ADD,0,2,0,1660,690,20,20,2,0,UI_PURPLE);/*JUMP*/ 
//interaction_figure_t _b8=CHARACTER(ADD,0,2,0,1700,130,17,15,2,0,UI_RB);/*JUMP1700,130,17,15 */ interaction_figure_t _r8=CHARACTER(ADD,0,2,0,1250,130,17,15,2,0,UI_RB);/*JUMP*/ 

interaction_figure_t _b6=CHARACTER(ADD,0,1,8,1600,700,20,20,2,0,UI_YELLOW);/*leg*/       interaction_figure_t _r6=CHARACTER(ADD,0,1,8,50,700,20,20,2,0,UI_YELLOW);/*leg*/

/*创建 组合图形对象*/
interaction_figure_4_t A;
interaction_figure_4_t AA;
interaction_figure_4_t AB;
interaction_figure_4_t AC;

/*创建 字符对象*/
client_custom_character_t B;uint8_t dataB[]="BULLET";
client_custom_character_t C;uint8_t dataC[]="LEVEL";
client_custom_character_t D;uint8_t dataD[]="JUMP";  //跳跃模式
client_custom_character_t E;uint8_t dataE[]="NORMAL";//正常
client_custom_character_t F;uint8_t dataF[]="SLOPE"; //飞坡
client_custom_character_t H;uint8_t dataH[]="GROUND"; //贴地模式
client_custom_character_t G;uint8_t dataG[]="LEG";

/*UI刷新主函数*/
void Client_Send_Handle(void)
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
    case 1:/*静态显示*///全添加
    {
        UI.ADD_7Graph(A,cap_down_arc,cap_up_arc,heat_down_arc,heat_up_arc,yaw_down_arc,yaw_up_arc,sight1);
    }
    break;
    case 2:
    {
        UI.ADD_7Graph(AA,bullet,level,battery_ract,battery,jump_hint_line1,jump_hint_line2,leg);
    }
    break;
    case 3:
    {
        UI.ADD_7Graph(AB,jump_distance,vision_c2,chassis_mode_circle,arrows1_l1,arrows1_l2,lollipop_line,lollipop_circle);
    }
    break;
    case 4:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(B,_r1,dataB,6);
        } else
        {
            UI.ADD_Char(B,_b1,dataB,6);
        }

    }
    break;
    case 5:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(C,_r2,dataC,5);
        } else
        {
            UI.ADD_Char(C,_b2,dataC,5);
        }
    }
    break;
    case 6:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(G,_r6,dataG,5);
        } else
        {
            UI.ADD_Char(G,_b6,dataG,5);
        }
    }
    break;
    case 7:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(D,_r8,dataD,5);//跳跃模式
        } else
        {
            UI.ADD_Char(D,_b8,dataD,5);
        }
    }
    break;
	  case 8:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(E,_r3,dataE,6);//NORMAL
        } else
        {
            UI.ADD_Char(E,_b3,dataE,6);
        }
    }
    break;
	  case 9:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(F,_r5,dataF,5);//SLOPE
        } else
        {
            UI.ADD_Char(F,_b5,dataF,5);
        }
    }
    break;
	  case 10:
    {
        if(UI.id < 50)
        {
            UI.ADD_Char(H,_r7,dataH,6);//GROUND
        } else
        {
            UI.ADD_Char(H,_b7,dataH,6);
        }
    }
    break;
    case 11:/*动态显示*///更改图像
    {
        UI.MODIFY_7Graph_0(A,cap_up_arc,heat_up_arc,yaw_up_arc,leg,bullet,level,battery);
    }
    break;
    case 12:
    {
        UI.MODIFY_7Graph_1(AA,sight1,vision_c2,jump_distance,arrows1_l1,arrows1_l2,lollipop_line,chassis_mode_circle);
    }
    break;
    case 13:
    {
        UI.MODIFY_7Graph_2(AB,jump_hint_line1,jump_hint_line2,lollipop_circle,l3,l4,l5,wheel);
    }
    break;
    default:
        break;
    }

    if(UI.circle_360<360)
    {
        UI.circle_360+=40;
    }
    else
    {
        UI.circle_360-=360;
    }

    UI.cnt++;
    if(UI.cnt>13)/*在需要刷新的图层刷新*/
        UI.cnt=1;

//    if(RC_CtrlData.Key_Flag.Key_R_Flag)
//    {
//        UI.cnt = 1;
//    }

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
    data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID,dddata,sizeof(UI_data.id_data)+2*sizeof(client_custom_character_t),DN_REG_ID,tx_buf);
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
    _7.interaction_figure[0].details_b=225+can_capacitance_message.cap_voltage_filte*90.0f/24.0f;
    VAL_LIMIT(_7.interaction_figure[0].details_b,225,315);
    if(can_capacitance_message.cap_voltage_filte<=0)
    {
        _7.interaction_figure[0].color=UI_YELLOW;
        _7.interaction_figure[0].details_b=315;
    }

    if(can_capacitance_message.cap_voltage_filte>5)
    {
        if(Chassis.USART_Chassis_Data.V_y==2.5)
        {
            _7.interaction_figure[0].color=UI_GREEN;
        } else
        {
            _7.interaction_figure[0].color=UI_YELLOW;
        }
    }
    /*第2个图形*/
    _7.interaction_figure[1].details_a=45+(judge_rece_mesg.power_heat_data.shooter_17mm_1_barrel_heat)*90.0f/judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit;//7878
    _7.interaction_figure[1].details_b = 135;

    if(Chassis.USART_Chassis_Data.fric_wheel_run==1)
    {
        _7.interaction_figure[1].color=UI_RB;
    }
    else
    {
        _7.interaction_figure[1].color=UI_YELLOW;
    }
    /*第3个图形*/
    float yaw__180_180;
    float yaw_0_360	= Chassis.USART_Chassis_Data.Yaw_Encoder_Angle;
    if(yaw_0_360<0) {
        yaw_0_360+=360;
    }
    if(yaw_0_360>=180)/*将0-2PI转换到-PI-PI范围内*/
    {
        yaw__180_180=yaw_0_360-360;
    }
    else
    {
        yaw__180_180=yaw_0_360;
    }

    _7.interaction_figure[2].details_a=yaw_0_360+15;//gimbal_gyro.yaw_Angle+15;

    if(yaw_0_360+345>360)
        yaw_0_360=yaw_0_360-360;
    _7.interaction_figure[2].details_b=yaw_0_360+345;//gimbal_gyro.yaw_Angle+345;

//    if(b_chassis.ctrl_mode==CHASSIS_REVERSE)
//    {
//        _7.interaction_figure[2].color = UI_PINK;
//    } else
//    {
//        _7.interaction_figure[2].color = UI_CYAN;
//    }
    /*第4个图形*/
// //   uint32_t  cap_temp3=(b_chassis.balance_loop.L0*1000.0f);
//    _7.interaction_figure[3].details_c=cap_temp3;
//    _7.interaction_figure[3].details_d=cap_temp3>>10;
//    _7.interaction_figure[3].details_e=cap_temp3>>21;
    if(UI.id < 50)
    {
        _7.interaction_figure[3].start_x = 230;
    } else
    {
        _7.interaction_figure[3].start_x = 1800;
    }
    /*第5个图形 */
//    uint32_t  cap_temp=((BULLET_SUM-already_shoot)*1000.0f);
//    _7.interaction_figure[4].details_c=cap_temp;
//    _7.interaction_figure[4].details_d=cap_temp>>10;
//    _7.interaction_figure[4].details_e=cap_temp>>21;
    if(UI.id < 50)
    {
        _7.interaction_figure[4].start_x = 230;
    } else
    {
        _7.interaction_figure[4].start_x = 1800;
    }
    /*第6个图形*/
    uint32_t  cap_temp1=(judge_rece_mesg.game_robot_state.robot_level*1000.0f);
    _7.interaction_figure[5].details_c=cap_temp1;
    _7.interaction_figure[5].details_d=cap_temp1>>10;
    _7.interaction_figure[5].details_e=cap_temp1>>21;
    if(UI.id < 50)
    {
        _7.interaction_figure[5].start_x = 230;
    } else
    {
        _7.interaction_figure[5].start_x = 1800;
    }
    /*第7个图形*/
    uint32_t  cap_temp2=(can_capacitance_message.cap_voltage_filte*1000.0f);
    _7.interaction_figure[6].details_c=cap_temp2;
    _7.interaction_figure[6].details_d=cap_temp2>>10;
    _7.interaction_figure[6].details_e=cap_temp2>>21;

    memcpy((uint8_t *)dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
    *(interaction_figure_4_t*)(&dddata[6])=_7;
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


//  if(usart_chassis_data.fric_wheel_run==1)
//    {
//		if(usart_chassis_data.lock_shoot_check==1)
//		{
//		  _7.interaction_figure[0].color=UI_BLACK;

//		}
//		else
//		{
//		 _7.interaction_figure[0].color=UI_YELLOW;

//		}
//        _7.interaction_figure[0].start_x = SIGHT1_X;
//        _7.interaction_figure[0].start_y = SIGHT1_Y;
//    } else
//    {
//        _7.interaction_figure[4].start_x = 0;
//        _7.interaction_figure[4].start_y = 0;
//    }
//    ///////////////////////////////////////////////////////////////////////////////////////
//    if(usart_chassis_data.UI_auto_aim_state!=3&&usart_chassis_data.UI_auto_aim_state!=0)
//    {
//        if(usart_chassis_data.UI_auto_aim_state==1)
//        {
//            _7.interaction_figure[1].color=UI_WHITE;
//        } else if(usart_chassis_data.UI_auto_aim_state==2)
//        {
//            _7.interaction_figure[1].color=UI_GREEN;
//        }
//    }
    //////////////////////////////////////////////////////////////////////////////////////////

//    uint32_t  cap_temp1=(mea_distance*1000.0f);
//    _7.interaction_figure[2].details_c=cap_temp1;
//    _7.interaction_figure[2].details_d=cap_temp1>>10;
//    _7.interaction_figure[2].details_e=cap_temp1>>21;

//    if(init_abnormal_state==1)
//    {
//        _7.interaction_figure[2].color=UI_CYAN;
//    }
//    else
//    {
//        if(mea_distance<=1.0||mea_distance>1.9)
//        {
//            _7.interaction_figure[2].color=UI_BLACK;
//        }
//        else
//        {
//            _7.interaction_figure[2].color=UI_GREEN;
//        }
//    }


    if(UI.id < 50)
    {
        _7.interaction_figure[3].start_x = 440;
        _7.interaction_figure[3].start_y = 850;
        _7.interaction_figure[3].details_d = 520;//x
        _7.interaction_figure[3].details_e = 650;
        _7.interaction_figure[3].color=UI_RB;

    } else
    {
        _7.interaction_figure[3].start_x = 1400;
        _7.interaction_figure[3].start_y = 650;
        _7.interaction_figure[3].details_d = 1480;
        _7.interaction_figure[3].details_e = 850;
        _7.interaction_figure[3].color=UI_CYAN;
    }

    if(UI.id < 50)
    {
        _7.interaction_figure[4].start_x = 440;
        _7.interaction_figure[4].start_y = 850;
        _7.interaction_figure[4].details_d = 480;//x
        _7.interaction_figure[4].details_e = 800;
        _7.interaction_figure[4].color=UI_RB;

    } else
    {
        _7.interaction_figure[4].start_x = 1480;
        _7.interaction_figure[4].start_y = 850;
        _7.interaction_figure[4].details_d = 1440;
        _7.interaction_figure[4].details_e = 800;
        _7.interaction_figure[4].color=UI_CYAN;
    }
//    if(UI.id < 50)
//    {
//        _7.interaction_figure[5].start_x = 440;
//        _7.interaction_figure[5].start_y = 850;
//        _7.interaction_figure[5].details_d = 440;//x
//        _7.interaction_figure[5].details_e = 800;
//        _7.interaction_figure[5].color=UI_RB;

//    } else
//    {
//        _7.interaction_figure[5].start_x = 1480;
//        _7.interaction_figure[5].start_y = 850;
//        _7.interaction_figure[5].details_d = 1480;
//        _7.interaction_figure[5].details_e = 800;
//        _7.interaction_figure[5].color=UI_CYAN;
//    }
	    if(UI.id < 50)
    {
        _7.interaction_figure[5].start_x = 1560;
        _7.interaction_figure[5].start_y = 450;
        _7.interaction_figure[5].details_d = 1520;//x
        _7.interaction_figure[5].details_e = 350;
        _7.interaction_figure[5].color=UI_RB;

    } else
    {
        _7.interaction_figure[5].start_x = 1760;
        _7.interaction_figure[5].start_y = 450;
        _7.interaction_figure[5].details_d = 1800;
        _7.interaction_figure[5].details_e = 350;
        _7.interaction_figure[5].color=UI_CYAN;
    }


//	if(gimbal_control_state==1)
//{
//	if(balance_chassis.Driving_Encoder[0].if_online&&balance_chassis.Driving_Encoder[1].if_online)
//  {
//	  if(usart_chassis_data.fn_2_trigger_flag==1)
//	  {
//		_7.interaction_figure[6].color=UI_PURPLE;

//	  }
//	  else
//	  {
//	   _7.interaction_figure[6].color=UI_GREEN;
//	  }
//  }
//  else
//  {
//	   _7.interaction_figure[6].color=UI_BLACK;
//  }
//	  
////	if(UI.id < 50)//红方
////	{
////		_7.interaction_figure[6].start_x = 1630;
////	}
////	else 
////	{
////		_7.interaction_figure[6].start_x = 190;
////	}
////		if(b_chassis.jump_flag==1)//跳跃
////    {
////        _7.interaction_figure[6].start_y = 682;       
////    } 
////	 else if(usart_chassis_data.overstep_cmd==1)//飞坡
////	 {
////		 _7.interaction_figure[6].start_y = 622;      
////	 }
////	 else if(usart_chassis_data.ctrl_mode==1)//倒地
////	 {
////		 _7.interaction_figure[6].start_y = 592;      
////	 }
////	 else if(b_chassis.ctrl_mode == CHASSIS_INIT)
////	 {
////		 _7.interaction_figure[6].start_y = 560; 
////	 }
////	else//正常
////    {
////        _7.interaction_figure[6].start_y = 652;	
////			
////    }
////  
////}
//else
//{
//	_7.interaction_figure[6].start_x = 0;
//	_7.interaction_figure[6].start_y = 0;
//}


    memcpy((uint8_t *)dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
    *(interaction_figure_4_t*)(&dddata[6])=_7;
//			memcpy((uint8_t *)(dddata+sizeof(UI_data.id_data)),(interaction_figure_4_t*)&_7,sizeof(interaction_figure_4_t));
    data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, dddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);


}


void MODIFY_7_Graph_DIY2(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6)
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

	//////////////////////////////////////////////////////////////////////////////



//	if(b_chassis.jump_flag==1)
//    {
//        _7.interaction_figure[0].start_x = 960;
//        _7.interaction_figure[0].start_y = 820;
//        _7.interaction_figure[0].details_d = 1100;//x
//        _7.interaction_figure[0].details_e = 760;
//		
//	    _7.interaction_figure[1].start_x = 960;
//        _7.interaction_figure[1].start_y = 820;
//        _7.interaction_figure[1].details_d = 820;//x
//        _7.interaction_figure[1].details_e = 760;
//    } else
//    {
//        _7.interaction_figure[0].start_x = 0;
//        _7.interaction_figure[0].start_y = 0;	
//	    _7.interaction_figure[1].start_x = 0;
//        _7.interaction_figure[1].start_y = 0;
//    }
//////////////////////////////////////////////////////////////
	    if(UI.id < 50)
    {
        _7.interaction_figure[2].start_x = 1518;
        _7.interaction_figure[2].color=UI_RB;


    } else
    {
        _7.interaction_figure[2].start_x = 1802;
        _7.interaction_figure[2].color=UI_CYAN;

    }

	
    memcpy((uint8_t *)dddata,(uint8_t *)&UI_data.id_data,sizeof(UI_data.id_data));
    *(interaction_figure_4_t*)(&dddata[6])=_7;
//	memcpy((uint8_t *)(dddata+sizeof(UI_data.id_data)),(interaction_figure_4_t*)&_7,sizeof(interaction_figure_4_t));
    data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, dddata,sizeof(UI_data.id_data)+sizeof(interaction_figure_4_t),DN_REG_ID,tx_buf);


}
//void Send_bullet_remaining_num(void)
//{

//    send_to_aerial.data_cmd_id = 0x0200 + (uint16_t)(judge_rece_mesg.game_robot_state.robot_id%100);//按兵种标号向后顺序排 0x0202 0x0203 0x0204----0x02FF
//    send_to_aerial.sender_id = judge_rece_mesg.game_robot_state.robot_id;
//    if(judge_rece_mesg.game_robot_state.robot_id < 50)//红方一号
//    {
//        send_to_aerial.receiver_id = 6;
//    }
//    else//蓝方一号
//    {
//        send_to_aerial.receiver_id = 106;
//    }

//    memcpy((uint8_t *)ddata,(uint8_t *)&send_to_aerial,sizeof(send_to_aerial));
//    ddata[6] = (uint8_t)judge_rece_mesg.ext_bullet_remaining.bullet_remaining_num_17mm;
//    ddata[7] = (uint8_t)(judge_rece_mesg.ext_bullet_remaining.bullet_remaining_num_17mm >> 8);
//    data_upload_handle(ROBOT_INTERACTIVE_DATA_ID,ddata,sizeof(send_to_aerial)+sizeof(judge_rece_mesg.ext_bullet_remaining.bullet_remaining_num_17mm),DN_REG_ID,tx_buf);

//}


