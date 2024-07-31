#ifndef _CLIENT_H_
#define _CLIENT_H_
#include "public.h"

#define ADD 1
#define MODIFY 2
//颜色
#define UI_RB     0   //红蓝主色
#define UI_YELLOW 1
#define UI_GREEN  2
#define UI_ORANGE 3
#define UI_PURPLE 4
#define UI_PINK   5
#define UI_CYAN   6   //青色
#define UI_BLACK  7
#define UI_WHITE  8

#define L1       150/2
#define L2       270/2
#define L3       270/2
#define L4       150/2
#define L5       150/2

#define LINE(op,n1,n2,n3,x1,x2,y1,y2,w,l,c) { \
				.figure_tpye=0, \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.details_d=x2, \
				.start_y=y1, \
				.details_e=y2, \
				.width=w, \
				.layer=l, \
	      .color=c, \
} \

#define RECTANGLE(op,n1,n2,n3,x1,x2,y1,y2,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.details_d=x2, \
				.start_y=y1, \
				.details_e=y2, \
				.width=w, \
				.layer=l, \
	      .color=c, \
				.figure_tpye=1, \
} \

#define CIRCLE(op,n1,n2,n3,x1,y1,r,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.start_y=y1, \
				.details_c=r, \
				.width=w, \
				.layer=l, \
	      .color=c, \
				.figure_tpye=2, \
} \

#define ELLIPSE(op,n1,n2,n3,x1,y1,rx,ry,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.start_y=y1, \
				.details_d=rx, \
				.details_e=ry, \
				.width=w, \
				.layer=l, \
	      .color=c, \
				.figure_tpye=3, \
} \

#define ARC(op,n1,n2,n3,x1,y1,rx,ry,a1,a2,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.start_y=y1, \
				.details_d=rx, \
				.details_e=ry, \
				.details_a=a1, \
				.details_b=a2, \
				.width=w, \
				.layer=l, \
	      .color=c, \
				.figure_tpye=4, \
} \

#define FLOAT_NUM(op,n1,n2,n3,x1,y1,_float,size,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.start_y=y1, \
				.details_c=(_float*1000), \
				.details_d=(_float*1000)>>10, \
				.details_e=(_float*1000)>>21, \
				.details_a=size, \
				.width=w, \
				.layer=l, \
	      .color=c, \
				.figure_tpye=5, \
} \

#define INT_NUM(op,n1,n2,n3,x1,y1,_int32,size,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.start_y=y1, \
				.details_c=(float)_int32, \
				.details_d=(float)_int32>>10, \
				.details_e=(float)_int32>>21, \
				.details_a=size, \
				.width=w, \
				.layer=l, \
	      .color=c, \
				.figure_tpye=6, \
} \

#define CHARACTER(op,n1,n2,n3,x1,y1,size,len,w,l,c) { \
				.operate_tpye=op, \
				.figure_name[0]=n1, \
				.figure_name[1]=n2, \
				.figure_name[2]=n3, \
				.start_x=x1, \
				.start_y=y1, \
				.details_a=size, \
				.details_b=len, \
				.width=w, \
				.layer=l, \
	      .color=c, \
			  .figure_tpye=7, \
} \

/*子内容ID：0x0100*/
typedef __packed struct 
{ 
uint8_t delete_type; /*0：空操作 1：删除图层 2：删除所有*/
uint8_t layer; /*图层数：0-9*/
}interaction_layer_delete_t;

/*子内容ID：0x0101*/
typedef __packed struct /*总计3个uint8 3个uint32*/
{ /*（ 0-2 3-5 6-9 10-13 14-31）（0-9 10-20 21-31）（0-31）*/
uint8_t figure_name[3];/*图形名 在图形删除、修改等操作中，作为索引*/
uint32_t operate_tpye:3;/*图形操作 1 增加 2修改图形 3删除单个图形*/  
uint32_t figure_tpye:3;/*图形类型 0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符*/ 
uint32_t layer:4;/*图层数（0-9）*/  
uint32_t color:4;/*颜色 0 红/蓝 1 黄 2 绿 3 橙 4 紫红 5 粉 6 青 7 黑 8 白 */  
uint32_t details_a:9; 
uint32_t details_b:9; /*图形细节参数 占18个字节*/
uint32_t width:10;/*线宽，建议字体大小与线宽比例为10：1 */  
uint32_t start_x:11;/*起点/圆心x坐标 */  
uint32_t start_y:11;/*起点/圆心y坐标 */
uint32_t details_c:10;  
uint32_t details_d:11;  
uint32_t details_e:11;  /*图形细节参数 占32个字节*/
}interaction_figure_t; 

/*表 2-26 图形细节参数说明 
类型 details_a details_b details_c details_d       details_e 
直线    - 				- 				- 			终点x坐标       终点y坐标 
矩形	  - 				- 				-      对角顶点x坐标    对角顶点y坐标
正圆    - 				-        半径        -                - 
椭圆	  - 				-				  - 			x半轴长度 				y半轴长度 
圆弧 起始角度   终止角度		  -			  x半轴长度 				y半轴长度 
浮点数 字体大小  无作用				 该值除以1000即实际显示值 
整型数 字体大小    -							 32位整型数，int32_t 
字符   字体大小 字符长度 			- 				 - 								- 

角度值含义为：0°指12点钟方向，顺时针绘制； 
屏幕位置：（0,0）为屏幕左下角（1920，1080）为屏幕右上角； 
浮点数：整型数均为32位，对于浮点数，实际显示的值为输入的值/1000，如在details_c、details_d、details_e对应的字节输入1234，选手端实际显示的值将为1.234。 
即使发送的数值超过对应数据类型的限制，图形仍有可能显示，但此时不保证显示的效果。 
*/
#define UI_DEFAULT { \
				.ADD_Char=&ADD_Character, \
				.ADD_7Graph=&ADD_7_Graph, \
				.MODIFY_7Graph_0=&MODIFY_7_Graph_DIY, \
				.MODIFY_7Graph_1=MODIFY_7_Graph_DIY1 \
} \

/*子内容ID：0x0102*/
typedef __packed struct/*绘制两个图形*/
{ 
  interaction_figure_t interaction_figure[2]; 
}interaction_figure_2_t;
/*子内容ID：0x0103*/
typedef __packed struct/*绘制5个图形*/
{ 
interaction_figure_t interaction_figure[5]; 
}interaction_figure_3_t; 
/*子内容ID：0x0104*/
typedef __packed struct/*绘制7个图形*/
{ 
interaction_figure_t interaction_figure[7]; 
}interaction_figure_4_t;

/*子内容ID：0x0110*/
typedef __packed struct 
{ 
interaction_figure_t  interaction_figure; 
uint8_t data[30]; 
}client_custom_character_t;
typedef __packed struct 
{
  uint16_t data_cmd_id; /*子内容ID 需为开放的子内容ID*/
  uint16_t sender_id; /*发送者ID 需与自身ID匹配，ID编号详见附录 */
  uint16_t receiver_id; /*接收者ID 仅限己方通信 需为规则允许的多机通讯接收者 若接收者为选手端，则仅可发送至发送者对应的选手端 ID编号详见附录*/
}id_data_t;
/*0x0301*/
typedef __packed struct 
{ 
	id_data_t id_data;
  uint8_t user_data[113]; /*内容数据段 x最大为113 */
}robot_interaction_data_t; 

typedef struct _UI
{	
	uint8_t id;
	uint8_t cnt; 
	uint16_t circle_360;
	void (*ADD_Char)        (client_custom_character_t _0,interaction_figure_t __0,uint8_t *data0,uint8_t size0);
	void (*ADD_7Graph)      (interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6);
	void (*MODIFY_7Graph_0)(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6);
	void (*MODIFY_7Graph_1)(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6);
}UI_t;

typedef struct
{
    int x[5];
    int y[5];
}VMC_t;

extern UI_t UI;
void Client_send_handle(void);
void ADD_7_Graph(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6);
void ADD_Character(client_custom_character_t _0,interaction_figure_t __0,uint8_t *data0,uint8_t size0);
void MODIFY_2_Character_Num(client_custom_character_t _0,interaction_figure_t __0,float data0,client_custom_character_t _1,interaction_figure_t __1,float data1);
void MODIFY_7_Graph_DIY(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6);
void MODIFY_7_Graph_DIY1(interaction_figure_4_t _7,interaction_figure_t _0,interaction_figure_t _1,interaction_figure_t _2,interaction_figure_t _3,interaction_figure_t _4,interaction_figure_t _5,interaction_figure_t _6);
void Client_Send_Handle(void);
void VMC_ui_cal(int x1,int y1);
void Send_bullet_remaining_num(void);
extern uint8_t  tx_buf[150];
#endif
