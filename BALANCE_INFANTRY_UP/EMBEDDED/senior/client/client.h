#ifndef _CLIENT_H_
#define _CLIENT_H_
#include "public.h"

#define ADD 1
#define MODIFY 2
//��ɫ
#define UI_RB     0   //������ɫ
#define UI_YELLOW 1
#define UI_GREEN  2
#define UI_ORANGE 3
#define UI_PURPLE 4
#define UI_PINK   5
#define UI_CYAN   6   //��ɫ
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

/*������ID��0x0100*/
typedef __packed struct 
{ 
uint8_t delete_type; /*0���ղ��� 1��ɾ��ͼ�� 2��ɾ������*/
uint8_t layer; /*ͼ������0-9*/
}interaction_layer_delete_t;

/*������ID��0x0101*/
typedef __packed struct /*�ܼ�3��uint8 3��uint32*/
{ /*�� 0-2 3-5 6-9 10-13 14-31����0-9 10-20 21-31����0-31��*/
uint8_t figure_name[3];/*ͼ���� ��ͼ��ɾ�����޸ĵȲ����У���Ϊ����*/
uint32_t operate_tpye:3;/*ͼ�β��� 1 ���� 2�޸�ͼ�� 3ɾ������ͼ��*/  
uint32_t figure_tpye:3;/*ͼ������ 0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�*/ 
uint32_t layer:4;/*ͼ������0-9��*/  
uint32_t color:4;/*��ɫ 0 ��/�� 1 �� 2 �� 3 �� 4 �Ϻ� 5 �� 6 �� 7 �� 8 �� */  
uint32_t details_a:9; 
uint32_t details_b:9; /*ͼ��ϸ�ڲ��� ռ18���ֽ�*/
uint32_t width:10;/*�߿����������С���߿����Ϊ10��1 */  
uint32_t start_x:11;/*���/Բ��x���� */  
uint32_t start_y:11;/*���/Բ��y���� */
uint32_t details_c:10;  
uint32_t details_d:11;  
uint32_t details_e:11;  /*ͼ��ϸ�ڲ��� ռ32���ֽ�*/
}interaction_figure_t; 

/*�� 2-26 ͼ��ϸ�ڲ���˵�� 
���� details_a details_b details_c details_d       details_e 
ֱ��    - 				- 				- 			�յ�x����       �յ�y���� 
����	  - 				- 				-      �ԽǶ���x����    �ԽǶ���y����
��Բ    - 				-        �뾶        -                - 
��Բ	  - 				-				  - 			x���᳤�� 				y���᳤�� 
Բ�� ��ʼ�Ƕ�   ��ֹ�Ƕ�		  -			  x���᳤�� 				y���᳤�� 
������ �����С  ������				 ��ֵ����1000��ʵ����ʾֵ 
������ �����С    -							 32λ��������int32_t 
�ַ�   �����С �ַ����� 			- 				 - 								- 

�Ƕ�ֵ����Ϊ��0��ָ12���ӷ���˳ʱ����ƣ� 
��Ļλ�ã���0,0��Ϊ��Ļ���½ǣ�1920��1080��Ϊ��Ļ���Ͻǣ� 
����������������Ϊ32λ�����ڸ�������ʵ����ʾ��ֵΪ�����ֵ/1000������details_c��details_d��details_e��Ӧ���ֽ�����1234��ѡ�ֶ�ʵ����ʾ��ֵ��Ϊ1.234�� 
��ʹ���͵���ֵ������Ӧ�������͵����ƣ�ͼ�����п�����ʾ������ʱ����֤��ʾ��Ч���� 
*/
#define UI_DEFAULT { \
				.ADD_Char=&ADD_Character, \
				.ADD_7Graph=&ADD_7_Graph, \
				.MODIFY_7Graph_0=&MODIFY_7_Graph_DIY, \
				.MODIFY_7Graph_1=MODIFY_7_Graph_DIY1 \
} \

/*������ID��0x0102*/
typedef __packed struct/*��������ͼ��*/
{ 
  interaction_figure_t interaction_figure[2]; 
}interaction_figure_2_t;
/*������ID��0x0103*/
typedef __packed struct/*����5��ͼ��*/
{ 
interaction_figure_t interaction_figure[5]; 
}interaction_figure_3_t; 
/*������ID��0x0104*/
typedef __packed struct/*����7��ͼ��*/
{ 
interaction_figure_t interaction_figure[7]; 
}interaction_figure_4_t;

/*������ID��0x0110*/
typedef __packed struct 
{ 
interaction_figure_t  interaction_figure; 
uint8_t data[30]; 
}client_custom_character_t;
typedef __packed struct 
{
  uint16_t data_cmd_id; /*������ID ��Ϊ���ŵ�������ID*/
  uint16_t sender_id; /*������ID ��������IDƥ�䣬ID��������¼ */
  uint16_t receiver_id; /*������ID ���޼���ͨ�� ��Ϊ��������Ķ��ͨѶ������ ��������Ϊѡ�ֶˣ�����ɷ����������߶�Ӧ��ѡ�ֶ� ID��������¼*/
}id_data_t;
/*0x0301*/
typedef __packed struct 
{ 
	id_data_t id_data;
  uint8_t user_data[113]; /*�������ݶ� x���Ϊ113 */
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
