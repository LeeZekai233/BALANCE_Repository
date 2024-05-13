#include "client.h"
u8  draw_cnt=0,draw_int=0;
u16 draw_data_ID=0x0101;
u16 data_ID=0xD180;
u16 client_custom_ID=0;
uint8_t  ddata[150];
uint8_t  tx_buf[150];
int Energy_organs_flag=0;
u8 security_attacked;
u8 base_attacked;

ext_client_custom_character_t Friction_state;                                
ext_client_custom_character_t Level;                         
ext_client_custom_character_t cap;                 //
ext_client_custom_character_t speed_up;             

ext_client_custom_character_t Level_sight;
ext_client_custom_character_t security_or_base;

ext_client_custom_graphic_seven_t   cap_sight;               //������ʾ
ext_client_custom_graphic_seven_t		cap_ract;										 //�������ݿ�
ext_client_custom_graphic_seven_t   sight_bead;                  //׼��
ext_client_custom_graphic_seven_t   chassis_graphics;          //С����ͼ��


ext_client_custom_graphic_seven_t graphic1;
ext_client_custom_graphic_seven_t graphic2;

ext_client_custom_character_t Friction_state;                                
ext_client_custom_character_t Level;                         
ext_client_custom_character_t cap;                 //
ext_client_custom_character_t speed_up;  
ext_client_custom_character_t bullet_data;
ext_client_custom_character_t aim_normal;
ext_client_custom_character_t aim_rotate;
ext_client_custom_character_t small_buff;
ext_client_custom_character_t big_buff;

ext_client_custom_character_t Level_sight;
ext_client_custom_character_t security_or_base;
ext_client_custom_character_t bullet_sight;

#define START_POINT_X 0
#define START_POINT_Y -50
#define END_POINT_X   0
#define END_POINT_Y   50
#define RADIOS    20
#define WIDTH    2
#define OFFSET_X 1600
#define OFFSET_Y 600

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


int qwe,qwer,last_qwe,qwert;
int NX_time_flag;
double NX_time,NX_time_qwe;

void Client_send_handle()
{
  u8 id;
  id=judge_rece_mesg.game_robot_state.robot_id;

  switch(id)
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
    case 103://��ɫ
      client_custom_ID=0x0167;
      break;
    case 104:
      client_custom_ID=0x0168;
      break;
    case 105:
      client_custom_ID=0x0169;
      break;
    }


//************************************************************************************************************************************/
//////////////////////////////////////////////////////////////��ʼ��///////////////////////////////////////////////////////////////////
//************************************************************************************************************************************/
    switch(draw_cnt)
    {
        case 1:
        {
            ddata[0]=0x0104;
              ddata[1]=0x0104>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
            
            /*********************׼����ʾ****************************************/
              graphic1.grapic_data_struct[0].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
              graphic1.grapic_data_struct[0].layer=1;   //ͼ��
              graphic1.grapic_data_struct[0].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
              graphic1.grapic_data_struct[0].graphic_name[0]=0;
              graphic1.grapic_data_struct[0].graphic_name[1]=0;
              graphic1.grapic_data_struct[0].graphic_name[2]=1;
              graphic1.grapic_data_struct[0].start_x=960;
              graphic1.grapic_data_struct[0].start_y=540;
            if(shoot.poke_run)
			{
				graphic1.grapic_data_struct[0].color=UI_RB;
			}else
			{
				graphic1.grapic_data_struct[0].color=UI_GREEN;
			}
			graphic1.grapic_data_struct[0].radius=1;
            graphic1.grapic_data_struct[0].width=3;//��
            if(new_location.flag||My_Auto_Shoot.Auto_Aim.Flag_Get_Target)
			{
                  graphic1.grapic_data_struct[1].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                  graphic1.grapic_data_struct[1].layer=1;   //ͼ��
                  graphic1.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                  graphic1.grapic_data_struct[1].graphic_name[0]=0;
                  graphic1.grapic_data_struct[1].graphic_name[1]=0;
                  graphic1.grapic_data_struct[1].graphic_name[2]=2;
                  graphic1.grapic_data_struct[1].start_x=960;
                  graphic1.grapic_data_struct[1].start_y=540;
                    graphic1.grapic_data_struct[1].color=UI_YELLOW;
                    graphic1.grapic_data_struct[1].radius=3;
                    graphic1.grapic_data_struct[1].width=3;//��
			}else
			{
				graphic1.grapic_data_struct[1].operate_type=3;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                graphic1.grapic_data_struct[1].layer=1;   //ͼ��
                graphic1.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                graphic1.grapic_data_struct[1].graphic_name[0]=0;
                graphic1.grapic_data_struct[1].graphic_name[1]=0;
                graphic1.grapic_data_struct[1].graphic_name[2]=2;
			}
            /*********************����ģʽ��ʾ****************************************/
                    graphic1.grapic_data_struct[2].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[2].layer=1;   //ͼ��
					graphic1.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[2].graphic_name[0]=0;
					graphic1.grapic_data_struct[2].graphic_name[1]=0;
					graphic1.grapic_data_struct[2].graphic_name[2]=3;					
					graphic1.grapic_data_struct[2].color = UI_PINK;
					graphic1.grapic_data_struct[2].start_x = 1320;
					graphic1.grapic_data_struct[2].start_y = 108;
					graphic1.grapic_data_struct[2].end_x = 1320;
					graphic1.grapic_data_struct[2].end_y = 195;
					graphic1.grapic_data_struct[2].width = 3;
					
					graphic1.grapic_data_struct[3].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[3].layer=1;   //ͼ��
					graphic1.grapic_data_struct[3].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[3].graphic_name[0]=0;
					graphic1.grapic_data_struct[3].graphic_name[1]=0;
					graphic1.grapic_data_struct[3].graphic_name[2]=4;					
					graphic1.grapic_data_struct[3].color = UI_YELLOW;
					graphic1.grapic_data_struct[3].start_x = 1285;
					graphic1.grapic_data_struct[3].start_y = 143;
					graphic1.grapic_data_struct[3].end_x = 1354;
					graphic1.grapic_data_struct[3].end_y = 143;
					graphic1.grapic_data_struct[3].width = 4;
					
					graphic1.grapic_data_struct[4].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[4].layer=1;   //ͼ��
					graphic1.grapic_data_struct[4].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[4].graphic_name[0]=0;
					graphic1.grapic_data_struct[4].graphic_name[1]=0;
					graphic1.grapic_data_struct[4].graphic_name[2]=5;				
					graphic1.grapic_data_struct[4].color = UI_YELLOW;
					graphic1.grapic_data_struct[4].start_x = 1285;
					graphic1.grapic_data_struct[4].start_y = 85;
					graphic1.grapic_data_struct[4].end_x = 1354;
					graphic1.grapic_data_struct[4].end_y = 85;
					graphic1.grapic_data_struct[4].width = 4;
                    
           /*************************��ѹ����ʾ*******************************/
                  graphic1.grapic_data_struct[5].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                  graphic1.grapic_data_struct[5].layer=1;   //ͼ��
                  graphic1.grapic_data_struct[5].graphic_type=1;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                  graphic1.grapic_data_struct[5].graphic_name[0]=0;
                  graphic1.grapic_data_struct[5].graphic_name[1]=0;
                  graphic1.grapic_data_struct[5].graphic_name[2]=6;
                  graphic1.grapic_data_struct[5].start_x=583;
                  graphic1.grapic_data_struct[5].start_y=85;
                  graphic1.grapic_data_struct[5].end_x=1208;
                  graphic1.grapic_data_struct[5].end_y=135;
                  graphic1.grapic_data_struct[5].color=0;
                  graphic1.grapic_data_struct[5].width=2;
                  
          /*************************��������ʾ********************************/
                    graphic1.grapic_data_struct[6].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                    graphic1.grapic_data_struct[6].layer=1;   //ͼ��
                    graphic1.grapic_data_struct[6].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                    graphic1.grapic_data_struct[6].graphic_name[0]=0;
                    graphic1.grapic_data_struct[6].graphic_name[1]=0;
                    graphic1.grapic_data_struct[6].graphic_name[2]=7;                                             
                    graphic1.grapic_data_struct[6].start_x=600;
                    graphic1.grapic_data_struct[6].start_y=110;
                    graphic1.grapic_data_struct[6].end_y=110;
                        
                  if(usart_gimbal_data.cap_v>=0.0f&&usart_gimbal_data.cap_v<=7.0f)
                    {
                      graphic1.grapic_data_struct[6].end_x=600;
                      graphic1.grapic_data_struct[6].color=UI_PINK;
                    }
                  else if(usart_gimbal_data.cap_v>7.0f)
                    {
                      graphic1.grapic_data_struct[6].end_x=600+((int)(usart_gimbal_data.cap_v-7.0))*35.29f;
                      graphic1.grapic_data_struct[6].color=UI_PINK;
                    }

                    graphic1.grapic_data_struct[6].width=30;
                    
               *(ext_client_custom_graphic_seven_t*)(&ddata[6])=graphic1;
                data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(graphic1),DN_REG_ID,tx_buf);
                    
        }break;
        case 2:
        {
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			Friction_state.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			Friction_state.grapic_data_struct.layer=1;   //ͼ��
			Friction_state.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			Friction_state.grapic_data_struct.graphic_name[0]=0;
			Friction_state.grapic_data_struct.graphic_name[1]=0;
			Friction_state.grapic_data_struct.graphic_name[2]=8;
			
			Friction_state.grapic_data_struct.start_x=100;//55
				Friction_state.grapic_data_struct.start_y=540;//215
				Friction_state.grapic_data_struct.width=WIDTH;
				Friction_state.grapic_data_struct.start_angle=20;
				Friction_state.grapic_data_struct.end_angle=15;
      
				Friction_state.data[0] = 'F';
				Friction_state.data[1] = 'R';
				Friction_state.data[2] = 'I';
				Friction_state.data[3] = 'C';
				Friction_state.data[4] = 'T';
				Friction_state.data[5] = 'I';
				Friction_state.data[6] = 'O';
				Friction_state.data[7] = 'N';
                Friction_state.data[8] = ':';
				
				
			   Friction_state.grapic_data_struct.color = UI_YELLOW;
				
				
				
					
      *(ext_client_custom_character_t*)(&ddata[6])=Friction_state;
		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Friction_state),DN_REG_ID,tx_buf);
        }break;
        case 3:
        {
            ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			Level.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			Level.grapic_data_struct.layer=1;   //ͼ��
			Level.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			Level.grapic_data_struct.graphic_name[0]=0;
			Level.grapic_data_struct.graphic_name[1]=0;
			Level.grapic_data_struct.graphic_name[2]=9;
			
			Level.grapic_data_struct.start_x=100;
				Level.grapic_data_struct.start_y=600;
				Level.grapic_data_struct.width=WIDTH;
				Level.grapic_data_struct.start_angle=20;
				Level.grapic_data_struct.end_angle=6;
      
				Level.data[0] = 'L';
				Level.data[1] = 'E';
				Level.data[2] = 'V';
				Level.data[3] = 'E';
				Level.data[4] = 'L';
				Level.data[5] = ':';


					Level.grapic_data_struct.color = UI_YELLOW;
				
		
			
      *(ext_client_custom_character_t*)(&ddata[6])=Level;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Level),DN_REG_ID,tx_buf);
        }break;
       case 4:
        {
            	//----------------------------------����ģʽ----------------------------------------//
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			speed_up.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			speed_up.grapic_data_struct.layer=1;   //ͼ��
			speed_up.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			speed_up.grapic_data_struct.graphic_name[0]=0;
			speed_up.grapic_data_struct.graphic_name[1]=1;
			speed_up.grapic_data_struct.graphic_name[2]=0;
			
			speed_up.grapic_data_struct.start_x=100;
				speed_up.grapic_data_struct.start_y=570;
				speed_up.grapic_data_struct.width=WIDTH;
				speed_up.grapic_data_struct.start_angle=20;
				speed_up.grapic_data_struct.end_angle=7;
      
				speed_up.data[0] = 'S';
				speed_up.data[1] = 'P';
				speed_up.data[2] = 'E';
				speed_up.data[3] = 'E';
				speed_up.data[4] = 'D';
				speed_up.data[5] = ':';

				
				
                speed_up.grapic_data_struct.color = UI_YELLOW;
				
			
			
			
			
			
      *(ext_client_custom_character_t*)(&ddata[6])=speed_up;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(speed_up),DN_REG_ID,tx_buf);
        }break;
       case 5:
       {
           //----------------------------------������ʾ----------------------------------------//
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			bullet_data.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			bullet_data.grapic_data_struct.layer=1;   //ͼ��
			bullet_data.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			bullet_data.grapic_data_struct.graphic_name[0]=0;
			bullet_data.grapic_data_struct.graphic_name[1]=1;
			bullet_data.grapic_data_struct.graphic_name[2]=1;
			
                bullet_data.grapic_data_struct.start_x=100;
				bullet_data.grapic_data_struct.start_y=630;
				bullet_data.grapic_data_struct.width=WIDTH;
				bullet_data.grapic_data_struct.start_angle=20;
				bullet_data.grapic_data_struct.end_angle=7;
                
				bullet_data.data[0] = 'B';
				bullet_data.data[1] = 'U';
				bullet_data.data[2] = 'L';
				bullet_data.data[3] = 'L';
				bullet_data.data[4] = 'E';
				bullet_data.data[5] = 'T';
                bullet_data.data[6] = ':';

				
				
                bullet_data.grapic_data_struct.color = UI_YELLOW;
				
			
			
			
			
			
      *(ext_client_custom_character_t*)(&ddata[6])=bullet_data;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(bullet_data),DN_REG_ID,tx_buf);
       }break;
       case 6:
       {
           //----------------------------------����----------------------------------------//
          ddata[0]=0x0110;
          ddata[1]=0x0110>>8;	 //��������id
          //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
          ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
          ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
          ddata[4]=client_custom_ID;
          ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			Level_sight.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			Level_sight.grapic_data_struct.layer=1;   //ͼ��
			Level_sight.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			Level_sight.grapic_data_struct.graphic_name[0]=0;
			Level_sight.grapic_data_struct.graphic_name[1]=1;
			Level_sight.grapic_data_struct.graphic_name[2]=2;
			
				Level_sight.grapic_data_struct.start_x=230;
				Level_sight.grapic_data_struct.start_y=600;
				Level_sight.grapic_data_struct.width=4;
				Level_sight.grapic_data_struct.start_angle=20;
				Level_sight.grapic_data_struct.end_angle=4;
      
				sprintf(Level_sight.data,"%f",judge_rece_mesg.game_robot_state.robot_level*1.0f);


					Level_sight.grapic_data_struct.color = UI_RB;
							
      *(ext_client_custom_character_t*)(&ddata[6])=Level_sight;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Level_sight),DN_REG_ID,tx_buf);
       }break;
       case 7:
       {
            //----------------------------------����----------------------------------------//
          ddata[0]=0x0110;
          ddata[1]=0x0110>>8;	 //��������id
          //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
          ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
          ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
          ddata[4]=client_custom_ID;
          ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			bullet_sight.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			bullet_sight.grapic_data_struct.layer=1;   //ͼ��
			bullet_sight.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			bullet_sight.grapic_data_struct.graphic_name[0]=0;
			bullet_sight.grapic_data_struct.graphic_name[1]=1;
			bullet_sight.grapic_data_struct.graphic_name[2]=3;
			
            bullet_sight.grapic_data_struct.start_x=230;
            bullet_sight.grapic_data_struct.start_y=630;
            bullet_sight.grapic_data_struct.width=4;
            bullet_sight.grapic_data_struct.start_angle=20;
            bullet_sight.grapic_data_struct.end_angle=4;

            sprintf(bullet_sight.data,"%f",already_shoot*1.0f);


					bullet_sight.grapic_data_struct.color = UI_RB;
							
      *(ext_client_custom_character_t*)(&ddata[6])=bullet_sight;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(bullet_sight),DN_REG_ID,tx_buf);
       }break;
       
       //----------------------------------�Ӿ�ģʽ��ʾ----------------------------------------//
       
       case 8:
       {
             
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
			aim_normal.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
			aim_normal.grapic_data_struct.layer=1;   //ͼ��
			aim_normal.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
			aim_normal.grapic_data_struct.graphic_name[0]=0;
			aim_normal.grapic_data_struct.graphic_name[1]=1;
			aim_normal.grapic_data_struct.graphic_name[2]=4;
			
                aim_normal.grapic_data_struct.start_x=1700;
				aim_normal.grapic_data_struct.start_y=540;
				aim_normal.grapic_data_struct.width=WIDTH;
				aim_normal.grapic_data_struct.start_angle=20;
				aim_normal.grapic_data_struct.end_angle=7;
                
				aim_normal.data[0] = 'A';
				aim_normal.data[1] = 'I';
				aim_normal.data[2] = 'M';
				aim_normal.data[3] = '_';
				aim_normal.data[4] = 'N';
				aim_normal.data[5] = 'O';
                aim_normal.data[6] = 'R';
                aim_normal.data[7] = 'M';
                aim_normal.data[8] = 'A';
                aim_normal.data[9] = 'L';
				
				
                aim_normal.grapic_data_struct.color = UI_PINK;
				
			
			
			
			
			
      *(ext_client_custom_character_t*)(&ddata[6])=aim_normal;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(aim_normal),DN_REG_ID,tx_buf);
       }break;
       case 9:
       {
             
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
                aim_rotate.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                aim_rotate.grapic_data_struct.layer=1;   //ͼ��
                aim_rotate.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                aim_rotate.grapic_data_struct.graphic_name[0]=0;
                aim_rotate.grapic_data_struct.graphic_name[1]=1;
                aim_rotate.grapic_data_struct.graphic_name[2]=5;
			    
                aim_rotate.grapic_data_struct.start_x=1700;
				aim_rotate.grapic_data_struct.start_y=570;
				aim_rotate.grapic_data_struct.width=WIDTH;
				aim_rotate.grapic_data_struct.start_angle=20;
				aim_rotate.grapic_data_struct.end_angle=7;
                
				aim_rotate.data[0] = 'A';
				aim_rotate.data[1] = 'I';
				aim_rotate.data[2] = 'M';
				aim_rotate.data[3] = '_';
				aim_rotate.data[4] = 'R';
				aim_rotate.data[5] = 'O';
                aim_rotate.data[6] = 'T';
                aim_rotate.data[7] = 'A';
                aim_rotate.data[8] = 'T';
                aim_rotate.data[9] = 'E';
			
                aim_rotate.grapic_data_struct.color = UI_PINK;
				
			
			
			
			
			
      *(ext_client_custom_character_t*)(&ddata[6])=aim_rotate;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(aim_rotate),DN_REG_ID,tx_buf);
       }break;
       case 10:
       {
             
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
                small_buff.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                small_buff.grapic_data_struct.layer=1;   //ͼ��
                small_buff.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                small_buff.grapic_data_struct.graphic_name[0]=0;
                small_buff.grapic_data_struct.graphic_name[1]=1;
                small_buff.grapic_data_struct.graphic_name[2]=6;
			    
                small_buff.grapic_data_struct.start_x=1700;
				small_buff.grapic_data_struct.start_y=600;
				small_buff.grapic_data_struct.width=WIDTH;
				small_buff.grapic_data_struct.start_angle=20;
				small_buff.grapic_data_struct.end_angle=7;
                
				small_buff.data[0] = 'S';
				small_buff.data[1] = 'M';
				small_buff.data[2] = 'A';
				small_buff.data[3] = 'L';
				small_buff.data[4] = 'L';
				small_buff.data[5] = '_';
                small_buff.data[6] = 'B';
                small_buff.data[7] = 'U';
                small_buff.data[8] = 'F';
                small_buff.data[9] = 'F';
			    
                small_buff.grapic_data_struct.color = UI_PINK;
				
			
			
			
			
			
      *(ext_client_custom_character_t*)(&ddata[6])=small_buff;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(small_buff),DN_REG_ID,tx_buf);
       }break;
       case 11:
       {
             
              ddata[0]=0x0110;
              ddata[1]=0x0110>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
			
                big_buff.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                big_buff.grapic_data_struct.layer=1;   //ͼ��
                big_buff.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                big_buff.grapic_data_struct.graphic_name[0]=0;
                big_buff.grapic_data_struct.graphic_name[1]=1;
                big_buff.grapic_data_struct.graphic_name[2]=7;
			    
                big_buff.grapic_data_struct.start_x=1700;
				big_buff.grapic_data_struct.start_y=630;
				big_buff.grapic_data_struct.width=WIDTH;
				big_buff.grapic_data_struct.start_angle=20;
				big_buff.grapic_data_struct.end_angle=7;
                
				big_buff.data[0] = 'B';
				big_buff.data[1] = 'I';
				big_buff.data[2] = 'G';
				big_buff.data[3] = '_';
				big_buff.data[4] = 'B';
				big_buff.data[5] = 'U';
                big_buff.data[6] = 'F';
                big_buff.data[7] = 'F';
                
			    
                big_buff.grapic_data_struct.color = UI_PINK;
				
			
			
			
			
			
      *(ext_client_custom_character_t*)(&ddata[6])=big_buff;
      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(big_buff),DN_REG_ID,tx_buf);
       }break;
       case 12:
       {
              ddata[0]=0x0104;
              ddata[1]=0x0104>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
                //Ħ����
              graphic2.grapic_data_struct[0].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
              graphic2.grapic_data_struct[0].layer=1;   //ͼ��
              graphic2.grapic_data_struct[0].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
              graphic2.grapic_data_struct[0].graphic_name[0]=0;
              graphic2.grapic_data_struct[0].graphic_name[1]=1;
              graphic2.grapic_data_struct[0].graphic_name[2]=8;
              graphic2.grapic_data_struct[0].start_x=300;
              graphic2.grapic_data_struct[0].start_y=530;
            if(shoot.fric_wheel_run==1)
			{
				graphic2.grapic_data_struct[0].color=UI_RB;
			}else
			{
				graphic2.grapic_data_struct[0].color=UI_YELLOW;
			}
			graphic2.grapic_data_struct[0].radius=9;
            graphic2.grapic_data_struct[0].width=5;//��
              //����
              graphic2.grapic_data_struct[1].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
              graphic2.grapic_data_struct[1].layer=1;   //ͼ��
              graphic2.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
              graphic2.grapic_data_struct[1].graphic_name[0]=0;
              graphic2.grapic_data_struct[1].graphic_name[1]=1;
              graphic2.grapic_data_struct[1].graphic_name[2]=9;
              graphic2.grapic_data_struct[1].start_x=300;
              graphic2.grapic_data_struct[1].start_y=560;
            if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag==1)
			{
				graphic2.grapic_data_struct[1].color=UI_RB;
			}else
			{
				graphic2.grapic_data_struct[1].color=UI_YELLOW;
			}
			graphic2.grapic_data_struct[1].radius=9;
            graphic2.grapic_data_struct[1].width=5;//��
            //�Ӿ�ģʽ
            switch(gimbal_data.vision_mode)
            {
                case AIM_NORMAL:
                {
                      graphic2.grapic_data_struct[2].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic2.grapic_data_struct[2].layer=1;   //ͼ��
                      graphic2.grapic_data_struct[2].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic2.grapic_data_struct[2].graphic_name[0]=0;
                      graphic2.grapic_data_struct[2].graphic_name[1]=2;
                      graphic2.grapic_data_struct[2].graphic_name[2]=0;
                      graphic2.grapic_data_struct[2].start_x=1650;
                      graphic2.grapic_data_struct[2].start_y=530;
                      graphic2.grapic_data_struct[2].color=UI_RB;
                      graphic2.grapic_data_struct[2].radius=9;
                      graphic2.grapic_data_struct[2].width=5;//��
                    
                }break;
                case AIM_ROTATE:
                {
                    graphic2.grapic_data_struct[2].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic2.grapic_data_struct[2].layer=1;   //ͼ��
                      graphic2.grapic_data_struct[2].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic2.grapic_data_struct[2].graphic_name[0]=0;
                      graphic2.grapic_data_struct[2].graphic_name[1]=2;
                      graphic2.grapic_data_struct[2].graphic_name[2]=0;
                      graphic2.grapic_data_struct[2].start_x=1650;
                      graphic2.grapic_data_struct[2].start_y=560;
                      graphic2.grapic_data_struct[2].color=UI_RB;
                      graphic2.grapic_data_struct[2].radius=9;
                      graphic2.grapic_data_struct[2].width=5;//��
                }break;
                case SMALL_BUFF:
                {
                    graphic2.grapic_data_struct[2].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic2.grapic_data_struct[2].layer=1;   //ͼ��
                      graphic2.grapic_data_struct[2].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic2.grapic_data_struct[2].graphic_name[0]=0;
                      graphic2.grapic_data_struct[2].graphic_name[1]=2;
                      graphic2.grapic_data_struct[2].graphic_name[2]=0;
                      graphic2.grapic_data_struct[2].start_x=1650;
                      graphic2.grapic_data_struct[2].start_y=590;
                      graphic2.grapic_data_struct[2].color=UI_RB;
                      graphic2.grapic_data_struct[2].radius=9;
                      graphic2.grapic_data_struct[2].width=5;//��
                }break;
                case BIG_BUFF:
                {
                    graphic2.grapic_data_struct[2].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic2.grapic_data_struct[2].layer=1;   //ͼ��
                      graphic2.grapic_data_struct[2].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic2.grapic_data_struct[2].graphic_name[0]=0;
                      graphic2.grapic_data_struct[2].graphic_name[1]=2;
                      graphic2.grapic_data_struct[2].graphic_name[2]=0;
                      graphic2.grapic_data_struct[2].start_x=1650;
                      graphic2.grapic_data_struct[2].start_y=620;
                      graphic2.grapic_data_struct[2].color=UI_RB;
                      graphic2.grapic_data_struct[2].radius=9;
                      graphic2.grapic_data_struct[2].width=5;//��
                }break;
            }
            *(ext_client_custom_graphic_seven_t*)(&ddata[6])=graphic2;
            data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(graphic2),DN_REG_ID,tx_buf);
              
       }break;
       
       
//************************************************************************************************************************************/
//////////////////////////////////////////////////////////////ˢ��UI///////////////////////////////////////////////////////////////////
//************************************************************************************************************************************/
                case 13:
                {
                                //----------------------------------����----------------------------------------//
                      ddata[0]=0x0110;
                      ddata[1]=0x0110>>8;	 //��������id
                      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
                      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
                      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
                      ddata[4]=client_custom_ID;
                      ddata[5]=client_custom_ID>>8;       //�ͻ���id
                        
                        Level_sight.grapic_data_struct.operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                        Level_sight.grapic_data_struct.layer=1;   //ͼ��
                        Level_sight.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                        Level_sight.grapic_data_struct.graphic_name[0]=0;
                        Level_sight.grapic_data_struct.graphic_name[1]=1;
                        Level_sight.grapic_data_struct.graphic_name[2]=2;
                        
                            Level_sight.grapic_data_struct.start_x=230;
                            Level_sight.grapic_data_struct.start_y=600;
                            Level_sight.grapic_data_struct.width=4;
                            Level_sight.grapic_data_struct.start_angle=20;
                            Level_sight.grapic_data_struct.end_angle=4;
                  
                            sprintf(Level_sight.data,"%f",judge_rece_mesg.game_robot_state.robot_level*1.0f);


                                Level_sight.grapic_data_struct.color = UI_RB;
                                        
                  *(ext_client_custom_character_t*)(&ddata[6])=Level_sight;
                  data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Level_sight),DN_REG_ID,tx_buf);
                }break;
                case 14:
                {
                    ddata[0]=0x0104;
              ddata[1]=0x0104>>8;	 //��������id
              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
              ddata[4]=client_custom_ID;
              ddata[5]=client_custom_ID>>8;       //�ͻ���id
            
            /*********************׼����ʾ****************************************/
              graphic1.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
              graphic1.grapic_data_struct[0].layer=1;   //ͼ��
              graphic1.grapic_data_struct[0].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
              graphic1.grapic_data_struct[0].graphic_name[0]=0;
              graphic1.grapic_data_struct[0].graphic_name[1]=0;
              graphic1.grapic_data_struct[0].graphic_name[2]=1;
              graphic1.grapic_data_struct[0].start_x=960;
              graphic1.grapic_data_struct[0].start_y=540;
            if(shoot.poke_run)
			{
				graphic1.grapic_data_struct[0].color=UI_RB;
			}else
			{
				graphic1.grapic_data_struct[0].color=UI_GREEN;
			}
			graphic1.grapic_data_struct[0].radius=1;
            graphic1.grapic_data_struct[0].width=3;//��
            if(new_location.flag||My_Auto_Shoot.Auto_Aim.Flag_Get_Target)
			{
                  graphic1.grapic_data_struct[1].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                  graphic1.grapic_data_struct[1].layer=1;   //ͼ��
                  graphic1.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                  graphic1.grapic_data_struct[1].graphic_name[0]=0;
                  graphic1.grapic_data_struct[1].graphic_name[1]=0;
                  graphic1.grapic_data_struct[1].graphic_name[2]=2;
                  graphic1.grapic_data_struct[1].start_x=960;
                  graphic1.grapic_data_struct[1].start_y=540;
                    graphic1.grapic_data_struct[1].color=UI_YELLOW;
                    graphic1.grapic_data_struct[1].radius=3;
                    graphic1.grapic_data_struct[1].width=3;//��
			}else
			{
				graphic1.grapic_data_struct[1].operate_type=3;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                graphic1.grapic_data_struct[1].layer=1;   //ͼ��
                graphic1.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                graphic1.grapic_data_struct[1].graphic_name[0]=0;
                graphic1.grapic_data_struct[1].graphic_name[1]=0;
                graphic1.grapic_data_struct[1].graphic_name[2]=2;
			}
            /*********************����ģʽ��ʾ****************************************/
                    graphic1.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[2].layer=1;   //ͼ��
					graphic1.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[2].graphic_name[0]=0;
					graphic1.grapic_data_struct[2].graphic_name[1]=0;
					graphic1.grapic_data_struct[2].graphic_name[2]=3;					
					graphic1.grapic_data_struct[2].color = UI_PINK;
					graphic1.grapic_data_struct[2].start_x = 1320;
					graphic1.grapic_data_struct[2].start_y = 108;
					graphic1.grapic_data_struct[2].end_x = 1320;
					graphic1.grapic_data_struct[2].end_y = 195;
					graphic1.grapic_data_struct[2].width = 3;
					
					graphic1.grapic_data_struct[3].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[3].layer=1;   //ͼ��
					graphic1.grapic_data_struct[3].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[3].graphic_name[0]=0;
					graphic1.grapic_data_struct[3].graphic_name[1]=0;
					graphic1.grapic_data_struct[3].graphic_name[2]=4;					
					graphic1.grapic_data_struct[3].color = UI_YELLOW;
					graphic1.grapic_data_struct[3].start_x = 1285;
					graphic1.grapic_data_struct[3].start_y = 143;
					graphic1.grapic_data_struct[3].end_x = 1354;
					graphic1.grapic_data_struct[3].end_y = 143;
					graphic1.grapic_data_struct[3].width = 4;
					
					graphic1.grapic_data_struct[4].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[4].layer=1;   //ͼ��
					graphic1.grapic_data_struct[4].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[4].graphic_name[0]=0;
					graphic1.grapic_data_struct[4].graphic_name[1]=0;
					graphic1.grapic_data_struct[4].graphic_name[2]=5;				
					graphic1.grapic_data_struct[4].color = UI_YELLOW;
					graphic1.grapic_data_struct[4].start_x = 1285;
					graphic1.grapic_data_struct[4].start_y = 85;
					graphic1.grapic_data_struct[4].end_x = 1354;
					graphic1.grapic_data_struct[4].end_y = 85;
					graphic1.grapic_data_struct[4].width = 4;
                    
                    switch(chassis.ctrl_mode)
			{
				case MANUAL_FOLLOW_GIMBAL:
				{
					graphic1.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[2].layer=1;   //ͼ��
					graphic1.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[2].graphic_name[0]=0;
					graphic1.grapic_data_struct[2].graphic_name[1]=0;
					graphic1.grapic_data_struct[2].graphic_name[2]=3;
					
					graphic1.grapic_data_struct[2].color = UI_PINK;
					graphic1.grapic_data_struct[2].start_x = 1320;
					graphic1.grapic_data_struct[2].start_y = 108;
					graphic1.grapic_data_struct[2].end_x = 1320;
					graphic1.grapic_data_struct[2].end_y = 195;
					graphic1.grapic_data_struct[2].width = 3;
					
					graphic1.grapic_data_struct[3].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[3].layer=1;   //ͼ��
					graphic1.grapic_data_struct[3].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[3].graphic_name[0]=0;
					graphic1.grapic_data_struct[3].graphic_name[1]=0;
					graphic1.grapic_data_struct[3].graphic_name[2]=4;
					
					graphic1.grapic_data_struct[3].color = UI_YELLOW;
					graphic1.grapic_data_struct[3].start_x = 1285;
					graphic1.grapic_data_struct[3].start_y = 143;
					graphic1.grapic_data_struct[3].end_x = 1354;
					graphic1.grapic_data_struct[3].end_y = 143;
					graphic1.grapic_data_struct[3].width = 4;
					
					graphic1.grapic_data_struct[4].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[4].layer=1;   //ͼ��
					graphic1.grapic_data_struct[4].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[4].graphic_name[0]=0;
					graphic1.grapic_data_struct[4].graphic_name[1]=0;
					graphic1.grapic_data_struct[4].graphic_name[2]=5;
					
					graphic1.grapic_data_struct[4].color = UI_YELLOW;
					graphic1.grapic_data_struct[4].start_x = 1285;
					graphic1.grapic_data_struct[4].start_y = 85;
					graphic1.grapic_data_struct[4].end_x = 1354;
					graphic1.grapic_data_struct[4].end_y = 85;
					graphic1.grapic_data_struct[4].width = 4;
					
				}break;
				case CHASSIS_REVERSE:
				{
					graphic1.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[2].layer=1;   //ͼ��
					graphic1.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[2].graphic_name[0]=0;
					graphic1.grapic_data_struct[2].graphic_name[1]=0;
					graphic1.grapic_data_struct[2].graphic_name[2]=3;
					graphic1.grapic_data_struct[2].color = UI_PINK;
					graphic1.grapic_data_struct[2].start_x = 1320;
					graphic1.grapic_data_struct[2].start_y = 108;
					graphic1.grapic_data_struct[2].end_x = 1320;
					graphic1.grapic_data_struct[2].end_y = 195;
					graphic1.grapic_data_struct[2].width = 3;
					
					graphic1.grapic_data_struct[3].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[3].layer=1;   //ͼ��
					graphic1.grapic_data_struct[3].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[3].graphic_name[0]=0;
					graphic1.grapic_data_struct[3].graphic_name[1]=0;
					graphic1.grapic_data_struct[3].graphic_name[2]=4;
					graphic1.grapic_data_struct[3].color = UI_YELLOW;
					graphic1.grapic_data_struct[3].start_x = 1285;
					graphic1.grapic_data_struct[3].start_y = 74;
					graphic1.grapic_data_struct[3].end_x = 1285;
					graphic1.grapic_data_struct[3].end_y = 146;
					graphic1.grapic_data_struct[3].width = 4;
					
					graphic1.grapic_data_struct[4].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[4].layer=1;   //ͼ��
					graphic1.grapic_data_struct[4].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[4].graphic_name[0]=0;
					graphic1.grapic_data_struct[4].graphic_name[1]=0;
					graphic1.grapic_data_struct[4].graphic_name[2]=5;
					graphic1.grapic_data_struct[4].color = UI_YELLOW;
					graphic1.grapic_data_struct[4].start_x = 1355;
					graphic1.grapic_data_struct[4].start_y = 74;
					graphic1.grapic_data_struct[4].end_x = 1355;
					graphic1.grapic_data_struct[4].end_y = 146;
					graphic1.grapic_data_struct[4].width = 4;
				}break;
				case CHASSIS_ROTATE:
				{
					graphic1.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[2].layer=1;   //ͼ��
					graphic1.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[2].graphic_name[0]=0;
					graphic1.grapic_data_struct[2].graphic_name[1]=0;
					graphic1.grapic_data_struct[2].graphic_name[2]=3;
					graphic1.grapic_data_struct[2].color = UI_PINK;
					graphic1.grapic_data_struct[2].start_x = 1320;
					graphic1.grapic_data_struct[2].start_y = 108;
					graphic1.grapic_data_struct[2].end_x = 1320;
					graphic1.grapic_data_struct[2].end_y = 195;
					graphic1.grapic_data_struct[2].width = 3;
					
					graphic1.grapic_data_struct[3].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[3].layer=1;   //ͼ��
					graphic1.grapic_data_struct[3].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[3].graphic_name[0]=0;
					graphic1.grapic_data_struct[3].graphic_name[1]=0;
					graphic1.grapic_data_struct[3].graphic_name[2]=4;
					graphic1.grapic_data_struct[3].start_x=1320;
					graphic1.grapic_data_struct[3].start_y=108;
					graphic1.grapic_data_struct[3].color=UI_YELLOW;
					graphic1.grapic_data_struct[3].radius=49;
					graphic1.grapic_data_struct[3].width=4;//��
					
					graphic1.grapic_data_struct[4].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
					graphic1.grapic_data_struct[4].layer=1;   //ͼ��
					graphic1.grapic_data_struct[4].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
					graphic1.grapic_data_struct[4].graphic_name[0]=0;
					graphic1.grapic_data_struct[4].graphic_name[1]=0;
					graphic1.grapic_data_struct[4].graphic_name[2]=5;
					graphic1.grapic_data_struct[4].color = UI_PINK;
					graphic1.grapic_data_struct[4].start_x = 1279;
					graphic1.grapic_data_struct[4].start_y = 108;
					graphic1.grapic_data_struct[4].end_x = 1368;
					graphic1.grapic_data_struct[4].end_y = 108;
					graphic1.grapic_data_struct[4].width = 3;
				}break;
			}
                    
           /*************************�Ӿ���ʾ*******************************/
                  //�Ӿ�ģʽ
            switch(gimbal_data.vision_mode)
            {
                case AIM_NORMAL:
                {
                      graphic1.grapic_data_struct[5].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic1.grapic_data_struct[5].layer=1;   //ͼ��
                      graphic1.grapic_data_struct[5].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic1.grapic_data_struct[5].graphic_name[0]=0;
                      graphic1.grapic_data_struct[5].graphic_name[1]=2;
                      graphic1.grapic_data_struct[5].graphic_name[2]=0;
                      graphic1.grapic_data_struct[5].start_x=1650;
                      graphic1.grapic_data_struct[5].start_y=530;
                      graphic1.grapic_data_struct[5].color=UI_RB;
                      graphic1.grapic_data_struct[5].radius=9;
                      graphic1.grapic_data_struct[5].width=5;//��
                    
                }break;
                case AIM_ROTATE:
                {
                      graphic1.grapic_data_struct[5].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic1.grapic_data_struct[5].layer=1;   //ͼ��
                      graphic1.grapic_data_struct[5].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic1.grapic_data_struct[5].graphic_name[0]=0;
                      graphic1.grapic_data_struct[5].graphic_name[1]=2;
                      graphic1.grapic_data_struct[5].graphic_name[2]=0;
                      graphic1.grapic_data_struct[5].start_x=1650;
                      graphic1.grapic_data_struct[5].start_y=560;
                      graphic1.grapic_data_struct[5].color=UI_RB;
                      graphic1.grapic_data_struct[5].radius=9;
                      graphic1.grapic_data_struct[5].width=5;//��
                }break;
                case SMALL_BUFF:
                {
                      graphic1.grapic_data_struct[5].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic1.grapic_data_struct[5].layer=1;   //ͼ��
                      graphic1.grapic_data_struct[5].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic1.grapic_data_struct[5].graphic_name[0]=0;
                      graphic1.grapic_data_struct[5].graphic_name[1]=2;
                      graphic1.grapic_data_struct[5].graphic_name[2]=0;
                      graphic1.grapic_data_struct[5].start_x=1650;
                      graphic1.grapic_data_struct[5].start_y=590;
                      graphic1.grapic_data_struct[5].color=UI_RB;
                      graphic1.grapic_data_struct[5].radius=9;
                      graphic1.grapic_data_struct[5].width=5;//��
                }break;
                case BIG_BUFF:
                {
                      graphic1.grapic_data_struct[5].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic1.grapic_data_struct[5].layer=1;   //ͼ��
                      graphic1.grapic_data_struct[5].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic1.grapic_data_struct[5].graphic_name[0]=0;
                      graphic1.grapic_data_struct[5].graphic_name[1]=2;
                      graphic1.grapic_data_struct[5].graphic_name[2]=0;
                      graphic1.grapic_data_struct[5].start_x=1650;
                      graphic1.grapic_data_struct[5].start_y=620;
                      graphic1.grapic_data_struct[5].color=UI_RB;
                      graphic1.grapic_data_struct[5].radius=9;
                      graphic1.grapic_data_struct[5].width=5;//��
                }break;
            }
                  
          /*************************��������ʾ********************************/
                    graphic1.grapic_data_struct[6].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                    graphic1.grapic_data_struct[6].layer=1;   //ͼ��
                    graphic1.grapic_data_struct[6].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                    graphic1.grapic_data_struct[6].graphic_name[0]=0;
                    graphic1.grapic_data_struct[6].graphic_name[1]=0;
                    graphic1.grapic_data_struct[6].graphic_name[2]=7;                                             
                    graphic1.grapic_data_struct[6].start_x=600;
                    graphic1.grapic_data_struct[6].start_y=110;
                    graphic1.grapic_data_struct[6].end_y=110;
                        
                  if(usart_gimbal_data.cap_v>=0.0f&&usart_gimbal_data.cap_v<=7.0f)
                    {
                      graphic1.grapic_data_struct[6].end_x=600;
                      graphic1.grapic_data_struct[6].color=UI_PINK;
                    }
                  else if(usart_gimbal_data.cap_v>7.0f)
                    {
                      graphic1.grapic_data_struct[6].end_x=600+((int)(usart_gimbal_data.cap_v-7.0))*35.29f;
                      graphic1.grapic_data_struct[6].color=UI_PINK;
                    }

                    graphic1.grapic_data_struct[6].width=30;
                    
               *(ext_client_custom_graphic_seven_t*)(&ddata[6])=graphic1;
                data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(graphic1),DN_REG_ID,tx_buf);
                }break;
                case 15:
                {
                      ddata[0]=0x0104;
                      ddata[1]=0x0104>>8;	 //��������id
                      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
                      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
                      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
                      ddata[4]=client_custom_ID;
                      ddata[5]=client_custom_ID>>8;       //�ͻ���id
                        //Ħ����
                      graphic2.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic2.grapic_data_struct[0].layer=1;   //ͼ��
                      graphic2.grapic_data_struct[0].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic2.grapic_data_struct[0].graphic_name[0]=0;
                      graphic2.grapic_data_struct[0].graphic_name[1]=1;
                      graphic2.grapic_data_struct[0].graphic_name[2]=8;
                      graphic2.grapic_data_struct[0].start_x=300;
                      graphic2.grapic_data_struct[0].start_y=530;
                    if(shoot.fric_wheel_run==1)
                    {
                        graphic2.grapic_data_struct[0].color=UI_RB;
                    }else
                    {
                        graphic2.grapic_data_struct[0].color=UI_YELLOW;
                    }
                    graphic2.grapic_data_struct[0].radius=9;
                    graphic2.grapic_data_struct[0].width=5;//��
                      //����
                      graphic2.grapic_data_struct[1].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                      graphic2.grapic_data_struct[1].layer=1;   //ͼ��
                      graphic2.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                      graphic2.grapic_data_struct[1].graphic_name[0]=0;
                      graphic2.grapic_data_struct[1].graphic_name[1]=1;
                      graphic2.grapic_data_struct[1].graphic_name[2]=9;
                      graphic2.grapic_data_struct[1].start_x=300;
                      graphic2.grapic_data_struct[1].start_y=560;
                    if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag==1)
                    {
                        graphic2.grapic_data_struct[1].color=UI_RB;
                    }else
                    {
                        graphic2.grapic_data_struct[1].color=UI_YELLOW;
                    }
                    graphic2.grapic_data_struct[1].radius=9;
                    graphic2.grapic_data_struct[1].width=5;//��
                    *(ext_client_custom_graphic_seven_t*)(&ddata[6])=graphic2;
                data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(graphic2),DN_REG_ID,tx_buf);
                }break;
                case 16:
                {
                      
                  
                  
                  //----------------------------------����----------------------------------------//
                      ddata[0]=0x0110;
                      ddata[1]=0x0110>>8;	 //��������id
                      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
                      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
                      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
                      ddata[4]=client_custom_ID;
                      ddata[5]=client_custom_ID>>8;       //�ͻ���id
                        
                        bullet_sight.grapic_data_struct.operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
                        bullet_sight.grapic_data_struct.layer=1;   //ͼ��
                        bullet_sight.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
                        bullet_sight.grapic_data_struct.graphic_name[0]=0;
                        bullet_sight.grapic_data_struct.graphic_name[1]=1;
                        bullet_sight.grapic_data_struct.graphic_name[2]=3;
                        
                        bullet_sight.grapic_data_struct.start_x=230;
                        bullet_sight.grapic_data_struct.start_y=630;
                        bullet_sight.grapic_data_struct.width=4;
                        bullet_sight.grapic_data_struct.start_angle=20;
                        bullet_sight.grapic_data_struct.end_angle=4;

                        sprintf(bullet_sight.data,"%f",already_shoot*1.0f);


                                bullet_sight.grapic_data_struct.color = UI_RB;
                                        
                  *(ext_client_custom_character_t*)(&ddata[6])=bullet_sight;
                  data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(bullet_sight),DN_REG_ID,tx_buf);
                }break;
                
    }
    
    draw_cnt++;
	draw_int++;
 
  if(draw_cnt>16)//����Ҫˢ�µ�ͼ��ˢ��
  {
      if(time_tick%1000 == 0)
          draw_cnt=13;
      else
          draw_cnt=14;
  }
	
	
	if(RC_CtrlData.Key_Flag.Key_R_Flag)
	{
		draw_cnt = 1;
	}
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
  
    
////************************************************************************************************************************************/
////////////////////////////////////////////////////////////////��ʼ��///////////////////////////////////////////////////////////////////
////************************************************************************************************************************************/
//	switch(draw_cnt)
//	{
//		case 1:   //׼�Ǻ����                   1-3
//    {
//      ddata[0]=0x0104;
//      ddata[1]=0x0104>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id


//      /*********************׼����ʾ****************************************/
//      sight_bead.grapic_data_struct[0].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      sight_bead.grapic_data_struct[0].layer=1;   //ͼ��
//      sight_bead.grapic_data_struct[0].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      sight_bead.grapic_data_struct[0].graphic_name[0]=0;
//      sight_bead.grapic_data_struct[0].graphic_name[1]=0;
//      sight_bead.grapic_data_struct[0].graphic_name[2]=1;
//      sight_bead.grapic_data_struct[0].start_x=960;
//      sight_bead.grapic_data_struct[0].start_y=540;
//			if(shoot.poke_run)
//			{
//				sight_bead.grapic_data_struct[0].color=UI_RB;
//			}else
//			{
//				sight_bead.grapic_data_struct[0].color=UI_GREEN;
//			}
//      
//			sight_bead.grapic_data_struct[0].radius=1;
//      sight_bead.grapic_data_struct[0].width=3;//��
//			
//			
//			if(new_location.flag||My_Auto_Shoot.Auto_Aim.Flag_Get_Target)
//			{
//				sight_bead.grapic_data_struct[1].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      sight_bead.grapic_data_struct[1].layer=1;   //ͼ��
//      sight_bead.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      sight_bead.grapic_data_struct[1].graphic_name[0]=1;
//      sight_bead.grapic_data_struct[1].graphic_name[1]=0;
//      sight_bead.grapic_data_struct[1].graphic_name[2]=0;
//      sight_bead.grapic_data_struct[1].start_x=960;
//      sight_bead.grapic_data_struct[1].start_y=540;

//			sight_bead.grapic_data_struct[1].color=UI_YELLOW;
//			
//			sight_bead.grapic_data_struct[1].radius=3;
//      sight_bead.grapic_data_struct[1].width=3;//��
//			}else
//			{
//				sight_bead.grapic_data_struct[1].operate_type=3;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      sight_bead.grapic_data_struct[1].layer=1;   //ͼ��
//      sight_bead.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      sight_bead.grapic_data_struct[1].graphic_name[0]=1;
//      sight_bead.grapic_data_struct[1].graphic_name[1]=0;
//      sight_bead.grapic_data_struct[1].graphic_name[2]=0;
//			}
//			
//         

//      *(ext_client_custom_graphic_seven_t*)(&ddata[6])=sight_bead;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(sight_bead),DN_REG_ID,tx_buf);
//    }break;
//		case 2:   //С����                       20
//		{
//				ddata[0]=0x0104;
//				ddata[1]=0x0104>>8;	 //��������id
//				//0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//				ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//				ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//				ddata[4]=client_custom_ID;
//				ddata[5]=client_custom_ID>>8;       //�ͻ���id
//					//*************************�Ƿ���С����*******************************//
//			
//					chassis_graphics.grapic_data_struct[0].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[0].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[0].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[0].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[2]=2;
//					
//					chassis_graphics.grapic_data_struct[0].color = UI_PINK;
//					chassis_graphics.grapic_data_struct[0].start_x = 1320;
//					chassis_graphics.grapic_data_struct[0].start_y = 108;
//					chassis_graphics.grapic_data_struct[0].end_x = 1320;
//					chassis_graphics.grapic_data_struct[0].end_y = 195;
//					chassis_graphics.grapic_data_struct[0].width = 3;
//					
//					
//					chassis_graphics.grapic_data_struct[1].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[1].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[1].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[1].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[2]=3;
//					
//					chassis_graphics.grapic_data_struct[1].color = UI_YELLOW;
//					chassis_graphics.grapic_data_struct[1].start_x = 1285;
//					chassis_graphics.grapic_data_struct[1].start_y = 143;
//					chassis_graphics.grapic_data_struct[1].end_x = 1354;
//					chassis_graphics.grapic_data_struct[1].end_y = 143;
//					chassis_graphics.grapic_data_struct[1].width = 4;
//					
//					
//					chassis_graphics.grapic_data_struct[2].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[2].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[2].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[2]=4;
//					
//					chassis_graphics.grapic_data_struct[2].color = UI_YELLOW;
//					chassis_graphics.grapic_data_struct[2].start_x = 1285;
//					chassis_graphics.grapic_data_struct[2].start_y = 85;
//					chassis_graphics.grapic_data_struct[2].end_x = 1354;
//					chassis_graphics.grapic_data_struct[2].end_y = 85;
//					chassis_graphics.grapic_data_struct[2].width = 4;
//				
//				

//				
//				
//			

//		*(ext_client_custom_graphic_seven_t*)(&ddata[6])=chassis_graphics;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(chassis_graphics),DN_REG_ID,tx_buf);
//		}break;
//    case 3:   //��ѹ��                       4
//    {
//			ddata[0]=0x0104;
//      ddata[1]=0x0104>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id

//      /*************************������ʾ*******************************/
//      cap_ract.grapic_data_struct[0].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      cap_ract.grapic_data_struct[0].layer=1;   //ͼ��
//      cap_ract.grapic_data_struct[0].graphic_type=1;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      cap_ract.grapic_data_struct[0].graphic_name[0]=0;
//      cap_ract.grapic_data_struct[0].graphic_name[1]=0;
//      cap_ract.grapic_data_struct[0].graphic_name[2]=5;

//      cap_ract.grapic_data_struct[0].start_x=583;
//      cap_ract.grapic_data_struct[0].start_y=85;
//      cap_ract.grapic_data_struct[0].end_x=1208;
//			cap_ract.grapic_data_struct[0].end_y=135;
//      cap_ract.grapic_data_struct[0].color=0;
//			cap_ract.grapic_data_struct[0].width=2;
//				
//      *(ext_client_custom_graphic_seven_t*)(&ddata[6])=cap_ract;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(cap_ract),DN_REG_ID,tx_buf);
//     }break;
//		case 4:   //��ѹֵ                       5
//		{
//			ddata[0]=0x0110;
//				ddata[1]=0x0110>>8;	 //��������id
//				//0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//				ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//				ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//				ddata[4]=client_custom_ID;
//				ddata[5]=client_custom_ID>>8;       //�ͻ���id
//					//*************************�Ƿ���С����*******************************//
//				cap.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//				cap.grapic_data_struct.layer=1;   //ͼ��
//				cap.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//				cap.grapic_data_struct.graphic_name[0]=0;
//				cap.grapic_data_struct.graphic_name[1]=0;
//				cap.grapic_data_struct.graphic_name[2]=6;
//        
//				cap.grapic_data_struct.start_x=540;
//				cap.grapic_data_struct.start_y=137;
//				cap.grapic_data_struct.width=WIDTH;
//				cap.grapic_data_struct.start_angle=20;
//				cap.grapic_data_struct.end_angle=4;
//			  
//				cap.grapic_data_struct.color=UI_YELLOW;
//				cap.data[0]='C';
//				cap.data[1]='A';
//				cap.data[2]='P';
//				cap.data[3]=':';
//				
//				*(ext_client_custom_character_t*)(&ddata[6])=cap;
//		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(cap),DN_REG_ID,tx_buf);
//				
//			
//    }break;
//    case 5:   //��������                     6
//		{
//      ddata[0]=0x0104;
//      ddata[1]=0x0104>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id

//      /*************************������ʾ*******************************/
//      cap_sight.grapic_data_struct[0].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      cap_sight.grapic_data_struct[0].layer=1;   //ͼ��
//      cap_sight.grapic_data_struct[0].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      cap_sight.grapic_data_struct[0].graphic_name[0]=0;
//      cap_sight.grapic_data_struct[0].graphic_name[1]=0;
//      cap_sight.grapic_data_struct[0].graphic_name[2]=7;
//                                   
//      cap_sight.grapic_data_struct[0].start_x=600;
//      cap_sight.grapic_data_struct[0].start_y=110;
//      cap_sight.grapic_data_struct[0].end_y=110;
//			
//      if(usart_gimbal_data.cap_v>=0.0f&&usart_gimbal_data.cap_v<=7.0f)
//        {
//          cap_sight.grapic_data_struct[0].end_x=600;
//          cap_sight.grapic_data_struct[0].color=UI_PINK;
//        }
//      else if(usart_gimbal_data.cap_v>7.0f)
//        {
//          cap_sight.grapic_data_struct[0].end_x=600+((int)(usart_gimbal_data.cap_v-7.0))*35.29f;
//          cap_sight.grapic_data_struct[0].color=UI_PINK;
//        }

//      cap_sight.grapic_data_struct[0].width=30;
//				
//      *(ext_client_custom_graphic_seven_t*)(&ddata[6])=cap_sight;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(cap_sight),DN_REG_ID,tx_buf);
//		}break;		
//    case 6:   //Ħ����                         7
//    {
//      //----------------------------------Ħ����ģʽ----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			Friction_state.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			Friction_state.grapic_data_struct.layer=1;   //ͼ��
//			Friction_state.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			Friction_state.grapic_data_struct.graphic_name[0]=0;
//			Friction_state.grapic_data_struct.graphic_name[1]=0;
//			Friction_state.grapic_data_struct.graphic_name[2]=8;
//			
//			Friction_state.grapic_data_struct.start_x=1700;//55
//				Friction_state.grapic_data_struct.start_y=540;//215
//				Friction_state.grapic_data_struct.width=WIDTH;
//				Friction_state.grapic_data_struct.start_angle=20;
//				Friction_state.grapic_data_struct.end_angle=15;
//      
//				Friction_state.data[0] = 'F';
//				Friction_state.data[1] = 'R';
//				Friction_state.data[2] = 'I';
//				Friction_state.data[3] = 'C';
//				Friction_state.data[4] = 'T';
//				Friction_state.data[5] = 'I';
//				Friction_state.data[6] = 'O';
//				Friction_state.data[7] = 'N';
//				
//				if(shoot.fric_wheel_run==1)
//				{
//					Friction_state.grapic_data_struct.color = UI_GREEN;
//				}else
//				{
//					Friction_state.grapic_data_struct.color = UI_YELLOW;
//				}
//				
//				
//					
//      *(ext_client_custom_character_t*)(&ddata[6])=Friction_state;
//		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Friction_state),DN_REG_ID,tx_buf);
//			}break;
//		case 7:   //����ģʽ                     8-13
//		{
//				
//			//----------------------------------����ģʽ----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			speed_up.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			speed_up.grapic_data_struct.layer=1;   //ͼ��
//			speed_up.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			speed_up.grapic_data_struct.graphic_name[0]=0;
//			speed_up.grapic_data_struct.graphic_name[1]=0;
//			speed_up.grapic_data_struct.graphic_name[2]=9;
//			
//			speed_up.grapic_data_struct.start_x=100;
//				speed_up.grapic_data_struct.start_y=570;
//				speed_up.grapic_data_struct.width=WIDTH;
//				speed_up.grapic_data_struct.start_angle=20;
//				speed_up.grapic_data_struct.end_angle=7;
//      
//				speed_up.data[0] = 'S';
//				speed_up.data[1] = 'P';
//				speed_up.data[2] = 'E';
//				speed_up.data[3] = 'E';
//				speed_up.data[4] = 'D';
//				speed_up.data[5] = 'U';
//				speed_up.data[6] = 'P';

//				
//				if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag==1)
//				{
//					speed_up.grapic_data_struct.color = UI_GREEN;
//				}else
//				{
//					speed_up.grapic_data_struct.color = UI_YELLOW;
//				}
//			
//			
//			
//			
//			
//      *(ext_client_custom_character_t*)(&ddata[6])=speed_up;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(speed_up),DN_REG_ID,tx_buf);
//    }break;
//		case 8:               
//		{
//			

//     //----------------------------------����----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			Level.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			Level.grapic_data_struct.layer=1;   //ͼ��
//			Level.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			Level.grapic_data_struct.graphic_name[0]=0;
//			Level.grapic_data_struct.graphic_name[1]=1;
//			Level.grapic_data_struct.graphic_name[2]=0;
//			
//			Level.grapic_data_struct.start_x=100;
//				Level.grapic_data_struct.start_y=600;
//				Level.grapic_data_struct.width=WIDTH;
//				Level.grapic_data_struct.start_angle=20;
//				Level.grapic_data_struct.end_angle=6;
//      
//				Level.data[0] = 'L';
//				Level.data[1] = 'E';
//				Level.data[2] = 'V';
//				Level.data[3] = 'E';
//				Level.data[4] = 'L';
//				Level.data[5] = ':';


//					Level.grapic_data_struct.color = UI_YELLOW;
//				
//		
//			
//      *(ext_client_custom_character_t*)(&ddata[6])=Level;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Level),DN_REG_ID,tx_buf);
//			}break;
//		case 9:   //������ʾ                     15-16
//		{
//				
//			
//			
//      //----------------------------------����----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			Level_sight.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			Level_sight.grapic_data_struct.layer=1;   //ͼ��
//			Level_sight.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			Level_sight.grapic_data_struct.graphic_name[0]=0;
//			Level_sight.grapic_data_struct.graphic_name[1]=1;
//			Level_sight.grapic_data_struct.graphic_name[2]=1;
//			
//				Level_sight.grapic_data_struct.start_x=210;
//				Level_sight.grapic_data_struct.start_y=600;
//				Level_sight.grapic_data_struct.width=4;
//				Level_sight.grapic_data_struct.start_angle=20;
//				Level_sight.grapic_data_struct.end_angle=4;
//      
//				sprintf(Level_sight.data,"%f",judge_rece_mesg.game_robot_state.robot_level*1.0f);


//					Level_sight.grapic_data_struct.color = UI_RB;
//							
//      *(ext_client_custom_character_t*)(&ddata[6])=Level_sight;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Level_sight),DN_REG_ID,tx_buf);
//		}break;	
//		case 10:
//		{
//			ddata[0]=0x0110;
//				ddata[1]=0x0110>>8;	 //��������id
//				//0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//				ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//				ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//				ddata[4]=client_custom_ID;
//				ddata[5]=client_custom_ID>>8;       //�ͻ���id
//					//*************************�ڱ��Ƿ񱻻���*******************************//
//			if(security_attacked==1)
//			{
//				security_or_base.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//				security_or_base.grapic_data_struct.layer=1;   //ͼ��
//				security_or_base.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//				security_or_base.grapic_data_struct.graphic_name[0]=0;
//				security_or_base.grapic_data_struct.graphic_name[1]=1;
//				security_or_base.grapic_data_struct.graphic_name[2]=6;
//        
//				security_or_base.grapic_data_struct.start_x=960;
//				security_or_base.grapic_data_struct.start_y=700;
//				security_or_base.grapic_data_struct.width=WIDTH;
//				security_or_base.grapic_data_struct.start_angle=24;
//				security_or_base.grapic_data_struct.end_angle=4;
//			  
//				security_or_base.grapic_data_struct.color=0;
//				security_or_base.data[0]='S';
//				security_or_base.data[1]='E';
//				security_or_base.data[2]='C';
//				security_or_base.data[3]='U';
//				security_or_base.data[4]='R';
//				security_or_base.data[5]='I';
//				security_or_base.data[6]='T';
//				security_or_base.data[7]='Y';
//				security_or_base.data[8]=' ';
//				security_or_base.data[9]='A';
//				security_or_base.data[10]='T';
//				security_or_base.data[11]='K';
//				security_attacked = 0;
//			}else
//			{
//				security_or_base.grapic_data_struct.operate_type=3;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//				security_or_base.grapic_data_struct.layer=1;   //ͼ��
//				security_or_base.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//				security_or_base.grapic_data_struct.graphic_name[0]=0;
//				security_or_base.grapic_data_struct.graphic_name[1]=1;
//				security_or_base.grapic_data_struct.graphic_name[2]=6;
//        
//				
//			}
//				*(ext_client_custom_character_t*)(&ddata[6])=security_or_base;
//		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(security_or_base),DN_REG_ID,tx_buf);
//		}break;
//		
//	
//	/*******************************************************************************************************************************/
//	/////////////////////////////////////////////////////////ˢ��ѭ��////////////////////////////////////////////////////////////////
//	/*******************************************************************************************************************************/
//		case 11:  //������                       20
//		{
//	  ddata[0]=0x0104;
//      ddata[1]=0x0104>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id

//      /*************************������ʾ*******************************/
//      cap_sight.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      cap_sight.grapic_data_struct[0].layer=1;   //ͼ��
//      cap_sight.grapic_data_struct[0].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      cap_sight.grapic_data_struct[0].graphic_name[0]=0;
//      cap_sight.grapic_data_struct[0].graphic_name[1]=0;
//      cap_sight.grapic_data_struct[0].graphic_name[2]=7;

//      cap_sight.grapic_data_struct[0].start_x=600;
//      cap_sight.grapic_data_struct[0].start_y=110;
//      cap_sight.grapic_data_struct[0].end_y=110;
//			
//      if(usart_gimbal_data.cap_v>=0.0f&&usart_gimbal_data.cap_v<=17.0f)
//        {
//          cap_sight.grapic_data_struct[0].end_x=600;
//          cap_sight.grapic_data_struct[0].color=UI_PINK;
//        }
//      else if(usart_gimbal_data.cap_v>17.0f)
//        {
//          cap_sight.grapic_data_struct[0].end_x=600+((int)(usart_gimbal_data.cap_v-17.0))*90.44f;
//          cap_sight.grapic_data_struct[0].color=UI_PINK;
//        }

//      cap_sight.grapic_data_struct[0].width=30;
//				
//      *(ext_client_custom_graphic_seven_t*)(&ddata[6])=cap_sight;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(cap_sight),DN_REG_ID,tx_buf);
//		}break;
//		case 12:  //׼��                   5
//		{
//              ddata[0]=0x0104;
//              ddata[1]=0x0104>>8;	 //��������id
//              //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//              ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//              ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//              ddata[4]=client_custom_ID;
//              ddata[5]=client_custom_ID>>8;       //�ͻ���id


//              /*********************׼����ʾ****************************************/
//              sight_bead.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//              sight_bead.grapic_data_struct[0].layer=1;   //ͼ��
//              sight_bead.grapic_data_struct[0].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//              sight_bead.grapic_data_struct[0].graphic_name[0]=0;
//              sight_bead.grapic_data_struct[0].graphic_name[1]=0;
//              sight_bead.grapic_data_struct[0].graphic_name[2]=1;
//              sight_bead.grapic_data_struct[0].start_x=960;
//              sight_bead.grapic_data_struct[0].start_y=540;
//			if(shoot.poke_run)
//			{
//				sight_bead.grapic_data_struct[0].color=UI_RB;
//			}else
//			{
//				sight_bead.grapic_data_struct[0].color=UI_GREEN;
//			}
//      
//			sight_bead.grapic_data_struct[0].radius=1;
//      sight_bead.grapic_data_struct[0].width=3;//��
//			
//			
//			if(new_location.flag||My_Auto_Shoot.Auto_Aim.Flag_Get_Target)
//			{
//				sight_bead.grapic_data_struct[1].operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      sight_bead.grapic_data_struct[1].layer=1;   //ͼ��
//      sight_bead.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      sight_bead.grapic_data_struct[1].graphic_name[0]=1;
//      sight_bead.grapic_data_struct[1].graphic_name[1]=0;
//      sight_bead.grapic_data_struct[1].graphic_name[2]=0;
//      sight_bead.grapic_data_struct[1].start_x=960;
//      sight_bead.grapic_data_struct[1].start_y=540;

//			sight_bead.grapic_data_struct[1].color=UI_YELLOW;
//			
//			sight_bead.grapic_data_struct[1].radius=3;
//      sight_bead.grapic_data_struct[1].width=3;//��
//			}else
//			{
//				sight_bead.grapic_data_struct[1].operate_type=3;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//      sight_bead.grapic_data_struct[1].layer=1;   //ͼ��
//      sight_bead.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//      sight_bead.grapic_data_struct[1].graphic_name[0]=1;
//      sight_bead.grapic_data_struct[1].graphic_name[1]=0;
//      sight_bead.grapic_data_struct[1].graphic_name[2]=0;
//			}
//			

//      *(ext_client_custom_graphic_seven_t*)(&ddata[6])=sight_bead;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(sight_bead),DN_REG_ID,tx_buf);
//		}break;
//		case 13:  //����ģʽ                     6
//		{
//      ddata[0]=0x0104;
//				ddata[1]=0x0104>>8;	 //��������id
//				//0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//				ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//				ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//				ddata[4]=client_custom_ID;
//				ddata[5]=client_custom_ID>>8;       //�ͻ���id
//					//*************************�Ƿ���С����*******************************//
//			switch(chassis.ctrl_mode)
//			{
//				case MANUAL_FOLLOW_GIMBAL:
//				{
//					chassis_graphics.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[0].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[0].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[0].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[2]=2;
//					
//					chassis_graphics.grapic_data_struct[0].color = UI_PINK;
//					chassis_graphics.grapic_data_struct[0].start_x = 1320;
//					chassis_graphics.grapic_data_struct[0].start_y = 108;
//					chassis_graphics.grapic_data_struct[0].end_x = 1320;
//					chassis_graphics.grapic_data_struct[0].end_y = 195;
//					chassis_graphics.grapic_data_struct[0].width = 3;
//					
//					
//					chassis_graphics.grapic_data_struct[1].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[1].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[1].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[1].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[2]=3;
//					
//					chassis_graphics.grapic_data_struct[1].color = UI_YELLOW;
//					chassis_graphics.grapic_data_struct[1].start_x = 1285;
//					chassis_graphics.grapic_data_struct[1].start_y = 143;
//					chassis_graphics.grapic_data_struct[1].end_x = 1354;
//					chassis_graphics.grapic_data_struct[1].end_y = 143;
//					chassis_graphics.grapic_data_struct[1].width = 4;
//					
//					
//					chassis_graphics.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[2].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[2].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[2]=4;
//					
//					chassis_graphics.grapic_data_struct[2].color = UI_YELLOW;
//					chassis_graphics.grapic_data_struct[2].start_x = 1285;
//					chassis_graphics.grapic_data_struct[2].start_y = 85;
//					chassis_graphics.grapic_data_struct[2].end_x = 1354;
//					chassis_graphics.grapic_data_struct[2].end_y = 85;
//					chassis_graphics.grapic_data_struct[2].width = 4;
//					
//				}break;
//				case CHASSIS_REVERSE:
//				{
//					chassis_graphics.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[0].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[0].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[0].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[2]=2;
//					
//					chassis_graphics.grapic_data_struct[0].color = UI_PINK;
//					chassis_graphics.grapic_data_struct[0].start_x = 1320;
//					chassis_graphics.grapic_data_struct[0].start_y = 108;
//					chassis_graphics.grapic_data_struct[0].end_x = 1320;
//					chassis_graphics.grapic_data_struct[0].end_y = 195;
//					chassis_graphics.grapic_data_struct[0].width = 3;
//					
//					
//					chassis_graphics.grapic_data_struct[1].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[1].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[1].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[1].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[2]=3;
//					
//					chassis_graphics.grapic_data_struct[1].color = UI_YELLOW;
//					chassis_graphics.grapic_data_struct[1].start_x = 1285;
//					chassis_graphics.grapic_data_struct[1].start_y = 74;
//					chassis_graphics.grapic_data_struct[1].end_x = 1285;
//					chassis_graphics.grapic_data_struct[1].end_y = 146;
//					chassis_graphics.grapic_data_struct[1].width = 4;
//					
//					
//					chassis_graphics.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[2].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[2].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[2]=4;
//					
//					chassis_graphics.grapic_data_struct[2].color = UI_YELLOW;
//					chassis_graphics.grapic_data_struct[2].start_x = 1355;
//					chassis_graphics.grapic_data_struct[2].start_y = 74;
//					chassis_graphics.grapic_data_struct[2].end_x = 1355;
//					chassis_graphics.grapic_data_struct[2].end_y = 146;
//					chassis_graphics.grapic_data_struct[2].width = 4;
//				}break;
//				case CHASSIS_ROTATE:
//				{
//					chassis_graphics.grapic_data_struct[0].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[0].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[0].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[0].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[0].graphic_name[2]=2;
//					
//					chassis_graphics.grapic_data_struct[0].color = UI_PINK;
//					chassis_graphics.grapic_data_struct[0].start_x = 1320;
//					chassis_graphics.grapic_data_struct[0].start_y = 108;
//					chassis_graphics.grapic_data_struct[0].end_x = 1320;
//					chassis_graphics.grapic_data_struct[0].end_y = 195;
//					chassis_graphics.grapic_data_struct[0].width = 3;
//					
//					
//					chassis_graphics.grapic_data_struct[1].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[1].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[1].graphic_type=2;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[1].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[1].graphic_name[2]=3;
//					chassis_graphics.grapic_data_struct[1].start_x=1320;
//					chassis_graphics.grapic_data_struct[1].start_y=108;

//					chassis_graphics.grapic_data_struct[1].color=UI_YELLOW;
//			
//      
//					chassis_graphics.grapic_data_struct[1].radius=49;
//					chassis_graphics.grapic_data_struct[1].width=4;//��
//					
//					chassis_graphics.grapic_data_struct[2].operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//					chassis_graphics.grapic_data_struct[2].layer=1;   //ͼ��
//					chassis_graphics.grapic_data_struct[2].graphic_type=0;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//					chassis_graphics.grapic_data_struct[2].graphic_name[0]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[1]=0;
//					chassis_graphics.grapic_data_struct[2].graphic_name[2]=4;
//					
//					chassis_graphics.grapic_data_struct[2].color = UI_PINK;
//					chassis_graphics.grapic_data_struct[2].start_x = 1279;
//					chassis_graphics.grapic_data_struct[2].start_y = 108;
//					chassis_graphics.grapic_data_struct[2].end_x = 1368;
//					chassis_graphics.grapic_data_struct[2].end_y = 108;
//					chassis_graphics.grapic_data_struct[2].width = 3;
//				}break;
//			}
//				


//			

//		*(ext_client_custom_graphic_seven_t*)(&ddata[6])=chassis_graphics;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(chassis_graphics),DN_REG_ID,tx_buf);
//		}break;
// 		case 14:  //�ȼ�                         7
//		{
//			//----------------------------------����----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			Level_sight.grapic_data_struct.operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			Level_sight.grapic_data_struct.layer=1;   //ͼ��
//			Level_sight.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			Level_sight.grapic_data_struct.graphic_name[0]=0;
//			Level_sight.grapic_data_struct.graphic_name[1]=1;
//			Level_sight.grapic_data_struct.graphic_name[2]=1;
//			
//				Level_sight.grapic_data_struct.start_x=210;
//				Level_sight.grapic_data_struct.start_y=600;
//				Level_sight.grapic_data_struct.width=4;
//				Level_sight.grapic_data_struct.start_angle=20;
//				Level_sight.grapic_data_struct.end_angle=4;
//      
//				sprintf(Level_sight.data,"%f",judge_rece_mesg.game_robot_state.robot_level*1.0f);


//					Level_sight.grapic_data_struct.color = UI_RB;
//							
//      *(ext_client_custom_character_t*)(&ddata[6])=Level_sight;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Level_sight),DN_REG_ID,tx_buf);
//			}break;
//		case 15:  //����                     10-13
//		{
//			//----------------------------------����ģʽ----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			speed_up.grapic_data_struct.operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			speed_up.grapic_data_struct.layer=1;   //ͼ��
//			speed_up.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			speed_up.grapic_data_struct.graphic_name[0]=0;
//			speed_up.grapic_data_struct.graphic_name[1]=0;
//			speed_up.grapic_data_struct.graphic_name[2]=9;
//			
//			speed_up.grapic_data_struct.start_x=100;
//				speed_up.grapic_data_struct.start_y=570;
//				speed_up.grapic_data_struct.width=WIDTH;
//				speed_up.grapic_data_struct.start_angle=20;
//				speed_up.grapic_data_struct.end_angle=7;
//      
//				speed_up.data[0] = 'S';
//				speed_up.data[1] = 'P';
//				speed_up.data[2] = 'E';
//				speed_up.data[3] = 'E';
//				speed_up.data[4] = 'D';
//				speed_up.data[5] = 'U';
//				speed_up.data[6] = 'P';

//				
//				if(RC_CtrlData.Key_Flag.Key_SHIFT_Flag==1)
//				{
//					speed_up.grapic_data_struct.color = UI_GREEN;
//				}else
//				{
//					speed_up.grapic_data_struct.color = UI_YELLOW;
//				}
//			
//			
//			
//			
//			
//      *(ext_client_custom_character_t*)(&ddata[6])=speed_up;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(speed_up),DN_REG_ID,tx_buf);
//		}break;
//		case 16:  //Ħ����                     14
//		{
//     //----------------------------------Ħ����ģʽ----------------------------------------//
//      ddata[0]=0x0110;
//      ddata[1]=0x0110>>8;	 //��������id
//      //0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//      ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//      ddata[4]=client_custom_ID;
//      ddata[5]=client_custom_ID>>8;       //�ͻ���id
//			
//			Friction_state.grapic_data_struct.operate_type=2;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//			Friction_state.grapic_data_struct.layer=1;   //ͼ��
//			Friction_state.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//			Friction_state.grapic_data_struct.graphic_name[0]=0;
//			Friction_state.grapic_data_struct.graphic_name[1]=0;
//			Friction_state.grapic_data_struct.graphic_name[2]=8;
//			
//			Friction_state.grapic_data_struct.start_x=1700;
//				Friction_state.grapic_data_struct.start_y=540;
//				Friction_state.grapic_data_struct.width=WIDTH;
//				Friction_state.grapic_data_struct.start_angle=20;
//				Friction_state.grapic_data_struct.end_angle=15;
//      
//				Friction_state.data[0] = 'F';
//				Friction_state.data[1] = 'R';
//				Friction_state.data[2] = 'I';
//				Friction_state.data[3] = 'C';
//				Friction_state.data[4] = 'T';
//				Friction_state.data[5] = 'I';
//				Friction_state.data[6] = 'O';
//				Friction_state.data[7] = 'N';
//				
//				if(shoot.fric_wheel_run==1)
//				{
//					Friction_state.grapic_data_struct.color = UI_GREEN;
//				}else
//				{
//					Friction_state.grapic_data_struct.color = UI_YELLOW;
//					
//				}
//				
//				
//					
//      *(ext_client_custom_character_t*)(&ddata[6])=Friction_state;
//      data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(Friction_state),DN_REG_ID,tx_buf);
//			}break;
//		case 17:
//			{
//				ddata[0]=0x0110;
//				ddata[1]=0x0110>>8;	 //��������id
//				//0x0100  ɾ��ͼ�� 0x0101 ����һ��ͼ�� 0x0102 ���ƶ���ͼ�� 0x0103 �������ͼ�� 0x0104�����߸�ͼ�� 0x0110�ͻ��˻����ַ�ͼ��
//				ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
//				ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //������id
//				ddata[4]=client_custom_ID;
//				ddata[5]=client_custom_ID>>8;       //�ͻ���id
//					//*************************�ڱ��Ƿ񱻻���*******************************//
//			if(security_attacked==1)
//			{
//				security_or_base.grapic_data_struct.operate_type=1;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//				security_or_base.grapic_data_struct.layer=1;   //ͼ��
//				security_or_base.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//				security_or_base.grapic_data_struct.graphic_name[0]=0;
//				security_or_base.grapic_data_struct.graphic_name[1]=1;
//				security_or_base.grapic_data_struct.graphic_name[2]=6;
//        
//				security_or_base.grapic_data_struct.start_x=960;
//				security_or_base.grapic_data_struct.start_y=700;
//				security_or_base.grapic_data_struct.width=WIDTH;
//				security_or_base.grapic_data_struct.start_angle=24;
//				security_or_base.grapic_data_struct.end_angle=4;
//			  
//				security_or_base.grapic_data_struct.color=0;
//				security_or_base.data[0]='S';
//				security_or_base.data[1]='E';
//				security_or_base.data[2]='C';
//				security_or_base.data[3]='U';
//				security_or_base.data[4]='R';
//				security_or_base.data[5]='I';
//				security_or_base.data[6]='T';
//				security_or_base.data[7]='Y';
//				security_or_base.data[8]=' ';
//				security_or_base.data[9]='A';
//				security_or_base.data[10]='T';
//				security_or_base.data[11]='K';
//				security_attacked = 0;
//			}else
//			{
//				security_or_base.grapic_data_struct.operate_type=3;  //1 ���� 2�޸�ͼ�� 3ɾ������ͼ�� 5ɾ��һ��ͼ���ͼ�� 6ɾ������ͼ��
//				security_or_base.grapic_data_struct.layer=1;   //ͼ��
//				security_or_base.grapic_data_struct.graphic_type=7;  //0 ֱ�� 1���� 2��Բ 3��Բ 4Բ�� 5������ 6������ 7�ַ�
//				security_or_base.grapic_data_struct.graphic_name[0]=0;
//				security_or_base.grapic_data_struct.graphic_name[1]=1;
//				security_or_base.grapic_data_struct.graphic_name[2]=6;
//        
//				
//			}
//				*(ext_client_custom_character_t*)(&ddata[6])=security_or_base;
//		data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(security_or_base),DN_REG_ID,tx_buf);
//			}break;
//		
//    default:
//    break;
//    }

//  draw_cnt++;
//	draw_int++;
// 
//  if(draw_cnt>17)//����Ҫˢ�µ�ͼ��ˢ��
//    draw_cnt=11;
//	
//	
//	if(RC_CtrlData.Key_Flag.Key_R_Flag)
//	{
//		draw_cnt = 1;
//	}

}

void delete_Coverage(u8 coverage)
{
  ddata[6]=4;//1����2�޸�3ɾ������4ɾ��ͼ��5ɾ������
  ddata[13]=coverage;//ͼ��0-9
}
