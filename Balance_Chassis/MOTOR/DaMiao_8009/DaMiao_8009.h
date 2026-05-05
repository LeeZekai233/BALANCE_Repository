#ifndef __DAMIAO_8009_H_
#define __DAMIAO_8009_H_
#include "stm32f4xx.h" 

#define P_MAX 3.141593 
#define P_MIN -3.141593
#define V_MAX 45.0f
#define V_MIN -45.0f
#define T_MAX 40.0f
#define T_MIN -40.0f
#define KP_MIN 0
#define KP_MAX 500
#define KD_MAX 5
#define KD_MIN 0
/************************************************** 结构体定义 *****************************************************************/
//首字母大写，_e枚举 _t结构体

typedef enum
{
    DM_DISABLE	    =0,
    DM_ENABLE       =1,
	U_MAX           =2,
	U_MIN		    =3,
	I_MAX		    =4,
	MOS_TEM_MAX		=5,         //MOS 过温
	ROTOR_TEM_MAX	=6,			//电机线圈过温
	CAN_BAG_LOST	=7,			//通讯丢失
	OVERLOAD	    =8,			//过载
}DaMiao_ERR_e;					//达妙电机状态

typedef struct
{
	int16_t  ID;
	DaMiao_ERR_e ERR;
	
	int16_t Kp;					// Kp 的范围为[0,500]，Kd 的范围为[0,5]
	int16_t Kd;					// P*Kp + V*Kd = T_ref—> I_ref
	
	float P_fdb;				//当前位置  rad
    
	float Angle_Deg_fdb;	          //当前单圈角度   单位°
    float Angle_Deg_fdb_last;     //电机上一次单圈角度 
    float Angle_Deg_Total_fdb;       //当前多圈角度
    
	float V_fdb;				//电机当前转速  rad/s
	
	float T_fdb;					//电机的扭矩信息   N·M	
	
	uint16_t Temperature_MOS;		//驱动上 MOS 的平均温度，单位℃
	uint16_t Temperature_Rotor;		//电机内部线圈的平均温度，单位℃
	
	int16_t round_cnt;  //圈数
	int8_t  Flag_init_direction;		//初始化时方向是正是负，解算±180在0°上
}DaMiao_8009_t;


#ifndef GENERIC_ENCODER
#define GENERIC_ENCODER


typedef struct
{
    
    float Angle_Deg_fdb;	         //当前单圈角度   单位°
    float Angle_Deg_Total_fdb;       //当前多圈角度   单位°
    
    float Angle_Rad_fdb;             //单圈角度反馈   单位rad
    float Angle_Rad_Total_fdb;       //多圈角度反馈   单位rad
    
    float Omega_Rad_fdb;		//电机当前转速  rad/s
    
    uint8_t online_flag;        //在线标志位
    
    float Torque;             //力矩
    
    uint32_t temperature;    //温度
    
    uint32_t heart_cnt;//
}Encoder_t;//通用编码器



#endif

/******************************************************************************************************************************/


/******************* 结构体or变量的声明 **********/
extern DaMiao_8009_t Joint_Motor[4];
/************************************************/


/****************** 函 数 声 明 ********************************/
void DaMiao_8009_Information_Send(CAN_TypeDef* CANx,uint16_t CAN_ID,float P_des,float V_ref,float T_ref,float Kp,float Kd);	//信息发送
void DaMiao_8009_Speed_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float V_des);
void DaMiao_8009_Position_Send(CAN_TypeDef* CANx,int16_t CAN_ID,float P_des,float V_des);
void DaMiao_8009_Position0_offset(CAN_TypeDef* CANx, int16_t CAN_ID);
void DaMiao_8009_Enable(CAN_TypeDef* CANx, int16_t CAN_ID);
void DaMiao_8009_Disable(CAN_TypeDef* CANx, int16_t CAN_ID);
void DaMiao_8009_Information_Receive(CanRxMsg *msg,DaMiao_8009_t *DaMiao_8009,float offset);		//信息接收
void DaMiao_8009_Claer_Error_Information(CAN_TypeDef* CANx, int16_t CAN_ID);
void DaMiao_8009_Position_Information_Send_Test(void);
void DaMiao_8009_To_Generic_Encoder(DaMiao_8009_t* DaMiao_8009,Encoder_t* Encoder);
float AngleWrap(float angle);
/***************************************************************/


#endif  /* _DAMIAO_8009_H_ */
