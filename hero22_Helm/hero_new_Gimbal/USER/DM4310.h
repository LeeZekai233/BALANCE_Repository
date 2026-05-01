#ifndef __DM4310_H__
#define __DM4310_H__


typedef enum
{
	NORMAL			=0,
	U_MAX       =1,
	U_MIN				=2,
	I_MAX				=3,
	MOS_TEM_MAX		=4,   	//MOS 过温
	ROTOR_TEM_MAX	=5,			//电机线圈过温
	CAN_BAG_LOST	=6,			//通讯丢失
	OVERLOAD			=7,			//过载
}DaMiao_ERR_e;


typedef struct
{
	int16_t  ID;
	DaMiao_ERR_e   ERR;				//故障类型
	
	int16_t Kp;					// Kp 的范围为[0,500]，Kd 的范围为[0,5]
	int16_t Kd;					// P*Kp + V*Kd = T_ref—> I_ref
	
	float P_des; 			//发送给电机的位置值				//闭电机内环
	float V_des; 			//发送给电机的最大速度值
	
	int16_t P_fdb;				//当前位置
	int16_t P_ref;				//当前位置
	
	float Angle_fdb;			//当前角度
	int16_t Angle_ref;			//目标角度
	
	int16_t V_fdb;				//电机当前转速
	int16_t V_ref;				//电机目标转速
	
	int16_t T_fdb;					//电机的扭矩信息
	int16_t T_ref;					//电机的扭矩信息
	
	uint16_t Temperature_MOS;		//驱动上 MOS 的平均温度，单位℃
	uint16_t Temperature_Rotor;		//电机内部线圈的平均温度，单位℃

	
}DaMiao_4310_t;

extern DaMiao_4310_t DM_4310;
void DaMiao_Exception_Clear(int16_t CAN_ID);
void DM4310_Progress(volatile DaMiao_4310_t *v,CanRxMsg *msg);
void DM4310_angle_control(CAN_TypeDef *CANx,float angle,float speed,uint32_t id);
void DaMiao_Run_Init(CAN_TypeDef *CANx,uint16_t CAN_ID);
void DaMiao_4310_Position0_offset(CAN_TypeDef *CANx,int16_t CAN_ID);
void DaMiao_4310_Information_Send(DaMiao_4310_t *DaMiao_4310,int16_t ID);
int float_to_uint(float x, float x_min, float x_max, int bits);
void DM_4310_MIT(CAN_TypeDef *CANx,float angle,float speed,float kp,float kd,float t,uint16_t id);
void DM_Speed_Mode(CAN_TypeDef *CANx,float _vel,uint16_t id);




#endif
