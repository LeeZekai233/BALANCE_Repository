#include "public.h"

Peripheral_State_t Peripheral_State;


/**********************
*@Brief:设备连接状态设置函数
*@Call:内部或外部
*@Param:设备状态结构体
				连接状态结构体
*@Note:无
*@RetVal:无
**********************/
void Link_State_Set(Equipment_State_t* Equipment_State,Link_State_e Link_State)
{
	Equipment_State->Link_State=Link_State;
}	

/**********************
*@Brief:设备计数器清零函数
*@Call:内部或外部
*@Param:设备状态结构体
*@Note:在相应中断中调用此函数，当计数器超过一定值时说明改设备连接异常。
*@RetVal:无
**********************/
void Equipment_Counter_Make_Zero(Equipment_State_t* Equipment_State)
{
	Equipment_State->Counter=0;
}


/**********************
*@Brief:设备状态初始化函数
*@Call:内部或外部
*@Param:
*@Note:无
*@RetVal:无
**********************/
void Peripheral_State_Init(Peripheral_State_t* Peripheral_State)
{
	Peripheral_State->Remote_Control.Link_State=DISCONNECT;
	Peripheral_State->Remote_Control.Counter=0xFFFFFF;
	
	Peripheral_State->VTM_Remote.Link_State=DISCONNECT;
	Peripheral_State->VTM_Remote.Counter=0xFFFFFF;
	
	Peripheral_State->Gimbal_Yaw_Gyro.Link_State=DISCONNECT;
	Peripheral_State->Gimbal_Yaw_Gyro.Counter=0xFFFFFF;
	
	Peripheral_State->Equipment_Visual_Equipment_Auto_Aim.Link_State=DISCONNECT;
	Peripheral_State->Equipment_Visual_Equipment_Auto_Aim.Counter=0xFFFFFF;

}
/**********************
*@Brief:设备状态判断函数
*@Call:内部或外部
*@Param:Equipment_State：设备状态结构体
				T,函数执行周期，单位：秒
*@Note:无
*@RetVal:无
**********************/
void Equipment_State_Judge(Equipment_State_t* Equipment_State,float T)
{
	Equipment_State->Counter++;
	if(Equipment_State->Counter*T>1)
		Equipment_State->Link_State=DISCONNECT;
	else
		Equipment_State->Link_State=CONNECTED;
}
/**********************
*@Brief:函数
*@Call:内部或外部
*@Param:
*@Note:无
*@RetVal:无
**********************/
void Peripheral_State_Judge(Peripheral_State_t* Peripheral_State,float T)
{
	Equipment_State_Judge(&Peripheral_State->Remote_Control,T);
	Equipment_State_Judge(&Peripheral_State->Gimbal_Yaw_Gyro,T);
	Equipment_State_Judge(&Peripheral_State->Equipment_Visual_Equipment_Auto_Aim,T);
	Equipment_State_Judge(&Peripheral_State->Super_Cap,T);
	Equipment_State_Judge(&Peripheral_State->VTM_Remote,T);
}
	

