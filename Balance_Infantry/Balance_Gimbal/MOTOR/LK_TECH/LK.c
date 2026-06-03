/**
  ******************************************************************************
  * @file    EMBEDDED\Motor\LK\LK.c
  * @author  William
  * @version V1.0.0
  * @date    17-April-2025
  * @brief   Configuration of LK Motor 
  ******************************************************************************
  * @attention	
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define Encoder_16bit_Angle_Conversion_Ratio 3.1415926/180;
/*****************************************************************************
**@Brief:	LK receiving function
**@Cal:		Internal or external
**@param:  	no
**@Note:    Accepted identifier ID = 0x180 + ESC ID (ESC ID ranges from 1 to 32)
**@RetVal: 	no
*****************************************************************************/
int id= 0;
void LK_Information_Receive (int count, CanRxMsg *msg, int16_t ID, volatile LK_InitTypeDef *LK)
{
	if (msg->StdId == (0x140 + ID))
	{
		switch (msg->Data[0])
		{
			//执行关闭命令时电机的反馈帧(与发送帧一样, 无用)
			case (0x80):	{break;}
			//执行运行命令时电机的反馈帧(与发送帧一样, 无用)
			case (0x88):	{break;}
			//执行停止命令时电机的反馈帧(与发送帧一样, 无用)
			case (0x81):	{break;}
			
			//抱闸器状态读取
			case (0x8C):	
			{
				if (msg->Data[1] == 0x00) 		{LK->Brake_Dvice= BD_ON;}
				else if (msg->Data[1] == 0x01) 	{LK->Brake_Dvice= BD_OFF;}
				
				break;
			}
			
			//Read encoder offset which has been cunrrently set
			case (0x19):
			{
				//读取编码器零偏值
				LK->encoder_offset= ((msg->Data[7] << 8) | msg->Data[6]);
				
				break;
			}
			
			//Read motor encoder data
			case (0x90):
			{
				//读取原始编码值
				LK->originate_raw_value= (uint16_t)((msg->Data[5] << 8) | msg->Data[4]);
				//读取电机编码值
				LK->raw_value          = (uint16_t)((msg->Data[3] << 8) | msg->Data[2]);
				//读取编码器零偏值
				LK->encoder_offset     = (uint16_t)((msg->Data[7] << 8) | msg->Data[6]);
				
				break;
			}
	
			case (0x92):
			{
				//读取电机角度(多圈，即可以累计)
				int64_t uncalulated_continuous_angle= 0;
				
				//由于msg, data, MG6012E_i8->ecd_angle的变量定义不同，
				//不能直接获取数据, 所以定义中间变量来获取数据
				uncalulated_continuous_angle |= (int64_t)msg->Data[1] << 0;   // 第0字节（最低位）
				uncalulated_continuous_angle |= (int64_t)msg->Data[2] << 8;   // 第1字节
				uncalulated_continuous_angle |= (int64_t)msg->Data[3] << 16;  // 第2字节
				uncalulated_continuous_angle |= (int64_t)msg->Data[4] << 24;  // 第3字节
				uncalulated_continuous_angle |= (int64_t)msg->Data[5] << 32;  // 第4字节
				uncalulated_continuous_angle |= (int64_t)msg->Data[6] << 40;  // 第5字节
				uncalulated_continuous_angle |= (int64_t)msg->Data[7] << 48;  // 第6字节（第7字节预留或符号位）
				
				LK->uncal_cont_angle= uncalulated_continuous_angle;
				
				break;
			}
			
			case (0x94):
			{
				//读取电机角度(单圈，用于计算连续角度)
				int64_t raw_angle= 0;
				
				//由于msg, data, MG6012E_i8->ecd_angle的变量定义不同，
				//不能直接获取数据, 所以定义中间变量来获取数据
				raw_angle |= (uint32_t)msg->Data[4] << 0;   // 第0字节（最低位）
				raw_angle |= (uint32_t)msg->Data[5] << 8;   // 第1字节
				raw_angle |= (uint32_t)msg->Data[6] << 16;  // 第2字节
				raw_angle |= (uint32_t)msg->Data[7] << 24;  // 第3字节
				
				LK->single_angle= raw_angle;
				
				break;
			}
			
			//Read moter station (主要是报错反馈)
			case (0x9A):
			{
				//读取电机温度
				LK->Temperature_Rotor= (int8_t)msg->Data[1];
				
				//判断错误(两种方法)
				// 法1:分别提取 Data[7] 的八个位, 判断每个位
				int bit0 = (msg->Data[7] >> 0) & 1;
				int bit1 = (msg->Data[7] >> 1) & 1;
				int bit2 = (msg->Data[7] >> 2) & 1;
				int bit3 = (msg->Data[7] >> 3) & 1;
				int bit4 = (msg->Data[7] >> 4) & 1;
				int bit5 = (msg->Data[7] >> 5) & 1;
				int bit6 = (msg->Data[7] >> 6) & 1;
				int bit7 = (msg->Data[7] >> 7) & 1;
				
				if 		(bit0 == 1) {LK->LK_ERR_e= U_DEFICIENCY;}
				else if (bit1 == 1) {LK->LK_ERR_e= U_OVERLOAD;}
				else if (bit2 == 1) {LK->LK_ERR_e= DRIVE_TEM_OVER;}
				else if (bit3 == 1) {LK->LK_ERR_e= MOTOR_TEM_OVER;}
				else if (bit4 == 1) {LK->LK_ERR_e= I_OVERLOAD;}
				else if (bit5 == 1) {LK->LK_ERR_e= MOTOR_SHORT_CIRCUIT;}
				else if (bit6 == 1) {LK->LK_ERR_e= MOTOR_STALL;}
				else if (bit7 == 1) {LK->LK_ERR_e= INPUT_SIGNAL_LOSSTIMEOUT;}
				
				//法2:把整个Data[7]看作整体判断
//				if (msg->Data[7] 	  == (int8_t) 0x01) {LK->LK_ERR_e= U_DEFICIENCY;}
//				else if (msg->Data[7] == (int8_t) 0x02) {LK->LK_ERR_e= U_OVERLOAD;}
//				else if (msg->Data[7] == (int8_t) 0x04) {LK->LK_ERR_e= DRIVE_TEM_OVER;}
//				else if (msg->Data[7] == (int8_t) 0x08) {LK->LK_ERR_e= MOTOR_TEM_OVER;}
//				else if (msg->Data[7] == (int8_t) 0x10) {LK->LK_ERR_e= I_OVERLOAD;}
//				else if (msg->Data[7] == (int8_t) 0x20) {LK->LK_ERR_e= MOTOR_SHORT_CIRCUIT;}
//				else if (msg->Data[7] == (int8_t) 0x40) {LK->LK_ERR_e= MOTOR_STALL;}
//				else if (msg->Data[7] == (int8_t) 0x80) {LK->LK_ERR_e= INPUT_SIGNAL_LOSSTIMEOUT;}
				
				//读取电机当前状态
				if ((uint8_t)(msg->Data[6] == 0x00)) 		{LK->Motor_State= LK_ON;}
				else if ((uint8_t)(msg->Data[6] == 0x10)) 	{LK->Motor_State= LK_OFF;}
				//读取电机母线电压
				LK->Bus_Voltage= (int16_t)(((msg->Data[3] << 8) | msg->Data[2]));
				//读取电机母线电流
				LK->Bus_Current= (int16_t)(((msg->Data[5] << 8) | msg->Data[4]));
				
				break;
			}
			
			//Read motor information
			case (0x9C):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}
			
			//Read three—phase current data
			case (0x9D):
			{
				//读取电机温度
				LK->Temperature_Rotor= (int8_t)msg->Data[1];
				
				//读取A相电流值
				LK->Current_APhase  = (int16_t)((msg->Data[3] << 8) | msg->Data[2]);		
				//读取B相电流值
				LK->Current_BPhase  = (int16_t)((msg->Data[5] << 8) | msg->Data[4]);
				//读取C相电流值
				LK->Current_CPhase  = (int16_t)((msg->Data[7] << 8) | msg->Data[6]);				

				break;
			}

			//开环控制命令的反馈帧
			case (0xA0):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}

			//转矩闭环指令时的反馈帧
			case (0xA1):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}

			//速度闭环指令时的反馈帧
			case (0xA2):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}	

			//多圈位置闭环指令1时的反馈帧
			case (0xA3):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}		

			//多圈位置闭环指令2时的反馈帧
			case (0xA4):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}

			//单圈位置闭环指令1时的反馈帧
			case (0xA5):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}	

			//单圈位置闭环指令2时的反馈帧
			case (0xA6):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}

			//单圈位置闭环增量式指令1时的反馈帧
			case (0xA7):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}	

			//单圈位置闭环增量式指令2时的反馈帧
			case (0xA8):
			{
				LK_Basic_Informaion_Read (count, msg, LK);
				
				break;
			}		
			
			//读取电机控制参数时的反馈帧
			case (0xC0):
			{
				LK_Parameter_Read(msg, LK);
			}
			
			//写入电机控制参数时的反馈帧
			case (0xC1):
			{
				LK_Parameter_Read(msg, LK);
			}
		}
	}
}

/*****************************************************************************
**@Brief;	LK motor status change command
**@Cal;  	Internal or external
**@param:	no
**@Note:  	
**			The identifier ID sent = 0x140 + ESC ID (ESC ID ranges from 1 to 32)
**			Staton决定调用相关的控制函数或者控制指令
**			set current position/angle to ROM as the encoder offset —— 0x19
**			电机关闭命令 —————————— 0x80
**				(将电机从开启状态[上电后默认状态]切换到关闭状态, 
**				清除电机转动圈数及之前接收的控制指令, LED由常亮转为慢闪.
**				此时电机仍然可以回复控制命令，但不会执行动作)
**			电机停止命令 —————————— 0x81
**				(停止电机, 但不清除电机运行状态.再次发送控制指令即可控制电机动作)
**			电机运行命令 —————————— 0x88
**				(将电机从关闭状态切换到开启状态, LED由慢闪转为常亮.此时再发送控制指令即可控制电机动作)
**			抱闸器控制命令 ————————— 0x8C
**				(未写在本函数, 在 LK_Break_Device_Station 中)
**			Read encoder data ——————— 0x90 
**			Read multiple turns angle ——— 0x92 
**			Read single turn angle ———— 0x94
**			Read motor State ——————— 0x9A
**			Clear the error flag bit ———— 0x9B
**			Read motor information ————— 0x9C
**			Read three—phase current data — 0x9D
**			MS电机开环控制指令 ———————— 0xA0
**				(未写在本函数，在 LK_Open_Loop_Control 中)
**			MF,MH,MG转矩闭环控制命令 ————— 0xA1
**				(未写在本函数, 在 LK_TorqueLoop_Out 中)
**			速度闭环控制命令 ————————— 0xA2
**				(未写在本函数, 在 LK_SpdLoop_Out 中)
**			多圈位置闭环控制命令 ——————— 0xA3 / 0xA4
**				(0xA3控制时不会限制速度最大值, 0xA4控制时可以设定速度最大值)
**				(0xA3控制时, 在 LK_MultiturnPos_Out1 中)
**				(0xA4控制时, 在 LK_MultiturnPos_Out1 中)
**			单圈位置闭环控制命令 ——————— 0xA5 / 0xA6
**				(0xA5控制时不会限制速度最大值, 0xA6控制时可以设定速度最大值)
**				(0xA5控制时, 在 LK_SingleturnPos_Out1 中)
**				(0xA6控制时, 在 LK_SingleturnPos_Out2 中)
**			单圈位置增量式闭环控制命令 ———— 0xA7 /0xA8
**				(0xA7控制时不会限制速度最大值, 0xA8控制时可以设定速度最大值)
**				(0xA7控制时, 在 LK_SingleturnPos_Delta_Out1 中)
**				(0xA8控制时, 在 LK_SingleturnPos_Delta_Out2 中)
**			设置当前位置为任意角度命令 ———— 0x95
**				(未写在本函数, 在 LK_Set_CurntPos_As_AnyAngle 中)
**			读取电机控制参数命令 ——————— 0xC0
**			写入电机控制参数命令 ——————— 0xC1
**				(未写在本函数, 在 LK_Set_CurntPos_As_AnyAngle 中)
**@RetVal:	no
*****************************************************************************/
void LK_Station (int16_t ID, int16_t Station, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= Station;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[5]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[6]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[7]= 0x00;		

	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK brake change command
**@Cal;  	Internal or external
**@param:	no
**@Note:  	读取和改变抱闸器状态
**			读取———————————0x10
**			断电, 刹车开启——————0x00
**			通电, 刹车关闭——————0x01
**@RetVal:	no
*****************************************************************************/
void LK_Break_Device_Station (int16_t ID, int16_t effect, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0x8C;
	LK_Motor_Init_CanTxMsg.Data[1]= effect;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[5]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[6]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[7]= 0x00;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	MS电机开环控制指令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	powerControl为输入的开环电压，范围-850~ 850
**			仅MS电机有用
**@RetVal:	no
*****************************************************************************/
void LK_Open_Loop_Control (int16_t ID, int16_t powerControl, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA0;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&powerControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&powerControl)+1) ;
	LK_Motor_Init_CanTxMsg.Data[6]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[7]= 0x00;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的转矩闭环控制命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	int16_t iqControl ranges from -2048 to 2048	
**			对应MG电机实际转矩电流范围-33A~33A
**			仅MF,MH,MG电机有用
**@RetVal:	no
*****************************************************************************/
void LK_TorqueLoop_Out (int16_t ID, int16_t iqControl, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&iqControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&iqControl)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[7]= 0x00;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的速度闭环控制命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	int16_t iqControl ranges from -2048 to 2048	
**			对应MG电机实际转矩电流范围-33A~33A
**			int32_t speedControl, 实际物理单位为0.01dps/LSB
**@RetVal:	no
*****************************************************************************/
void LK_SpdLoop_Out (int16_t ID, int16_t iqControl, int32_t speedControl, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
	
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA2;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t *)(&iqControl);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(&iqControl)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&speedControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&speedControl)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&speedControl)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&speedControl)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的多圈位置闭环控制命令1
**@Cal;  	Internal or external
**@param:	no
**@Note:  	int32_t angleControl, 物理单位为0.01degree/LSB, 即36000代表360°
**			电机转动方向由目标位置和当前位置的差值决定
**@RetVal:	no
*****************************************************************************/
void LK_MultiturnPos_Out1 (int16_t ID, int32_t angleControl, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
			
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA3;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&angleControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&angleControl)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&angleControl)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&angleControl)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的多圈位置闭环控制命令2
**@Cal;  	Internal or external
**@param:	no
**@Note:  	int32_t angleControl, 物理单位为0.01degree/LSB, 即36000代表360°
**			uint16_t maxSpeed, 物理单位为1dps/LSB, 即360代表360dps
**			电机转动方向由目标位置和当前位置的差值决定
**@RetVal:	no
*****************************************************************************/
void LK_MultiturnPos_Out2 (int16_t ID, int32_t angleControl, uint16_t maxSpeed, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA4;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t *)(&maxSpeed);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(&maxSpeed)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&angleControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&angleControl)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&angleControl)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&angleControl)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的单圈位置闭环控制命令1
**@Cal;  	Internal or external
**@param:	no
**@Note:  	uint8_t spinDirection, 电机转动方向由该参数决定.0x00代表顺时针，0x01代表逆时针
**			uint32_t angleControl, 实际物理意义为0.01degree/LSB, 即36000代表360
**@RetVal:	no
*****************************************************************************/
void LK_SingleturnPos_Out1 (int16_t ID, uint8_t spinDirection, uint32_t angleControl, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA5;
	LK_Motor_Init_CanTxMsg.Data[1]= spinDirection;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&angleControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&angleControl)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&angleControl)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&angleControl)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的单圈位置闭环控制命令2
**@Cal;  	Internal or external
**@param:	no
**@Note:  	uint8_t spinDirection, 电机转动方向由该参数决定.0x00代表顺时针，0x01代表逆时针
**			uint32_t angleControl, 实际物理意义为0.01degree/LSB, 即36000代表360
**			控制值maxSpeed限制了电机转动的最大速度, 为uint32_t类型, 对应实际转速1dps/LSB, 即360代表 360dps
**			uint16_t maxSpeed, 360 代表 360dps
**@RetVal:	no
*****************************************************************************/
void LK_SingleturnPos_Out2 (int16_t ID, uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA6;
	LK_Motor_Init_CanTxMsg.Data[1]= spinDirection;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t*)(&maxSpeed);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(&maxSpeed)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&angleControl);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&angleControl)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&angleControl)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&angleControl)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的单圈位置闭环增量式控制命令1
**@Cal;  	Internal or external
**@param:	no
**@Note:  	int32_t anglelncrement, 实际物理意义为0.01degree/LSB，即36000代表360°，
**			电机转动方向由该参数的符号决定
**@RetVal:	no
*****************************************************************************/
void LK_SingleturnPos_Delta_Out1 (int16_t ID, int32_t anglelncrement, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA7;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&anglelncrement);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&anglelncrement)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&anglelncrement)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&anglelncrement)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的单圈位置闭环增量式控制命令2
**@Cal;  	Internal or external
**@param:	no
**@Note:  	int32_t anglelncrement, 实际物理意义为0.01degree/LSB，即36000代表360°，
**			电机转动方向由该参数的符号决定
**			uint32_t maxSpeed， 对应实际物理意义1dps/LSB，即360代表 360dps	
**@RetVal:	no
*****************************************************************************/
void LK_SingleturnPos_Delta_Out2 (int16_t ID, int32_t anglelncrement, uint32_t maxSpeed, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xA8;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t *)(&maxSpeed);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(&maxSpeed)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&anglelncrement);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(&anglelncrement)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(&anglelncrement)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(&anglelncrement)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的设置当前位置作为任意角度命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	no	
**@RetVal:	no
*****************************************************************************/
void LK_Set_CurntPos_As_AnyAngle (int16_t ID, int32_t motorAngle, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0x95;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(&motorAngle);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& motorAngle)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(& motorAngle)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& motorAngle)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的读取控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令读取当前电机的控制参数，读取的参数由序号controlParamID确定
**			0x0A——————角度环pid
**			0x0B——————速度环pid
**			0x0C——————电流环pid
**			0x1E——————最大力矩电流
**			0x20——————最大速度
**			0x22——————角度限制
**			0x24——————电流斜率
**			0x26——————速度斜率
**@RetVal:	no
*****************************************************************************/
void LK_Read_Control_Parameter (int16_t ID, int32_t controlParamID, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC0;
	LK_Motor_Init_CanTxMsg.Data[1]= controlParamID;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[5]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[6]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[7]= 0x00;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的角度环pid写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_AnglePid_Parameter (int16_t ID, uint16_t anglePidKp, uint16_t anglePidKi, uint16_t anglePidKd, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x0A;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t *)(& anglePidKp);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(& anglePidKp)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& anglePidKi);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& anglePidKi)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *(uint8_t *)(& anglePidKd);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& anglePidKd)+1);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的速度环pid写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_SpeedPid_Parameter (int16_t ID, uint16_t speedPidKp, uint16_t speedPidKi, uint16_t speedPidKd, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x0B;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t *)(& speedPidKp);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(& speedPidKp)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& speedPidKi);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& speedPidKi)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *(uint8_t *)(& speedPidKd);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& speedPidKd)+1);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的电流环pid写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_CurrentPid_Parameter (int16_t ID, uint16_t currentPidKp, uint16_t currentPidKi, uint16_t currentPidKd, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x0C;
	LK_Motor_Init_CanTxMsg.Data[2]= *(uint8_t *)(& currentPidKp);
	LK_Motor_Init_CanTxMsg.Data[3]= *((uint8_t *)(& currentPidKp)+1);
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& currentPidKi);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& currentPidKi)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *(uint8_t *)(& currentPidKd);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& currentPidKd)+1);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的最大力矩电流写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_inputTorqueMax_Parameter (int16_t ID, int16_t inputTorqueLimit, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x1E;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& inputTorqueLimit);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& inputTorqueLimit)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[7]= 0x00;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的最大速度写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_inputSpeedMax_Parameter (int16_t ID, int32_t inputSpeedLimit, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x20;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& inputSpeedLimit);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& inputSpeedLimit)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(& inputSpeedLimit)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& inputSpeedLimit)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的角度限制写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_inputAngleLimit_Parameter (int16_t ID, int32_t inputAngleLimit, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x22;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& inputAngleLimit);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& inputAngleLimit)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(& inputAngleLimit)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& inputAngleLimit)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的电流斜率写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_inputCurrentRamp_Parameter (int16_t ID, int32_t inputCurrentRamp, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x24;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& inputCurrentRamp);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& inputCurrentRamp)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(& inputCurrentRamp)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& inputCurrentRamp)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的速度斜率写入控制参数命令
**@Cal;  	Internal or external
**@param:	no
**@Note:  	主机发送该命令写入控制参数到RAM中，即时生效，断电后失效，
**@RetVal:	no
*****************************************************************************/
void LK_Control_inputSpeedRamp_Parameter (int16_t ID, int32_t inputSpeedRamp, CAN_TypeDef *CANx)
{
	CanTxMsg LK_Motor_Init_CanTxMsg;
	
	LK_Motor_Init_CanTxMsg.StdId= (0x140 + ID);
	LK_Motor_Init_CanTxMsg.IDE 	= CAN_Id_Standard;
	LK_Motor_Init_CanTxMsg.RTR 	= CAN_RTR_Data;
	LK_Motor_Init_CanTxMsg.DLC 	= 0x08;
		
	LK_Motor_Init_CanTxMsg.Data[0]= 0xC1;
	LK_Motor_Init_CanTxMsg.Data[1]= 0x26;
	LK_Motor_Init_CanTxMsg.Data[2]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[3]= 0x00;
	LK_Motor_Init_CanTxMsg.Data[4]= *(uint8_t *)(& inputSpeedRamp);
	LK_Motor_Init_CanTxMsg.Data[5]= *((uint8_t *)(& inputSpeedRamp)+1);
	LK_Motor_Init_CanTxMsg.Data[6]= *((uint8_t *)(& inputSpeedRamp)+2);
	LK_Motor_Init_CanTxMsg.Data[7]= *((uint8_t *)(& inputSpeedRamp)+3);
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&LK_Motor_Init_CanTxMsg));	
}

/*****************************************************************************
**@Brief;	LK的基本电机信息读取函数
**@Cal;  	Internal
**@param:	no
**@Note:  	用在 LK_Information_Receive 函数内
**@RetVal:	no
*****************************************************************************/
void LK_Basic_Informaion_Read (int count, CanRxMsg *msg, volatile LK_InitTypeDef *LK)
{
	LK_EncoderProcess(LK,msg);
	if(count <= 100)//进入函数的次数不大于100次的话
	{
		//随着接收值
		if((LK->ecd_bias - LK->ecd_value) < -30000)
		{
			LK->ecd_bias= 32768 + 65536;
		}
		else if((LK->ecd_bias - LK->ecd_value) > 30000)  
		{
			LK->ecd_bias= 32768 - 65536;
		}
	}
	
	//Get the initial deviation value of the encoder
	if(count <= 50)	{LK_GetEncoderBias(LK, msg);}
	else			{LK_EncoderProcess(LK, msg);}
}

/*****************************************************************************
**@Brief;	LK的电机参数信息读取函数
**@Cal;  	Internal
**@param:	no
**@Note:  	用在 LK_Information_Receive 函数内
**@RetVal:	no
*****************************************************************************/
void LK_Parameter_Read (CanRxMsg *msg, volatile LK_InitTypeDef *LK)
{
	switch (msg->Data[1])
	{
		//读取角度PID
		case (0x0A):
		{
			LK->LK_Control_Parameter.Angleloop_PID.Kp= (uint16_t)((msg->Data[3] << 8) | msg->Data[2]);
			LK->LK_Control_Parameter.Angleloop_PID.Ki= (uint16_t)((msg->Data[5] << 8) | msg->Data[4]);
			LK->LK_Control_Parameter.Angleloop_PID.Kd= (uint16_t)((msg->Data[7] << 8) | msg->Data[6]);
			
			break;
		}
			
		//读取转速PID
		case (0x0B):
		{
			LK->LK_Control_Parameter.RotSpdloop_PID.Kp= (uint16_t)((msg->Data[3] << 8) | msg->Data[2]);
			LK->LK_Control_Parameter.RotSpdloop_PID.Ki= (uint16_t)((msg->Data[5] << 8) | msg->Data[4]);
			LK->LK_Control_Parameter.RotSpdloop_PID.Kd= (uint16_t)((msg->Data[7] << 8) | msg->Data[6]);
			
			break;
		}
			
		//读取电流PID
		case (0x0C):
		{
			LK->LK_Control_Parameter.Curentloop_PID.Kp= (uint16_t)((msg->Data[3] << 8) | msg->Data[2]);
			LK->LK_Control_Parameter.Curentloop_PID.Ki= (uint16_t)((msg->Data[5] << 8) | msg->Data[4]);
			LK->LK_Control_Parameter.Curentloop_PID.Kd= (uint16_t)((msg->Data[7] << 8) | msg->Data[6]);
			
			break;
		}
			
		//读取最大力矩电流
		case (0x1E):
		{
			LK->LK_Control_Parameter.TorqueCurrentLimit= (int16_t)((msg->Data[5] << 8) | msg->Data[4]);
			
			break;
		}

		//读取最大速度
		case (0x20):
		{
			LK->LK_Control_Parameter.SpeedLimit |= (int32_t)msg->Data[4] << 0;
			LK->LK_Control_Parameter.SpeedLimit |= (int32_t)msg->Data[5] << 8;
			LK->LK_Control_Parameter.SpeedLimit |= (int32_t)msg->Data[6] << 16;
			LK->LK_Control_Parameter.SpeedLimit |= (int32_t)msg->Data[7] << 24;
			
			break;
		}

		//读取角度限制
		case (0x22):
		{
			LK->LK_Control_Parameter.Anglelimit |= (int32_t)msg->Data[4] << 0;
			LK->LK_Control_Parameter.Anglelimit |= (int32_t)msg->Data[5] << 8;
			LK->LK_Control_Parameter.Anglelimit |= (int32_t)msg->Data[6] << 16;
			LK->LK_Control_Parameter.Anglelimit |= (int32_t)msg->Data[7] << 24;

			break;
		}			

		//读取电流斜率
		case (0x24):
		{
			LK->LK_Control_Parameter.CurrentSlope |= (int32_t)msg->Data[4] << 0;
			LK->LK_Control_Parameter.CurrentSlope |= (int32_t)msg->Data[5] << 8;
			LK->LK_Control_Parameter.CurrentSlope |= (int32_t)msg->Data[6] << 16;
			LK->LK_Control_Parameter.CurrentSlope |= (int32_t)msg->Data[7] << 24;

			break;
		}	

		//读取速度斜率
		case (0x26):
		{
			LK->LK_Control_Parameter.SpeedSlope |= (int32_t)msg->Data[4] << 0;
			LK->LK_Control_Parameter.SpeedSlope |= (int32_t)msg->Data[5] << 8;
			LK->LK_Control_Parameter.SpeedSlope |= (int32_t)msg->Data[6] << 16;
			LK->LK_Control_Parameter.SpeedSlope |= (int32_t)msg->Data[7] << 24;
			
			break;
		}						
	}
}

/*****************************************************************************
**@Brief:	Obtain deviation
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void LK_GetEncoderBias (volatile LK_InitTypeDef *LK, CanRxMsg* msg)
{
    LK->ecd_bias		= (uint16_t)((msg->Data[7] << 8) | msg->Data[6]);;	//Save the initial encoded value as the bias
    LK->ecd_value 		= LK->ecd_bias;										//The continuous encoder value after processing == the saved initial encoder value
    LK->last_raw_value 	= LK->ecd_bias;										//Update the previous raw encoder value
    LK->temp_count++;														//count
}

/*****************************************************************************
**@Brief:	Encoding to obtain continuous values
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void LK_EncoderProcess (volatile LK_InitTypeDef *LK, CanRxMsg* msg)
{
	static int count_lk= 1;
		
	int i= 0;
	float rate_sum= 0;
	count_lk ++;
	
	LK->last_raw_value= LK->raw_value;									//Update the last return value
	LK->raw_value  = (uint16_t)((msg->Data[7] << 8) | msg->Data[6]);	//Update this return value
	LK->diff= LK->raw_value - LK->last_raw_value;						//Update the difference between two return values
	
	//读取电机转矩电流
	LK->Current	   = (int16_t)((msg->Data[3] << 8) | msg->Data[2]);
	//读取电机转速
	LK->RotSpd	   = (int16_t)((msg->Data[5] << 8) | msg->Data[4]);
	
	//Judge whether the number of turns has changed. If the difference between the two return values is too large, it means that the number of turns has changed.
	if(LK->diff < -32768)              	
	{
		LK->round_cnt++;
		LK->ecd_raw_rate= LK->diff + 65536;
	}
	else if(LK->diff > 32768)
	{
		LK->round_cnt--;
		LK->ecd_raw_rate= LK->diff- 65536;
	}		
	else
	{
		LK->ecd_raw_rate= LK->diff;
	}
	
	//Calculate the continuous encoder output value (make the encoding range be from positive infinity to negative infinity)
	LK->ecd_value= LK->raw_value + (LK->round_cnt * 65536);
	
	//Calculate the angle value and the range is from positive infinity to negative infinity
	LK->ecd_angle= (float)((LK->raw_value - LK->ecd_bias)*0.0054931641f + (LK->round_cnt * 360.0f)) /10;
		
	//Original value of storage speed (store six)
	LK->rate_buf[LK->buf_count++]= LK->ecd_raw_rate;	//First use the value of buf_count and then increment it
	if(LK->buf_count == 6)
	{
		LK->buf_count= 0;
	}

	
	//Calculate the average speed
	for(i = 0;i < 6; i++)
	{ 
		rate_sum += LK->rate_buf[i];
	}
	LK->filter_rate= (int32_t)(rate_sum/6);	
//	V->filter_rate= (int32_t)(rate_sum/3);

	//Obtain motor temperature
	LK->Temperature_Rotor= (int8_t)msg->Data[1];
}

/*****************************************************************************
**@Brief:	Encoding to obtain continuous values
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void LK_Parameter_Init (volatile LK_InitTypeDef *v, int ID)
{
	v->ID= ID;
	
	v->LK_ERR_e= MOTOR_NORMAL;
	
	v->Motor_State= LK_STANDBY;
	
	v->Brake_Dvice= BD_OFF;
	
	v->LK_Control_Parameter.Angleloop_PID.Kp=  0;
	v->LK_Control_Parameter.Angleloop_PID.Ki=  0;
	v->LK_Control_Parameter.Angleloop_PID.Kd=  0;

	v->LK_Control_Parameter.RotSpdloop_PID.Kp=  0;
	v->LK_Control_Parameter.RotSpdloop_PID.Ki=  0;
	v->LK_Control_Parameter.RotSpdloop_PID.Kd=  0;

	v->LK_Control_Parameter.Curentloop_PID.Kp=  0;
	v->LK_Control_Parameter.Curentloop_PID.Ki=  0;
	v->LK_Control_Parameter.Curentloop_PID.Kd=  0;
	
	v->LK_Control_Parameter.TorqueCurrentLimit= 0;
	
	v->LK_Control_Parameter.SpeedLimit=	  0;
	
	v->LK_Control_Parameter.Anglelimit=   0;
	
	v->LK_Control_Parameter.CurrentSlope= 0;
	
	v->LK_Control_Parameter.SpeedSlope=   0;
	
	v->Bus_Voltage= 0;
	v->Bus_Current= 0;
	
	v->Current= 0;
	
	v->RotSpd= 0;
	v->rate_buf[0]= 0;
	v->rate_buf[1]= 0;
	v->rate_buf[2]= 0;
	v->rate_buf[3]= 0;
	v->rate_buf[4]= 0;
	v->rate_buf[5]= 0;
	v->buf_count=   0;
	v->ecd_raw_rate=0;
	v->filter_rate= 0;
	
	v->Temperature_Rotor= 0;
	
	v->encoder_offset=      0;
	v->originate_raw_value= 0;
	
	v->temp_count= 		0;
	v->diff=     		0;
	v->ecd_bias= 		0;
	v->last_raw_value= 	0;
	v->raw_value= 		0;
	v->ecd_value= 		0;
	
	v->uncal_cont_angle= 0;
	v->single_angle= 	 0;
	v->ecd_angle= 		 0;
	
	v->round_cnt= 0;
	
	v->Current_APhase= 0;
	v->Current_BPhase= 0;
	v->Current_CPhase= 0;
}


void LK4005_Encoder_To_Generic_Encoder(LK_InitTypeDef* LK,Encoder_t* Encoder)
{
    
}
