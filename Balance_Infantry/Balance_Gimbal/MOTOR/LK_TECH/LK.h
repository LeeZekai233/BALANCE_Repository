/**
  ******************************************************************************
  * @file    EMBEDDED\Motor\LK\LK.h
  * @author  William
  * @version V1.0.0
  * @date    17-April-2025
  * @brief   The file contains the headers of LK Motor configuration  
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LK_H
#define __LK_H
#include "stm32f4xx.h"                  // Device header
/*******************************************************************************#definition***************************************************************************************/


/****************************************************************************Enum Definition**************************************************************************************/


/****************************************************************************Struct Definition************************************************************************************/

#ifndef GENERIC_ENCODER
#define GENERIC_ENCODER
typedef struct
{
    
    float Angle_Deg_fdb;	         //单圈角度反馈   单位°
    float Angle_Deg_Total_fdb;       //多圈角度反馈   单位°
    
    float Angle_Rad_fdb;             //单圈角度反馈   单位rad
    float Angle_Rad_Total_fdb;       //多圈角度反馈   单位rad
    
    float Omega_Rad_fdb;		//电机转速反馈  单位rad/s
    
    uint8_t online_flag;
    
    float Torque;             //力矩
    
    uint32_t temperature;    //温度
    
    uint32_t heart_cnt;//
}Encoder_t;//通用编码器
#endif


typedef __packed struct
{
	int16_t  ID;
	
	enum
	{
		MOTOR_NORMAL 					= 0,			//正常
		U_DEFICIENCY	 		= 1,			//低压保护
		U_OVERLOAD			 	= 2,			//高压保护
		DRIVE_TEM_OVER			= 3,			//驱动过温
		MOTOR_TEM_OVER			= 4,			//电机过温
		I_OVERLOAD			 	= 5,			//电机过流
		MOTOR_SHORT_CIRCUIT	 	= 6,			//电机短路
		MOTOR_STALL			 	= 7,			//电机堵转
		INPUT_SIGNAL_LOSSTIMEOUT= 8, 			//输入信号丢失超时
	}LK_ERR_e;							//故障类型
	
	enum
	{
		LK_OFF,
		LK_STANDBY,
		LK_ON,
	}Motor_State;						//电机是否开启的标志位
	
	enum
	{
		BD_OFF,
		BD_ON,
	}Brake_Dvice;						//抱闸器是否开启的标志位
	
	  __packed	struct
	{
		  __packed	struct
		{
			uint16_t Kp;
			uint16_t Ki;
			uint16_t Kd;		
		}Angleloop_PID;					//参数控制——角度环PID
		
		  __packed	struct
		{
			uint16_t Kp;
			uint16_t Ki;
			uint16_t Kd;
		}RotSpdloop_PID;				//参数控制——转速环PID
		
		  __packed	struct
		{
			uint16_t Kp;
			uint16_t Ki;
			uint16_t Kd;
		}Curentloop_PID;				//电流环PID

		int16_t TorqueCurrentLimit;		//最大力矩电流限制

		int32_t SpeedLimit;				//最大速度限制

		int32_t Anglelimit;				//最大角度限制
		
		int32_t CurrentSlope;			//电流斜率
		
		int32_t SpeedSlope;				//速度斜率
		
	}LK_Control_Parameter;				//控制参数
	
	int16_t Bus_Voltage;				//母线电压, 0.01V/LSB
	int16_t Bus_Current;				//母线电流, 0.01A/LSB，MS系列为输出功率
	
	int16_t Current;					//转矩电流值
	
	int16_t RotSpd;						//电机转速 1dps/LSB
	int32_t rate_buf[6];	            //Store the original speed for each time [a total of 6]
	uint8_t buf_count;					//Filtering update  Used for receiving values
	int32_t ecd_raw_rate;				//The original value of speed calculated by the encoder
	int32_t filter_rate;				//(continuous/average)[current]speed
	
	int8_t  Temperature_Rotor;			//电机温度，单位℃/LSB	
	
	int32_t encoder_offset;				//Encoder zero offset	
	int32_t originate_raw_value;		//the real encoder data
	
	//Be used to calculate Continuous encoder value
	int32_t temp_count;                 //For counting
	int32_t diff;						//The difference between two encoders
	int32_t ecd_bias;					//Bias of encoder	
	int32_t last_raw_value;				//Last raw value of encoder
	int32_t raw_value;   				//Raw value of encoder without processing
	//(actually raw_value is the real encoder value subtract the Encoder zero offset)
	int32_t ecd_value;                  //Continuous encoder value after processing
	
	int64_t uncal_cont_angle;			//Continuous angle directly comes from motor
	int64_t single_angle;				//Single angle directly comes from motor
	float   ecd_angle;					//Continuous angle after encoder processing

	int32_t round_cnt;

	int16_t Current_APhase;				//A相电流
	int16_t Current_BPhase;				//B相电流(MG电机相电流分辨率为(66/4096 A) / LSB)
	int16_t Current_CPhase;				//C相电流

}LK_InitTypeDef;


/****************************************************************************Extern variable**************************************************************************************/
extern LK_InitTypeDef Poke_MG4005;
/****************************************************************************Extern Function**************************************************************************************/


/****************************************************************************Function Declaration*********************************************************************************/	
void LK_Information_Receive (int count, CanRxMsg *msg, int16_t ID, volatile LK_InitTypeDef *LK);
void LK_Station (int16_t ID, int16_t Station, CAN_TypeDef *CANx);
void LK_Break_Device_Station (int16_t ID, int16_t effect, CAN_TypeDef *CANx);
void LK_Open_Loop_Control (int16_t ID, int16_t powerControl, CAN_TypeDef *CANx);
void LK_TorqueLoop_Out (int16_t ID, int16_t iqControl, CAN_TypeDef *CANx);
void LK_SpdLoop_Out (int16_t ID, int16_t iqControl, int32_t speedControl, CAN_TypeDef *CANx);
void LK_MultiturnPos_Out1 (int16_t ID, int32_t angleControl, CAN_TypeDef *CANx);
void LK_MultiturnPos_Out2 (int16_t ID, int32_t angleControl, uint16_t maxSpeed, CAN_TypeDef *CANx);
void LK_SingleturnPos_Out1 (int16_t ID, uint8_t spinDirection, uint32_t angleControl, CAN_TypeDef *CANx);
void LK_SingleturnPos_Out2 (int16_t ID, uint8_t spinDirection, uint32_t angleControl, uint16_t maxSpeed, CAN_TypeDef *CANx);
void LK_SingleturnPos_Delta_Out1 (int16_t ID, int32_t anglelncrement, CAN_TypeDef *CANx);
void LK_SingleturnPos_Delta_Out2 (int16_t ID, int32_t anglelncrement, uint32_t maxSpeed, CAN_TypeDef *CANx);
void LK_Set_CurntPos_As_AnyAngle (int16_t ID, int32_t motorAngle, CAN_TypeDef *CANx);
void LK_Read_Control_Parameter (int16_t ID, int32_t controlParamID, CAN_TypeDef *CANx);
void LK_Control_AnglePid_Parameter (int16_t ID, uint16_t anglePidKp, uint16_t anglePidKi, uint16_t anglePidKd, CAN_TypeDef *CANx);
void LK_Control_SpeedPid_Parameter (int16_t ID, uint16_t speedPidKp, uint16_t speedPidKi, uint16_t speedPidKd, CAN_TypeDef *CANx);
void LK_Control_CurrentPid_Parameter (int16_t ID, uint16_t currentPidKp, uint16_t currentPidKi, uint16_t currentPidKd, CAN_TypeDef *CANx);
void LK_Control_inputTorqueMax_Parameter (int16_t ID, int16_t inputTorqueLimit, CAN_TypeDef *CANx);
void LK_Control_inputSpeedMax_Parameter (int16_t ID, int32_t inputSpeedLimit, CAN_TypeDef *CANx);
void LK_Control_inputAngleLimit_Parameter (int16_t ID, int32_t inputAngleLimit, CAN_TypeDef *CANx);
void LK_Control_inputCurrentRamp_Parameter (int16_t ID, int32_t inputCurrentRamp, CAN_TypeDef *CANx);
void LK_Control_inputSpeedRamp_Parameter (int16_t ID, int32_t inputSpeedRamp, CAN_TypeDef *CANx);
void LK_Basic_Informaion_Read (int count, CanRxMsg *msg, volatile LK_InitTypeDef *LK);
void LK_Parameter_Read (CanRxMsg *msg, volatile LK_InitTypeDef *LK);
void LK_GetEncoderBias (volatile LK_InitTypeDef *LK, CanRxMsg* msg);
void LK_EncoderProcess (volatile LK_InitTypeDef *LK, CanRxMsg* msg);
void LK_Parameter_Init (volatile LK_InitTypeDef *v, int ID);
void LK4005_Encoder_To_Generic_Encoder(LK_InitTypeDef* LK,Encoder_t* Encoder);

#endif
