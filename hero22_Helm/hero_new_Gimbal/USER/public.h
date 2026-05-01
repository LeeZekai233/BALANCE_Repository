#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#define PI  3.14159265358979f
#define VTM_RC_FRAME_LENGTH  21u
#define VTM_REMOTE_USART6_DMA_RX_BUF_LEN	152u
#define BSP_USART6_DMA_RX_BUF_LEN 				VTM_REMOTE_USART6_DMA_RX_BUF_LEN
#define BSP_USART6_RX_BUF_LENGTH  152

#include "stm32f4xx.h"                  // Device header
#include <stdio.h>

#include <math.h>

#include <LESO.h>

#include "can.h"
#include "stm32f4xx_conf.h"
#include "configuration.h"
#include "BSP.h"
#include "Dj6020.h"
#include "timer.h"
#include "usart.h"
#include "ch040.h"
#include "sensor.h"
#include "string.h"
#include "CanBus.h"
#include "pid.h"


#include <signal.h>
#include <data_sample.h>
#include "signal.h"


#include "control_task.h"

#include "chassis_task.h"
#include "remote.h"
#include "remote_ctrl.h"
#include "gimbal_task.h"
#include "LK_MG5010.h"
#include "LK_new.h"
#include "Hero_Mode_Select_Task.h"
#include "can_send_task.h"
#include "42mm_shoot_task.h"
#include "VTM.h"
#include "DM4310.h"
#include "Radar.h"

#include "LESO.h"
#include "auto_shoot.h"
#include "judge.h"
#include "UI.h"
#include "Super_Cap.h"
#include "power_task.h"
#include "CAN_Chassis_Transmit.h"

#include "Fault_Diagnosis_Task.h"

