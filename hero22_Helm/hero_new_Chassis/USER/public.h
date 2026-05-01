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

#include "usart.h"
#include "can.h"
#include "stm32f4xx_conf.h"
#include "configuration.h"
#include "BSP.h"
#include "Dj6020.h"
#include "timer.h"
#include "sensor.h"
#include "string.h"
#include "CanBus.h"
#include "pid.h"
#include "CH040.h"


#include <signal.h>
#include <data_sample.h>
#include "signal.h"


#include "control_task.h"

#include "chassis_task.h"
#include "remote.h"
#include "can_send_task.h"

#include "Super_Cap.h"
#include "power_task.h"
#include "CAN_Gimbal_Transmit.h"

#include "Fault_Diagnosis_Task.h"

