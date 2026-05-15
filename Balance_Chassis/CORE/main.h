#ifndef __MAIN_H__
#define __MAIN_H__


#include <stm32f4xx.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "arm_math.h"


//SYSTEM
#include "USART1.h"
#include "USART2.h"
#include "USART3.h"                                                                  
#include "USART4.h"
#include "USART6.h"
#include "TIM6.h"
#include "TIM2.h"
#include "CAN1.h"
#include "CAN2.h"
#include "BSP.h"


//TASK
#include "Chassis_Task.h"
#include "Control_Task.h"
#include "Leg_Task.h"
#include "Board_Connected_Task.h"
#include "Remote_Task.h"
#include "Whell_Observe_Task.h"
#include "Log_Task.h"
#include "Can_Bus_Task.h"
#include "Super_Cap_Task.h"


//Judge_System
#include "Judge_System.h"


//MOTOR
#include "DaMiao_8009.h"
#include "DJI_Motor.h"


//Control_algorithm
#include "PID.h"
#include "Ramp.h"
#include "Low_Pass_Filter.h"
#include "High_Pass_Filter.h"
#include "Signal.h"




#endif

