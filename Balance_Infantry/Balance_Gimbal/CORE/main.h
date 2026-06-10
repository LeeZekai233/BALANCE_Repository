#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"                  // Device header
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>


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
#include "Control_Task.h"
#include "Board_Connected_Task.h"
#include "Remote_Task.h"
#include "Can_Bus_Task.h"
#include "Gimbal_Task.h"
#include "Shooter_Task.h"


//Judge_System
#include "Judge_System.h"


//MOTOR
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "LK.h"


//Control_algorithm
#include "PID.h"
#include "Ramp.h"
#include "Low_Pass_Filter.h"
#include "High_Pass_Filter.h"
#include "Signal.h"
#include "Shooter_Kalman_Filter.h"


#define PI 3.141593f


#endif

