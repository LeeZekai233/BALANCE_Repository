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

#include "USART1.h"
#include "USART3.h"                                                                  
#include "USART4.h"
#include "USART6.h"
#include "TIM6.h"
#include "TIM2.h"
#include "CAN1.h"
#include "CAN2.h"
#include "BSP.h"

#include "Remote_Task.h"
#include "Chassis_Task.h"
#include "Control_Task.h"
#include "Leg_Task.h"
#include "Board_Connected_Task.h"
#include "Whell_Observe_Task.h"
#include "Judge_System.h"

#include "DaMiao_8009.h"
#include "DJI_Motor.h"
#include "Generic_Encoder.h"

#include "PID.h"

#endif

