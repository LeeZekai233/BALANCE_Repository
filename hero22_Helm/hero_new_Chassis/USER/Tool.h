///**
//  ******************************************************************************
//  * @file    BAL\Tool\Tool.h 
//  * @author  William
//  * @version V1.0.0
//  * @date    17-April-2025
//  * @brief   no  
//  ******************************************************************************
//  * @attention
//  *
//  ******************************************************************************
//  */
//  
///* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef __TOOL_H
//#define __TOOL_H
//#include "stm32f4xx.h"  
//#include <math.h>


///*******************************************************************************#definition***************************************************************************************/
//#define ln2		0.69315f
//#define PI  	3.1416f
//#define PI2		6.2832f
//#define Encoder_16bit_Angle_Conversion_Ratio 0.0054932f

///****************************************************************************Enum Definition**************************************************************************************/
//typedef enum {DeSys_To_RaSys= 0, RaSys_To_DeSys= 1}Angle_TransformSystrm;//Degree system to radian system, radian system to Degree system
//typedef enum {Positive= 0, Reverse= 1}CoordinateSystem_Trasform;
//typedef enum {NOW= 0, LAST= 1, LLAST= 2,}range;


///****************************************************************************Struct Definition************************************************************************************/
//typedef struct {double Alpha; double Beta; double Gamma;}CS_TrsforAngle_InitTypeDef;
//typedef struct {double x; double y; double z;}CoordinateSystem_InitTypeDef;


///****************************************************************************Extern variable**************************************************************************************/


///****************************************************************************Extern Function**************************************************************************************/


///****************************************************************************Function Declaration*********************************************************************************/
//float Transfor_Original_Data_To_Meaningful_Data(int x_int, float x_min, float x_max, int bits) ;int isLittleEndian(uint32_t a);
//double factorial (int n);
//int Judge (double num);
//double Angle_Trasnform (double num, int transform_system);
//void CoordinateSystem_PosTransform (volatile void* o, int change, volatile void* c, volatile void* a);
//float Nearest_Neighbor_Search (float originate_value, float positive_value, float reverse_value, float range);

//	
//#endif