///**
//  ******************************************************************************
//  * @file    BAL\Tool\Tool.c 
//  * @author  William
//  * @version V1.0.0
//  * @date    17-April-2025
//  * @brief   no
//  ******************************************************************************
//  * @attention
//  ******************************************************************************
//  */

///* Includes ------------------------------------------------------------------*/
//#include "Tool.h"
//#include "stm32f4xx_conf.h"

////三角函数
////double __cdecl sin(double _X);     // 计算正弦值（输入为弧度）
////double __cdecl cos(double _X);     // 计算余弦值（输入为弧度）
////double __cdecl tan(double _X);     // 计算正切值（输入为弧度）

////反三角函数
////double __cdecl asin(double _X);    // 计算反正弦值（返回值为弧度）
////double __cdecl acos(double _X);    // 计算反余弦值（返回值为弧度）
////double __cdecl atan(double _X);    // 计算反正切值（返回值为弧度）
////double __cdecl atan2(double _Y, double _X); // 计算Y/X的反正切值（返回值为弧度）

////双曲函数
////double __cdecl sinh(double _X);    // 计算双曲正弦值
////double __cdecl cosh(double _X);    // 计算双曲余弦值
////double __cdecl tanh(double _X);    // 计算双曲正切值

////指数与对数函数
////double __cdecl exp(double _X);     // 计算e的X次幂（e^X）
////double __cdecl log(double _X);     // 计算自然对数（以e为底）
////double __cdecl log10(double _X);   // 计算常用对数（以10为底）

////幂函数与平方根函数
////double __cdecl pow(double _X, double _Y); // 计算X的Y次幂（X^Y）
////double __cdecl sqrt(double _X);    // 计算平方根

////取整函数
////double __cdecl ceil(double _X);    // 向上取整，返回不小于X的最小整数
////double __cdecl floor(double _X);   // 向下取整，返回不大于X的最大整数

////绝对值函数
////double fabs(double x);       // 计算 double 类型的绝对值
////float fabsf(float x);        // 计算 float 类型的绝对值（C99新增）
////long double fabsl(long double x); // 计算 long double 类型的绝对值（C99新增）
///*****************************************************************************
//**@Brief:	转换外设的原始数据为有实际意义的数据
//**@Cal:		no
//**@param:  	no
//**@Note:   	no
//**@RetVal: 	no
//*****************************************************************************/
//float Transfor_Original_Data_To_Meaningful_Data(int x_int, float x_min, float x_max, int bits) 
//{
//	 //converts unsigned int to float, given range and number of bits
//	 float span = x_max- x_min;

//	 float offset = x_min;
//	 return (((float)x_int) * span / ((float)((1<<bits)-1))) + offset;
//}

///*****************************************************************************
//**@Brief:	检测大小端字序
//**@Cal:		no
//**@param:  	a必须为1
//**@Note:   	no
//**@RetVal: 	返回值=1, 为小端字节序;反之, 则为大端字节序
//*****************************************************************************/
//int isLittleEndian(uint32_t a) 
//{
//    return (*(uint8_t *)&a == 1);
//}

///**
//************************************************************************************************************************
//* @Name     : fputc/_sys_exit
//* @brief    : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
//* @param    : ch
//* @param    : FILE *f
//* @retval   : void
//* @Note     : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
//************************************************************************************************************************
//**/ 
////#pragma import(__use_no_semihosting) 
//////标准库需要的支持函数                 
////struct __FILE 
////{ 
////	int handle; 
////}; 

////FILE __stdout;       
//////定义_sys_exit()以避免使用半主机模式    
////void _sys_exit(int x) 
////{ 
////	x = x; 
////} 
//////重定义fputc函数 
////int fputc(int ch, FILE *f)
////{ 	
////	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
////	USART2 ->DR = (u8) ch;      
////	return ch;
////}

///*****************************************************************************
//**@Brief:	计算阶乘
//**@Cal:		no
//**@param:  	no
//**@Note:   	no
//**@RetVal: 	no
//*****************************************************************************/
//double factorial (int n) 
//{
//	double result= 1.0f;
//	for (int i = 1; i <= n; i++) { result *= i;}
//    return result;
//}

///*****************************************************************************
//**@Brief:	判断正负(0单独作为返回值)
//**@Cal:		no
//**@param:  	no
//**@Note:   	弧度制计算
//**@RetVal: 	no
//*****************************************************************************/
//int Judge (double num)
//{
//	int flag= 0;
//	
//	if (num > 0) { flag= 1;}
//	else if (num < 0) { flag= -1;}
//	
//	return flag;
//}

///*****************************************************************************
//**@Brief:	角度制转换
//**@Cal:		no
//**@param:  	num是传入的值; transform_system是个枚举变量, 分别是角度到弧度和弧度到角度
//**@Note:   	no
//**@RetVal: 	no
//*****************************************************************************/
//double Angle_Trasnform (double num, int transform_system)
//{
//	switch (transform_system)
//	{
//		case DeSys_To_RaSys:	{ num= num * (PI / 180); /**单位为rad/s**/ break;}
//			
//		case RaSys_To_DeSys:	{ num= (num / PI) * 180; /**单位为°**/ break;}
//	}
//	
//	return num;
//}

///*****************************************************************************
//**@Brief:	坐标系转换(自然坐标系与非自然坐标系)
//**@Cal:		no
//**@param:  	o——自然坐标系结构体, 
//**			change——转换方向
//**			Positive——正向，即自然坐标系转换到非自然坐标系; 
//**			Reverse——逆向, 即非自然坐标系转换到自然坐标系;
//**			c——非自然坐标系结构体, 
//**			a——自然坐标系三个个坐标轴以不同面为基准面时的转动角度结构体
//**@Note:   	以xOy为基准面，此时z轴与坐标原点重合，其转动角度为γ;
//**			以zOx为基准面，此时y轴与坐标原点重合，其转动角度为β;
//**			以yOz为基准面，此时x轴与坐标原点重合，其转动角度为α;
//**			原坐标系中点的坐标为(x, y, z), 转换后坐标系中点的坐标为(x_, y_, z_)
//**			结构体为匿名结构体, 只要满足传入结构体的结构与该函数的结构体形式一致即可解算
//**			任意两个坐标系之间也可以转换，只要知道其一坐标系的坐标以及相对另一个坐标系的轴的旋转角度即可
//**@RetVal: 	
//*****************************************************************************/
//void CoordinateSystem_PosTransform (volatile void* o, int change, volatile void* c, volatile void* a)
//{
//	CoordinateSystem_InitTypeDef *Natural_CS= (CoordinateSystem_InitTypeDef* )o;
//	CoordinateSystem_InitTypeDef *Unnatrl_CS= (CoordinateSystem_InitTypeDef* )c;
//	CS_TrsforAngle_InitTypeDef	 *Trsfor_Ang= (CS_TrsforAngle_InitTypeDef*	 )a;
//	
//	switch (change)
//	{
//		case (Positive):
//		{	
////			//yOz为基准面，进行α角度的转动
////			double Unnatrl_CS_xAlpha= + 1 * Natural_CS->x + 0					    * Natural_CS->y - 0 		  			  * Natural_CS->z;
////			double Unnatrl_CS_yAlpha= + 0 * Natural_CS->x + cos(Trsfor_Ang->Alpha)  * Natural_CS->y + sin(Trsfor_Ang->Alpha ) * Natural_CS->z;
////			double Unnatrl_CS_zAlpha= + 0 * Natural_CS->x - sin(Trsfor_Ang->Alpha)	* Natural_CS->y + cos(Trsfor_Ang->Alpha ) * Natural_CS->z;
////			
////			//zOx为基准面，进行β角度的转动
////			double Unnatrl_CS_xBeta= + cos(Trsfor_Ang->Beta) * Natural_CS->x + 0 * Natural_CS->y - sin(Trsfor_Ang->Beta) * Natural_CS->z;
////			double Unnatrl_CS_yBeta= + 0					 * Natural_CS->x + 1 * Natural_CS->y + 0 					 * Natural_CS->z;
////			double Unnatrl_CS_zBeta= + sin(Trsfor_Ang->Beta) * Natural_CS->x + 0 * Natural_CS->y + cos(Trsfor_Ang->Beta) * Natural_CS->z;
////			
////			//xOy为基准面，进行γ角度的转动
////			double Unnatrl_CS_xGamma= + cos(Trsfor_Ang->Gamma) * Natural_CS->x + sin(Trsfor_Ang->Gamma) * Natural_CS->y + 0 * Natural_CS->z;
////			double Unnatrl_CS_yGamma= - sin(Trsfor_Ang->Gamma) * Natural_CS->x + cos(Trsfor_Ang->Gamma) * Natural_CS->y + 0 * Natural_CS->z;
////			double Unnatrl_CS_zGamma= + 0					   * Natural_CS->x + 0						* Natural_CS->y + 1 * Natural_CS->z;
//			
//			//分别得到三个轴的旋转矩阵后, 按照γ, β, α的顺序来算三维旋转矩阵			
//			Unnatrl_CS->x= +(cos(Trsfor_Ang->Gamma) * cos(Trsfor_Ang->Beta)) * Natural_CS->x
//						   +((sin(Trsfor_Ang->Gamma) * cos(Trsfor_Ang->Alpha)) - (cos(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta) * sin(Trsfor_Ang->Alpha))) * Natural_CS->y
//						   +((sin(Trsfor_Ang->Alpha) * sin(Trsfor_Ang->Gamma)) + (cos(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta) * cos(Trsfor_Ang->Alpha))) * Natural_CS->z;
//			
//			Unnatrl_CS->y= -(sin(Trsfor_Ang->Gamma) * cos(Trsfor_Ang->Beta)) * Natural_CS->x
//						   +((cos(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Gamma)) + (sin(Trsfor_Ang->Alpha) * sin(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta))) * Natural_CS->y
//						   +((sin(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Gamma)) - (sin(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta) * cos(Trsfor_Ang->Alpha))) * Natural_CS->z;
//			
//			Unnatrl_CS->z= +sin(Trsfor_Ang->Beta) * Natural_CS->x
//						   -(sin(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Beta)) * Natural_CS->y
//						   + (cos(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Beta))* Natural_CS->z;

//			break;
//		}
//		
//		case (Reverse):
//		{
//			//即上面三维旋转矩阵的逆阵			
//			Natural_CS->x= +(cos(Trsfor_Ang->Gamma) * cos(Trsfor_Ang->Beta))* Unnatrl_CS->x
//						   -(sin(Trsfor_Ang->Gamma) * cos(Trsfor_Ang->Beta))* Unnatrl_CS->y
//						   +sin(Trsfor_Ang->Beta) * Unnatrl_CS->z;
//			
//			Natural_CS->y= +((sin(Trsfor_Ang->Gamma) * cos(Trsfor_Ang->Alpha)) - (cos(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta) * sin(Trsfor_Ang->Alpha))) * Unnatrl_CS->x
//						   +((cos(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Gamma)) + (sin(Trsfor_Ang->Alpha) * sin(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta))) * Unnatrl_CS->y
//						   -(sin(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Beta)) * Unnatrl_CS->z;
//			
//			Natural_CS->z= +((sin(Trsfor_Ang->Alpha) * sin(Trsfor_Ang->Gamma)) + (cos(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta) * cos(Trsfor_Ang->Alpha))) * Unnatrl_CS->x
//						   +((sin(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Gamma)) - (sin(Trsfor_Ang->Gamma) * sin(Trsfor_Ang->Beta) * cos(Trsfor_Ang->Alpha))) * Unnatrl_CS->y
//						   +(cos(Trsfor_Ang->Alpha) * cos(Trsfor_Ang->Beta)) * Unnatrl_CS->z;			
//			break;
//		}
//	}
//}

///*****************************************************************************
//**@Brief:	找出距某连续值最近的一界限值(在两个界限中)，并且求出离当前值最近的该界限值的非连续数值(单圈数值)
//**@Cal:		no
//**@param:  	originate_value——输入的连续值
//**			positive_value ——正向界限的单圈值(此处写为Yaw轴车头指向车正方向时Yaw轴电机的编码值)
//**			reverse_value  ——负向界限的单圈值(此处写为Yaw轴车头指向车负方向时Yaw轴电机的编码值)
//**			period_range   ——非连续值的范围(此处写为Yaw轴电机的单圈编码器值范围)
//**			Nearest_Value  ——最近界限的单圈值
//**@Note:   	
//**@RetVal: 	
//*****************************************************************************/
//float Nearest_Neighbor_Search (float originate_value, float positive_value, float reverse_value, float period_range)
//{
//	float Nearest_Value;

//	if (
//		fabs(((int)originate_value % (int)period_range) - positive_value) >=
//		fabs(((int)originate_value % (int)period_range) - reverse_value)
//	   )
//	{ Nearest_Value= positive_value;}
//	else { Nearest_Value= reverse_value;}
//	
//	return Nearest_Value;
//}
