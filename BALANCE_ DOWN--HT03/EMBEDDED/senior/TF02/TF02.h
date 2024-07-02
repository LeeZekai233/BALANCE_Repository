#ifndef __TF02_H
#define __TF02_H
#include "public.h"


#define TF02_BUF_LEN	      	9u
#define TF02_FrameHeader        0x59

/* 9字节数据编码说明
 * 帧头0x59
 * 帧头0x59
 * Dist_L
 * Dist_H
 * Strength_L
 * Strength_H
 * Temp_L
 * Temp_H
 * Checksum
 */

/* distance_raw :
		当信号强度值Strength≤60时，Dist的测量值被认为不可信，Dist输出值为4500。
		当信号强度大于60且实际距离在45~60m时，Dist输出值为4500。
		当信号强度大于60且实际距离超过60m时，会有过周期数据出现，此时数据为0或其他异常值。*/

/* strength_raw :
		信号强度，默认输出值会在0-65535之间。
		当测距档位一定时，测距越远，信号强度越低；
		目标物反射率越低，信号强度越低。 */
		
/* temp_raw:
		表征芯片内部温度值。摄氏度=Temp/8-256. */
			
typedef struct
{
    uint16_t distance_raw;	//cm	0-4500
    uint16_t strength_raw;	 
    int16_t temp_raw;
    float temp_centi;       // 摄氏温度
    uint8_t checkSum;
} TF02_t;


void TF02_DataProgress(uint8_t *pdata,float *distance);


extern TF02_t TF02;


#endif

