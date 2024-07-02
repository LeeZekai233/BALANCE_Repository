#ifndef __TF02_H
#define __TF02_H
#include "public.h"


#define TF02_BUF_LEN	      	9u
#define TF02_FrameHeader        0x59

/* 9�ֽ����ݱ���˵��
 * ֡ͷ0x59
 * ֡ͷ0x59
 * Dist_L
 * Dist_H
 * Strength_L
 * Strength_H
 * Temp_L
 * Temp_H
 * Checksum
 */

/* distance_raw :
		���ź�ǿ��ֵStrength��60ʱ��Dist�Ĳ���ֵ����Ϊ�����ţ�Dist���ֵΪ4500��
		���ź�ǿ�ȴ���60��ʵ�ʾ�����45~60mʱ��Dist���ֵΪ4500��
		���ź�ǿ�ȴ���60��ʵ�ʾ��볬��60mʱ�����й��������ݳ��֣���ʱ����Ϊ0�������쳣ֵ��*/

/* strength_raw :
		�ź�ǿ�ȣ�Ĭ�����ֵ����0-65535֮�䡣
		����൵λһ��ʱ�����ԽԶ���ź�ǿ��Խ�ͣ�
		Ŀ���ﷴ����Խ�ͣ��ź�ǿ��Խ�͡� */
		
/* temp_raw:
		����оƬ�ڲ��¶�ֵ�����϶�=Temp/8-256. */
			
typedef struct
{
    uint16_t distance_raw;	//cm	0-4500
    uint16_t strength_raw;	 
    int16_t temp_raw;
    float temp_centi;       // �����¶�
    uint8_t checkSum;
} TF02_t;


void TF02_DataProgress(uint8_t *pdata,float *distance);


extern TF02_t TF02;


#endif

