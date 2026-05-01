#ifndef __SENSOR_H__
#define __SENSOR_H__


typedef struct
{
	volatile Encoder Driving_Encoder[4];//3508
	volatile Encoder Heading_Encoder[4];//6020
}Helm_wheel_t;



#endif
