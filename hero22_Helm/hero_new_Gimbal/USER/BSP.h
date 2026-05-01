#ifndef __BSP_H__
#define __BSP_H__

#define ANGLE_TO_RAD 0.01745329251994329576923690768489f

void BSP_Init(void);
float convert_ecd_angle_to_0_2180(double ecd_angle,float _0_2pi_angle);
float convert_ecd_angle_to__pi_pi(double ecd_angle,float __pi_pi_angle);


#endif

