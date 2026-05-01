
#ifndef __CH040_H__
#define __CH040_H__

__packed typedef struct
{
	uint8_t tag; //数据包标签0x91
	uint8_t id; 
	uint8_t rev[2];//保留
	float prs; //气压
	uint32_t ts; //时间戳信息，从系统开机开始累加，每毫秒增加1
	float acc[3]; 
	float gyr[3]; //角速度，单位deg/s，顺序xyz，为啥用他妈gry表示角速度，gry不是陀螺仪吗?用palstance更合适
	float mag[3]; //磁强度，顺序xyz
	float eul[3]; //节点欧拉角
	float quat[4];//节点四元数集合
}id0x91_t;


typedef struct
{
    float yaw_angle;
    float pitch_angle;
    float roll_angle;
    float yaw_palstance;
    float pitch_palstance;
    float roll_palstance;
    float x_acc;
    float y_acc;
    float z_acc;
	int	  Yaw_count;
}general_gyro_t;

extern general_gyro_t gimbal_gyro;
extern uint8_t Pitch_Init_Flag;
void CH040_getDATA(uint8_t *addr_data,general_gyro_t*gyro);

#endif
