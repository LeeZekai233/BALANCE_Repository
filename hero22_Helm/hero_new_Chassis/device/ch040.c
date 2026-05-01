#include "public.h"
general_gyro_t chassis_gyro;
 uint8_t Pitch_Init_Flag;
void CH040_getDATA(uint8_t *addr_data,general_gyro_t*gyro)
{        
    static __align(4) id0x91_t dat;
    memcpy(&dat, &addr_data[6], sizeof(id0x91_t));
    
    volatile static float Last_yaw_temp1, Yaw_temp1;
    
    Last_yaw_temp1=Yaw_temp1;
    Yaw_temp1=dat.eul[2];
    
    if(Yaw_temp1-Last_yaw_temp1<=-320)
    {
        gyro->Yaw_count++;
    }
    else if(Yaw_temp1-Last_yaw_temp1>=320)
    {
        gyro->Yaw_count--;
    }
        
    gyro->pitch_angle=-dat.eul[0];
    gyro->roll_angle=dat.eul[1];
    gyro->yaw_angle=Yaw_temp1+gyro->Yaw_count*360;
    
    gyro->x_acc=-dat.acc[0];
    gyro->y_acc=dat.acc[2];
    gyro->z_acc=dat.acc[1];
    
    gyro->pitch_palstance=-dat.gyr[1];
    gyro->roll_palstance=dat.gyr[0];
    gyro->yaw_palstance=dat.gyr[2];
    
    
}